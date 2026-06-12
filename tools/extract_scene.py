"""Extract scene assembly for the DTM_GRID campus terrain.

Reads (tagged properties only -- no geometry decode):
  MESHES/DTM_GRID/GRID_DTM_COMBINED.uasset          Blueprint tile placement
  MESHES/DTM_GRID/Materials/*.uasset                MaterialInstanceConstant -> texture
  MESHES/DTM_GRID/Meshes/*.uasset                   StaticMesh -> material + bounds

Writes:
  extracted/manifest-scene.json

Transform conventions (UE 4.24):
  FRotator serialized order = (Pitch, Yaw, Roll), degrees.
  Row-vector convention: v_world = v_local * M(rot) * scale + T, composed
  child-to-parent up the SCS tree.  See rot_matrix() (FRotationMatrix port).
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uasset import Package, Reader

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GRID = os.path.join(ROOT, 'MESHES', 'DTM_GRID')
OUT = os.path.join(ROOT, 'extracted')


# ---------------------------------------------------------------------------
# UE rotation math (FRotationMatrix / FMatrix::Rotator ports, row-vector)
# ---------------------------------------------------------------------------
def rot_matrix(pitch, yaw, roll):
    """UE FRotationMatrix: rows are the rotated X/Y/Z axes (degrees in)."""
    p, y, r = (math.radians(v) for v in (pitch, yaw, roll))
    SP, CP = math.sin(p), math.cos(p)
    SY, CY = math.sin(y), math.cos(y)
    SR, CR = math.sin(r), math.cos(r)
    return [
        [CP * CY,                CP * SY,               SP],
        [SR * SP * CY - CR * SY, SR * SP * SY + CR * CY, -SR * CP],
        [-(CR * SP * CY + SR * SY), CY * SR - CR * SP * SY, CR * CP],
    ]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def vec_mat(v, m):
    """Row vector times matrix: v * M."""
    return [sum(v[i] * m[i][j] for i in range(3)) for j in range(3)]


def matrix_to_rotator(m):
    """UE FMatrix::Rotator(): returns (pitch, yaw, roll) degrees."""
    xaxis, yaxis, zaxis = m[0], m[1], m[2]
    pitch = math.degrees(math.atan2(xaxis[2],
                                    math.sqrt(xaxis[0] ** 2 + xaxis[1] ** 2)))
    yaw = math.degrees(math.atan2(xaxis[1], xaxis[0]))
    sy = rot_matrix(pitch, yaw, 0.0)[1]  # Y axis of the pitch/yaw-only matrix
    dot = lambda a, b: sum(x * y for x, y in zip(a, b))
    roll = math.degrees(math.atan2(dot(zaxis, sy), dot(yaxis, sy)))
    return [pitch, yaw, roll]


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def props_of(pkg, export):
    r = Reader(pkg.data, export['serial_offset'])
    return pkg.read_properties(r)


def prop(props, name, default=None):
    for t in props:
        if t['name'] == name:
            return t['value']
    return default


def struct_items(arr_value):
    """Items of an ArrayProperty-of-StructProperty value."""
    if not isinstance(arr_value, dict):
        return []
    return arr_value.get('items') or []


def import_package_path(pkg, import_index):
    """Resolve an import (negative FPackageIndex) to its outermost package
    object path, e.g. /Game/Campus/Meshes_old/DTM_GRID/Meshes/15626E185064N."""
    idx = import_index
    while True:
        im = pkg.imports[-idx - 1]
        if im['outer_index'] == 0:
            return im['object_name']
        idx = im['outer_index']


# ---------------------------------------------------------------------------
# Blueprint: SimpleConstructionScript -> per-tile world transforms
# ---------------------------------------------------------------------------
def parse_blueprint(path):
    p = Package(path)
    exports = p.exports
    by_index = {i + 1: e for i, e in enumerate(exports)}

    scs = next(e for e in exports
               if p.class_of(e) == 'SimpleConstructionScript')
    scs_props = props_of(p, scs)
    root_nodes = prop(scs_props, 'RootNodes', [])

    nodes = {}      # export index -> parsed SCS node dict

    def parse_node(idx):
        e = by_index[idx]
        np = props_of(p, e)
        tmpl_idx = prop(np, 'ComponentTemplate')
        tmpl = by_index[tmpl_idx]
        tp = props_of(p, tmpl)
        node = {
            'scs_export': idx,
            'scs_name': e['object_name'],
            'var_name': prop(np, 'InternalVariableName'),
            'component_class': p.obj_name(prop(np, 'ComponentClass')),
            'template_name': tmpl['object_name'],
            'children': prop(np, 'ChildNodes', []) or [],
            'relative_location': list(prop(tp, 'RelativeLocation',
                                           (0.0, 0.0, 0.0))),
            'relative_rotation': list(prop(tp, 'RelativeRotation',
                                           (0.0, 0.0, 0.0))),  # P,Y,R deg
            'relative_scale': list(prop(tp, 'RelativeScale3D',
                                        (1.0, 1.0, 1.0))),
            'visible': bool(prop(tp, 'bVisible', True)),
            'hidden_in_game': bool(prop(tp, 'bHiddenInGame', False)),
        }
        sm = prop(tp, 'StaticMesh')
        if sm is not None and sm != 0:
            node['static_mesh'] = p.obj_name(sm)
            node['static_mesh_pkg'] = import_package_path(p, sm)
        om = prop(tp, 'OverrideMaterials')
        if om:
            node['override_materials'] = [p.obj_name(x) for x in om]
        nodes[idx] = node
        return node

    # walk tree, composing transforms (UE row-vector: v*M_child*M_parent...)
    tiles = []
    def walk(idx, parent_m, parent_t, parent_s, parent_rot_chain):
        n = parse_node(idx)
        pch, yw, rl = n['relative_rotation']
        m_rel = rot_matrix(pch, yw, rl)
        s_rel = n['relative_scale']
        t_loc = [n['relative_location'][i] * parent_s[i]
                 for i in range(3)]  # scale then rotate into parent frame
        t_world = [vec_mat(t_loc, parent_m)[i] + parent_t[i]
                   for i in range(3)]
        m_world = mat_mul(m_rel, parent_m)
        s_world = [s_rel[i] * parent_s[i] for i in range(3)]
        n['world_translation_cm'] = t_world
        n['world_rotation_deg_pyr'] = matrix_to_rotator(m_world)
        n['world_scale'] = s_world
        n['parent_chain'] = parent_rot_chain
        if 'static_mesh' in n:
            tiles.append(n)
        for c in n['children']:
            walk(c, m_world, t_world, s_world,
                 parent_rot_chain + [n['var_name']])

    ident = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    for r in root_nodes:
        walk(r, ident, [0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [])
    return p, nodes, tiles


# ---------------------------------------------------------------------------
# MaterialInstanceConstant -> texture / params
# ---------------------------------------------------------------------------
def parse_material_instance(path):
    p = Package(path)
    e = next(x for x in p.exports
             if p.class_of(x) == 'MaterialInstanceConstant')
    pr = props_of(p, e)
    out = {'name': e['object_name'], 'class': 'MaterialInstanceConstant'}
    parent = prop(pr, 'Parent')
    if parent:
        out['parent'] = p.obj_name(parent)

    def param_entries(arr_name):
        res = []
        for item in struct_items(prop(pr, arr_name, {})):
            info = prop(item, 'ParameterInfo', [])
            res.append({
                'name': prop(info, 'Name') if isinstance(info, list) else None,
                'value': prop(item, 'ParameterValue'),
            })
        return res

    texs = []
    for t in param_entries('TextureParameterValues'):
        v = t['value']
        texs.append({
            'param': t['name'],
            'texture': p.obj_name(v),
            'texture_pkg': import_package_path(p, v) if v and v < 0 else None,
        })
    out['texture_parameters'] = texs
    out['scalar_parameters'] = [
        {'param': t['name'], 'value': t['value']}
        for t in param_entries('ScalarParameterValues')]
    out['vector_parameters'] = [
        {'param': t['name'], 'value': t['value']}
        for t in param_entries('VectorParameterValues')]
    bpo = prop(pr, 'BasePropertyOverrides')
    if isinstance(bpo, list):
        out['base_property_overrides'] = {
            t['name']: t['value'] for t in bpo}
    return out


def parse_base_material(path):
    """M_CAMPUS_BASE: report the expression graph compactly."""
    p = Package(path)
    mat = next(x for x in p.exports if p.class_of(x) == 'Material')
    pr = props_of(p, mat)
    out = {'name': mat['object_name'], 'class': 'Material', 'inputs': {}}
    # ColorMaterialInput/ScalarMaterialInput are native structs:
    # i32 Expression(FPackageIndex), i32 OutputIndex, FName InputName(8B),
    # ... constants.  First i32 = expression export index.
    for t in pr:
        if t.get('struct_name', '') in ('ColorMaterialInput',
                                        'ScalarMaterialInput',
                                        'VectorMaterialInput'):
            raw = p.data[t['value_offset']:t['value_offset'] + 4]
            expr_idx = int.from_bytes(raw, 'little', signed=True)
            desc = None
            if expr_idx > 0:
                ee = p.exports[expr_idx - 1]
                ep = props_of(p, ee)
                cls = p.class_of(ee)
                if cls == 'MaterialExpressionConstant':
                    desc = {'expr': cls, 'value': prop(ep, 'R', 0.0)}
                elif cls == 'MaterialExpressionTextureSampleParameter2D':
                    desc = {'expr': cls,
                            'parameter': prop(ep, 'ParameterName')}
                else:
                    desc = {'expr': cls}
            out['inputs'][t['name']] = desc
    return out


# ---------------------------------------------------------------------------
# StaticMesh tagged props -> material reference + bounds (no geometry)
# ---------------------------------------------------------------------------
def parse_mesh(path):
    p = Package(path)
    e = next(x for x in p.exports if p.class_of(x) == 'StaticMesh')
    pr = props_of(p, e)
    out = {'name': e['object_name']}
    mats = []
    for item in struct_items(prop(pr, 'StaticMaterials', {})):
        mi = prop(item, 'MaterialInterface')
        mats.append({
            'material': p.obj_name(mi),
            'material_pkg': import_package_path(p, mi) if mi and mi < 0
                            else None,
            'slot': prop(item, 'MaterialSlotName'),
        })
    out['static_materials'] = mats
    eb = prop(pr, 'ExtendedBounds')
    if isinstance(eb, list):
        out['extended_bounds'] = {
            'origin': list(prop(eb, 'Origin', (0, 0, 0))),
            'box_extent': list(prop(eb, 'BoxExtent', (0, 0, 0))),
            'sphere_radius': prop(eb, 'SphereRadius'),
        }
    return out


# ---------------------------------------------------------------------------
def write_report(manifest, path):
    m = manifest
    T = m['origin_node']['translation_cm']
    rows = []
    for t in sorted(m['tiles'], key=lambda t: t['name']):
        o = t['extended_bounds']['origin']
        e = t['extended_bounds']['box_extent']
        wc = [o[0] + T[0], -o[2] + T[1], o[1] + T[2]]      # world = (x,-z,y)+T
        we = [e[0], e[2], e[1]]
        rows.append(
            f"{t['name']}  ({wc[0]/100:7.1f},{wc[1]/100:8.1f},{wc[2]/100:6.1f})"
            f"  ({we[0]/100:5.1f},{we[1]/100:5.1f},{we[2]/100:5.1f})"
            + ('' if t['visible'] else '  HIDDEN'))
    table = '\n'.join(rows)
    unmatched = ', '.join(m['unmatched_textures'])
    unused = ', '.join(m['unused_materials'])

    text = f"""# Scene assembly extraction (domain: scene)

Source: `MESHES/DTM_GRID/GRID_DTM_COMBINED.uasset` (Blueprint),
`MESHES/DTM_GRID/Materials/*.uasset` (17 MaterialInstanceConstant + 1 Material),
`MESHES/DTM_GRID/Meshes/*.uasset` (16 StaticMesh, tagged props only).
Tool: `tools/extract_scene.py` -> `extracted/manifest-scene.json` (re-runnable;
this report is generated by the same script).

## Headline result -- THE ONE TRANSFORM THE VIEWER MUST APPLY

The 16 tile StaticMeshComponents have **no per-tile offsets**. They are all
children of a single SceneComponent named `origin` which carries the only
non-trivial transform in the Blueprint:

```
origin.RelativeLocation  = ( 25604.033203125, -77057.078125, -36982.9921875 )  cm
origin.RelativeRotation  = ( Pitch=0, Yaw=0, Roll=-90 )  degrees
origin.RelativeScale3D   = ( 1, 1, 1 )
```

The mesh vertices as stored in the .uasset files are in the FBX-import local
frame: **X = easting, Y = elevation (up), Z = northing** (Y-up). The Roll=-90
(rotation about UE +X) converts that to UE world Z-up. Net effect:

```
world_cm = ( x_local,  -z_local,  y_local ) + ( 25604.033, -77057.078, -36982.992 )
```

(UE world: X = easting-ish, **+Y = south** -- northing increases toward -Y --
Z = up.) This is NOT optional: per the data contract mesh .bin files keep
vertices exactly as stored in the asset, so the integration/viewer agent must
apply the transform above to every terrain tile (identical for all 16).

### Verification
Applying the transform to each mesh's `ExtendedBounds` (read from StaticMesh
tagged props) makes the visible tiles span world
x in [-912.8, 914.8] m, y in [-1688.1, 1694.7] m, z in [-86.9, -45.7] m,
which matches the LiDAR octree bounds (+-915.25, +-1703.97, +-168.73 m)
almost exactly -- terrain and point cloud register in the same world frame
with this transform; the LiDAR needs NO extra offset relative to it.

## Blueprint structure (SimpleConstructionScript)

```
SCS_Node_0  DefaultSceneRoot (SceneComponent, identity)
  \\- SCS_Node_3  origin (SceneComponent, transform above)
       \\- 16 SCS_Nodes -> StaticMeshComponent templates, one per tile
```

Per-component template props (export `<TILE>_GEN_VARIABLE`):
- `StaticMesh` (ObjectProperty -> import) -- always the same-name StaticMesh
  package `/Game/Campus/Meshes_old/DTM_GRID/Meshes/<TILE>`.
- NO `RelativeLocation` / `RelativeScale3D` on any tile component.
- `RelativeRotation` present on only two components -- `15626E192984N` and
  `15652E185064N` -- value (0, 0, 1.0245e-05 deg) roll: numerically zero,
  ignore.
- `OverrideMaterials` present only on `15626E185064N` -> MIC `15626E185064N`,
  the same material the mesh already references. Harmless.
- **Two tiles are hidden in the Blueprint**: `15626E185064N` and
  `15678E185064N` have `bVisible=False, bHiddenInGame=True`. They are the two
  thin southern sliver tiles (y-extent 19 m / 27 m; also the two smallest mesh
  files). Manifest field `visible` records this; viewer may honour or ignore.
- Cosmetic flags on all: `CastShadow=False`, `bTreatAsBackgroundForOcclusion`,
  `LDMaxDrawDistance=50000` (UE draw-distance cull -- ignore for viewer).

Rotator convention in the manifest (`rotation_deg`): UE FRotator serialized
order **[Pitch, Yaw, Roll]** degrees; transforms composed with the UE
FRotationMatrix row-vector convention (world = v * M_child * M_parent + T).
All 16 tiles compose to rotation [0, 0, -90] (two are -89.9999898 from the
1e-5 noise) with identical translation.

## Materials -> textures (1:1 by name, with 2 exceptions)

Each of the 17 MaterialInstanceConstant packages contains:
- `Parent` -> `M_CAMPUS_BASE`
- `TextureParameterValues`: exactly one entry, parameter name **`TEXTURE`** ->
  ObjectProperty import `Texture2D <same name as the material>`
  (package `/Game/Campus/Meshes_old/DTM_GRID/Textures/<NAME>`).
- `ScalarParameterValues`: one entry `RefractionDepthBias = 0.0` (irrelevant).
- No `VectorParameterValues`.
- `BasePropertyOverrides`: `OpacityMaskClipValue = 0.3333` (irrelevant; the
  base material is opaque, `bCanMaskedBeAssumedOpaque=True`).

`M_CAMPUS_BASE` (Material, 3-node graph) decoded from its native
FExpressionInput structs (first int32 of `ColorMaterialInput` /
`ScalarMaterialInput` raw bytes = expression FPackageIndex):

| Material input | Source |
|---|---|
| BaseColor | TextureSampleParameter2D `TEXTURE` |
| Metallic  | Constant 0.0 |
| Specular  | Constant 0.0 |
| Roughness | Constant 1.0 |

Viewer replication: fully rough, non-metallic diffuse texture, i.e.
MeshStandardMaterial(roughness=1, metalness=0) or simply MeshBasicMaterial /
Lambert with the ortho JPEG. No brightness/tint multipliers exist.

## Meshes -> materials

Each of the 16 StaticMesh packages: `StaticMaterials` array has exactly one
slot, `MaterialSlotName = "Default OBJ"`, `MaterialInterface` -> import of the
**same-name** MaterialInstanceConstant. The chain is strictly
mesh NAME -> material NAME -> texture NAME for all 16 tiles.

## Inventory accounting (16 meshes / 17+1 materials / 18 textures)

- Meshes (16): columns 15626/15652/15678 x rows 185064..195624, plus
  15652E198264N. **Missing meshes:** 15626E198264N, 15678E198264N.
- Materials: 17 MICs + M_CAMPUS_BASE. **15626E198264N has no material.**
- Textures (18): all 3x6 grid cells.
- `unmatched_textures` = [{unmatched}] -- textures with no mesh to map onto
  (15678E198264N also has an orphan, unused MIC; 15626E198264N has neither
  material nor mesh). The two N-row corner tiles of the 3x6 grid were never
  meshed (outside DTM coverage).
- `unused_materials` = [{unused}].

## Grid geometry sanity (from ExtendedBounds, local frame)

Northing rows step local z by ~80,440 cm = 2640 ft (half mile) between full
rows; full tiles are ~804.4 m square (extent 414.0-414.2 m incl. UE
ExtendedBounds padding). Edge tiles are partial: row 185064 = thin south
slivers, row 198264 = 68 m strip, column 15626 ~ 183-186 m wide, column
15678 ~ 294-309 m. Elevation (local y) ~ 286-315 m -> world z ~ -84..-46 m.

World-space tile centers and extents (m, after transform; x, y, z):

```
{table}
```

## Manifest format (`extracted/manifest-scene.json`)

```
{{ domain, source_blueprint, coordinate_note,
  origin_node: {{translation_cm, rotation_pyr_deg, scale}},
  tiles: [ {{ name, mesh_pkg, mesh_file, material, texture,
             translation_cm[3],          # identical for all tiles
             rotation_deg[3],            # UE FRotator [pitch,yaw,roll] = [0,0,-90]
             scale[3],                   # [1,1,1]
             relative: {{location, rotation_pyr_deg, scale}},  # raw component level
             visible,                    # false for the 2 hidden slivers
             material_from_override,
             extended_bounds: {{origin, box_extent, sphere_radius}}  # LOCAL frame
         }} x16 ],
  unmatched_textures, unused_materials, all_textures[18],
  base_material,            # M_CAMPUS_BASE decoded graph
  material_instances }}      # full per-MIC params
```

## Notes for integration/viewer agent (MUST READ)

1. Apply world = (x, -z, y) + (25604.033, -77057.078, -36982.992) cm to every
   terrain tile vertex (or as a node transform). Same for all 16 tiles.
2. LiDAR shares this world frame directly (verified by bounds match); do NOT
   apply the tile transform to the point cloud.
3. +Y is SOUTH in this UE world frame (northing -> -Y). The usual UE->three.js
   conversion (x, z, y) then handles axis/handedness as per PLAN.md.
4. Two tiles flagged visible:false (15626E185064N, 15678E185064N) are tiny
   slivers; safe to load anyway or honour the flag.
5. Texture/material params carry nothing visually important: plain diffuse
   ortho texture, roughness 1, metalness 0.
"""
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    print('wrote', path)


def main():
    os.makedirs(OUT, exist_ok=True)

    bp_path = os.path.join(GRID, 'GRID_DTM_COMBINED.uasset')
    print('parsing blueprint', bp_path)
    bp_pkg, nodes, bp_tiles = parse_blueprint(bp_path)

    mat_dir = os.path.join(GRID, 'Materials')
    mesh_dir = os.path.join(GRID, 'Meshes')
    tex_dir = os.path.join(GRID, 'Textures')

    materials = {}
    base_material = None
    for f in sorted(os.listdir(mat_dir)):
        if not f.endswith('.uasset'):
            continue
        path = os.path.join(mat_dir, f)
        name = f[:-7]
        if name == 'M_CAMPUS_BASE':
            base_material = parse_base_material(path)
            print('  base material', name)
        else:
            materials[name] = parse_material_instance(path)
            print('  material', name)

    meshes = {}
    for f in sorted(os.listdir(mesh_dir)):
        if not f.endswith('.uasset'):
            continue
        name = f[:-7]
        meshes[name] = parse_mesh(os.path.join(mesh_dir, f))
        print('  mesh', name)

    texture_names = sorted(f[:-7] for f in os.listdir(tex_dir)
                           if f.endswith('.uasset'))

    # ---- assemble tiles --------------------------------------------------
    tiles = []
    used_textures = set()
    for n in sorted(bp_tiles, key=lambda x: x['var_name']):
        name = n['var_name']
        mesh = meshes.get(name, {})
        mesh_mat = (mesh.get('static_materials') or [{}])[0]
        mat_name = None
        if n.get('override_materials'):
            mat_name = n['override_materials'][0]
        elif mesh_mat.get('material'):
            mat_name = mesh_mat['material']
        mat = materials.get(mat_name, {})
        tex = None
        for tp in mat.get('texture_parameters', []):
            if tp['param'] == 'TEXTURE':
                tex = tp['texture']
        if tex:
            used_textures.add(tex)
        tiles.append({
            'name': name,
            'mesh_pkg': n.get('static_mesh_pkg'),
            'mesh_file': os.path.join('MESHES/DTM_GRID/Meshes',
                                      name + '.uasset').replace('\\', '/'),
            'material': mat_name,
            'texture': tex,
            'translation_cm': n['world_translation_cm'],
            'rotation_deg': n['world_rotation_deg_pyr'],  # [pitch,yaw,roll]
            'scale': n['world_scale'],
            'relative': {
                'location': n['relative_location'],
                'rotation_pyr_deg': n['relative_rotation'],
                'scale': n['relative_scale'],
            },
            'visible': n['visible'] and not n['hidden_in_game'],
            'material_from_override': bool(n.get('override_materials')),
            'extended_bounds': mesh.get('extended_bounds'),
        })

    unmatched_textures = [t for t in texture_names if t not in used_textures]
    unused_materials = [m for m in sorted(materials)
                        if m not in {t['material'] for t in tiles}]

    origin = next((n for n in nodes.values() if n['var_name'] == 'origin'),
                  None)
    manifest = {
        'domain': 'scene',
        'source_blueprint': 'MESHES/DTM_GRID/GRID_DTM_COMBINED.uasset',
        'coordinate_note': (
            'UE world cm, Z-up.  world = v_local * M(rotation_deg as UE '
            'FRotator [pitch,yaw,roll]) * scale + translation_cm.  With the '
            'shared origin rotation (0,0,-90): world = (x, -z, y) + T.'),
        'origin_node': {
            'translation_cm': origin['relative_location'],
            'rotation_pyr_deg': origin['relative_rotation'],
            'scale': origin['relative_scale'],
        } if origin else None,
        'tiles': tiles,
        'unmatched_textures': unmatched_textures,
        'unused_materials': unused_materials,
        'all_textures': texture_names,
        'base_material': base_material,
        'material_instances': materials,
    }

    out_path = os.path.join(OUT, 'manifest-scene.json')
    with open(out_path, 'w') as f:
        json.dump(manifest, f, indent=2)
    print('wrote', out_path)

    write_report(manifest, os.path.join(OUT, 'REPORT-scene.md'))

    # quick console sanity table
    print(f'\n{len(tiles)} tiles; {len(unmatched_textures)} unmatched '
          f'textures: {unmatched_textures}; unused materials: '
          f'{unused_materials}')
    for t in tiles:
        eb = t['extended_bounds']
        o = eb['origin'] if eb else None
        print(f"  {t['name']}: mat={t['material']} tex={t['texture']} "
              f"vis={t['visible']} rot={['%g' % v for v in t['rotation_deg']]} "
              f"local_bounds_origin={o}")


if __name__ == '__main__':
    main()
