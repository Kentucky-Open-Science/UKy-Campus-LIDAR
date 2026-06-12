"""Extract StaticMesh terrain tiles (UE 4.24 FMeshDescription) to web .bin format.

Usage:
    python tools/extract_mesh.py MESHES/DTM_GRID/Meshes/15626E185064N.uasset
    python tools/extract_mesh.py --all

Output (little-endian, per data contract):
    web/data/meshes/<NAME>.bin:
        u32 vert_count, u32 index_count,
        f32 positions[vert_count*3]  (UE cm, world coords as stored),
        f32 uvs[vert_count*2]        (UV channel 0, raw),
        u32 indices[index_count]     (triangle list, winding as stored)

Reverse-engineered FMeshDescription layout (UE 4.24.3, verified empirically):
    Element array (TMeshElementArray): u32 ArraySize, allocation bitmask of
        ceil(ArraySize/32) u32 words, then per-ALLOCATED-element payload:
        - Vertex: nothing
        - VertexInstance: i32 parent VertexID
        - Edge: 2 x i32 VertexIDs
        - Polygon: TArray<i32> contour (count then ids; empty in these assets,
          superseded by the triangle block) + i32 PolygonGroupID
        - PolygonGroup: nothing
    Order: VertexArray, VertexInstanceArray, EdgeArray, PolygonArray,
        PolygonGroupArray, then 5 attribute sets (vertex, vertexinstance,
        edge, polygon, polygongroup), then TriangleArray:
        u32 size + bitmask + per-tri {i32 vi0, vi1, vi2, i32 PolygonID},
        then triangle attribute set {u32 NumElements, u32 NumAttributes(=0)}.
    Attribute set: u32 NumElements, u32 NumAttributes, per attribute:
        FString Name (FName as string, may carry trailing space),
        u32 Type (0=FVector4,1=FVector,2=FVector2D,3=float,4=int32,5=bool,6=FName),
        u32 NumElements, u32 NumIndices,
        per index: numeric -> BulkSerialize {u32 ElemSize, u32 Count, raw};
                   FName   -> u32 Count, Count x FString,
        DefaultValue (ElemSize bytes raw; FString for FName),
        u32 Flags.

Vertex output policy: dedupe identical (parent VertexID, UV0) pairs across
vertex instances (TIN ortho UVs are per-vertex constant, so output vert count
== source vertex count); indices remap triangle vertex-instance ids.
"""
import argparse
import json
import os
import struct
import sys

# tools/inspect.py shadows stdlib 'inspect' (needed by numpy) when this dir is
# on sys.path; import numpy with the tools dir removed, then restore it.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path if os.path.abspath(p or '.') != _HERE]
import numpy as np  # noqa: E402

sys.path.insert(0, _HERE)
from uasset import Package, Reader, decompress_chunked  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESH_DIR = os.path.join(ROOT, 'MESHES', 'DTM_GRID', 'Meshes')
OUT_DIR = os.path.join(ROOT, 'web', 'data', 'meshes')

# per-element sizes in BulkSerialized arrays (bool packs to 1 byte)
TYPE_SIZES = {0: 16, 1: 12, 2: 8, 3: 4, 4: 4, 5: 1}
# serialized size of the DEFAULT value (bool serializes as u32 via FArchive)
DEFAULT_SIZES = {0: 16, 1: 12, 2: 8, 3: 4, 4: 4, 5: 4}


class MeshDescError(Exception):
    pass


def read_element_array(r, label):
    """Returns (size, allocated_count). Asserts dense allocation (no holes)."""
    size = r.u32()
    nwords = (size + 31) // 32
    words = np.frombuffer(r.read(nwords * 4), dtype=np.uint32)
    nalloc = int(np.unpackbits(words.view(np.uint8), bitorder='little').sum())
    if nalloc != size:
        raise MeshDescError(f'{label}: sparse holes ({nalloc}/{size}) unsupported')
    return size


def read_attribute_set(r, label):
    """Parse one TAttributesSet; return dict name->list of per-index raw
    payloads (bytes for numeric, list[str] for FName) plus metadata."""
    num_elements = r.u32()
    num_attribs = r.u32()
    attrs = {}
    for _ in range(num_attribs):
        name = r.fstring().strip().strip('\x00')
        atype = r.u32()
        n_elem = r.u32()
        n_indices = r.u32()
        indices = []
        elem_size = None
        if atype == 6:  # FName -> string array
            for _ in range(n_indices):
                cnt = r.u32()
                indices.append([r.fstring() for _ in range(cnt)])
            default = r.fstring()
        else:
            if atype not in DEFAULT_SIZES:
                raise MeshDescError(f'{label}.{name}: unknown type {atype}')
            for _ in range(n_indices):
                es = r.u32()
                cnt = r.u32()
                elem_size = es
                indices.append(r.read(es * cnt))
                if cnt != n_elem:
                    raise MeshDescError(
                        f'{label}.{name}: count {cnt} != NumElements {n_elem}')
            default = r.read(DEFAULT_SIZES[atype])
        flags = r.u32()
        attrs[name] = {'type': atype, 'num_elements': n_elem,
                       'indices': indices, 'default': default, 'flags': flags}
    return num_elements, attrs


def parse_mesh_description(dec):
    """Parse decompressed FMeshDescription blob -> dict of numpy arrays."""
    r = Reader(dec)
    n_verts = read_element_array(r, 'VertexArray')

    n_inst = read_element_array(r, 'VertexInstanceArray')
    inst_vertex = np.frombuffer(r.read(n_inst * 4), dtype=np.int32)

    n_edges = read_element_array(r, 'EdgeArray')
    r.read(n_edges * 8)  # 2 x i32 vertex ids; not needed

    n_polys = read_element_array(r, 'PolygonArray')
    # fast path: peek if all contours empty (8 bytes/poly, contour count == 0)
    peek = np.frombuffer(dec[r.tell():r.tell() + n_polys * 8], dtype=np.int32)
    poly_contours = None
    if len(peek) == n_polys * 2 and not peek[0::2].any():
        poly_groups = peek[1::2].copy()
        r.read(n_polys * 8)
    else:  # general path: {count, ids..., group}
        poly_contours, poly_groups = [], np.empty(n_polys, np.int32)
        for i in range(n_polys):
            cnt = r.i32()
            if not (0 <= cnt <= 256):
                raise MeshDescError(f'poly[{i}]: bad contour count {cnt}')
            poly_contours.append([r.i32() for _ in range(cnt)])
            poly_groups[i] = r.i32()

    n_groups = read_element_array(r, 'PolygonGroupArray')

    _, vattr = read_attribute_set(r, 'VertexAttributes')
    _, viattr = read_attribute_set(r, 'VertexInstanceAttributes')
    _, eattr = read_attribute_set(r, 'EdgeAttributes')
    _, pattr = read_attribute_set(r, 'PolygonAttributes')
    _, gattr = read_attribute_set(r, 'PolygonGroupAttributes')

    n_tris = read_element_array(r, 'TriangleArray')
    tri_raw = np.frombuffer(r.read(n_tris * 16), dtype=np.int32).reshape(-1, 4)

    tn_elem, tattr = read_attribute_set(r, 'TriangleAttributes')
    leftover = len(dec) - r.tell()
    if leftover:
        raise MeshDescError(f'{leftover} unconsumed bytes at {r.tell()}')

    positions = np.frombuffer(vattr['Position']['indices'][0],
                              dtype='<f4').reshape(-1, 3)
    if len(positions) != n_verts:
        raise MeshDescError('position count mismatch')
    uvs = np.frombuffer(viattr['TextureCoordinate']['indices'][0],
                        dtype='<f4').reshape(-1, 2)
    if len(uvs) != n_inst:
        raise MeshDescError('uv count mismatch')

    slot_names = gattr.get('ImportedMaterialSlotName', {}).get('indices', [[]])[0]
    return {
        'n_verts': n_verts, 'n_inst': n_inst, 'n_edges': n_edges,
        'n_polys': n_polys, 'n_groups': n_groups, 'n_tris': n_tris,
        'inst_vertex': inst_vertex, 'tris': tri_raw[:, :3],
        'tri_poly': tri_raw[:, 3], 'poly_groups': poly_groups,
        'positions': positions, 'uvs': uvs, 'slot_names': slot_names,
        'vi_attr_names': sorted(viattr), 'v_attr_names': sorted(vattr),
        'e_attr_names': sorted(eattr), 'p_attr_names': sorted(pattr),
        'g_attr_names': sorted(gattr), 't_attr_names': sorted(tattr),
        'uv_channels': len(viattr['TextureCoordinate']['indices']),
    }


def find_extended_bounds(p):
    """Decode ExtendedBounds (FBoxSphereBounds) from the StaticMesh export's
    tagged properties (editor packages serialize it as tagged inner props)."""
    for e in p.exports:
        if p.class_of(e) != 'StaticMesh':
            continue
        r = Reader(p.data, e['serial_offset'])
        props = p.read_properties(r)
        for t in props:
            if t['name'] == 'ExtendedBounds':
                origin = extent = radius = None
                for inner in t['value']:
                    if inner['name'] == 'Origin':
                        origin = inner['value']
                    elif inner['name'] == 'BoxExtent':
                        extent = inner['value']
                    elif inner['name'] == 'SphereRadius':
                        radius = inner['value']
                if origin and extent:
                    return origin, extent, radius
    return None


def extract(path, out_dir=OUT_DIR, verbose=True):
    name = os.path.splitext(os.path.basename(path))[0]
    p = Package(path)
    blob = p.data[p.bulk_data_start_offset:len(p.data) - 4]
    dec = decompress_chunked(blob)
    # confirm the bulk region is exactly one SerializeCompressed blob
    rb = Reader(blob)
    rb.i64(); rb.i64()
    comp_total = rb.i64(); rb.i64()
    n_chunks = (len(dec) + 0x20000 - 1) // 0x20000
    consumed = 32 + n_chunks * 16 + comp_total
    md = parse_mesh_description(dec)

    inst_vertex = md['inst_vertex']
    tris = md['tris']
    uvs = md['uvs']
    positions = md['positions']

    # dedupe (parent vertex id, uv0 bit pattern) over vertex instances
    key = np.empty((md['n_inst'], 3), dtype=np.uint32)
    key[:, 0] = inst_vertex.view(np.uint32)
    key[:, 1:] = uvs.view(np.uint32)
    uniq, inverse = np.unique(key, axis=0, return_inverse=True)
    out_vid = uniq[:, 0].view(np.int32)
    out_pos = positions[out_vid]
    out_uv = uniq[:, 1:].view(np.float32)
    indices = inverse[tris.reshape(-1)].astype(np.uint32)

    if indices.max() >= len(uniq) or len(indices) != md['n_tris'] * 3:
        raise MeshDescError('index remap failed')

    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, name + '.bin')
    with open(out_path, 'wb') as f:
        f.write(struct.pack('<II', len(uniq), len(indices)))
        f.write(out_pos.astype('<f4').tobytes())
        f.write(out_uv.astype('<f4').tobytes())
        f.write(indices.tobytes())

    bmin = out_pos.min(axis=0)
    bmax = out_pos.max(axis=0)
    eb = find_extended_bounds(p)
    eb_err = None
    if eb:
        origin, extent, _ = eb
        eb_min = np.array(origin) - np.array(extent)
        eb_max = np.array(origin) + np.array(extent)
        eb_err = float(max(np.abs(eb_min - bmin).max(), np.abs(eb_max - bmax).max()))

    info = {
        'name': name,
        'file': f'data/meshes/{name}.bin',
        'vert_count': int(len(uniq)),
        'tri_count': int(md['n_tris']),
        'index_count': int(len(indices)),
        'bounds_min_cm': [float(v) for v in bmin],
        'bounds_max_cm': [float(v) for v in bmax],
        'uv_min': [float(v) for v in uvs.min(axis=0)],
        'uv_max': [float(v) for v in uvs.max(axis=0)],
        'source': {
            'uasset': os.path.relpath(path, ROOT).replace('\\', '/'),
            'vertices': md['n_verts'], 'vertex_instances': md['n_inst'],
            'edges': md['n_edges'], 'polygons': md['n_polys'],
            'polygon_groups': md['n_groups'], 'uv_channels': md['uv_channels'],
            'material_slots': md['slot_names'],
            'decompressed_bytes': len(dec),
            'blob_fully_consumed': consumed == len(blob),
        },
        'extended_bounds_max_abs_err_cm': eb_err,
    }
    if verbose:
        print(f'{name}: verts {md["n_verts"]} -> out {len(uniq)}, '
              f'tris {md["n_tris"]}, bounds_min {bmin.round(1).tolist()}, '
              f'bounds_max {bmax.round(1).tolist()}, eb_err {eb_err}, '
              f'uv [{uvs.min():.4f},{uvs.max():.4f}], '
              f'attrs vi={md["vi_attr_names"]}')
    return info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('paths', nargs='*')
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--manifest', default=os.path.join(ROOT, 'extracted',
                                                       'manifest-meshes.json'))
    args = ap.parse_args()
    paths = args.paths
    if args.all:
        paths = sorted(os.path.join(MESH_DIR, f) for f in os.listdir(MESH_DIR)
                       if f.endswith('.uasset'))
    if not paths:
        ap.error('no input files')
    tiles = []
    for path in paths:
        tiles.append(extract(path))
    if args.all:
        manifest = {
            'domain': 'meshes',
            'coordinate_space': 'UE world centimeters, Z-up, as stored',
            'format': 'u32 vert_count, u32 index_count, f32 pos[v*3], '
                      'f32 uv[v*2], u32 idx[i] (little-endian, tri list)',
            'winding': 'as stored in FMeshDescription triangle array',
            'tiles': tiles,
        }
        os.makedirs(os.path.dirname(args.manifest), exist_ok=True)
        with open(args.manifest, 'w') as f:
            json.dump(manifest, f, indent=1)
        print('wrote', args.manifest)


if __name__ == '__main__':
    main()
