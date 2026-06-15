#!/usr/bin/env python
"""Drop floating buildings onto the terrain (no building hovers above / sinks below the
ground texture), EXCEPT bridges that cross over a road — those keep their elevation.

The building meshes are LiDAR-derived, so their base elevation often differs from the
DTM terrain the viewer renders: ~1250 of ~3100 buildings float >0.5 m (some by tens of
metres) and ~400 sink. This offline post-processor samples the terrain heightmap under
each footprint and bakes a `ground_y_m` (scene-metre foundation level) into
web/data/manifest.json; the viewer drops each building so its base sits on that level.
A building whose footprint a road passes UNDER while it floats well above the ground is
treated as a bridge/overpass and left where it is (flagged `bridge: true`).

Run AFTER tools.smooth_roads (it reads the smoothed roads.json for bridge detection).

Usage:  python -m tools.ground_buildings
"""
import json, math, os
import numpy as np

from tools.smooth_roads import build_heightmap, drape_scene, DATA

BRIDGE_FLOAT_M = 2.5    # a road-spanning building only counts as a bridge if it floats this high
GROUND_PCTL    = 15     # foundation = this percentile of sampled ground under the footprint
ROAD_CELL_M    = 20.0   # spatial-grid cell for the road-vertex index


def road_vertex_index(roads):
    """Grid-bucket every road vertex by scene XZ so a building can cheaply find roads
    crossing its footprint. Returns (grid dict, cell size)."""
    from collections import defaultdict
    grid = defaultdict(list)
    for ri, rd in enumerate(roads):
        for p in rd['pts']:
            grid[(int(p[0] // ROAD_CELL_M), int(p[2] // ROAD_CELL_M))].append((ri, p[0], p[2]))
    return grid


def road_crosses(grid, x0, x1, z0, z1):
    """True if a single road traverses the [x0,x1]x[z0,z1] footprint — vertices of the
    same road appear in opposite thirds (so the centreline passes under, not just clips a
    corner)."""
    from collections import defaultdict
    hits = defaultdict(list)
    cx0, cx1 = int(x0 // ROAD_CELL_M) - 1, int(x1 // ROAD_CELL_M) + 1
    cz0, cz1 = int(z0 // ROAD_CELL_M) - 1, int(z1 // ROAD_CELL_M) + 1
    for cx in range(cx0, cx1 + 1):
        for cz in range(cz0, cz1 + 1):
            for (ri, x, z) in grid.get((cx, cz), ()):
                if x0 <= x <= x1 and z0 <= z <= z1:
                    hits[ri].append((x, z))
    mx, mz = (x0 + x1) / 2, (z0 + z1) / 2
    for ri, pts in hits.items():
        if len(pts) < 2:
            continue
        # spans the footprint if it has points on both sides of the mid-line (either axis)
        if (any(x < mx for x, z in pts) and any(x > mx for x, z in pts)) or \
           (any(z < mz for x, z in pts) and any(z > mz for x, z in pts)):
            return True
    return False


def main():
    manifest = json.load(open(os.path.join(DATA, 'manifest.json')))
    if 'buildings' not in manifest:
        raise SystemExit('no buildings in manifest')
    o = manifest['origin_cm']
    elev = build_heightmap(manifest)
    roads = []
    rp = os.path.join(DATA, 'roads.json')
    if os.path.exists(rp):
        roads = json.load(open(rp)).get('roads', [])
    grid = road_vertex_index(roads)

    dropped = raised = bridges = 0
    moves = []
    for b in manifest['buildings']['tiles']:
        mn, mx = b['bounds_min_cm'], b['bounds_max_cm']
        # footprint in scene metres: sceneX=(ue_x-o0)/100, sceneZ=(ue_y-o1)/100
        x0, x1 = (mn[0] - o[0]) / 100.0, (mx[0] - o[0]) / 100.0
        z0, z1 = (mn[1] - o[1]) / 100.0, (mx[1] - o[1]) / 100.0
        if x0 > x1:
            x0, x1 = x1, x0
        if z0 > z1:
            z0, z1 = z1, z0
        base_y = (mn[2] - o[2]) / 100.0                     # current base scene y
        # sample the ground over a grid inside the footprint; found = low percentile
        gx = np.linspace(x0, x1, 4); gz = np.linspace(z0, z1, 4)
        GX, GZ = np.meshgrid(gx, gz)
        g = elev(GX.ravel() * 100.0, -GZ.ravel() * 100.0) / 100.0
        ground = float(np.percentile(g, GROUND_PCTL))
        float_h = base_y - ground
        is_bridge = float_h > BRIDGE_FLOAT_M and road_crosses(grid, x0, x1, z0, z1)
        if is_bridge:
            b['bridge'] = True
            b.pop('ground_y_m', None)
            bridges += 1
            continue
        b['ground_y_m'] = round(ground, 2)
        b.pop('bridge', None)
        moves.append(float_h)
        if float_h > 0:
            dropped += 1
        elif float_h < 0:
            raised += 1

    out = os.path.join(DATA, 'manifest.json')
    json.dump(manifest, open(out, 'w'), indent=2)
    moves = np.array(moves) if moves else np.array([0.0])
    print(f'grounded {len(moves)} buildings (dropped {dropped}, raised {raised}); '
          f'{bridges} bridges left elevated')
    print(f'  float removed (m): median {np.median(np.abs(moves)):.2f}  max {np.max(np.abs(moves)):.2f}')
    print(f'  wrote {out}')


if __name__ == '__main__':
    main()
