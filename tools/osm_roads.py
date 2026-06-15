#!/usr/bin/env python
"""Build the campus road network from OpenStreetMap and write web/data/roads.json.

The viewer scene is georeferenced: from the manifest's lidar.original_coordinates
+ origin_cm, the viewer's own cursor read-out implies an EXACT, rotation-free,
unit-scale map between scene metres and UTM zone 16N (EPSG:32616):

    easting  = A + sceneX            A = (original_coordinates[0] + origin_cm[0]) / 100
    northing = B - sceneZ            B = -(original_coordinates[1] + origin_cm[1]) / 100

(Verified: the scene bbox centre projects to lon/lat -84.505, 38.030 = UK campus,
and OSM ways overlaid through this transform land on the aerial streets.)

So we fetch OSM highways for the campus bbox, project lon/lat -> UTM 16N -> scene,
drape every point onto the terrain elevation (heightmap baked from the terrain
meshes, same as tools/extract_roads.py), and emit scene-metre [x,y,z] polylines
with a per-road width plus intersection nodes — the exact contract roads.js reads.

Usage:  python -m tools.osm_roads  [--osm-cache PATH] [--service]
        (run from the repo root so the tools package imports cleanly)
"""
import argparse, json, math, os, urllib.request
import numpy as np
from pyproj import Transformer

from tools.extract_roads import DATA, load_mesh, build_heightmap

# width (m) by OSM highway class
WIDTH = {
    'motorway': 15, 'trunk': 14, 'primary': 13, 'secondary': 11, 'tertiary': 9,
    'residential': 7.5, 'unclassified': 7.5, 'living_street': 6.5, 'service': 5.5,
    'motorway_link': 7, 'trunk_link': 7, 'primary_link': 7,
    'secondary_link': 6.5, 'tertiary_link': 6.5,
}
KEEP = set(WIDTH)
RESAMPLE_M = 5.0
OVERPASS = 'https://overpass-api.de/api/interpreter'


def terrain_extent(manifest):
    gx0 = gz0 = 1e18; gx1 = gz1 = -1e18
    for t in manifest['terrain']['tiles']:
        mp = os.path.join(DATA, t['mesh'])
        if not os.path.exists(mp):
            continue
        P, _ = load_mesh(mp)
        if len(P) < 3:
            continue
        gx0 = min(gx0, P[:, 0].min()); gx1 = max(gx1, P[:, 0].max())
        gz0 = min(gz0, P[:, 2].min()); gz1 = max(gz1, P[:, 2].max())
    return gx0, gz0, gx1, gz1


def fetch_osm(s, w, n, e):
    types = '|'.join(sorted(KEEP))
    q = (f'[out:json][timeout:120];'
         f'way["highway"~"^({types})$"]({s},{w},{n},{e});(._;>;);out body;')
    req = urllib.request.Request(OVERPASS, data=q.encode('utf-8'),
                                 headers={'User-Agent': 'uky-campus-viewer/1.0'})
    return json.load(urllib.request.urlopen(req, timeout=180))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--osm-cache', help='use this OSM json instead of fetching')
    ap.add_argument('--service', action='store_true',
                    help='also keep service roads (parking aisles/driveways)')
    args = ap.parse_args()

    manifest = json.load(open(os.path.join(DATA, 'manifest.json')))
    oc, O = manifest['lidar']['original_coordinates'], manifest['origin_cm']
    A = (oc[0] + O[0]) / 100.0
    B = -(oc[1] + O[1]) / 100.0     # easting = A + sceneX ; northing = B - sceneZ

    print('[1/5] terrain extent + elevation heightmap ...')
    gx0, gz0, gx1, gz1 = terrain_extent(manifest)
    mpp = 50.0
    W = int(math.ceil((gx1 - gx0) / mpp)); H = int(math.ceil((gz1 - gz0) / mpp))
    elev = build_heightmap(manifest, (gx0, gz0, mpp, W, H))

    to_utm = Transformer.from_crs(4326, 32616, always_xy=True)   # lon/lat -> UTM16N
    to_ll = Transformer.from_crs(32616, 4326, always_xy=True)

    # campus bbox (scene -> UTM -> lon/lat) with a small margin.
    # sceneX = local_x/100 ; sceneZ = -local_z/100 ; easting = A + sceneX ;
    # northing = B - sceneZ = B + local_z/100.
    pad = 60.0
    E = [A + gx0 / 100 - pad, A + gx1 / 100 + pad]
    N = [B + gz0 / 100 - pad, B + gz1 / 100 + pad]
    # scene-space terrain bounds (to clip far-away OSM ways)
    sxmin, sxmax = gx0 / 100 - pad, gx1 / 100 + pad
    szmin, szmax = -gz1 / 100 - pad, -gz0 / 100 + pad
    lls = [to_ll.transform(e, n) for e in E for n in N]
    lons = [p[0] for p in lls]; lats = [p[1] for p in lls]
    s, w, nn, e = min(lats), min(lons), max(lats), max(lons)

    if args.osm_cache:
        print(f'[2/5] loading OSM cache {args.osm_cache} ...')
        osm = json.load(open(args.osm_cache))
    else:
        print(f'[2/5] fetching OSM highways for bbox ({s:.4f},{w:.4f},{nn:.4f},{e:.4f}) ...')
        osm = fetch_osm(s, w, nn, e)

    els = osm['elements']
    nodes = {x['id']: (x['lon'], x['lat']) for x in els if x['type'] == 'node'}
    ways = [x for x in els if x['type'] == 'way' and x.get('tags', {}).get('highway')]

    def to_scene(nid):
        lon, lat = nodes[nid]
        ee, nnn = to_utm.transform(lon, lat)
        return (ee - A, B - nnn)        # (sceneX, sceneZ)

    def drape(sx, sz):
        return float(elev(sx * 100.0, -sz * 100.0)) / 100.0   # sceneY (m)

    def resample(coords, step):
        out = [coords[0]]; carry = step
        for i in range(len(coords) - 1):
            ax, az = coords[i]; bx, bz = coords[i + 1]
            seg = math.hypot(bx - ax, bz - az)
            if seg < 1e-6:
                continue
            d = carry
            while d < seg:
                tt = d / seg
                out.append((ax + (bx - ax) * tt, az + (bz - az) * tt)); d += step
            carry = d - seg
        out.append(coords[-1])
        return out

    print('[3/5] projecting + draping roads ...')
    keep = set(KEEP)
    if not args.service:
        keep.discard('service')
    roads_out = []
    node_use = {}                       # node id -> count of kept ways using it
    total_km = 0.0

    def emit_road(out, run, hw, wy):
        nonlocal total_km
        ded = [run[0]]                  # drop coincident points (duplicate OSM nodes)
        for p in run[1:]:
            if math.hypot(p[0] - ded[-1][0], p[1] - ded[-1][1]) > 1e-6:
                ded.append(p)
        if len(ded) < 2:
            return
        run = ded
        pts = [[round(sx, 2), round(drape(sx, sz), 2), round(sz, 2)] for sx, sz in run]
        for i in range(len(run) - 1):
            total_km += math.hypot(run[i + 1][0] - run[i][0],
                                   run[i + 1][1] - run[i][1]) / 1000
        out.append({'pts': pts, 'width': WIDTH.get(hw, 7), 'class': hw,
                    'name': wy['tags'].get('name')})

    for wy in ways:
        hw = wy['tags']['highway']
        if hw not in keep:
            continue
        # skip unnamed service roads even when service is enabled (parking aisles)
        if hw == 'service' and 'name' not in wy.get('tags', {}):
            continue
        nids = [i for i in wy['nodes'] if i in nodes]
        if len(nids) < 2:
            continue
        xz = [to_scene(i) for i in nids]
        inb = lambda x, z: sxmin <= x <= sxmax and szmin <= z <= szmax
        if not any(inb(x, z) for x, z in xz):       # fully outside the terrain
            continue
        xz = resample(xz, RESAMPLE_M)
        # split into contiguous in-bounds runs so roads never float past the map
        run = []
        for sx, sz in xz:
            if inb(sx, sz):
                run.append((sx, sz))
            elif len(run) >= 2:
                emit_road(roads_out, run, hw, wy)
                run = []
            else:
                run = []
        if len(run) >= 2:
            emit_road(roads_out, run, hw, wy)
        for i in nids:                              # intersection node usage (in-bounds)
            sx, sz = to_scene(i)
            if inb(sx, sz):
                node_use[i] = node_use.get(i, 0) + 1

    print('[4/5] intersections ...')
    inter = []
    for nid, cnt in node_use.items():
        if cnt >= 2:                    # node shared by >=2 kept roads
            sx, sz = to_scene(nid)
            inter.append([round(sx, 2), round(drape(sx, sz), 2), round(sz, 2)])

    out = {
        'note': 'road network from OpenStreetMap, projected UTM16N->scene metres and '
                'draped on terrain; points are [x, y, z]; source (c) OpenStreetMap contributors',
        'source': 'openstreetmap',
        'roads': roads_out,
        'intersections': inter,
    }
    outpath = os.path.join(DATA, 'roads.json')
    json.dump(out, open(outpath, 'w'))
    print(f'[5/5] wrote {outpath}: {len(roads_out)} roads ({total_km:.1f} km), '
          f'{len(inter)} intersections')


if __name__ == '__main__':
    main()
