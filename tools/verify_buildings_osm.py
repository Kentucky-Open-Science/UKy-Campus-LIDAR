#!/usr/bin/env python
"""Verify extracted LiDAR building footprints against OpenStreetMap ground truth.

The viewer scene is georeferenced (see tools/osm_roads.py): scene metres map to
UTM zone 16N (EPSG:32616) by a rotation-free, unit-scale transform

    easting  = A + sceneX            A = (original_coordinates[0] + origin_cm[0]) / 100
    northing = B - sceneZ            B = -(original_coordinates[1] + origin_cm[1]) / 100

and the viewer places each building .bin (UE cm) at

    sceneX = (vx - origin_cm[0]) / 100      sceneZ = (vy - origin_cm[1]) / 100

(vx, vy = .bin vertex components 0 and 1; component 2 is up). So we can put both
the extracted footprints and the OSM building footprints into the *same* scene-
metre plane the user sees, plus the roads.json road polygons, and measure:

  - merging   : one extracted footprint overlapping >1 distinct OSM building
  - road-cross: extracted footprint overlapping a road surface polygon
  - spurious  : extracted footprint with no OSM building under it
  - missed    : OSM building with no extracted footprint over it
  - shape IoU : best intersection-over-union vs OSM for matched footprints

Outputs:
  extracted/REPORT-buildings-osm.md     human-readable summary
  extracted/verify-buildings-osm.json   per-building metrics
  extracted/verify-buildings-osm.png    top-down overlay (OSM green / extracted
                                        red / roads grey)
  extracted/osm_buildings.json          cached Overpass response

Usage:
  python -m tools.verify_buildings_osm [--osm-cache PATH] [--no-image]
  (run from repo root)
"""
import argparse
import glob
import json
import math
import os
import struct
import sys
import time
import urllib.request

import numpy as np
from pyproj import Transformer
from shapely.geometry import Polygon, LineString, MultiPolygon
from shapely.ops import polygonize, unary_union
from shapely import STRtree

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, 'web', 'data')
OUT = os.path.join(ROOT, 'extracted')
OVERPASS_MIRRORS = [
    'https://overpass-api.de/api/interpreter',
    'https://overpass.kumi.systems/api/interpreter',
    'https://maps.mail.ru/osm/tools/overpass/api/interpreter',
    'https://overpass.openstreetmap.fr/api/interpreter',
]

# An extracted footprint counts as "covering" an OSM building when their overlap
# is at least this fraction of the smaller of the two areas.
OVERLAP_FRAC = 0.10


# --------------------------------------------------------------- georef -----
def load_georef():
    m = json.load(open(os.path.join(DATA, 'manifest.json')))
    oc = m['lidar']['original_coordinates']
    O = m['origin_cm']
    A = (oc[0] + O[0]) / 100.0
    B = -(oc[1] + O[1]) / 100.0
    return A, B, O


def bin_footprint(path, O):
    """Read a building .bin and return its base-ring footprint as a scene-metre
    shapely Polygon (or None). Walls are vertical extrusions, so the bottom ring
    (first (vc-1)//2 verts) is the footprint, in order."""
    with open(path, 'rb') as f:
        raw = f.read()
    if len(raw) < 8:
        return None
    vc, ic = struct.unpack_from('<II', raw, 0)
    need = 8 + vc * 12 + ic * 4
    if len(raw) < need or vc < 7:
        return None
    pos = np.frombuffer(raw, dtype='<f4', count=vc * 3, offset=8).reshape(vc, 3)
    n = (vc - 1) // 2                       # generate_mesh(): 2n ring verts + 1 cap
    ring = pos[:n]
    sx = (ring[:, 0] - O[0]) / 100.0
    sz = (ring[:, 1] - O[1]) / 100.0
    coords = list(zip(sx.tolist(), sz.tolist()))
    if len(coords) < 3:
        return None
    poly = Polygon(coords)
    if not poly.is_valid:
        poly = poly.buffer(0)
    if poly.is_empty or poly.area <= 0:
        return None
    return poly


# ----------------------------------------------------------------- OSM ------
def scene_bbox_lonlat(manifest_blds, A, B, O, pad_m=200.0):
    """Scene bbox of all extracted buildings (from manifest bounds), padded,
    converted to a lon/lat bbox for Overpass."""
    sx0 = sz0 = 1e18
    sx1 = sz1 = -1e18
    for b in manifest_blds:
        mn, mx = b['bounds_min_cm'], b['bounds_max_cm']
        for ux in (mn[0], mx[0]):
            sx = (ux - O[0]) / 100.0
            sx0, sx1 = min(sx0, sx), max(sx1, sx)
        for uy in (mn[1], mx[1]):
            sz = (uy - O[1]) / 100.0
            sz0, sz1 = min(sz0, sz), max(sz1, sz)
    sx0 -= pad_m; sz0 -= pad_m; sx1 += pad_m; sz1 += pad_m
    to_ll = Transformer.from_crs(32616, 4326, always_xy=True)
    lls = []
    for sx in (sx0, sx1):
        for sz in (sz0, sz1):
            e = A + sx
            nth = B - sz
            lls.append(to_ll.transform(e, nth))
    lons = [p[0] for p in lls]
    lats = [p[1] for p in lls]
    return (min(lats), min(lons), max(lats), max(lons),
            (sx0, sz0, sx1, sz1))


def fetch_osm_buildings(s, w, n, e):
    q = (f'[out:json][timeout:180];'
         f'(way["building"]({s},{w},{n},{e});'
         f'relation["building"]["type"="multipolygon"]({s},{w},{n},{e}););'
         f'(._;>;);out body;')
    last = None
    for attempt in range(3):
        for url in OVERPASS_MIRRORS:
            try:
                req = urllib.request.Request(
                    url, data=q.encode('utf-8'),
                    headers={'User-Agent': 'uky-campus-viewer/1.0'})
                print(f'      try {url} (attempt {attempt + 1}) ...')
                return json.load(urllib.request.urlopen(req, timeout=240))
            except Exception as ex:
                last = ex
                print(f'      {type(ex).__name__}: {ex}')
        time.sleep(5)
    raise RuntimeError(f'all Overpass mirrors failed: {last}')


def osm_to_scene_polys(osm, A, B):
    """Build scene-metre polygons from OSM ways + multipolygon relations."""
    to_utm = Transformer.from_crs(4326, 32616, always_xy=True)
    els = osm['elements']
    nodes = {x['id']: (x['lon'], x['lat']) for x in els if x['type'] == 'node'}
    ways = {x['id']: x for x in els if x['type'] == 'way'}

    def ring_scene(node_ids):
        pts = []
        for nid in node_ids:
            if nid not in nodes:
                return None
            lon, lat = nodes[nid]
            ee, nn = to_utm.transform(lon, lat)
            pts.append((ee - A, B - nn))
        return pts

    polys = []
    used_in_rel = set()

    # multipolygon relations first (so their member ways aren't double-counted)
    for r in els:
        if r['type'] != 'relation':
            continue
        outer_lines = []
        for mem in r.get('members', []):
            if mem['type'] != 'way' or mem['ref'] not in ways:
                continue
            if mem.get('role') not in ('outer', ''):
                if mem.get('role') == 'inner':
                    used_in_rel.add(mem['ref'])
                continue
            used_in_rel.add(mem['ref'])
            pts = ring_scene(ways[mem['ref']]['nodes'])
            if pts and len(pts) >= 2:
                outer_lines.append(LineString(pts))
        if outer_lines:
            for geom in polygonize(unary_union(outer_lines)):
                g = geom if geom.is_valid else geom.buffer(0)
                if not g.is_empty and g.area > 0:
                    polys.append((g, r['id'], 'relation'))

    # standalone building ways
    for wid, w in ways.items():
        if wid in used_in_rel:
            continue
        if 'building' not in w.get('tags', {}):
            continue
        nids = w['nodes']
        if len(nids) < 4:
            continue
        pts = ring_scene(nids)
        if not pts or len(pts) < 3:
            continue
        g = Polygon(pts)
        if not g.is_valid:
            g = g.buffer(0)
        if g.is_empty or g.area <= 0:
            continue
        polys.append((g, wid, 'way'))
    return polys


def load_road_polys():
    """Road surface polygons (scene metres) from roads.json: each polyline
    buffered by half its width."""
    rp = os.path.join(DATA, 'roads.json')
    if not os.path.exists(rp):
        return []
    rj = json.load(open(rp))
    polys = []
    for r in rj.get('roads', []):
        pts = [(p[0], p[2]) for p in r['pts']]          # (sceneX, sceneZ)
        if len(pts) < 2:
            continue
        line = LineString(pts)
        polys.append(line.buffer(max(r.get('width', 7), 1) / 2.0,
                                 cap_style=2, join_style=2))
    return polys


# --------------------------------------------------------------- image ------
def render_overlay(path, ext_polys, osm_polys, road_polys, scene_box):
    from PIL import Image, ImageDraw
    sx0, sz0, sx1, sz1 = scene_box
    W = 1600
    span_x = max(sx1 - sx0, 1.0)
    span_z = max(sz1 - sz0, 1.0)
    scale = W / span_x
    H = int(span_z * scale)
    img = Image.new('RGB', (W, H), (18, 20, 24))
    d = ImageDraw.Draw(img, 'RGBA')

    def to_px(x, z):
        return ((x - sx0) * scale, (z - sz0) * scale)

    def draw_poly(geom, outline, fill):
        gs = geom.geoms if geom.geom_type == 'MultiPolygon' else [geom]
        for g in gs:
            if g.is_empty:
                continue
            ring = [to_px(x, z) for x, z in g.exterior.coords]
            if len(ring) >= 2:
                d.polygon(ring, outline=outline, fill=fill)

    for g in road_polys:
        draw_poly(g, None, (120, 120, 130, 110))
    for g, _, _ in osm_polys:
        draw_poly(g, (60, 230, 120, 255), (60, 230, 120, 55))
    for g in ext_polys:
        draw_poly(g, (240, 70, 70, 220), (240, 70, 70, 70))

    img.save(path)


# ---------------------------------------------------------------- main ------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--osm-cache', help='use this OSM json instead of fetching')
    ap.add_argument('--no-image', action='store_true')
    args = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)
    A, B, O = load_georef()
    print(f'[georef] A={A:.3f} B={B:.3f} origin_cm={O}')

    man = json.load(open(os.path.join(OUT, 'manifest-buildings.json')))
    blds = man['tiles']
    print(f'[1/5] extracted buildings in manifest: {len(blds)}')

    # bbox -> OSM
    s, w, n, e, scene_box = scene_bbox_lonlat(blds, A, B, O)
    cache = args.osm_cache or os.path.join(OUT, 'osm_buildings.json')
    if args.osm_cache and os.path.exists(args.osm_cache):
        print(f'[2/5] loading OSM cache {args.osm_cache}')
        osm = json.load(open(args.osm_cache))
    elif os.path.exists(cache) and not args.osm_cache:
        print(f'[2/5] loading cached OSM {cache}')
        osm = json.load(open(cache))
    else:
        print(f'[2/5] fetching OSM buildings bbox ({s:.4f},{w:.4f},{n:.4f},{e:.4f})')
        osm = fetch_osm_buildings(s, w, n, e)
        json.dump(osm, open(cache, 'w'))
    osm_polys = osm_to_scene_polys(osm, A, B)
    print(f'      OSM building footprints: {len(osm_polys)}')

    # extracted footprints (read every .bin via the viewer transform)
    print('[3/5] reading extracted footprints from .bin ...')
    ext = []           # (poly, name, manifest_record)
    for b in blds:
        p = os.path.join(DATA, b['file'])
        if not os.path.exists(p):
            continue
        poly = bin_footprint(p, O)
        if poly is not None:
            ext.append((poly, b['name'], b))
    print(f'      readable extracted footprints: {len(ext)}')

    road_polys = load_road_polys()
    road_union = unary_union(road_polys) if road_polys else None
    print(f'[4/5] road surface polygons: {len(road_polys)}')

    # spatial index over OSM footprints
    osm_geoms = [g for g, _, _ in osm_polys]
    tree = STRtree(osm_geoms) if osm_geoms else None

    print('[5/5] scoring ...')
    results = []
    osm_hit = set()
    for poly, name, rec in ext:
        area = poly.area
        entry = {
            'name': name,
            'provenance': rec.get('provenance', 'unknown'),
            'area_m2': round(area, 1),
            'osm_overlap_count': 0,
            'best_iou': 0.0,
            'merged': False,
            'spurious': False,
            'road_overlap_m2': 0.0,
            'road_overlap_frac': 0.0,
        }
        # OSM overlaps
        matches = []
        if tree is not None:
            for j in tree.query(poly):
                g = osm_geoms[j]
                inter = poly.intersection(g).area
                if inter <= 0:
                    continue
                small = min(area, g.area)
                if small > 0 and inter / small >= OVERLAP_FRAC:
                    union = poly.area + g.area - inter
                    iou = inter / union if union > 0 else 0.0
                    matches.append((j, inter, iou))
        if matches:
            entry['osm_overlap_count'] = len(matches)
            entry['best_iou'] = round(max(m[2] for m in matches), 3)
            entry['merged'] = len(matches) > 1
            for j, _, _ in matches:
                osm_hit.add(j)
        else:
            entry['spurious'] = True
        # road crossing
        if road_union is not None:
            rinter = poly.intersection(road_union).area
            if rinter > 1.0:
                entry['road_overlap_m2'] = round(rinter, 1)
                entry['road_overlap_frac'] = round(rinter / area, 3)
        results.append(entry)

    n_missed = len(osm_geoms) - len(osm_hit)
    merged = [r for r in results if r['merged']]
    # lidar_only footprints have no OSM building under them BY DESIGN; only count
    # an osm+lidar footprint with no OSM match as genuinely spurious.
    spurious = [r for r in results
                if r['spurious'] and r['provenance'] != 'lidar_only']
    lidar_only = [r for r in results if r['provenance'] == 'lidar_only']
    road_cross = [r for r in results if r['road_overlap_frac'] >= 0.05]
    matched = [r for r in results if not r['spurious']]
    ious = sorted(r['best_iou'] for r in matched)
    med_iou = ious[len(ious) // 2] if ious else 0.0
    big = sorted(results, key=lambda r: -r['area_m2'])[:10]

    summary = {
        'extracted_total': len(results),
        'osm_total': len(osm_geoms),
        'osm_matched': len(osm_hit),
        'osm_missed': n_missed,
        'merged_footprints': len(merged),
        'osm_swallowed_by_merges': sum(r['osm_overlap_count'] for r in merged),
        'spurious_footprints': len(spurious),
        'lidar_only_footprints': len(lidar_only),
        'road_crossing_footprints': len(road_cross),
        'median_iou_matched': round(med_iou, 3),
        'overlap_frac_threshold': OVERLAP_FRAC,
    }
    json.dump({'summary': summary, 'buildings': results},
              open(os.path.join(OUT, 'verify-buildings-osm.json'), 'w'), indent=1)

    # report
    lines = []
    L = lines.append
    L('# Building geometry verification vs OpenStreetMap\n')
    L('Source of truth: OpenStreetMap building footprints (c) OSM contributors, ')
    L('projected into scene metres through the same verified georeference the ')
    L('viewer and roads.json use. Roads from web/data/roads.json.\n')
    L('## Headline\n')
    L(f'- Extracted footprints: **{summary["extracted_total"]}**')
    L(f'- OSM buildings in area: **{summary["osm_total"]}**')
    L(f'- OSM buildings covered by an extracted footprint: '
      f'**{summary["osm_matched"]}** ({100*summary["osm_matched"]/max(summary["osm_total"],1):.0f}%)')
    L(f'- OSM buildings missed entirely: **{summary["osm_missed"]}**')
    L(f'- Extracted footprints that merge >1 OSM building: '
      f'**{summary["merged_footprints"]}** '
      f'(swallowing {summary["osm_swallowed_by_merges"]} OSM buildings)')
    L(f'- Extracted footprints overlapping a road surface (>=5% of their area): '
      f'**{summary["road_crossing_footprints"]}**')
    L(f'- Genuinely spurious footprints (osm+lidar, no OSM building under them): '
      f'**{summary["spurious_footprints"]}**')
    L(f'- LiDAR-only footprints (intentional: real LiDAR returns where OSM has no '
      f'building): **{summary["lidar_only_footprints"]}**')
    L(f'- Median shape IoU of matched footprints vs OSM: '
      f'**{summary["median_iou_matched"]:.2f}** (1.0 = perfect)\n')
    L('## 10 largest extracted footprints (merge suspects)\n')
    L('| name | provenance | area m² | OSM under it | road overlap m² | IoU |')
    L('|---|---|---:|---:|---:|---:|')
    for r in big:
        L(f'| {r["name"]} | {r["provenance"]} | {r["area_m2"]:,.0f} '
          f'| {r["osm_overlap_count"]} | {r["road_overlap_m2"]:,.0f} '
          f'| {r["best_iou"]:.2f} |')
    L('\nSee verify-buildings-osm.json for per-building detail and '
      'verify-buildings-osm.png for the top-down overlay '
      '(green = OSM, red = extracted, grey = roads).')
    open(os.path.join(OUT, 'REPORT-buildings-osm.md'), 'w',
         encoding='utf-8').write('\n'.join(lines))

    if not args.no_image:
        try:
            render_overlay(os.path.join(OUT, 'verify-buildings-osm.png'),
                           [p for p, _, _ in ext], osm_polys, road_polys, scene_box)
            print('      wrote extracted/verify-buildings-osm.png')
        except Exception as ex:
            print(f'      image render skipped: {ex}')

    print('\n=== SUMMARY ===')
    for k, v in summary.items():
        print(f'  {k}: {v}')
    print('\nwrote extracted/REPORT-buildings-osm.md, verify-buildings-osm.json')


if __name__ == '__main__':
    main()
