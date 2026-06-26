#!/usr/bin/env python
"""Hybrid building extraction: OSM footprints split the LiDAR, LiDAR gives shape.

The pure-LiDAR extractor (tools/extract_buildings.py) DBSCAN-clusters building-
class points with eps=5 m, which fuses anything connected by trees / walkways /
adjacent roofs into giant blobs that cross roads and merge whole blocks (verified
against OSM by tools/verify_buildings_osm.py: 99 blobs swallowing 2217 distinct
OSM buildings, 144 crossing roads).

This tool keeps the LiDAR-derived shape but uses OSM building footprints as the
splitting/bounding authority:

  1. Load building-class LiDAR points (classification 6) from the source uasset.
  2. Project to scene metres; assign each point to the OSM footprint it falls in
     (footprints buffered +2 m to catch eaves). This SPLITS merged clusters: each
     OSM building only ever gets its own points.
  3. Per OSM footprint with enough points: footprint = concave hull of its points
     CLIPPED to the OSM polygon (LiDAR shape where it agrees; never exceeds the
     OSM boundary, so it can't cross a road or swallow a neighbour). Fall back to
     the OSM polygon when LiDAR is sparse. Height from the points' z-range.
  4. Residual points (no OSM building above them) are recovered conservatively:
     DBSCAN with a tight eps, kept only if building-shaped, off the roads, and not
     overlapping any OSM footprint. Flagged provenance='lidar_only'.

Output is byte-compatible with the old extractor (same .bin format, same manifest
schema) so build_all.py / the viewer need no changes.

Usage:
  python -m tools.extract_buildings_hybrid [--no-residual] [--verbose]
  (run from repo root)
"""
import argparse
import json
import os
import struct
import sys
import time

import numpy as np
import shapely
from shapely.geometry import Polygon, MultiPoint
from shapely.ops import unary_union
from shapely import STRtree, concave_hull

import tools.extract_buildings as eb
from tools.extract_buildings import (load_from_source_uasset, generate_mesh,
                                     compute_tile_name)
from tools.verify_buildings_osm import (load_georef, scene_bbox_lonlat,
                                        fetch_osm_buildings, osm_to_scene_polys,
                                        load_road_polys)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, 'web', 'data')
OUT_DIR = os.path.join(DATA, 'buildings')
EXTRACTED_DIR = os.path.join(ROOT, 'extracted')

ASSIGN_BUFFER_M = 2.0       # grow OSM footprints when gathering points (eaves)
MIN_PTS_OSM = 8             # min LiDAR points to call an OSM footprint "scanned"
HULL_RATIO = 0.35
LIDAR_SHAPE_MIN_FRAC = 0.5  # use the LiDAR hull only if it fills >=50% of OSM
Z_LO, Z_HI = 2.0, 98.0      # robust height percentiles (drop spikes / pits)

# residual (LiDAR-only) recovery
RESIDUAL_EPS_CM = 300.0
RESIDUAL_MIN_SAMPLES = 20
RES_AREA_MIN, RES_AREA_MAX = 40.0, 4000.0     # m^2
RES_ROAD_MAX_FRAC = 0.15
RES_OSM_MAX_FRAC = 0.20
RES_DECIMATE_CAP = 2_500_000   # cap residual point count fed to DBSCAN


def scene_poly_to_ue(poly, O):
    """scene-metre polygon -> UE-cm polygon (for .bin vertices)."""
    ext = [((x * 100.0) + O[0], (z * 100.0) + O[1]) for x, z in poly.exterior.coords]
    return Polygon(ext)


def largest_polygon(geom):
    """Reduce a (Multi)Polygon to its largest single Polygon."""
    if geom.is_empty:
        return None
    if geom.geom_type == 'Polygon':
        return geom
    if geom.geom_type in ('MultiPolygon', 'GeometryCollection'):
        polys = [g for g in geom.geoms if g.geom_type == 'Polygon' and g.area > 0]
        if not polys:
            return None
        return max(polys, key=lambda g: g.area)
    return None


def write_building(name, footprint_scene, z_min_cm, z_max_cm, pts_ue, O,
                   provenance, osm_id, out_files):
    """Mesh + write one building .bin; return its manifest record or None."""
    ue_poly = scene_poly_to_ue(footprint_scene, O)
    if not ue_poly.is_valid:
        ue_poly = ue_poly.buffer(0)
        ue_poly = largest_polygon(ue_poly)
        if ue_poly is None:
            return None
    height = z_max_cm - z_min_cm
    if height <= 0:
        height = 100.0
    verts, indices = generate_mesh(ue_poly, z_min_cm, z_min_cm + height)
    if len(verts) == 0 or len(indices) % 3 != 0:
        return None

    fname = f'{name}.bin'
    fpath = os.path.join(OUT_DIR, fname)
    with open(fpath, 'wb') as f:
        f.write(struct.pack('<II', len(verts), len(indices)))
        f.write(verts.astype('<f4').tobytes())
        f.write(indices.tobytes())
    out_files.append(fpath)

    mn = pts_ue.min(axis=0)
    mx = pts_ue.max(axis=0)
    rec = {
        'name': name,
        'file': f'buildings/{fname}',
        'bounds_min_cm': [float(v) for v in mn],
        'bounds_max_cm': [float(v) for v in mx],
        'height_cm': round(float(height), 1),
        'footprint_area_m2': round(float(footprint_scene.area), 2),
        'point_count': int(len(pts_ue)),
        'vertex_count': int(len(verts)),
        'index_count': int(len(indices)),
        'provenance': provenance,
    }
    if osm_id is not None:
        rec['osm_id'] = osm_id
    return rec, os.path.getsize(fpath)


def robust_z(pts_z_cm):
    lo = float(np.percentile(pts_z_cm, Z_LO))
    hi = float(np.percentile(pts_z_cm, Z_HI))
    if hi <= lo:
        lo, hi = float(pts_z_cm.min()), float(pts_z_cm.max())
    return lo, hi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-residual', action='store_true',
                    help='skip LiDAR-only recovery of buildings OSM lacks')
    ap.add_argument('--osm-cache', help='OSM json to use instead of fetching')
    ap.add_argument('--verbose', '-v', action='store_true')
    args = ap.parse_args()

    t0 = time.time()
    os.makedirs(OUT_DIR, exist_ok=True)
    A, B, O = load_georef()
    print(f'[georef] A={A:.3f} B={B:.3f} origin_cm={O}')

    # ---- 1. LiDAR building-class points (UE cm) -------------------------
    print('[1/6] loading building-class LiDAR points from source uasset ...')
    pts = load_from_source_uasset()           # (N,3) float32, UE cm (x,y,z)
    if len(pts) == 0:
        print('  no building-class points; aborting')
        return
    print(f'  {len(pts):,} points')
    sx = (pts[:, 0] - O[0]) / 100.0
    sz = (pts[:, 1] - O[1]) / 100.0
    pz = pts[:, 2]                              # up, cm

    # ---- 2. OSM footprints (scene metres) -------------------------------
    print('[2/6] OSM footprints ...')
    # OSM fetch bbox = the extent of the building-class LiDAR we just loaded (UE cm bounds
    # -> padded lon/lat). This previously read extracted/manifest-buildings.json — the file
    # THIS tool writes only at the very end (line ~331) — so a from-scratch build had nothing
    # to read and crashed. The point cloud is the correct, self-contained source for the bbox.
    bmn = [float(pts[:, 0].min()), float(pts[:, 1].min()), float(pts[:, 2].min())]
    bmx = [float(pts[:, 0].max()), float(pts[:, 1].max()), float(pts[:, 2].max())]
    s, w, n, e, _ = scene_bbox_lonlat([{'bounds_min_cm': bmn, 'bounds_max_cm': bmx}], A, B, O)
    cache = args.osm_cache or os.path.join(EXTRACTED_DIR, 'osm_buildings.json')
    if os.path.exists(cache):
        osm = json.load(open(cache))
        print(f'  loaded OSM cache {cache}')
    else:
        osm = fetch_osm_buildings(s, w, n, e)
        json.dump(osm, open(cache, 'w'))
    osm_polys = osm_to_scene_polys(osm, A, B)
    osm_geoms = [g for g, _, _ in osm_polys]
    osm_ids = [oid for _, oid, _ in osm_polys]
    print(f'  {len(osm_geoms)} OSM footprints')

    # ---- 3. assign points to OSM footprints -----------------------------
    print('[3/6] assigning points to footprints (STRtree) ...')
    pt_geoms = shapely.points(sx, sz)
    grown = [g.buffer(ASSIGN_BUFFER_M) for g in osm_geoms]
    tree = STRtree(grown)
    # pairs: (point_idx, geom_idx) where grown[geom] covers point
    pi, gi = tree.query(pt_geoms, predicate='intersects')
    print(f'  {len(pi):,} point-in-footprint hits')
    # one winning OSM geom per point (first hit wins for overlapping footprints),
    # then group point indices by geom — all vectorised (hits run to millions).
    assigned = np.full(len(pts), -1, dtype=np.int64)
    assigned[pi[::-1]] = gi[::-1]              # reverse so earliest hit wins
    vp = np.flatnonzero(assigned >= 0)
    vg = assigned[vp]
    order = np.argsort(vg, kind='stable')
    vp, vg = vp[order], vg[order]
    uniq, starts = np.unique(vg, return_index=True)
    ends = np.append(starts[1:], len(vg))
    buckets = {int(g): vp[s:e] for g, s, e in zip(uniq, starts, ends)}

    road_polys = load_road_polys()
    road_union = unary_union(road_polys) if road_polys else None

    # ---- 4. build OSM+LiDAR footprints ----------------------------------
    print('[4/6] meshing OSM-split / LiDAR-shaped footprints ...')
    buildings = []
    out_files = []
    name_counts = {}
    total_bytes = 0
    osm_with_lidar = 0

    def add(name_base, footprint, zlo, zhi, pts_ue, prov, osm_id):
        nonlocal total_bytes
        if name_base not in name_counts:
            name_counts[name_base] = 0
        name_counts[name_base] += 1
        name = f'{name_base}-{name_counts[name_base]:03d}'
        res = write_building(name, footprint, zlo, zhi, pts_ue, O, prov,
                             osm_id, out_files)
        if res is None:
            return False
        rec, nbytes = res
        total_bytes += nbytes
        buildings.append(rec)
        return True

    for g_idx, idxs in buckets.items():
        if len(idxs) < MIN_PTS_OSM:
            continue
        idxs = np.asarray(idxs)
        osm_poly = osm_geoms[g_idx]
        bsx, bsz, bz = sx[idxs], sz[idxs], pz[idxs]
        pts_ue = pts[idxs]
        # LiDAR concave hull, clipped to the OSM polygon
        footprint = None
        if len(idxs) >= 3:
            try:
                hull = concave_hull(MultiPoint(np.column_stack([bsx, bsz])),
                                    ratio=HULL_RATIO)
            except Exception:
                hull = MultiPoint(np.column_stack([bsx, bsz])).convex_hull
            if hull is not None and not hull.is_empty:
                clip = largest_polygon(hull.intersection(osm_poly))
                if clip is not None and clip.area >= LIDAR_SHAPE_MIN_FRAC * osm_poly.area:
                    footprint = clip
        if footprint is None:
            footprint = largest_polygon(osm_poly)
        if footprint is None or footprint.area <= 0:
            continue
        zlo, zhi = robust_z(bz)
        cx, cy = float(pts_ue[:, 0].mean()), float(pts_ue[:, 1].mean())
        if add(compute_tile_name(cx, cy), footprint, zlo, zhi, pts_ue,
               'osm+lidar', osm_ids[g_idx]):
            osm_with_lidar += 1

    print(f'  OSM footprints with LiDAR: {osm_with_lidar}')

    # ---- 5. residual LiDAR-only recovery --------------------------------
    residual_added = 0
    if not args.no_residual:
        print('[5/6] residual LiDAR-only recovery ...')
        res_mask = assigned < 0
        res_idx = np.flatnonzero(res_mask)
        print(f'  residual points: {len(res_idx):,}')
        if len(res_idx) > RES_DECIMATE_CAP:
            stride = int(np.ceil(len(res_idx) / RES_DECIMATE_CAP))
            res_idx = res_idx[::stride]
            print(f'  decimated to {len(res_idx):,} (stride {stride}) for clustering')
        if len(res_idx) >= RESIDUAL_MIN_SAMPLES:
            res_pts = pts[res_idx]
            eb.EPS_CM = RESIDUAL_EPS_CM
            eb.MIN_SAMPLES = RESIDUAL_MIN_SAMPLES
            clusters = eb.dbscan_cluster(res_pts)
            osm_tree = STRtree(osm_geoms)
            for cidx, centroid in clusters:
                cpts = res_pts[cidx]
                if len(cpts) < RESIDUAL_MIN_SAMPLES:
                    continue
                csx = (cpts[:, 0] - O[0]) / 100.0
                csz = (cpts[:, 1] - O[1]) / 100.0
                fp = eb.extract_footprint(np.column_stack([csx, csz]))
                if fp is None or fp.area < RES_AREA_MIN or fp.area > RES_AREA_MAX:
                    continue
                # reject if it sits on a road or overlaps an OSM building
                if road_union is not None:
                    if fp.intersection(road_union).area / fp.area > RES_ROAD_MAX_FRAC:
                        continue
                osm_ov = 0.0
                for j in osm_tree.query(fp):
                    osm_ov += fp.intersection(osm_geoms[j]).area
                if osm_ov / fp.area > RES_OSM_MAX_FRAC:
                    continue
                zlo, zhi = robust_z(cpts[:, 2])
                if (zhi - zlo) < 200 or (zhi - zlo) > 12000:    # 2 m .. 120 m
                    continue
                cx, cy = float(cpts[:, 0].mean()), float(cpts[:, 1].mean())
                if add(compute_tile_name(cx, cy), fp, zlo, zhi, cpts,
                       'lidar_only', None):
                    residual_added += 1
        print(f'  LiDAR-only buildings recovered: {residual_added}')
    else:
        print('[5/6] residual recovery skipped (--no-residual)')

    # ---- 6. clear stale .bin, write manifest ----------------------------
    print('[6/6] writing manifest + cleaning stale files ...')
    keep = {os.path.abspath(p) for p in out_files}
    removed = 0
    for f in os.listdir(OUT_DIR):
        if f.endswith('.bin') and os.path.abspath(os.path.join(OUT_DIR, f)) not in keep:
            os.remove(os.path.join(OUT_DIR, f)); removed += 1
    print(f'  removed {removed} stale .bin files')

    manifest = {
        'domain': 'buildings',
        'method': 'hybrid-osm-split-lidar-shape',
        'format': 'u32 vc, u32 ic, f32 pos[vc*3], u32 idx[ic] -- UE cm, no UVs',
        'source': 'LiDAR building-class points (shape/height) + OpenStreetMap '
                  'footprints (split/bound), (c) OpenStreetMap contributors',
        'assign_buffer_m': ASSIGN_BUFFER_M,
        'min_pts_osm': MIN_PTS_OSM,
        'hull_ratio': HULL_RATIO,
        'count': len(buildings),
        'count_osm_lidar': sum(1 for b in buildings if b['provenance'] == 'osm+lidar'),
        'count_lidar_only': sum(1 for b in buildings if b['provenance'] == 'lidar_only'),
        'total_mesh_bytes': int(total_bytes),
        'tiles': buildings,
    }
    mpath = os.path.join(EXTRACTED_DIR, 'manifest-buildings.json')
    json.dump(manifest, open(mpath, 'w'), indent=1)
    print(f'  wrote {mpath}')

    print(f'\n=== DONE: {len(buildings)} buildings '
          f'({manifest["count_osm_lidar"]} osm+lidar, '
          f'{manifest["count_lidar_only"]} lidar-only), '
          f'{total_bytes/1e6:.1f} MB in {time.time()-t0:.1f}s ===')
    print('Next: python tools/build_all.py --skip-textures --skip-meshes '
          '--skip-lidar --skip-buildings   (merges manifest)')


if __name__ == '__main__':
    main()
