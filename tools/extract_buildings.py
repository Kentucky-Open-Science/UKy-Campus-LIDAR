"""Extract 3D building meshes from LiDAR building-class points.

Reads all LiDAR point cloud chunks from web/data/lidar/, filters for
building-class points (classification code 6), clusters into individual
structures via DBSCAN, computes concave-hull footprints, extrudes to
building height, and writes simplified meshes to web/data/buildings/.

Algorithm (per research.md):
  - Load all chunks into a single global array (handles cross-chunk buildings)
  - Filter classification == 6
  - DBSCAN via scipy.spatial.KDTree: eps=500cm, min_samples=10
  - Concave hull footprint per cluster via shapely.concave_hull(ratio=0.4)
  - Mesh: wall quads (extrude footprint to max_z) + flat roof cap
  - Output: binary format per contracts/building-mesh-format.md

Usage:
  python tools/extract_buildings.py
  python tools/extract_buildings.py --verbose
"""

import argparse
import json
import os
import struct
import sys
import time

_here = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path
            if os.path.abspath(p if p else '.') != _here]
import numpy as np
from scipy.spatial import KDTree
from shapely.geometry import Polygon, MultiPoint
from shapely import concave_hull

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIDAR_DIR = os.path.join(ROOT, 'web', 'data', 'lidar')
OUT_DIR = os.path.join(ROOT, 'web', 'data', 'buildings')
EXTRACTED_DIR = os.path.join(ROOT, 'extracted')

# DBSCAN parameters (research.md)
EPS_CM = 500.0         # 5 meters
MIN_SAMPLES = 10
# Concave hull ratio: 0=convex, 1=highly concave (research.md recommends 0.3-0.5)
HULL_RATIO = 0.4
# Determinism seed
SEED = 2019

# Tile grid constants for naming (matched to PLAN.md coordinate notes)
# LiDAR box: (-91525, -170396.5) to (+91525, +170396.5)
# Tile naming = KY State Plane-ish grid steps of 26 in easting, 2640 in northing
EASTING_STEP = 2600   # cm (26 * 100)
NORTHING_STEP = 2640  # cm (feet, half-mile)

# The LiDAR box min: these are the base offsets for computing tile indices
BOX_MIN_X = -91525.0
BOX_MIN_Y = -170396.5


def load_lidar_chunks(lidar_dir):
    """Read all lidar chunk files into a single (N, 4) numpy array [x, y, z, cls].
    Handles cross-chunk buildings by loading everything first."""
    import glob
    all_xyz = []
    all_cls = []
    files = sorted(glob.glob(os.path.join(lidar_dir, 'chunk_*.bin')))
    if not files:
        raise FileNotFoundError(
            f'No lidar chunks found in {lidar_dir}. '
            'Run tools/build_all.py first.')

    for path in files:
        with open(path, 'rb') as f:
            raw = f.read()
        count = struct.unpack_from('<I', raw, 0)[0]
        if count == 0:
            continue
        # Records: 16 bytes = 4 x f32 (x,y,z,rgba_as_f32) per spec
        stride = 16
        expected = 4 + count * stride
        if len(raw) < expected:
            print(f'  WARNING: {os.path.basename(path)} truncated '
                  f'(expected {expected}, got {len(raw)})', file=sys.stderr)
            count = (len(raw) - 4) // stride
            if count <= 0:
                continue
        # xyz as float32 view
        recs = np.frombuffer(raw, dtype=np.float32, count=count * 4, offset=4)
        recs = recs.reshape(count, 4)
        xyz = recs[:, :3].copy()
        # The 4th float32 contains RGBA packed; classification is not in this
        # format. We need to parse raw bytes to get the classification byte.
        # The lidar extract_lidar.py stores x,y,z,r,g,b,a where r,g,b are u8.
        # Classification is NOT stored in the chunk files (it was used during
        # lidar extraction for stats but not persisted). 
        # 
        # However, the original LiDAR had classification byte at offset 17.
        # Since the chunk files only store xyz+rgb, we need to re-read
        # the original .uasset or use a different strategy.
        #
        # APPROACH: Use the raw byte-level parsing directly.
        # Each point in the chunk files is 16 bytes (contra spec):
        # Actually let me check the extract_lidar.py output format more carefully.
        #
        # From extract_lidar.py line 170-188: the record is:
        #   rec_dt = [('x','<f4'),('y','<f4'),('z','<f4'),
        #             ('r','u1'),('g','u1'),('b','u1'),('a','u1')]
        # That's a 12+4=16 byte record stored as numpy structured array, so the
        # file is u32 count then count * 16 bytes with x,y,z as floats and
        # r,g,b,a as u8. Classification is NOT stored in output chunks.
        #
        # We need classification. The original LiDAR had it at byte 17 of each
        # 18-byte point record. We must go back to the source .uasset.
        
    return load_from_source_uasset()


def load_from_source_uasset():
    """Read building-class points (code 6) from the source uasset octree.
    Uses proven byte-offset walk from extract_lidar.py (no Reader dependency
    for the octree body — direct byte-level parsing with position tracking)."""
    uasset_path = os.path.join(ROOT, 'LIDAR', 'POINT_CLOUD_2019.uasset')
    if not os.path.exists(uasset_path):
        raise FileNotFoundError(f'Source uasset not found: {uasset_path}')

    sys.path.insert(0, _here)
    from uasset import Package
    sys.path.remove(_here)

    print('Reading LiDAR data from source uasset (building-class only)...')
    p = Package(uasset_path)
    data = p.data
    e = p.exports[0]

    # The octree root body starts at byte 1964 in the file
    # (tagged props + header bytes = 1964 total, per extract_lidar.py research)
    HDR_OFFSET = 1964
    EXPORT_END = e['serial_offset'] + e['serial_size']
    BOX = (-91525.0, -170396.5, -16872.5, 91525.0, 170396.5, 16872.5)

    t0 = time.time()
    points_xyz = []
    building_raw = 0

    u32 = lambda p: struct.unpack_from('<I', data, p)[0]

    def walk_body(pos, depth):
        nonlocal building_raw
        n = u32(pos); pos += 4
        for i in range(n):
            p_off = pos + i * 18
            cls_byte = data[p_off + 17]
            if cls_byte == 6:
                x, y, z = struct.unpack_from('<3f', data, p_off)
                points_xyz.append((x, y, z))
        building_raw += n
        pos += n * 18
        ne = u32(pos); pos += 4
        pos += ne * 18  # skip extra
        nc = u32(pos); pos += 4
        for _ in range(nc):
            pos += 13  # u8 idx + f32*3 center
            pos = walk_body(pos, depth + 1)
        return pos

    end = walk_body(HDR_OFFSET, 0)
    assert end == EXPORT_END - 25, f'octree end mismatch: {end} vs {EXPORT_END - 25}'
    tail = struct.unpack_from('<6f', data, end)
    assert tail == BOX, f'tail box mismatch: {tail} vs {BOX}'
    assert data[end + 24] == 1, 'tail flag not 1'

    elapsed = time.time() - t0
    print(f'  Loaded {len(points_xyz)} building-class points '
          f'(from {building_raw} raw records) in {elapsed:.1f}s')
    return np.array(points_xyz, dtype=np.float32) if points_xyz else np.empty((0, 3), dtype=np.float32)


def dbscan_cluster(points_xyz):
    """Run density-based clustering on XY coordinates.
    Returns list of (indices_array, centroid) tuples for each cluster."""
    t0 = time.time()
    xy = points_xyz[:, :2]
    tree = KDTree(xy)
    print(f'  Built KDTree ({len(points_xyz)} pts) in {time.time()-t0:.1f}s')

    # Perform DBSCAN manually via KDTree
    visited = np.zeros(len(points_xyz), dtype=bool)
    labels = np.full(len(points_xyz), -1, dtype=np.int32)
    cluster_id = 0

    for i in range(len(points_xyz)):
        if visited[i]:
            continue
        visited[i] = True
        neighbors = tree.query_ball_point(xy[i], r=EPS_CM)
        if len(neighbors) < MIN_SAMPLES:
            continue  # noise point
        # Expand cluster
        labels[neighbors] = cluster_id
        visited[neighbors] = True
        seed = list(neighbors)
        idx = 0
        while idx < len(seed):
            j = seed[idx]; idx += 1
            nbrs = tree.query_ball_point(xy[j], r=EPS_CM)
            if len(nbrs) >= MIN_SAMPLES:
                for nb in nbrs:
                    if not visited[nb]:
                        visited[nb] = True
                        labels[nb] = cluster_id
                        seed.append(nb)
                    elif labels[nb] == -1:
                        labels[nb] = cluster_id
        cluster_id += 1

    elapsed = time.time() - t0
    print(f'  DBSCAN: {cluster_id} clusters in {elapsed:.1f}s')

    clusters = []
    for cid in range(cluster_id):
        mask = labels == cid
        idx = np.flatnonzero(mask)
        centroid = points_xyz[idx].mean(axis=0)
        clusters.append((idx, centroid))

    return clusters


def extract_footprint(xy_points):
    """Compute concave hull footprint from 2D point projection."""
    if len(xy_points) < 3:
        return None
    mp = MultiPoint(xy_points.tolist())
    try:
        hull = concave_hull(mp, ratio=HULL_RATIO)
        if hull is None or hull.is_empty:
            hull = mp.convex_hull
    except Exception:
        hull = mp.convex_hull
    if hull is None or hull.is_empty:
        return None
    if hull.geom_type == 'Point' or hull.geom_type == 'LineString':
        hull = mp.convex_hull
    if hull is not None and hull.geom_type == 'Polygon':
        return hull
    return None


def generate_mesh(polygon, z_min, z_max):
    """Extrude polygon footprint into 3D mesh: walls + roof cap.
    Returns (vertices, indices) as numpy arrays."""
    if polygon is None or polygon.is_empty:
        return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.uint32)

    exterior = polygon.exterior
    if len(exterior.coords) < 4:
        return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.uint32)

    ring = np.array(exterior.coords[:-1])  # drop closing duplicate
    n = len(ring)

    # Vertices: bottom ring + top ring
    verts = np.zeros((n * 2, 3), dtype=np.float32)
    verts[:n, :2] = ring
    verts[:n, 2] = z_min
    verts[n:, :2] = ring
    verts[n:, 2] = z_max

    # Triangles: walls (2 tris per wall segment) + roof cap (triangulate)
    tri_list = []
    for i in range(n):
        a, b = i, (i + 1) % n
        # Wall quad: bottom[a], bottom[b], top[b]; bottom[a], top[b], top[a]
        tri_list.append([a, b, b + n])
        tri_list.append([a, b + n, a + n])
    # Roof cap: fan triangulation from center
    center_idx = len(verts)
    center = np.array([[ring[:, 0].mean(), ring[:, 1].mean(), z_max]], dtype=np.float32)
    verts = np.vstack([verts, center])
    for i in range(n):
        tri_list.append([center_idx, n + i, n + ((i + 1) % n)])

    indices = np.array(tri_list, dtype=np.uint32).flatten()
    return verts, indices


def compute_tile_name(centroid_x, centroid_y):
    """Generate building name from centroid location using tile grid."""
    e_idx = int(np.floor((centroid_x - BOX_MIN_X) / EASTING_STEP))
    n_idx = int(np.floor((centroid_y - BOX_MIN_Y) / NORTHING_STEP))
    e_value = 15626 + e_idx * 26  # Base easting + step
    n_value = 185064 + n_idx * 2640  # Base northing + step
    return f'B-{e_value}E-{n_value}N'


def main():
    ap = argparse.ArgumentParser(
        description='Extract 3D building meshes from LiDAR building-class points')
    ap.add_argument('--verbose', '-v', action='store_true')
    args = ap.parse_args()

    np.random.seed(SEED)

    t_total = time.time()
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(EXTRACTED_DIR, exist_ok=True)

    # Phase 1: Load + filter building-class points (T005, T006)
    print('=== Building Extraction ===')
    print('T005/T006: Loading building-class LiDAR points...')
    xyz_cls = load_lidar_chunks(LIDAR_DIR)
    if hasattr(xyz_cls, 'shape') and xyz_cls.shape[1] >= 3:
        points_xyz = xyz_cls[:, :3]

    if len(points_xyz) == 0:
        print('  No building-class points found.')
        # Write empty manifest
        manifest = {'domain': 'buildings', 'count': 0, 'tiles': []}
        with open(os.path.join(EXTRACTED_DIR, 'manifest-buildings.json'), 'w') as f:
            json.dump(manifest, f, indent=1)
        print('  Wrote empty manifest.')
        return

    print(f'  Total building-class points: {len(points_xyz)}')

    # T007: DBSCAN clustering
    print('T007: Clustering...')
    clusters = dbscan_cluster(points_xyz)
    print(f'  Found {len(clusters)} clusters')

    # T008-T012: Process each cluster
    print('T008-T012: Generating meshes...')
    buildings = []
    all_files = []
    name_counts = {}
    total_mesh_bytes = 0

    for i, (idx, centroid) in enumerate(clusters):
        cluster_pts = points_xyz[idx]
        n_pts = len(cluster_pts)

        # Compute z bounds
        z_min = float(cluster_pts[:, 2].min())
        z_max = float(cluster_pts[:, 2].max())
        height = z_max - z_min
        if height <= 0:
            height = 100.0  # minimum 1m height for degenerate clusters

        # T008: Footprint extraction
        xy = cluster_pts[:, :2]
        footprint = extract_footprint(xy)
        if footprint is None:
            continue

        # T028: Footprint area
        area_m2 = float(footprint.area) * 0.0001  # cm² -> m²

        # T009: Mesh generation
        verts, indices = generate_mesh(footprint, z_min, z_max)
        if len(verts) == 0:
            continue

        # T027: Naming
        base_name = compute_tile_name(centroid[0], centroid[1])
        if base_name not in name_counts:
            name_counts[base_name] = 0
        name_counts[base_name] += 1
        name = f'{base_name}-{name_counts[base_name]:03d}'

        # T010: Write mesh file
        fname = f'{name}.bin'
        fpath = os.path.join(OUT_DIR, fname)
        with open(fpath, 'wb') as f:
            f.write(struct.pack('<II', len(verts), len(indices)))
            f.write(verts.astype('<f4').tobytes())
            f.write(indices.tobytes())
        file_size = os.path.getsize(fpath)
        total_mesh_bytes += file_size
        all_files.append(fpath)

        # T011: Build metadata
        bld = {
            'name': name,
            'file': f'buildings/{fname}',
            'bounds_min_cm': [float(v) for v in cluster_pts.min(axis=0)],
            'bounds_max_cm': [float(v) for v in cluster_pts.max(axis=0)],
            'height_cm': round(height, 1),
            'footprint_area_m2': round(area_m2, 2),
            'point_count': int(n_pts),
            'vertex_count': int(len(verts)),
            'index_count': int(len(indices)),
        }

        # T031: Validate metadata + flag suspicious clusters
        assert bld['height_cm'] > 0, f'{name}: height <= 0'
        assert bld['footprint_area_m2'] > 0, f'{name}: area <= 0'
        assert bld['vertex_count'] > 0, f'{name}: vc <= 0'
        assert bld['index_count'] % 3 == 0, f'{name}: ic not multiple of 3'

        # Aspect ratio check for non-building structures
        sqrt_area = np.sqrt(bld['footprint_area_m2'])
        aspect = bld['height_cm'] / 100.0 / max(sqrt_area, 0.01)  # height in m / sqrt(area in m²)
        if aspect > 10:
            bld['suspicious'] = 'very_tall'  # possible tower/spire
        elif aspect < 0.1:
            bld['suspicious'] = 'very_flat'  # possible bridge/wall/noise

        buildings.append(bld)

        if args.verbose:
            print(f'  [{i+1}/{len(clusters)}] {name}: {n_pts} pts, '
                  f'{len(verts)} verts, {len(indices)} idx, '
                  f'area={area_m2:.1f}m², h={height/100:.1f}m')

    # T012: Write manifest with size check
    print(f'\nT012: Writing manifest...')
    total_mb = total_mesh_bytes / (1024 * 1024)
    print(f'  Total mesh size: {total_mb:.1f} MB')
    if total_mb > 50:
        print(f'  WARNING: Exceeds 50 MB budget (SC-003)', file=sys.stderr)

    # T029: Count metadata fields consistency
    required_fields = ['name', 'file', 'bounds_min_cm', 'bounds_max_cm',
                       'height_cm', 'footprint_area_m2', 'point_count',
                       'vertex_count', 'index_count']
    for bld in buildings:
        for rf in required_fields:
            assert rf in bld, f'Missing field {rf} in building {bld.get("name", "?")}'

    suspicious_count = sum(1 for b in buildings if 'suspicious' in b)
    if suspicious_count:
        print(f'  Flagged {suspicious_count} suspicious clusters '
              f'(aspect ratio extreme)')

    manifest = {
        'domain': 'buildings',
        'format': 'u32 vc, u32 ic, f32 pos[vc*3], u32 idx[ic] -- UE cm, no UVs',
        'eps_cm': EPS_CM,
        'min_samples': MIN_SAMPLES,
        'hull_ratio': HULL_RATIO,
        'seed': SEED,
        'count': len(buildings),
        'total_mesh_bytes': int(total_mesh_bytes),
        'tiles': buildings,
    }

    mpath = os.path.join(EXTRACTED_DIR, 'manifest-buildings.json')
    with open(mpath, 'w') as f:
        json.dump(manifest, f, indent=1)
    print(f'  Wrote {mpath}')

    elapsed = time.time() - t_total
    print(f'\n=== DONE: {len(buildings)} buildings in {elapsed:.1f}s ===')


if __name__ == '__main__':
    main()