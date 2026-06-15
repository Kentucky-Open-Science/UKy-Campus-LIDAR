#!/usr/bin/env python
"""Extract a road network from the campus aerial textures.

Reads the terrain tiles (web/data/manifest.json), stitches their aerial JPGs into
a single world-space raster (using each tile's exact planar UV->world mapping, the
SAME convention three.js renders with: texture flipY=false => image row = v*height),
detects asphalt road surfaces, isolates the thin *linear* road structures from wide
blobs (parking lots / plazas) and from building rooftops (masked out using the
building footprints in the manifest), skeletonises to centrelines, vectorises into a
graph of polylines, and writes web/data/roads.json in viewer scene coordinates
(metres):  sceneX = local_x/100 ,  sceneZ = -local_z/100 .

The viewer (roads layer) drapes these polylines onto the terrain by raycasting, so we
only need the horizontal (X,Z) centrelines + per-road width here.

Usage:  python tools/extract_roads.py [--mpp 50] [--qc]
"""
import argparse, json, os, struct, array, math
import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage as ndi
from skimage.morphology import (disk, closing, dilation,
                                skeletonize, remove_small_objects)

Image.MAX_IMAGE_PIXELS = None
HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, '..', 'web', 'data'))

# ----- tunables (cm unless noted) -----
ROAD_MAX_W_CM   = 1900    # widest thing we still call a road (drop wider blobs)
ROAD_MIN_LEN_M  = 25      # discard centreline edges shorter than this (spurs)
MIN_COMPONENT_M = 75      # drop skeleton components smaller than this (lot speckle)
RESAMPLE_M      = 5.0     # densify polylines so ribbons follow terrain over hills
SIMPLIFY_M      = 1.5     # Douglas-Peucker tolerance for polylines
ISECT_MERGE_M   = 16      # merge junction nodes closer than this into one intersection
# asphalt colour gate (HSV, 0..1). Roads here are mid-LIGHT grey (weathered
# asphalt / concrete), not dark — measured from the imagery, not assumed.
S_MAX, V_MIN, V_MAX = 0.20, 0.42, 0.76


def load_mesh(mp):
    with open(mp, 'rb') as f:
        vc = struct.unpack('<I', f.read(4))[0]; struct.unpack('<I', f.read(4))
        pos = array.array('f'); pos.frombytes(f.read(vc * 12))
        uv = array.array('f'); uv.frombytes(f.read(vc * 8))
    return np.array(pos).reshape(-1, 3), np.array(uv).reshape(-1, 2)


def load_building_footprint(path, O):
    """Return (Nx2) local (x,z) vertices of a building, or None."""
    try:
        with open(path, 'rb') as f:
            vc = struct.unpack('<I', f.read(4))[0]; struct.unpack('<I', f.read(4))
            pos = array.array('f'); pos.frombytes(f.read(vc * 12))
    except Exception:
        return None
    P = np.array(pos).reshape(-1, 3)
    lx = P[:, 0] - O[0]
    lz = O[1] - P[:, 1]          # local_z = O1 - world_y   (validated)
    return np.c_[lx, lz]


def build_world_raster(manifest, mpp):
    tiles = manifest['terrain']['tiles']
    info = []
    gx0 = gz0 = 1e18; gx1 = gz1 = -1e18
    for t in tiles:
        mp = os.path.join(DATA, t['mesh']); tp = os.path.join(DATA, t['texture'])
        if not (os.path.exists(mp) and os.path.exists(tp)):
            continue
        P, U = load_mesh(mp)
        if len(P) < 10:
            continue
        x, z = P[:, 0], P[:, 2]
        A = np.c_[x, z, np.ones(len(x))]
        cu = np.linalg.lstsq(A, U[:, 0], rcond=None)[0]
        cv = np.linalg.lstsq(A, U[:, 1], rcond=None)[0]
        info.append((tp, cu, cv, x.min(), x.max(), z.min(), z.max()))
        gx0, gx1 = min(gx0, x.min()), max(gx1, x.max())
        gz0, gz1 = min(gz0, z.min()), max(gz1, z.max())
    W = int(math.ceil((gx1 - gx0) / mpp)); H = int(math.ceil((gz1 - gz0) / mpp))
    rgb = np.zeros((H, W, 3), np.uint8)
    valid = np.zeros((H, W), bool)
    for (tp, cu, cv, x0, x1, z0, z1) in info:
        im = np.asarray(Image.open(tp).convert('RGB')); th, tw = im.shape[:2]
        px0 = max(0, int((x0 - gx0) / mpp)); px1 = min(W, int(math.ceil((x1 - gx0) / mpp)))
        pz0 = max(0, int((z0 - gz0) / mpp)); pz1 = min(H, int(math.ceil((z1 - gz0) / mpp)))
        if px1 <= px0 or pz1 <= pz0:
            continue
        WX, WZ = np.meshgrid(gx0 + (np.arange(px0, px1) + 0.5) * mpp,
                             gz0 + (np.arange(pz0, pz1) + 0.5) * mpp)
        u = cu[0] * WX + cu[1] * WZ + cu[2]; v = cv[0] * WX + cv[1] * WZ + cv[2]
        sx = np.clip((u * tw).astype(int), 0, tw - 1)
        sy = np.clip((v * th).astype(int), 0, th - 1)   # flipY=false => row = v*height
        inb = (u >= 0) & (u <= 1) & (v >= 0) & (v <= 1)
        block = im[sy, sx]; block[~inb] = 0
        dst = rgb[pz0:pz1, px0:px1]
        m = inb & (block.sum(2) > 0)
        dst[m] = block[m]
        valid[pz0:pz1, px0:px1] |= m
    return rgb, valid, (gx0, gz0, mpp, W, H)


def build_heightmap(manifest, geom, hm_mpp=400.0):
    """Rasterise terrain elevation (local_y, cm) over the world (local x,z) extent
    so road points can be draped without runtime raycasting. Returns a sampler
    elev(lx, lz) -> cm (bilinear, with holes filled by nearest)."""
    gx0, gz0, mpp, W, H = geom
    tiles = manifest['terrain']['tiles']
    hw = int(math.ceil(W * mpp / hm_mpp)) + 1
    hh = int(math.ceil(H * mpp / hm_mpp)) + 1
    acc = np.zeros((hh, hw), np.float64); cnt = np.zeros((hh, hw), np.int64)
    for t in tiles:
        mp = os.path.join(DATA, t['mesh'])
        if not os.path.exists(mp):
            continue
        P, _ = load_mesh(mp)
        if len(P) < 3:
            continue
        cx = ((P[:, 0] - gx0) / hm_mpp).astype(int)
        cy = ((P[:, 2] - gz0) / hm_mpp).astype(int)
        ok = (cx >= 0) & (cx < hw) & (cy >= 0) & (cy < hh)
        np.add.at(acc, (cy[ok], cx[ok]), P[ok, 1])      # local_y = elevation
        np.add.at(cnt, (cy[ok], cx[ok]), 1)
    have = cnt > 0
    grid = np.zeros((hh, hw), np.float64)
    grid[have] = acc[have] / cnt[have]
    if not have.all():                                   # fill holes with nearest
        idx = ndi.distance_transform_edt(~have, return_distances=False,
                                         return_indices=True)
        grid = grid[tuple(idx)]

    def elev(lx, lz):
        fx = np.clip((lx - gx0) / hm_mpp, 0, hw - 1.001)
        fy = np.clip((lz - gz0) / hm_mpp, 0, hh - 1.001)
        x0 = np.floor(fx).astype(int); y0 = np.floor(fy).astype(int)
        dx = fx - x0; dy = fy - y0
        return ((grid[y0, x0] * (1 - dx) + grid[y0, x0 + 1] * dx) * (1 - dy) +
                (grid[y0 + 1, x0] * (1 - dx) + grid[y0 + 1, x0 + 1] * dx) * dy)
    return elev


def building_mask(manifest, geom, O):
    gx0, gz0, mpp, W, H = geom
    mask_img = Image.new('1', (W, H), 0)
    d = ImageDraw.Draw(mask_img)
    from scipy.spatial import ConvexHull
    for b in manifest.get('buildings', {}).get('tiles', []):
        fp = b.get('file')
        verts = load_building_footprint(os.path.join(DATA, fp), O) if fp else None
        if verts is None or len(verts) < 3:
            mn, mx = b.get('bounds_min_cm'), b.get('bounds_max_cm')
            if not mn or not mx:
                continue
            lx0, lx1 = mn[0] - O[0], mx[0] - O[0]
            lz0, lz1 = O[1] - mn[1], O[1] - mx[1]
            verts = np.array([[lx0, lz0], [lx1, lz0], [lx1, lz1], [lx0, lz1]])
        px = (verts[:, 0] - gx0) / mpp; py = (verts[:, 1] - gz0) / mpp
        pts = np.c_[px, py]
        if len(pts) > 3:
            try:
                pts = pts[ConvexHull(pts).vertices]
            except Exception:
                pass
        d.polygon([tuple(p) for p in pts], fill=1)
    return np.asarray(mask_img, bool)


def neighbors8(r, c):
    return [(r-1, c-1), (r-1, c), (r-1, c+1), (r, c-1),
            (r, c+1), (r+1, c-1), (r+1, c), (r+1, c+1)]


def trace_graph(skel):
    """Skeleton bool image -> list of polylines (each a list of (r,c))."""
    sk = skel.copy()
    deg = ndi.convolve(sk.astype(np.uint8),
                       np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]]),
                       mode='constant') * sk
    pix = set(zip(*np.where(sk)))
    is_node = lambda p: sk[p] and (deg[p] == 1 or deg[p] >= 3)
    nodes = set(p for p in pix if is_node(p))
    polylines = []
    used = set()  # undirected first-step edges already consumed

    def nbrs(p):
        r, c = p
        return [q for q in neighbors8(r, c) if 0 <= q[0] < sk.shape[0]
                and 0 <= q[1] < sk.shape[1] and sk[q]]

    for n in nodes:
        for nb in nbrs(n):
            if (n, nb) in used:
                continue
            path = [n]; prev, cur = n, nb
            used.add((n, nb))
            while True:
                path.append(cur)
                if cur in nodes:
                    used.add((cur, prev))
                    break
                nxt = [q for q in nbrs(cur) if q != prev]
                # prefer non-diagonal continuity if branching on deg-2 noise
                if not nxt:
                    break
                used.add((cur, prev)); used.add((prev, cur))
                prev, cur = cur, nxt[0]
                used.add((prev, path[-2]))
            polylines.append(path)

    # isolated loops (cycles with no endpoint/junction node)
    seen = set(p for pl in polylines for p in pl)
    remaining = pix - seen
    while remaining:
        start = next(iter(remaining))
        path = [start]; remaining.discard(start)
        cur, prev = start, None
        while True:
            opts = [q for q in nbrs(cur) if q != prev and q in remaining]
            if not opts:
                break
            prev, cur = cur, opts[0]
            path.append(cur); remaining.discard(cur)
        if len(path) > 3:
            polylines.append(path)
    return polylines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mpp', type=float, default=50.0, help='cm per pixel of work raster')
    ap.add_argument('--qc', action='store_true', help='write QC overlay png')
    args = ap.parse_args()

    manifest = json.load(open(os.path.join(DATA, 'manifest.json')))
    O = manifest['origin_cm']
    mpp = args.mpp
    print(f'[1/6] stitching world raster @ {mpp:.0f} cm/px ...')
    rgb, valid, geom = build_world_raster(manifest, mpp)
    gx0, gz0, mpp, W, H = geom
    print(f'      raster {W}x{H}  ({W*H/1e6:.1f} MP)')

    print('[2/6] colour + building gating ...')
    f = rgb.astype(np.float32) / 255.0
    mx = f.max(2); mn = f.min(2); V = mx
    S = np.where(mx > 1e-6, (mx - mn) / np.maximum(mx, 1e-6), 0)
    asphalt = (S < S_MAX) & (V > V_MIN) & (V < V_MAX) & valid
    bld = building_mask(manifest, geom, O)
    asphalt &= ~dilation(bld, disk(2))
    asphalt = closing(asphalt, disk(2))
    asphalt = remove_small_objects(asphalt, 60, connectivity=2)

    print('[3/6] removing parking lots / plazas (wide asphalt) ...')
    # A road is THIN; parking lots / plazas / building aprons are WIDE. Fill the
    # car-gaps in lots (closing), measure local width via distance transform, then
    # delete every region wider than a road plus a margin. Thin roads survive even
    # where they touch a lot (only the lot body + margin is removed).
    roadhalf = max(3, int(round((ROAD_MAX_W_CM / 2) / mpp)))   # px
    filled = closing(asphalt, disk(6))
    dist_f = ndi.distance_transform_edt(filled)
    wide = dist_f > (roadhalf + 3)
    blob = dilation(wide, disk(roadhalf + 6))
    roads = asphalt & ~blob
    roads = remove_small_objects(roads, int(ROAD_MIN_LEN_M * 100 / mpp),
                                 connectivity=2)
    roads = closing(roads, disk(2))
    dist = ndi.distance_transform_edt(roads)            # for per-road width

    print('[4/6] skeletonise + vectorise ...')
    skel = skeletonize(roads)
    # The real street network is large + connected; parking-lot speckle and stray
    # edges form small isolated components. Keep only big connected components.
    min_comp_px = int(MIN_COMPONENT_M * 100 / mpp)
    skel = remove_small_objects(skel, min_comp_px, connectivity=2)
    polylines = trace_graph(skel)
    print(f'      raw polylines: {len(polylines)}')

    print('      baking terrain elevation (heightmap) ...')
    elev = build_heightmap(manifest, geom)
    from shapely.geometry import LineString

    def rc_to_local(rc):
        r, c = rc
        return (gx0 + (c + 0.5) * mpp, gz0 + (r + 0.5) * mpp)   # local cm (x, z)

    def local_to_scene(lx, lz):
        return (lx / 100.0, float(elev(lx, lz)) / 100.0, -lz / 100.0)  # scene (x,y,z)

    def resample(coords, step_cm):
        ls = LineString(coords)
        if ls.length <= step_cm:
            return list(ls.coords)
        n = int(math.ceil(ls.length / step_cm))
        return [ls.interpolate(i / n, normalized=True).coords[0] for i in range(n + 1)]

    roads_out = []
    junctions = []
    deg = ndi.convolve(skel.astype(np.uint8),
                       np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]]), mode='constant') * skel
    for pl in polylines:
        if len(pl) < 2:
            continue
        local = [rc_to_local(p) for p in pl]
        ls = LineString(local)
        if ls.length < ROAD_MIN_LEN_M * 100:
            continue
        ls = ls.simplify(SIMPLIFY_M * 100, preserve_topology=False)   # clean XZ shape
        local = resample(list(ls.coords), RESAMPLE_M * 100)           # follow hills
        if len(local) < 2:
            continue
        pts3 = [local_to_scene(lx, lz) for lx, lz in local]
        ds = [dist[p] for p in pl]
        width_m = float(np.clip(2 * np.median(ds) * mpp / 100.0, 5.5, ROAD_MAX_W_CM / 100.0))
        roads_out.append({'pts': [[round(x, 2), round(y, 2), round(z, 2)] for x, y, z in pts3],
                          'width': round(width_m, 2)})
        for end in (pl[0], pl[-1]):
            if deg[end] >= 3:
                junctions.append(rc_to_local(end))

    # greedy distance dedup of junction nodes (keep first within ISECT_MERGE_M)
    inter = []
    merge2 = (ISECT_MERGE_M * 100) ** 2
    for p in junctions:
        for q in inter:
            if (p[0]-q[0])**2 + (p[1]-q[1])**2 < merge2:
                break
        else:
            inter.append(p)
    inter3 = [local_to_scene(lx, lz) for lx, lz in inter]

    out = {
        'note': 'road centrelines extracted from aerial textures; scene metres; '
                'points are [x, y, z] already draped on terrain elevation',
        'source_mpp_cm': mpp,
        'roads': roads_out,
        'intersections': [[round(x, 2), round(y, 2), round(z, 2)] for x, y, z in inter3],
    }
    outpath = os.path.join(DATA, 'roads.json')
    json.dump(out, open(outpath, 'w'))
    total_km = sum(LineString([(p[0], p[2]) for p in r['pts']]).length
                   for r in roads_out) / 1000.0
    print(f'[5/6] wrote {outpath}: {len(roads_out)} roads ({total_km:.1f} km), '
          f'{len(inter3)} intersections')

    if args.qc:
        print('[6/6] QC overlay ...')
        ov = (rgb.astype(np.float32) * 0.7).astype(np.uint8)
        ov[dilation(skel, disk(1))] = (255, 40, 40)
        img = Image.fromarray(ov); d = ImageDraw.Draw(img)
        for lx, lz in inter:                         # inter is in local cm
            px = (lx - gx0) / mpp; py = (lz - gz0) / mpp
            d.ellipse([px-4, py-4, px+4, py+4], outline=(60, 200, 255), width=2)
        img.thumbnail((1400, 2400))
        img.save('/tmp/roads_qc.png'); print('      saved /tmp/roads_qc.png', img.size)
    print('done.')


if __name__ == '__main__':
    main()
