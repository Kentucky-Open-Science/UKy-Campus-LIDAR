"""Extract the LidarPointCloud octree from POINT_CLOUD_2019.uasset into
web-ready decimated point-cloud chunks.

Reverse-engineered serialization (verified byte-exact over the full 448 MB,
see extracted/REPORT-lidar.md):

  export native data (after tagged props + 4-byte guid flag, offset 1775):
    u32 = 0
    FString source path  (".../LIDAR/points_colorized.las")
    u32 = 1
    u32 = 0x000D0001     (PCPF data version pair, twice)
    u32 = 0x000D0001
    u32 = 1
    f32[6]  bounds box: min xyz, max xyz  [UE cm]
    u32 = 1
    18 zero bytes        (one all-zero point-sized record)
    <root node_body>
    f32[6]  bounds box again
    u8 = 1
    == export end ==

  node_body :=
    u32 nPoints,   nPoints * point(18B)
    u32 nExtra,    nExtra  * point(18B)     ("padding"/overflow points)
    u32 nChildren  (0..8)
    nChildren * { u8 octant_idx, f32[3] ABSOLUTE node center, node_body }

  point (18 bytes) :=
    f32 x, f32 y, f32 z          [UE world cm, absolute]
    u8 B, u8 G, u8 R, u8 A       (FColor; A = intensity-ish, varies)
    u8 flags                     (1 = bVisible)
    u8 classification            (LAS class id)

The octree is a uniform cube: root center (0,0,0), half-extent 170396.5 cm
(= max axis of the bounds box).  Points exist at EVERY depth (LOD subsets);
the full cloud is the union of all node arrays (main + extra).

Output (data contract):
  web/data/lidar/chunk_NNN.bin : u32 count, then count * {f32 x,y,z, u8 r,g,b,a}
  extracted/manifest-lidar.json
  extracted/lidar_topdown.png  (sanity render)

Decimation: uniform random keep with fixed seed (rng PCG64 seed 2019) at
ratio TARGET/total, applied to the full concatenated point stream.
"""
import os
import sys
import json
import struct
import time

# tools/inspect.py shadows the stdlib 'inspect' that numpy needs; make sure
# this script's directory is not on sys.path before importing numpy.
_here = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path
            if os.path.abspath(p if p else '.') != _here]
import numpy as np  # noqa: E402

# Repo root (the CAMPUS/ layout: LIDAR/, MESHES/, tools/, extracted/ live at the
# same level as this file's parent). Was a hardcoded dev-machine path; resolved
# relative to the repo so the tool works on any checkout.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH = os.path.join(ROOT, 'LIDAR', 'POINT_CLOUD_2019.uasset')
OUT_DIR = os.path.join(ROOT, 'web', 'data', 'lidar')
EXTRACTED = os.path.join(ROOT, 'extracted')
EXPORT_END = 448664675
ROOT_EXTENT = 170396.5
BOX = (-91525.0, -170396.5, -16872.5, 91525.0, 170396.5, 16872.5)
TARGET = 12_000_000
SEED = 2019

sys.setrecursionlimit(10000)


def walk_collect(data):
    """Walk the octree, returning list of (byte_offset, count) point slices."""
    u32 = lambda p: struct.unpack_from('<I', data, p)[0]
    slices = []

    def body(pos, depth):
        n = u32(pos); pos += 4
        if n:
            slices.append((pos, n, depth, 0))
        pos += n * 18
        ne = u32(pos); pos += 4
        if ne:
            slices.append((pos, ne, depth, 1))
        pos += ne * 18
        nc = u32(pos); pos += 4
        assert nc <= 8, (nc, pos)
        for _ in range(nc):
            pos += 13  # u8 idx + absolute center fvector
            pos = body(pos, depth + 1)
        return pos

    end = body(1964, 0)
    assert end == EXPORT_END - 25, end
    # trailing 25B: bounds box repeated + u8(1)
    tail_box = struct.unpack_from('<6f', data, end)
    assert tail_box == BOX, tail_box
    assert data[end + 24] == 1
    return slices


def main():
    t0 = time.time()
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(EXTRACTED, exist_ok=True)
    with open(PATH, 'rb') as f:
        data = f.read()
    print(f'loaded {len(data)} bytes  [{time.time()-t0:.1f}s]')

    slices = walk_collect(data)
    total = sum(n for _, n, _, _ in slices)
    print(f'{len(slices)} point arrays, total {total} points  [{time.time()-t0:.1f}s]')

    # ---- gather all points -------------------------------------------------
    pos = np.empty((total, 3), np.float32)
    col = np.empty((total, 4), np.uint8)   # B,G,R,A as stored
    cls = np.empty(total, np.uint8)
    flg = np.empty(total, np.uint8)
    d = 0
    for off, n, depth, kind in slices:
        raw = np.frombuffer(data, np.uint8, n * 18, off).reshape(n, 18)
        pos[d:d+n] = raw[:, :12].copy().view('<f4')
        col[d:d+n] = raw[:, 12:16]
        flg[d:d+n] = raw[:, 16]
        cls[d:d+n] = raw[:, 17]
        d += n
    assert d == total
    del data
    print(f'points gathered  [{time.time()-t0:.1f}s]')

    # ---- validation stats --------------------------------------------------
    mn = pos.min(axis=0); mx = pos.max(axis=0)
    print('point bounds min:', mn, ' max:', mx)
    print('expected box   :', BOX)
    uniq_f, cnt_f = np.unique(flg, return_counts=True)
    print('flag byte histogram:', dict(zip(uniq_f.tolist(), cnt_f.tolist())))
    uniq_c, cnt_c = np.unique(cls, return_counts=True)
    cls_hist = dict(zip(uniq_c.tolist(), cnt_c.tolist()))
    print('classification histogram:', cls_hist)
    cmean = col.astype(np.float64).mean(axis=0)
    print(f'color means B={cmean[0]:.1f} G={cmean[1]:.1f} R={cmean[2]:.1f} A={cmean[3]:.1f}')
    pct = np.percentile(col[:, :3].astype(np.float32), [1, 25, 50, 75, 99], axis=0)
    print('BGR percentiles (1,25,50,75,99):\n', pct)
    zpct = np.percentile(pos[:, 2], [0, 1, 25, 50, 75, 99, 100])
    print('z percentiles (0,1,25,50,75,99,100):', zpct)

    # ---- decimation ---------------------------------------------------------
    rng = np.random.default_rng(SEED)
    keep = rng.random(total) < (TARGET / total)
    kidx = np.flatnonzero(keep)
    kept = len(kidx)
    print(f'decimated {total} -> {kept}  [{time.time()-t0:.1f}s]')
    kpos = pos[kidx]; kcol = col[kidx]
    del pos, col, cls, flg, keep, kidx

    # ---- spatial chunking ---------------------------------------------------
    for nx, ny in ((8, 8), (8, 16), (16, 16), (16, 32)):
        ix = np.clip(((kpos[:, 0] - BOX[0]) / (BOX[3] - BOX[0]) * nx).astype(np.int32), 0, nx - 1)
        iy = np.clip(((kpos[:, 1] - BOX[1]) / (BOX[4] - BOX[1]) * ny).astype(np.int32), 0, ny - 1)
        cell = iy * nx + ix
        counts = np.bincount(cell, minlength=nx * ny)
        if counts.max() <= 1_000_000:
            break
    print(f'grid {nx}x{ny}, max cell {counts.max()}, nonempty {np.count_nonzero(counts)}')

    order = np.argsort(cell, kind='stable')
    kpos = kpos[order]; kcol = kcol[order]; cell = cell[order]
    bounds = np.searchsorted(cell, np.arange(nx * ny + 1))

    rec_dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                       ('r', 'u1'), ('g', 'u1'), ('b', 'u1'), ('a', 'u1')])
    chunks = []
    ci = 0
    for c in range(nx * ny):
        a, b = bounds[c], bounds[c + 1]
        n = b - a
        if n == 0:
            continue
        rec = np.empty(n, rec_dt)
        rec['x'] = kpos[a:b, 0]; rec['y'] = kpos[a:b, 1]; rec['z'] = kpos[a:b, 2]
        rec['r'] = kcol[a:b, 2]  # stored BGRA -> output RGB
        rec['g'] = kcol[a:b, 1]
        rec['b'] = kcol[a:b, 0]
        rec['a'] = 255
        fname = f'chunk_{ci:03d}.bin'
        with open(f'{OUT_DIR}/{fname}', 'wb') as f:
            f.write(struct.pack('<I', n))
            rec.tofile(f)
        cmn = kpos[a:b].min(axis=0); cmx = kpos[a:b].max(axis=0)
        chunks.append({'file': fname, 'count': int(n),
                       'bounds_min_cm': [float(v) for v in cmn],
                       'bounds_max_cm': [float(v) for v in cmx]})
        ci += 1
    print(f'wrote {ci} chunks  [{time.time()-t0:.1f}s]')

    # ---- top-down sanity render --------------------------------------------
    W = 1024
    # square pixels over the full XY box (Y is the long axis: 340793 cm)
    span = max(BOX[3] - BOX[0], BOX[4] - BOX[1])
    H = W
    px = np.clip(((kpos[:, 0] - BOX[0]) / span * W).astype(np.int32), 0, W - 1)
    py = np.clip(((kpos[:, 1] - BOX[1]) / span * H).astype(np.int32), 0, H - 1)
    lin = py * W + px
    npx = W * H
    cnts = np.bincount(lin, minlength=npx).astype(np.float64)
    img = np.zeros((npx, 3), np.float64)
    for ch, src in ((0, 2), (1, 1), (2, 0)):  # RGB from BGR
        img[:, ch] = np.bincount(lin, weights=kcol[:, src].astype(np.float64), minlength=npx)
    nz = cnts > 0
    img[nz] /= cnts[nz, None]
    img = img.reshape(H, W, 3).astype(np.uint8)
    img = img[::-1]  # +Y up
    from PIL import Image
    Image.fromarray(img, 'RGB').save(f'{EXTRACTED}/lidar_topdown.png')
    print(f'render saved  [{time.time()-t0:.1f}s]')

    # ---- manifest -----------------------------------------------------------
    manifest = {
        'domain': 'lidar',
        'source': 'LIDAR/POINT_CLOUD_2019.uasset',
        'coordinate_frame': 'UE world centimeters, Z-up (raw, not recentered)',
        'offset_cm': [0.0, 0.0, 0.0],
        'total_points_full': int(total),
        'total_points_kept': int(kept),
        'decimation': f'uniform random keep p={TARGET/total:.6f}, numpy PCG64 seed {SEED}',
        'bounds_min_cm': [BOX[0], BOX[1], BOX[2]],
        'bounds_max_cm': [BOX[3], BOX[4], BOX[5]],
        'chunk_grid': [int(nx), int(ny)],
        'point_record': 'f32 x,y,z (UE cm) + u8 r,g,b,a(=255), 16B stride, little-endian; file = u32 count + records',
        'original_coordinates': ORIGINAL_COORDS,
        'classifications': [2, 6, 3, 5, 4, 17, 8, 1, 7],
        'classification_histogram': {str(k): int(v) for k, v in cls_hist.items()},
        'chunks': chunks,
    }
    with open(f'{EXTRACTED}/manifest-lidar.json', 'w') as f:
        json.dump(manifest, f, indent=1)
    print(f'manifest written; ALL DONE  [{time.time()-t0:.1f}s]')


ORIGINAL_COORDS = None  # filled by __main__ via read_original_coords()


def read_original_coords():
    """Parse OriginalCoordinates (DoubleVector tagged struct) + classifications."""
    sys.path.insert(0, _here)
    from uasset import Package, Reader
    sys.path.remove(_here)
    p = Package(PATH)
    r = Reader(p.data, p.exports[0]['serial_offset'])
    props = p.read_properties(r)
    coords = None
    for t in props:
        if t['name'] == 'OriginalCoordinates':
            inner = {q['name']: q['value'] for q in t['value']}
            coords = [inner.get('X', 0.0), inner.get('Y', 0.0), inner.get('Z', 0.0)]
    return coords


if __name__ == '__main__':
    ORIGINAL_COORDS = read_original_coords()
    print('OriginalCoordinates:', ORIGINAL_COORDS)
    main()
