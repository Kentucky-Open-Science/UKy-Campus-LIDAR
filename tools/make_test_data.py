#!/usr/bin/env python3
"""Generate SYNTHETIC viewer test data into web/data_test/ per the shared
data contract, then re-read every .bin the same way app.js does (DataView
semantics) to validate the parsing logic.

Open the viewer with  http://localhost:8000/?data=data_test

Scene: two adjacent 200 m (20000 cm) inclined tiles along +X (east):
  TEST_A at UE (100000..120000, 200000..220000) cm, verts stored ABSOLUTE,
         translation_cm = [0,0,0]  (mirrors how real meshes likely ship).
  TEST_B at UE (120000..140000, 200000..220000) cm, verts stored LOCAL,
         translation_cm = [120000, 200000, 0]  (exercises the offset path).
Checkerboard JPEGs have a RED square at image top-left = NW tile corner when
the viewer's default UV V-flip is ON (v stored as (y-y0)/size, flipY=false).
LiDAR: 2 chunks of 150k points each on/above tile A, xyz relative to
offset_cm = [110000, 210000, 0].
"""
import json
import os
import struct
import sys
from pathlib import Path

# tools/inspect.py (shared, must not rename) shadows the stdlib 'inspect'
# module that numpy needs — drop the script dir from sys.path before importing.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "web" / "data_test"

TILE_CM = 20000.0          # 200 m tiles
GRID_N = 33                # verts per side
BASE_Z = 3000.0            # cm
ORIGIN_CM = [120000.0, 210000.0, 3000.0]  # center seam of the two tiles
LIDAR_OFFSET_CM = [110000.0, 210000.0, 0.0]
ORIGINAL_COORDS = [71898655.0, 12345678.0, 0.0]  # fake georef doubles


def height_cm(x_abs, y_abs):
    """Synthetic terrain height (cm) from absolute UE x/y (cm)."""
    return (BASE_Z
            + 0.02 * (x_abs - 100000.0)            # incline to the east
            + 400.0 * np.sin(x_abs / 3000.0)
            + 300.0 * np.cos(y_abs / 2500.0))


def make_tile(name, x0, y0, absolute):
    """Build one grid tile. Returns (bin_bytes, translation_cm)."""
    n = GRID_N
    step = TILE_CM / (n - 1)
    jj, ii = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
    # ii = column (x/east), jj = row (y/north)
    xa = x0 + ii.astype(np.float64) * step
    ya = y0 + jj.astype(np.float64) * step
    za = height_cm(xa, ya)

    if absolute:
        px, py, tr = xa, ya, [0.0, 0.0, 0.0]
        pz = za
    else:
        px, py, tr = xa - x0, ya - y0, [x0, y0, 0.0]
        pz = za  # keep z absolute-ish; translation only in x/y here

    pos = np.stack([px, py, pz], axis=-1).reshape(-1, 3).astype("<f4")

    # UVs: u east, v_raw = (y-y0)/size  ->  with viewer default v=1-v_raw the
    # image TOP row lands on the NORTH edge (map orientation).
    u = ((xa - x0) / TILE_CM)
    v = ((ya - y0) / TILE_CM)
    uv = np.stack([u, v], axis=-1).reshape(-1, 2).astype("<f4")

    # indices: CCW seen from +Z in UE coords (viewer flips winding after the
    # Y/Z swap, yielding +Y-up front faces in three.js)
    vidx = (jj * n + ii)  # vertex index grid [row, col]
    c00 = vidx[:-1, :-1].ravel()
    c10 = vidx[:-1, 1:].ravel()    # +x
    c01 = vidx[1:, :-1].ravel()    # +y
    c11 = vidx[1:, 1:].ravel()
    tris = np.empty((c00.size * 2, 3), dtype="<u4")
    tris[0::2] = np.stack([c00, c10, c11], axis=-1)
    tris[1::2] = np.stack([c00, c11, c01], axis=-1)

    vc = pos.shape[0]
    ic = tris.size
    blob = (struct.pack("<II", vc, ic)
            + pos.tobytes() + uv.tobytes() + tris.tobytes())
    print(f"  {name}: {vc} verts, {ic} indices, {len(blob)} B "
          f"({'absolute' if absolute else 'local+translation'})")
    return blob, tr


def make_texture(name, hue_rgb):
    """512px checkerboard, RED marker square at image top-left, name label."""
    img = Image.new("RGB", (512, 512))
    d = ImageDraw.Draw(img)
    a = tuple(hue_rgb)
    b = tuple(int(c * 0.55) for c in hue_rgb)
    for ty in range(8):
        for tx in range(8):
            d.rectangle([tx * 64, ty * 64, tx * 64 + 63, ty * 64 + 63],
                        fill=a if (tx + ty) % 2 else b)
    d.rectangle([0, 0, 63, 63], fill=(220, 30, 30))  # top-left marker
    d.text((8, 70), "TL=NW", fill=(255, 255, 0))
    d.text((200, 240), name, fill=(255, 255, 255))
    return img


def make_lidar_chunks(rng):
    """Two 150k-point chunks above tile A; xyz relative to LIDAR_OFFSET_CM."""
    chunks = []
    for ci in range(2):
        npts = 150_000
        xa = rng.uniform(100000.0, 120000.0, npts)
        ya = rng.uniform(200000.0, 220000.0, npts)
        za = height_cm(xa, ya) + rng.uniform(5.0, 60.0, npts)
        # a few "buildings": columns of points
        nb = npts // 10
        xa[:nb] = rng.uniform(106000.0, 114000.0, nb)
        ya[:nb] = rng.uniform(206000.0, 214000.0, nb)
        za[:nb] = height_cm(xa[:nb], ya[:nb]) + rng.uniform(0.0, 2500.0, nb)

        rel = np.stack([xa - LIDAR_OFFSET_CM[0],
                        ya - LIDAR_OFFSET_CM[1],
                        za - LIDAR_OFFSET_CM[2]], axis=-1).astype("<f4")
        # color by height + per-chunk tint so chunk boundaries are visible
        t = np.clip((za - BASE_Z) / 3000.0, 0, 1)
        r = (60 + 195 * t).astype(np.uint8)
        g = (200 - 120 * t).astype(np.uint8) if ci == 0 else \
            (90 + 100 * t).astype(np.uint8)
        bcol = np.full(npts, 220 if ci == 0 else 60, dtype=np.uint8)
        alpha = np.full(npts, 255, dtype=np.uint8)

        rec = np.zeros(npts, dtype=[("xyz", "<f4", 3), ("rgba", "u1", 4)])
        rec["xyz"] = rel
        rec["rgba"] = np.stack([r, g, bcol, alpha], axis=-1)
        blob = struct.pack("<I", npts) + rec.tobytes()
        assert len(blob) == 4 + npts * 16
        chunks.append(blob)
        print(f"  chunk_{ci:03d}: {npts} pts, {len(blob)} B")
    return chunks


def verify():
    """Re-read every .bin exactly like app.js does (struct/offset math)."""
    print("verify: re-reading per contract (mimics JS DataView parsing)")
    ok = True
    man = json.loads((OUT / "manifest.json").read_text())
    origin = man["origin_cm"]

    for tile in man["terrain"]["tiles"]:
        buf = (OUT / tile["mesh"]).read_bytes()
        vc, ic = struct.unpack_from("<II", buf, 0)
        need = 8 + vc * 12 + vc * 8 + ic * 4
        assert len(buf) == need, f"{tile['name']}: size {len(buf)} != {need}"
        pos = np.frombuffer(buf, "<f4", vc * 3, 8).reshape(-1, 3)
        uv = np.frombuffer(buf, "<f4", vc * 2, 8 + vc * 12).reshape(-1, 2)
        idx = np.frombuffer(buf, "<u4", ic, 8 + vc * 20)
        assert idx.max() < vc and ic % 3 == 0
        assert uv.min() >= 0.0 and uv.max() <= 1.0
        tr = tile["translation_cm"]
        world = pos.astype(np.float64) + tr
        scene = (world - origin) * 0.01           # meters, pre-swizzle
        print(f"  {tile['name']}: verts={vc} tris={ic//3} "
              f"world_x=[{world[:,0].min():.0f},{world[:,0].max():.0f}]cm "
              f"world_y=[{world[:,1].min():.0f},{world[:,1].max():.0f}]cm "
              f"scene_extent_m=({np.ptp(scene[:,0]):.1f},"
              f"{np.ptp(scene[:,1]):.1f},{np.ptp(scene[:,2]):.1f})")

    ofs = man["lidar"]["offset_cm"]
    for ch in man["lidar"]["chunks"]:
        buf = (OUT / ch["file"]).read_bytes()
        (count,) = struct.unpack_from("<I", buf, 0)
        assert len(buf) == 4 + count * 16 and count == ch["count"]
        rec = np.frombuffer(buf, dtype=[("xyz", "<f4", 3), ("rgba", "u1", 4)],
                            count=count, offset=4)
        world = rec["xyz"].astype(np.float64) + ofs
        print(f"  {ch['file']}: {count} pts "
              f"world_x=[{world[:,0].min():.0f},{world[:,0].max():.0f}]cm "
              f"rgb_mean=({rec['rgba'][:,0].mean():.0f},"
              f"{rec['rgba'][:,1].mean():.0f},{rec['rgba'][:,2].mean():.0f})")
    print("verify: OK" if ok else "verify: FAILED")
    return ok


def main():
    rng = np.random.default_rng(42)
    for sub in ("meshes", "textures", "lidar"):
        (OUT / sub).mkdir(parents=True, exist_ok=True)

    print("tiles:")
    blob_a, tr_a = make_tile("TEST_A", 100000.0, 200000.0, absolute=True)
    blob_b, tr_b = make_tile("TEST_B", 120000.0, 200000.0, absolute=False)
    (OUT / "meshes/TEST_A.bin").write_bytes(blob_a)
    (OUT / "meshes/TEST_B.bin").write_bytes(blob_b)

    make_texture("TEST_A", (90, 170, 90)).save(
        OUT / "textures/TEST_A.jpg", quality=88)
    make_texture("TEST_B", (100, 130, 200)).save(
        OUT / "textures/TEST_B.jpg", quality=88)

    print("lidar:")
    for i, blob in enumerate(make_lidar_chunks(rng)):
        (OUT / f"lidar/chunk_{i:03d}.bin").write_bytes(blob)

    manifest = {
        "version": 1,
        "generated_by": "tools/make_test_data.py (SYNTHETIC)",
        "origin_cm": ORIGIN_CM,
        "terrain": {
            "tiles": [
                {"name": "TEST_A", "mesh": "meshes/TEST_A.bin",
                 "texture": "textures/TEST_A.jpg", "translation_cm": tr_a},
                {"name": "TEST_B", "mesh": "meshes/TEST_B.bin",
                 "texture": "textures/TEST_B.jpg", "translation_cm": tr_b},
            ]
        },
        "lidar": {
            "offset_cm": LIDAR_OFFSET_CM,
            "original_coordinates": ORIGINAL_COORDS,
            "chunks": [
                {"file": "lidar/chunk_000.bin", "count": 150000},
                {"file": "lidar/chunk_001.bin", "count": 150000},
            ],
        },
    }
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"wrote {OUT}")
    return 0 if verify() else 1


if __name__ == "__main__":
    sys.exit(main())
