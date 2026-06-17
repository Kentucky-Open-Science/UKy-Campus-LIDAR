#!/usr/bin/env python
"""Bake the 3,109 per-building meshes into ONE packed buffer for fast loading.

The viewer's per-building path fetches one tiny .bin per building — ~3,100 HTTP
round-trips that dominate load time and create ~3,100 draw calls. This tool merges
them all into a single `web/data/buildings.pack.bin` (+ a small JSON sidecar of
per-building ranges/metadata), so the viewer makes ONE request and builds ONE
geometry / ONE draw call.

Crucially it bakes the transforms the viewer used to do per vertex at runtime:
  * UE cm -> scene metres with the (x, z, y) axis swap and origin subtraction,
  * the tools/ground_buildings.py drop (ground_y_m) so each base sits on the
    terrain — except `bridge` buildings, which keep their modelled elevation,
  * the winding flip for the handedness change.
The packed positions are therefore final scene-space; the viewer uploads them
directly (no per-vertex work) and computes normals once.

Picking + colour-by-height still work: the JSON sidecar records each building's
vertex/index range, name, and height, so a raycast hit's faceIndex maps back to a
building, and the viewer fills a per-vertex colour from the per-building height.

    python -m tools.pack_buildings        # reads web/data/manifest.json

Output: web/data/buildings.pack.bin, web/data/buildings.pack.json
"""
import json
import os
import struct

import numpy as np

from tools.transit_common import DATA

MAGIC = b"BPK1"


def parse_building(buf):
    vc, ic = struct.unpack_from("<II", buf, 0)
    need = 8 + vc * 12 + ic * 4
    if len(buf) < need:
        raise ValueError(f"truncated: need {need}, have {len(buf)}")
    pos = np.frombuffer(buf, dtype="<f4", count=vc * 3, offset=8).reshape(vc, 3)
    idx = np.frombuffer(buf, dtype="<u4", count=ic, offset=8 + vc * 12)
    return vc, ic, pos, idx


def main():
    manifest = json.load(open(os.path.join(DATA, "manifest.json")))
    o = manifest["origin_cm"]
    blds = (manifest.get("buildings") or {}).get("tiles") or []
    if not blds:
        print("no buildings in manifest; nothing to pack")
        return

    all_pos = []          # list of (n,3) float32 scene-space arrays
    all_idx = []          # list of (m,) uint32 arrays, already offset
    records = []
    vbase = ibase = 0
    skipped = 0

    for b in blds:
        path = os.path.join(DATA, b["file"])
        try:
            with open(path, "rb") as f:
                vc, ic, pos, idx = parse_building(f.read())
        except (FileNotFoundError, ValueError) as e:
            skipped += 1
            continue

        # UE cm -> scene metres with the viewer's axis swap (x, z, y) and origin sub
        sx = (pos[:, 0] - o[0]) * 0.01
        sy = (pos[:, 2] - o[2]) * 0.01
        sz = (pos[:, 1] - o[1]) * 0.01

        # bake the ground drop (ground_y_m - bbox.min.y), unless this is a bridge
        gy = b.get("ground_y_m", b.get("groundYm"))
        if gy is not None and not b.get("bridge"):
            sy = sy + (float(gy) - float(sy.min()))

        scene = np.empty((vc, 3), dtype="<f4")
        scene[:, 0] = sx; scene[:, 1] = sy; scene[:, 2] = sz

        # winding flip for the handedness change (swap each triangle's 2nd/3rd idx)
        tri = idx[: (ic // 3) * 3].reshape(-1, 3).copy()
        tri[:, [1, 2]] = tri[:, [2, 1]]
        gidx = (tri.reshape(-1) + vbase).astype("<u4")

        all_pos.append(scene)
        all_idx.append(gidx)
        records.append({
            "name": b["name"],
            "vStart": vbase, "vCount": vc,
            "iStart": ibase, "iCount": int(gidx.shape[0]),
            "heightM": round((b.get("height_cm") or b.get("heightCm") or 0) / 100.0, 2),
            "footprintM2": round(b.get("footprint_area_m2") or b.get("footprintAreaM2") or 0, 1),
            # scene-space AABB so the agent collision broad-phase + ground probe can
            # use per-building boxes without un-merging the render mesh.
            "min": [round(float(sx.min()), 2), round(float(sy.min()), 2), round(float(sz.min()), 2)],
            "max": [round(float(sx.max()), 2), round(float(sy.max()), 2), round(float(sz.max()), 2)],
        })
        vbase += vc
        ibase += int(gidx.shape[0])

    P = np.concatenate(all_pos) if all_pos else np.zeros((0, 3), "<f4")
    I = np.concatenate(all_idx) if all_idx else np.zeros((0,), "<u4")
    total_v, total_i = P.shape[0], I.shape[0]

    bin_path = os.path.join(DATA, "buildings.pack.bin")
    with open(bin_path, "wb") as f:
        f.write(MAGIC)
        f.write(struct.pack("<III", len(records), total_v, total_i))
        f.write(P.tobytes())
        f.write(I.tobytes())

    meta = {
        "note": "Packed building geometry: positions are final scene metres (axis-"
                "swapped, origin-subtracted, ground-dropped); indices are global + "
                "winding-flipped. One fetch, one draw call. See tools/pack_buildings.py.",
        "format": "BPK1",
        "count": len(records), "totalVerts": int(total_v), "totalIndices": int(total_i),
        "bin": "buildings.pack.bin",
        "buildings": records,
    }
    json_path = os.path.join(DATA, "buildings.pack.json")
    with open(json_path, "w") as f:
        json.dump(meta, f)

    mb = (16 + total_v * 12 + total_i * 4) / 1e6
    print(f"packed {len(records)} buildings ({skipped} missing/bad) -> {bin_path}")
    print(f"  {total_v:,} verts, {total_i:,} indices, {mb:.1f} MB in ONE file "
          f"(was {len(blds)} fetches)")
    print(f"  metadata -> {json_path}")


if __name__ == "__main__":
    main()
