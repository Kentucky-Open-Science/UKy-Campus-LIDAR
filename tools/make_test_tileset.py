#!/usr/bin/env python3
"""Generate a tiny local 3D Tiles tileset (one glTF cube) anchored at a real Lexington
lat/lon, for offline end-to-end testing of the photorealistic-tiles render + alignment
path WITHOUT a Google key. The cube is placed via the tile `transform` (ENU->ECEF), so
after the viewer's baked ECEF->scene alignment it must land at the expected scene
coordinates — which tools/verify_photoreal.py checks.

    python -m tools.make_test_tileset [out_dir]   # default: web/_testtileset
"""
import base64
import json
import math
import os
import struct
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]

from pyproj import Transformer

# Anchor at the scene georef origin (so the cube centre maps to scene ~0,0 horizontally).
LON, LAT = -84.50179519085518, 38.0369173929931
GEOID_N = -33.5
H_ORTHO = 270.0
HALF = 150.0  # cube half-extent (m) -> 300 m cube, easily detectable


def cube_gltf():
    """Minimal glTF 2.0: one indexed box, bright material. Single embedded buffer."""
    v = HALF
    pos = [
        -v, -v, -v,  v, -v, -v,  v, v, -v,  -v, v, -v,
        -v, -v, v,   v, -v, v,   v, v, v,   -v, v, v,
    ]
    idx = [
        0, 1, 2, 0, 2, 3,  4, 6, 5, 4, 7, 6,  0, 4, 5, 0, 5, 1,
        1, 5, 6, 1, 6, 2,  2, 6, 7, 2, 7, 3,  3, 7, 4, 3, 4, 0,
    ]
    pos_b = struct.pack("<%df" % len(pos), *pos)
    idx_b = struct.pack("<%dH" % len(idx), *idx)
    if len(idx_b) % 4:  # 4-byte align the buffer concatenation
        idx_b += b"\x00" * (4 - len(idx_b) % 4)
    buf = idx_b + pos_b
    uri = "data:application/octet-stream;base64," + base64.b64encode(buf).decode()
    return {
        "asset": {"version": "2.0", "generator": "uky-twin test tileset"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": [{
            "attributes": {"POSITION": 1}, "indices": 0, "material": 0,
        }]}],
        "materials": [{
            "pbrMetallicRoughness": {"baseColorFactor": [1.0, 0.15, 0.1, 1.0],
                                     "metallicFactor": 0.0, "roughnessFactor": 1.0},
            "emissiveFactor": [0.6, 0.05, 0.03], "doubleSided": True,
        }],
        "buffers": [{"uri": uri, "byteLength": len(buf)}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": len(idx_b), "target": 34963},
            {"buffer": 0, "byteOffset": len(idx_b), "byteLength": len(pos_b), "target": 34962},
        ],
        "accessors": [
            {"bufferView": 0, "componentType": 5123, "count": len(idx),
             "type": "SCALAR", "max": [max(idx)], "min": [min(idx)]},
            {"bufferView": 1, "componentType": 5126, "count": 8, "type": "VEC3",
             "max": [v, v, v], "min": [-v, -v, -v]},
        ],
    }


def enu_to_ecef_transform(lon, lat, h):
    """Column-major 4x4 mapping local ENU metres -> ECEF, for the 3D Tiles `transform`."""
    to_ecef = Transformer.from_crs(4326, 4978, always_xy=True)
    px, py, pz = to_ecef.transform(lon, lat, h)
    rlon, rlat = math.radians(lon), math.radians(lat)
    east = [-math.sin(rlon), math.cos(rlon), 0.0]
    up = [math.cos(rlat) * math.cos(rlon), math.cos(rlat) * math.sin(rlon), math.sin(rlat)]
    north = [up[1] * east[2] - up[2] * east[1],
             up[2] * east[0] - up[0] * east[2],
             up[0] * east[1] - up[1] * east[0]]
    # columns: east, north, up, position
    return [east[0], east[1], east[2], 0.0,
            north[0], north[1], north[2], 0.0,
            up[0], up[1], up[2], 0.0,
            px, py, pz, 1.0]


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(_HERE, "..", "web", "_testtileset")
    out = os.path.abspath(out)
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "cube.gltf"), "w") as f:
        json.dump(cube_gltf(), f)
    tileset = {
        "asset": {"version": "1.0"},
        "geometricError": 1000,
        "root": {
            "boundingVolume": {"sphere": [0, 0, 0, HALF * 2]},
            "geometricError": 0,
            "refine": "REPLACE",
            "transform": enu_to_ecef_transform(LON, LAT, H_ORTHO + GEOID_N),
            "content": {"uri": "cube.gltf"},
        },
    }
    with open(os.path.join(out, "tileset.json"), "w") as f:
        json.dump(tileset, f, indent=1)
    print("wrote", out, "(anchor lon/lat %.5f,%.5f)" % (LON, LAT))


if __name__ == "__main__":
    main()
