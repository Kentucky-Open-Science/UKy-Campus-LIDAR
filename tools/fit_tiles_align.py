#!/usr/bin/env python3
"""Fit the ECEF -> scene similarity transform for the photorealistic 3D-tile basemap.

The photorealistic tiles (Google / Cesium) are delivered in geocentric ECEF
(EPSG:4978). The viewer's world is the UTM-16N-derived scene frame
(scene_x = E - A, scene_z = B - N, scene_y = NAVD88 orthometric metres). This
script finds the best-fit similarity transform (uniform scale s, rotation R with
det +1, translation t) such that

    scene ≈ s * R @ ecef + t

by sampling a city-wide grid of control points and projecting each one through the
EXACT pipeline projection (tools.transit_common.Projector, pyproj 4326->32616). A
single global similarity absorbs the UTM grid convergence (~1.5° here) and point
scale (~0.0002) automatically, so the tiles line up with the OSM buildings/roads
across the whole city — exactly at the centre, within a couple of metres at the
edges (residuals are printed).

Output: web/lib/tiles_align.json  (column-major 4x4 `elements` for THREE.Matrix4,
plus metadata). tiles3d.js loads it (with an embedded fallback). Re-run after any
change to the georef anchor:

    python -m tools.fit_tiles_align

Vertical datum: scene_y is NAVD88 orthometric; tiles are ellipsoidal. We map with a
constant geoid undulation (GEOID18 central KY ≈ -33.5 m); the viewer also exposes a
vertical nudge for the residual.
"""
import json
import os
import sys

# Run as a module from repo root; drop tools/ from sys.path so tools/inspect.py
# doesn't shadow stdlib inspect (project convention).
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]

import numpy as np
from pyproj import Transformer

from tools.transit_common import Projector, load_manifest, georef

GEOID_N = -33.5  # GEOID18 undulation, central KY (ellipsoidal = orthometric + N)
OUT = os.path.join(_HERE, "..", "web", "lib", "tiles_align.json")


def umeyama(src, dst):
    """Similarity transform mapping src->dst (Umeyama 1991, with scale)."""
    n = src.shape[0]
    mu_s, mu_d = src.mean(0), dst.mean(0)
    sc, dc = src - mu_s, dst - mu_d
    cov = (dc.T @ sc) / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_s = (sc ** 2).sum() / n
    s = float(np.trace(np.diag(D) @ S) / var_s)
    t = mu_d - s * R @ mu_s
    return s, R, t


def main():
    manifest = load_manifest()
    A, B = georef(manifest)
    proj = Projector(manifest)
    to_ecef = Transformer.from_crs(4326, 4978, always_xy=True)  # lon/lat/h_ellip -> ECEF

    # City bbox (lon/lat) straight from the OSM city context.
    with open(os.path.join(_HERE, "..", "web", "data", "city.json")) as f:
        city = json.load(f)
    minlon, minlat, maxlon, maxlat = city["bbox_lonlat"]

    # Control grid over the city, at three altitudes so the vertical axis of R is
    # well constrained (city-scale points are nearly coplanar otherwise).
    lons = np.linspace(minlon, maxlon, 9)
    lats = np.linspace(minlat, maxlat, 9)
    H0 = 270.0  # representative ground (NAVD88 m); relief comes from the tiles
    src, dst = [], []
    for H in (H0, H0 + 1500.0, H0 + 4000.0):
        for lo in lons:
            for la in lats:
                sx, sz = proj(lo, la)
                X, Y, Z = to_ecef.transform(lo, la, H + GEOID_N)
                src.append([X, Y, Z])
                dst.append([sx, H, sz])
    src, dst = np.asarray(src), np.asarray(dst)

    s, R, t = umeyama(src, dst)

    # Residuals.
    pred = (s * (R @ src.T).T) + t
    err = np.linalg.norm(pred - dst, axis=1)
    herr = np.linalg.norm((pred - dst)[:, [0, 2]], axis=1)  # horizontal only

    # Column-major 4x4 for THREE.Matrix4.fromArray (scene = M * ecef_homogeneous).
    sR = s * R
    elements = [
        sR[0, 0], sR[1, 0], sR[2, 0], 0.0,
        sR[0, 1], sR[1, 1], sR[2, 1], 0.0,
        sR[0, 2], sR[1, 2], sR[2, 2], 0.0,
        t[0],     t[1],     t[2],     1.0,
    ]

    # Origin geodetic (scene 0,0,0) for documentation / runtime ENU fallback.
    to_ll = Transformer.from_crs(32616, 4326, always_xy=True)
    olon, olat = to_ll.transform(A, B)

    # Corner control points (lon/lat + EXACT pipeline-projector scene x,z) emitted so the
    # offline matrix check (tools/verify_tiles_align.mjs) reads them instead of embedding
    # hand-copied literals that could silently drift from the regenerated matrix.
    check_h = H0 + GEOID_N
    corners = [{"lon": float(lo), "lat": float(la), "sx": float(proj(lo, la)[0]),
                "sz": float(proj(lo, la)[1])}
               for lo, la in [(minlon, minlat), (maxlon, maxlat),
                              (minlon, maxlat), (maxlon, minlat)]]

    out = {
        "note": "ECEF(EPSG:4978) -> viewer scene metres. scene = M * ecef. "
                "Column-major 'elements' for new THREE.Matrix4().fromArray(elements). "
                "Regenerate with: python -m tools.fit_tiles_align",
        "elements": elements,
        "scale": s,
        "georef": {"A": A, "B": B, "epsg": 32616},
        "originLonLat": [olon, olat],
        "geoidN": GEOID_N,
        "checkHeight": check_h,
        "corners": corners,
        "residual_m": {
            "rms": float(err.mean()), "max": float(err.max()),
            "horiz_rms": float(herr.mean()), "horiz_max": float(herr.max()),
        },
        "controlPoints": int(src.shape[0]),
    }
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        json.dump(out, f, indent=2)

    # Sanity: project the 4 bbox corners and compare scene vs city.json bbox.
    print("scale s = %.8f  (point-scale ~%.5f)" % (s, s))
    print("residual horiz rms=%.2fm max=%.2fm | 3D rms=%.2fm max=%.2fm  (n=%d)" % (
        out["residual_m"]["horiz_rms"], out["residual_m"]["horiz_max"],
        out["residual_m"]["rms"], out["residual_m"]["max"], src.shape[0]))
    print("origin lon/lat = %.6f, %.6f" % (olon, olat))
    print("bbox corner check (scene_x, scene_z) via fit vs pipeline projector:")
    for lo, la in [(minlon, minlat), (maxlon, maxlat), (minlon, maxlat), (maxlon, minlat)]:
        X, Y, Z = to_ecef.transform(lo, la, H0 + GEOID_N)
        p = (s * (R @ np.array([X, Y, Z])) + t)
        px, pz = proj(lo, la)
        print("  (%.4f,%.4f) fit=(%.1f,%.1f) proj=(%.1f,%.1f) d=%.2fm" % (
            lo, la, p[0], p[2], px, pz, ((p[0] - px) ** 2 + (p[2] - pz) ** 2) ** 0.5))
    print("wrote", os.path.normpath(OUT))


if __name__ == "__main__":
    main()
