"""Shared reader for the KYAPED citywide bare-ground elevation grid (web/data/ground.f32).

Built by `tools/ky_lidar.py --heightmap`. Used by the citywide road tools
(osm_roads/smooth_roads draping) and mirrors the same ground.json schema that
tools/build_city.py's Ground sampler reads (cell, x0, z0, nx, nz). Keeping one loader
here avoids re-implementing the cm<->scene<->cell math in every tool.
"""
import json
import os

import numpy as np


def load_grid(data_dir):
    """Return (grid float32 [nz][nx], meta dict) or (None, None) if absent."""
    try:
        gm = json.load(open(os.path.join(data_dir, "ground.json")))
        arr = np.fromfile(os.path.join(data_dir, "ground.f32"), np.float32).reshape(gm["nz"], gm["nx"])
        return arr, gm
    except Exception:  # noqa: BLE001 — optional layer
        return None, None


def sampler_cm(data_dir, fallback_y):
    """elev(lx_cm, lz_cm) -> elevation in cm, matching the osm_roads/smooth_roads heightmap
    interface (lx_cm = sceneX*100, lz_cm = -sceneZ*100). Scalar or array in/out. NaN cells
    (gaps) and out-of-grid points fall back to fallback_y metres. Returns None if no grid."""
    arr, gm = load_grid(data_dir)
    if arr is None:
        return None
    nx, nz, cell, x0, z0 = gm["nx"], gm["nz"], gm["cell"], gm["x0"], gm["z0"]
    fb_cm = float(fallback_y) * 100.0

    def elev(lx_cm, lz_cm):
        scalar = np.ndim(lx_cm) == 0
        sx = np.asarray(lx_cm, np.float64) / 100.0
        sz = -np.asarray(lz_cm, np.float64) / 100.0
        ix = np.clip(((sx - x0) / cell).astype(np.int64), 0, nx - 1)
        iz = np.clip(((sz - z0) / cell).astype(np.int64), 0, nz - 1)
        v = arr[iz, ix].astype(np.float64) * 100.0
        v = np.where(np.isfinite(v), v, fb_cm)
        return float(v) if scalar else v

    return elev
