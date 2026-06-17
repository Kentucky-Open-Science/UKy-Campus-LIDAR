"""Citywide 3D buildings for Lexington: OSM footprints extruded to KYAPED LiDAR heights.

KYAPED LiDAR has no building class (6), so we can't cluster building points the way the
campus pipeline does. Instead we take OpenStreetMap building FOOTPRINTS and give each a
height from the LiDAR: a max-elevation (DSM) grid built from the non-ground (class 1)
returns gives the roof, the bare-ground grid (tools/ky_lidar --heightmap, ground.f32)
gives the base, and height = roof - base. Each footprint is extruded into a flat-roofed
prism in final scene metres and written straight into the viewer's packed building buffer
(BPK1, same format as tools/pack_buildings.py), so it renders in one draw call and the
sim's per-building collision boxes come for free.

    python -m tools.city_buildings --test          # small box around campus (fast)
    python -m tools.city_buildings                 # full Lextran service area (~130k)

Needs: ground.f32 (run `python -m tools.ky_lidar --heightmap`) + downloaded KYAPED tiles.
"""
import argparse
import concurrent.futures as cf
import json
import math
import os
import struct
import sys
import time
import urllib.parse
import urllib.request

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(_HERE, ".."))
sys.path[:] = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]
sys.path.insert(0, ROOT)
import numpy as np  # noqa: E402
import mapbox_earcut  # noqa: E402
from shapely.geometry import Polygon  # noqa: E402
from shapely import set_precision  # noqa: E402
from pyproj import Transformer  # noqa: E402

from tools.ky_lidar import (DATA, SCRATCH, georef, query_aoi, tile_path,  # noqa: E402
                            read_tile_scene, _transformer)

DSM_CELL = 3.0            # m, roof-elevation grid resolution
MIN_H, MAX_H = 2.5, 180.0  # clamp building heights (m)
OVERPASS = ["https://overpass-api.de/api/interpreter",
            "https://overpass.kumi.systems/api/interpreter"]
MAGIC = b"BPK1"
TO_LL = Transformer.from_crs(32616, 4326, always_xy=True)


# ---- elevation grids -----------------------------------------------------------
def load_ground():
    gm = json.load(open(os.path.join(DATA, "ground.json")))
    arr = np.fromfile(os.path.join(DATA, "ground.f32"), np.float32).reshape(gm["nz"], gm["nx"])
    return arr, gm


def grid_sample(arr, gm, x, z):
    ix = int((x - gm["x0"]) / gm["cell"]); iz = int((z - gm["z0"]) / gm["cell"])
    if 0 <= ix < gm["nx"] and 0 <= iz < gm["nz"]:
        v = arr[iz, ix]
        if v == v:
            return float(v)
    return None


def header_scene_bbox(path, A, B):
    """Scene-metre bbox of a tile from its LAS header only (no point load)."""
    import laspy
    from pyproj import CRS
    with laspy.open(path) as f:
        h = f.header
        src = h.parse_crs() or CRS.from_epsg(6473)
        mins, maxs = h.mins, h.maxs
    xs, ys = [mins[0], maxs[0]], [mins[1], maxs[1]]
    E, N = _transformer(src).transform([x for x in xs for _ in ys], [y for _ in xs for y in ys])
    sx = np.asarray(E) - A; sz = B - np.asarray(N)
    return float(sx.min()), float(sz.min()), float(sx.max()), float(sz.max())


def build_dsm(bbox_scene, A, B, cell=DSM_CELL):
    """Max non-ground (class 1) elevation grid over bbox_scene = (x0,z0,x1,z1)."""
    x0, z0, x1, z1 = bbox_scene
    nx = max(1, int(math.ceil((x1 - x0) / cell))); nz = max(1, int(math.ceil((z1 - z0) / cell)))
    grid = np.full(nx * nz, -np.inf, np.float32)
    tiles = [t for t in query_aoi(json.load(open(os.path.join(DATA, "city.json")))["bbox_lonlat"])
             if os.path.exists(tile_path(t["tile"], t))]
    used = 0
    for info in tiles:
        p = tile_path(info["tile"], info)
        bx0, bz0, bx1, bz1 = header_scene_bbox(p, A, B)
        if bx1 < x0 or bx0 > x1 or bz1 < z0 or bz0 > z1:
            continue                                   # tile doesn't touch the bbox
        scene, _ = read_tile_scene(p, A, B, classes={1})
        sx, sy, sz = scene[:, 0], scene[:, 1], scene[:, 2]
        m = (sx >= x0) & (sx < x1) & (sz >= z0) & (sz < z1)
        if not m.any():
            continue
        ix = ((sx[m] - x0) / cell).astype(np.int32); iz = ((sz[m] - z0) / cell).astype(np.int32)
        np.maximum.at(grid, iz * nx + ix, sy[m])
        used += 1
    grid[~np.isfinite(grid)] = np.nan
    print(f"  DSM {nx}x{nz} @ {cell} m from {used} tiles")
    return grid.reshape(nz, nx), {"x0": x0, "z0": z0, "cell": cell, "nx": nx, "nz": nz}


# ---- OSM footprints ------------------------------------------------------------
CACHE = os.path.join(SCRATCH, "osm_buildings")    # one JSON per tile (resumable)


def _overpass(query, start=0, tries=4):
    last = None
    for k in range(tries):
        url = OVERPASS[(start + k) % len(OVERPASS)]
        try:
            req = urllib.request.Request(url, data=urllib.parse.urlencode({"data": query}).encode(),
                                         headers={"User-Agent": "uky-campus-twin/1.0 (city buildings)"})
            return json.load(urllib.request.urlopen(req, timeout=180))
        except Exception as e:  # noqa: BLE001 — Overpass throttles; back off + try the other mirror
            last = e; time.sleep(3 * (k + 1))
    raise last


def _fetch_tile(task):
    i, j, box = task
    cpath = os.path.join(CACHE, f"b_{i}_{j}.json")
    if os.path.exists(cpath):
        try:
            return json.load(open(cpath))
        except Exception:  # noqa: BLE001
            pass
    ts, tw, tn, te = box
    q = f'[out:json][timeout:120];way["building"]({ts},{tw},{tn},{te});out geom;'
    data = _overpass(q, start=i + j)
    foots = {el["id"]: [(p["lon"], p["lat"]) for p in el["geometry"]]
             for el in data.get("elements", [])
             if el.get("type") == "way" and el.get("geometry") and len(el["geometry"]) >= 4}
    json.dump(foots, open(cpath, "w"))
    return foots


def fetch_footprints(bbox_lonlat, step=0.03, workers=4):
    """All OSM building ways in bbox=[w,s,e,n], fetched concurrently in cached tiles.
    Returns list of (way_id, [(lon,lat), ...] outer ring)."""
    os.makedirs(CACHE, exist_ok=True)
    w, s, e, n = bbox_lonlat
    nx = max(1, int(math.ceil((e - w) / step))); nz = max(1, int(math.ceil((n - s) / step)))
    tasks = [(i, j, (s + j * step, w + i * step, min(s + (j + 1) * step, n), min(w + (i + 1) * step, e)))
             for j in range(nz) for i in range(nx)]
    print(f"fetching OSM footprints: {len(tasks)} tiles x {workers} workers (cache {CACHE}) ...", flush=True)
    out = {}; done = 0; failed = 0
    with cf.ThreadPoolExecutor(max_workers=workers) as ex:
        futs = {ex.submit(_fetch_tile, t): t for t in tasks}
        for fut in cf.as_completed(futs):
            done += 1
            try:
                for k, v in fut.result().items():
                    out[int(k)] = v
            except Exception as ez:  # noqa: BLE001
                failed += 1
                print(f"  tile {futs[fut][:2]} failed: {ez}", flush=True)
            if done % 5 == 0 or done == len(tasks):
                print(f"  [{done}/{len(tasks)}] {len(out):,} footprints ({failed} tile fails)", flush=True)
    return list(out.items())


# ---- extrusion -----------------------------------------------------------------
def extrude(ring_xz, base_y, roof_y):
    """Outer ring (k,2 scene x,z, CCW, no repeat) -> (verts (m,3) f32, idx (p,) u32)."""
    k = len(ring_xz)
    V = np.empty((2 * k, 3), np.float32)
    V[:k, 0] = ring_xz[:, 0]; V[:k, 1] = base_y; V[:k, 2] = ring_xz[:, 1]   # base ring
    V[k:, 0] = ring_xz[:, 0]; V[k:, 1] = roof_y; V[k:, 2] = ring_xz[:, 1]   # roof ring
    idx = []
    for i in range(k):                          # walls (outward winding)
        j = (i + 1) % k
        idx += [i, k + i, k + j, i, k + j, j]
    tris = mapbox_earcut.triangulate_float64(np.ascontiguousarray(ring_xz, np.float64),
                                             np.array([k], np.uint32))
    roof = (np.asarray(tris, np.int64).reshape(-1, 3) + k)
    roof[:, [1, 2]] = roof[:, [2, 1]]           # flip so the roof faces up
    idx = np.concatenate([np.asarray(idx, np.uint32), roof.reshape(-1).astype(np.uint32)])
    return V, idx


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--bbox", nargs=4, type=float, metavar=("W", "S", "E", "N"),
                    help="lon/lat bbox (default: full city.json extent)")
    ap.add_argument("--test", action="store_true", help="small box around campus")
    ap.add_argument("--step", type=float, default=0.03, help="Overpass tile size (deg)")
    ap.add_argument("--workers", type=int, default=4, help="concurrent Overpass fetchers")
    ap.add_argument("--out", default="buildings.pack", help="output basename in web/data/")
    args = ap.parse_args()

    A, B, _ = georef()
    city = json.load(open(os.path.join(DATA, "city.json")))
    if args.test:
        bbox_ll = [-84.520, 38.030, -84.490, 38.055]     # ~campus + a bit
    elif args.bbox:
        bbox_ll = args.bbox
    else:
        bbox_ll = city["bbox_lonlat"]
    print(f"bbox lon/lat {bbox_ll}")

    foots = fetch_footprints(bbox_ll, step=args.step, workers=args.workers)
    print(f"{len(foots):,} OSM building footprints")

    # project rings to scene; compute the scene bbox actually covered
    to_utm = Transformer.from_crs(4326, 32616, always_xy=True)
    rings = []
    sxmin = szmin = 1e18; sxmax = szmax = -1e18
    for wid, ll in foots:
        lon = [p[0] for p in ll]; lat = [p[1] for p in ll]
        E, Nn = to_utm.transform(lon, lat)
        xz = np.column_stack([np.asarray(E) - A, B - np.asarray(Nn)]).astype(np.float64)
        rings.append((wid, xz))
        sxmin = min(sxmin, xz[:, 0].min()); sxmax = max(sxmax, xz[:, 0].max())
        szmin = min(szmin, xz[:, 1].min()); szmax = max(szmax, xz[:, 1].max())

    ground, gm = load_ground()
    dsm, dm = build_dsm((sxmin, szmin, sxmax, szmax), A, B)

    def dsm_at(x, z):
        ix = int((x - dm["x0"]) / dm["cell"]); iz = int((z - dm["z0"]) / dm["cell"])
        if 0 <= ix < dm["nx"] and 0 <= iz < dm["nz"]:
            v = dsm[iz, ix]
            if v == v:
                return float(v)
        return None

    all_V, all_I, recs = [], [], []
    vbase = ibase = 0
    skipped = 0
    t0 = time.time()
    for wid, xz in rings:
        if len(xz) > 1 and (xz[0] == xz[-1]).all():
            xz = xz[:-1]                                   # drop the OSM closing point
        if len(xz) < 3:
            skipped += 1; continue
        poly = Polygon(xz)
        if not poly.is_valid or poly.area < 4.0:           # skip slivers (<4 m^2)
            skipped += 1; continue
        if poly.exterior.is_ccw:                           # ensure CCW outer ring
            ring = xz
        else:
            ring = xz[::-1]
        cx, cz = poly.centroid.x, poly.centroid.y
        base = grid_sample(ground, gm, cx, cz)
        if base is None:
            base = float(city["groundY"])      # gap in the ground grid -> flat-plane fallback
        # roof = max DSM at centroid + interior sample points
        samples = [(cx, cz)] + [(0.5 * (cx + x), 0.5 * (cz + z)) for x, z in ring]
        roofs = [dsm_at(x, z) for x, z in samples]
        roofs = [r for r in roofs if r is not None]
        h = (max(roofs) - base) if roofs else 0.0
        h = min(max(h, MIN_H), MAX_H)
        V, I = extrude(np.asarray(ring, np.float64), base, base + h)
        all_V.append(V); all_I.append((I + vbase).astype(np.uint32))
        recs.append({"name": f"osm_{wid}", "vStart": vbase, "vCount": len(V),
                     "iStart": ibase, "iCount": len(I),
                     "heightM": round(h, 2), "footprintM2": round(poly.area, 1),
                     "min": [round(float(V[:, 0].min()), 2), round(base, 2), round(float(V[:, 2].min()), 2)],
                     "max": [round(float(V[:, 0].max()), 2), round(base + h, 2), round(float(V[:, 2].max()), 2)]})
        vbase += len(V); ibase += len(I)
    print(f"extruded {len(recs):,} buildings ({skipped:,} skipped) in {time.time()-t0:.0f}s")

    P = np.concatenate(all_V) if all_V else np.zeros((0, 3), np.float32)
    Idx = np.concatenate(all_I) if all_I else np.zeros((0,), np.uint32)
    binp = os.path.join(DATA, args.out + ".bin")
    bak = os.path.join(DATA, "buildings.pack.campus.bak.bin")
    if args.out == "buildings.pack" and os.path.exists(binp) and not os.path.exists(bak):
        os.replace(binp, bak); os.replace(os.path.join(DATA, "buildings.pack.json"),
                                          os.path.join(DATA, "buildings.pack.campus.bak.json"))
        print(f"  backed up campus pack -> buildings.pack.campus.bak.*")
    with open(binp, "wb") as f:
        f.write(MAGIC); f.write(struct.pack("<III", len(recs), len(P), len(Idx)))
        f.write(P.astype("<f4").tobytes()); f.write(Idx.astype("<u4").tobytes())
    json.dump({"note": "Citywide OSM footprints extruded to KYAPED LiDAR heights "
                       "(tools/city_buildings.py). Positions are final scene metres.",
               "format": "BPK1", "count": len(recs), "totalVerts": int(len(P)),
               "totalIndices": int(len(Idx)), "bin": args.out + ".bin", "buildings": recs},
              open(os.path.join(DATA, args.out + ".json"), "w"))
    mb = (16 + len(P) * 12 + len(Idx) * 4) / 1e6
    print(f"wrote {len(recs):,} buildings -> {binp}  ({len(P):,} verts, {mb:.1f} MB)")


if __name__ == "__main__":
    main()
