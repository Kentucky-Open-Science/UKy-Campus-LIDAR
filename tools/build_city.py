#!/usr/bin/env python
"""Full-city 3D building geometry for the digital twin (no UE source needed).

Footprint SHAPE comes from OpenStreetMap; roof HEIGHT comes from the KyFromAbove /
KYAPED LiDAR we already downloaded (tools/ky_lidar.py); the BASE is the flat city
ground plane (`city.json` groundY). Each OSM footprint is extruded from that plane
up to a robust roof percentile of the non-ground (class 1) LiDAR returns that fall
inside it. KYAPED has no building class, so non-ground-over-a-footprint is the roof.

Why this design:
  * OSM footprints give clean, crisp building outlines (no LiDAR blob-merging).
  * LiDAR gives real per-building heights (median ~8 m, up to ~123 m downtown).
  * A flat base keeps every building sitting cleanly on the simple ground plane;
    the real terrain still shows when the LiDAR point cloud is toggled on.
  * Footprints are fetched tiled + cached (resumable); the LiDAR is streamed one
    tile at a time and accumulated per footprint, so memory stays bounded even
    over the whole ~19x18 km service area (~114k buildings).

Prerequisites (all from committed tools):
  * web/data/manifest.json  — scene georef (build_all.py, or the ky_lidar setup)
  * extracted/ky/*.laz       — KYAPED tiles (python -m tools.ky_lidar --download-aoi)
  * web/data/ground.f32/.json— citywide ground grid (python -m tools.ky_lidar --heightmap)
  * web/data/city.json       — bbox + ground plane (python -m tools.osm_city)

Outputs:
  * web/data/buildings/*.bin           — one extruded mesh per building (UE cm, no UVs)
  * extracted/manifest-buildings.json  — building manifest (the contract pack_buildings reads)
  * merges the buildings section into web/data/manifest.json

Usage (run as a module from the repo root):
    python -m tools.build_city [--grid N] [--sample N] [--roof-pct P]
    python -m tools.pack_buildings          # then pack -> ONE fetch / ONE draw call

OSM data (c) OpenStreetMap contributors; LiDAR: KyFromAbove / KYAPED.
"""
import argparse
import json
import os
import struct
import sys
import time
from collections import defaultdict

# tools/inspect.py shadows the stdlib `inspect` that numpy/laspy need — drop our
# own dir from sys.path (mirrors tools/ky_lidar.py) and make `import tools.*` work.
_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(_HERE, ".."))
sys.path[:] = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402
import shapely  # noqa: E402
from shapely.geometry import Polygon  # noqa: E402
from shapely import STRtree  # noqa: E402

from tools.verify_buildings_osm import (load_georef, fetch_osm_buildings,  # noqa: E402
                                        osm_to_scene_polys)
from tools.extract_buildings import generate_mesh, compute_tile_name  # noqa: E402
from tools.ky_lidar import query_aoi, read_tile_scene, tile_path  # noqa: E402

DATA = os.path.join(ROOT, "web", "data")
OUT_DIR = os.path.join(DATA, "buildings")
EXTRACTED = os.path.join(ROOT, "extracted")

# tunables (CLI-overridable)
GRID = 4               # OSM building fetch is tiled GRID x GRID over the city bbox
SAMPLE = 400_000       # non-ground (class 1) points sampled per LiDAR tile
ROOF_PCT = 85.0        # roof = this percentile of in-footprint return elevations
MIN_PTS = 5            # min LiDAR returns to trust a measured roof
MIN_H, MAX_H = 3.0, 200.0   # clamp building height (m)
SIMPLIFY_M = 0.5       # footprint simplification (m) for vertex economy
MIN_AREA = 15.0        # drop footprints smaller than this (m^2)
DEFAULT_LEVH = 3.5     # m per OSM building level when no LiDAR/height tag


def largest(geom):
    """Reduce a (Multi)Polygon to its largest single Polygon (or None)."""
    if geom is None or geom.is_empty:
        return None
    if geom.geom_type == "Polygon":
        return geom
    if geom.geom_type in ("MultiPolygon", "GeometryCollection"):
        ps = [g for g in geom.geoms if g.geom_type == "Polygon" and g.area > 0]
        return max(ps, key=lambda g: g.area) if ps else None
    return None


def tag_height(t):
    """Explicit building height (m) from OSM tags, else None."""
    try:
        if "height" in t:
            return float(str(t["height"]).split()[0])
        if "building:levels" in t:
            return float(t["building:levels"]) * DEFAULT_LEVH
    except (ValueError, TypeError):
        return None
    return None


class Ground:
    """Sampler over web/data/ground.f32 (NaN = gap; nearest-fill within a window)."""

    def __init__(self):
        m = json.load(open(os.path.join(DATA, "ground.json")))
        self.cell = m["cell"]; self.x0 = m["x0"]; self.z0 = m["z0"]
        self.nx = m["nx"]; self.nz = m["nz"]
        self.g = np.fromfile(os.path.join(DATA, "ground.f32"),
                             np.float32).reshape(self.nz, self.nx)

    def at(self, sx, sz):
        ix = int((sx - self.x0) / self.cell); iz = int((sz - self.z0) / self.cell)
        if not (0 <= ix < self.nx and 0 <= iz < self.nz):
            return None
        v = self.g[iz, ix]
        if np.isfinite(v):
            return float(v)
        for r in range(1, 6):
            sub = self.g[max(0, iz - r):iz + r + 1, max(0, ix - r):ix + r + 1]
            fin = sub[np.isfinite(sub)]
            if fin.size:
                return float(np.median(fin))
        return None


def fetch_city_osm(s, w, n, e, grid):
    """Tiled OSM building fetch over [s,w,n,e], cached per cell, resumable."""
    seen = {}
    for i in range(grid):
        for j in range(grid):
            cs = s + (n - s) * i / grid; cn = s + (n - s) * (i + 1) / grid
            cw = w + (e - w) * j / grid; ce = w + (e - w) * (j + 1) / grid
            cache = os.path.join(EXTRACTED, f"osm_bld_{i}_{j}.json")
            if os.path.exists(cache):
                d = json.load(open(cache))
            else:
                print(f"  [osm] cell {i},{j} ...", flush=True)
                try:
                    d = fetch_osm_buildings(cs, cw, cn, ce)
                    json.dump(d, open(cache, "w"))
                except Exception as ex:  # noqa: BLE001 — keep going, report at end
                    print(f"    cell {i},{j} FAILED: {ex}"); continue
            for el in d["elements"]:
                seen[(el["type"], el["id"])] = el
    return {"elements": list(seen.values())}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--grid", type=int, default=GRID,
                    help="OSM fetch grid NxN over the city bbox (default 4)")
    ap.add_argument("--sample", type=int, default=SAMPLE,
                    help="class-1 LiDAR points sampled per tile (default 400000)")
    ap.add_argument("--roof-pct", type=float, default=ROOF_PCT,
                    help="roof height percentile of in-footprint returns (default 85)")
    args = ap.parse_args()

    t0 = time.time()
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(EXTRACTED, exist_ok=True)
    A, B, O = load_georef()
    city = json.load(open(os.path.join(DATA, "city.json")))
    groundY = float(city["groundY"])            # flat base elevation (the ground plane)
    w, s, e, n = city["bbox_lonlat"]            # [w, s, e, n]
    bb = city["bbox_scene"]                      # [xmin, zmin, xmax, zmax]
    print(f"[georef] A={A:.3f} B={B:.3f} groundY={groundY} | city {bb}")

    print("[1/5] OSM footprints (tiled) ...")
    osm = fetch_city_osm(s, w, n, e, args.grid)
    polys = osm_to_scene_polys(osm, A, B)
    tags = {el["id"]: el.get("tags", {}) for el in osm["elements"]
            if el["type"] in ("way", "relation")}
    foot = []
    for g, oid, _ in polys:
        g = largest(g)
        if g is None or g.area < MIN_AREA:
            continue
        c = g.centroid
        if not (bb[0] <= c.x <= bb[2] and bb[1] <= c.y <= bb[3]):
            continue
        gs = largest(g.simplify(SIMPLIFY_M, preserve_topology=True)) or g
        foot.append((gs, oid, tag_height(tags.get(oid, {}))))
    print(f"  {len(foot)} footprints across the city (of {len(polys)} fetched)")

    print("[2/5] streaming class-1 roof points over the city tiles ...")
    tree = STRtree([g.buffer(2.0) for g, _, _ in foot])
    tiles = [t for t in query_aoi([w, s, e, n])
             if os.path.exists(tile_path(t["tile"], t))]
    accum = defaultdict(list)
    for ti, t in enumerate(tiles, 1):
        try:
            scene, _ = read_tile_scene(tile_path(t["tile"], t), A, B,
                                       sample=args.sample, classes={1, 6})
        except Exception as ex:  # noqa: BLE001
            print(f"  {t['tile']}: {ex}"); continue
        pts = shapely.points(scene[:, 0], scene[:, 2])
        pi, gi = tree.query(pts, predicate="intersects")
        if len(gi):
            order = np.argsort(gi, kind="stable")
            gi_s, pi_s = gi[order], pi[order]
            uniq, st = np.unique(gi_s, return_index=True)
            en = np.append(st[1:], len(gi_s))
            for gg, a, b in zip(uniq, st, en):
                accum[int(gg)].append(scene[pi_s[a:b], 1].copy())
        if ti % 20 == 0 or ti == len(tiles):
            print(f"  [{ti}/{len(tiles)}] tiles, {len(accum)} footprints lit")
    print(f"  {len(accum)} footprints have LiDAR roof points")

    print("[3/5] meshing ...")
    grnd = Ground()
    blds, files, total = [], [], 0
    names = {}
    nlid = nosm = 0
    for k, (g, oid, tagh) in enumerate(foot):
        c = g.centroid
        gref = grnd.at(c.x, c.y)
        arrs = accum.get(k)
        ev = np.concatenate(arrs) if arrs is not None else None
        if ev is not None and len(ev) >= MIN_PTS:
            roof = float(np.percentile(ev, args.roof_pct))
            base_ref = gref if gref is not None else float(np.percentile(ev, 3.0)) - 1.0
            h = roof - base_ref; prov = "osm+lidar"; nlid += 1
        else:
            h = tagh if tagh else DEFAULT_LEVH * 2; prov = "osm"; nosm += 1
        h = float(np.clip(h, MIN_H, MAX_H))
        z_min_cm = groundY * 100.0 + O[2]          # FLAT base on the ground plane
        z_max_cm = z_min_cm + h * 100.0
        ue = Polygon([(x * 100.0 + O[0], z * 100.0 + O[1])
                      for x, z in g.exterior.coords])
        if not ue.is_valid:
            ue = largest(ue.buffer(0))
            if ue is None:
                continue
        verts, idx = generate_mesh(ue, z_min_cm, z_max_cm)
        if len(verts) == 0 or len(idx) % 3 != 0:
            continue
        nb = compute_tile_name(float(ue.centroid.x), float(ue.centroid.y))
        names[nb] = names.get(nb, 0) + 1
        name = f"{nb}-{names[nb]:03d}"
        fp = os.path.join(OUT_DIR, name + ".bin")
        with open(fp, "wb") as f:
            f.write(struct.pack("<II", len(verts), len(idx)))
            f.write(verts.astype("<f4").tobytes()); f.write(idx.tobytes())
        files.append(fp); total += os.path.getsize(fp)
        mn = verts.min(axis=0); mx = verts.max(axis=0)
        blds.append({"name": name, "file": f"buildings/{name}.bin",
                     "bounds_min_cm": [float(mn[0]), float(mn[1]), float(mn[2])],
                     "bounds_max_cm": [float(mx[0]), float(mx[1]), float(mx[2])],
                     "height_cm": round(h * 100.0, 1),
                     "footprint_area_m2": round(float(g.area), 2),
                     "point_count": int(len(ev) if ev is not None else 0),
                     "vertex_count": int(len(verts)), "index_count": int(len(idx)),
                     "provenance": prov})

    print("[4/5] clearing stale .bin ...")
    keep = {os.path.abspath(p) for p in files}
    rm = 0
    for f in os.listdir(OUT_DIR):
        if f.endswith(".bin") and os.path.abspath(os.path.join(OUT_DIR, f)) not in keep:
            os.remove(os.path.join(OUT_DIR, f)); rm += 1
    print(f"  removed {rm} stale")

    print("[5/5] manifest ...")
    man = {"domain": "buildings",
           "method": "full-city osm-footprint + kyaped-roof, flat base on ground plane",
           "source": "OpenStreetMap footprints (c) OSM contributors + KyFromAbove/KYAPED "
                     "LiDAR roof heights; base = flat city ground plane (groundY)",
           "groundY": groundY, "count": len(blds),
           "count_osm_lidar": nlid, "count_osm_only": nosm,
           "total_mesh_bytes": int(total), "tiles": blds}
    json.dump(man, open(os.path.join(EXTRACTED, "manifest-buildings.json"), "w"))
    # merge straight into the viewer manifest so the next step is just pack_buildings
    mpath = os.path.join(DATA, "manifest.json")
    wm = json.load(open(mpath))
    wm["buildings"] = {"tiles": blds, "total_mesh_bytes": int(total)}
    json.dump(wm, open(mpath, "w"), indent=2)

    print(f"\n=== {len(blds)} buildings ({nlid} osm+lidar, {nosm} osm-only), "
          f"{total/1e6:.1f} MB in {time.time()-t0:.1f}s ===")
    print("merged into web/data/manifest.json")
    print("Next: python -m tools.pack_buildings   "
          "# -> buildings.pack.bin/.json (ONE fetch, ONE draw call)")


if __name__ == "__main__":
    main()
