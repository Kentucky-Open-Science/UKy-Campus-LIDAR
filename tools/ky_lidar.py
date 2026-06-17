"""Import KyFromAbove / KYAPED LiDAR tiles into the campus digital twin.

The campus point cloud was extracted from a single UE asset (POINT_CLOUD_2019);
this pulls the authoritative statewide LiDAR (https://kyfromabove.ky.gov, the
"Ky_KYAPED_Point_Cloud_Index" feature service) so we can fill in the REST of
Lexington and let the newer scans supersede the campus-only data.

Pipeline per 5,000-ft tile:
  1. pick the newest available phase (2025 > 2019 > 2010) download URL from the index
  2. download the .laz / .copc.laz from S3 (kyfromabove.s3.us-west-2.amazonaws.com)
  3. read with laspy; reproject the points from the tile's native CRS
     (NAD83(2011) / Kentucky Single Zone, US survey feet) into the scene's UTM-16N
     frame, then into scene metres with the SAME georef the rest of the twin uses
  4. colour by elevation (hypsometric) — KYAPED point format 6 has no RGB
  5. decimate + spatially bin into web/data/lidar chunks (UE-cm, the viewer's format)
  6. accumulate ground-class (2) points into a city-wide elevation heightmap

Scene frame (matches web/app.js line 6 + tools/twin_server.Ground):
    sx = easting  - A          ue.x = sx*100 + origin_cm[0]
    sz = B - northing          ue.y = sz*100 + origin_cm[1]
    sy = elevation (m)         ue.z = sy*100 + origin_cm[2]
  where A, B and origin_cm come from web/data/manifest.json.

CLI:
    python -m tools.ky_lidar --list                 # tiles over the city AOI
    python -m tools.ky_lidar --tile N091E300        # download+import one tile -> render
    python -m tools.ky_lidar --aoi                  # full run (download+import all)
"""
import argparse
import json
import os
import struct
import sys
import time
import urllib.parse
import urllib.request

# tools/inspect.py shadows the stdlib `inspect` that numpy needs; drop our own dir.
_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(_HERE, ".."))
sys.path[:] = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]
import numpy as np  # noqa: E402

DATA = os.path.join(ROOT, "web", "data")
LIDAR_DIR = os.path.join(DATA, "lidar")
SCRATCH = os.path.join(ROOT, "extracted", "ky")        # gitignored downloads/intermediates
INDEX = ("https://services3.arcgis.com/ghsX9CKghMvyYjBU/arcgis/rest/services/"
         "Ky_KYAPED_Point_Cloud_Index_WM_gdb/FeatureServer/0")
FT_US = 0.3048006096          # US survey foot -> metre (KY SP + NAVD88 are ftUS)
UTM16N = "EPSG:32616"
# fixed city-wide elevation range (m) for consistent hypsometric colour across tiles;
# Lexington ground runs ~240-320 m NAVD88, buildings push higher and clamp to white.
ELEV_LO, ELEV_HI = 235.0, 315.0


# ---- scene georef (read from the manifest so nothing is hard-coded) -------------
def georef():
    m = json.load(open(os.path.join(DATA, "manifest.json")))
    oc = m["lidar"]["original_coordinates"]
    o = m["origin_cm"]
    A = (oc[0] + o[0]) / 100.0
    B = -(oc[1] + o[1]) / 100.0
    return A, B, [float(v) for v in o]


# ---- tile index ----------------------------------------------------------------
def _get(url):
    with urllib.request.urlopen(url, timeout=60) as r:
        return json.load(r)


def pick_phase(attrs):
    """Newest available (phase3 2025 > phase2 2019 > phase1 2010) -> (url, year, phase)."""
    for ph in ("phase3", "phase2", "phase1"):
        u = attrs.get(ph + "_aws_url")
        if u and u != "None":
            return u, attrs.get(ph + "_year"), ph
    return None, None, None


def query_aoi(bbox_lonlat):
    """All tiles intersecting [lon0,lat0,lon1,lat1] -> list of dicts."""
    env = {"xmin": bbox_lonlat[0], "ymin": bbox_lonlat[1],
           "xmax": bbox_lonlat[2], "ymax": bbox_lonlat[3],
           "spatialReference": {"wkid": 4326}}
    q = {"where": "1=1", "geometry": json.dumps(env), "geometryType": "esriGeometryEnvelope",
         "inSR": "4326", "spatialRel": "esriSpatialRelIntersects",
         "outFields": "tilename,phase1_year,phase1_aws_url,phase2_year,phase2_aws_url,"
                      "phase3_year,phase3_aws_url",
         "returnGeometry": "false", "f": "json"}
    feats = _get(INDEX + "/query?" + urllib.parse.urlencode(q)).get("features", [])
    out = []
    for f in feats:
        a = f["attributes"]
        url, year, phase = pick_phase(a)
        if url:
            out.append({"tile": a["tilename"], "url": url, "year": year, "phase": phase})
    out.sort(key=lambda d: d["tile"])
    return out


def query_tile(tilename):
    q = {"where": f"tilename='{tilename}'",
         "outFields": "tilename,phase1_year,phase1_aws_url,phase2_year,phase2_aws_url,"
                      "phase3_year,phase3_aws_url",
         "returnGeometry": "false", "f": "json"}
    feats = _get(INDEX + "/query?" + urllib.parse.urlencode(q)).get("features", [])
    if not feats:
        raise SystemExit(f"tile {tilename} not in the index")
    a = feats[0]["attributes"]
    url, year, phase = pick_phase(a)
    return {"tile": tilename, "url": url, "year": year, "phase": phase}


def download(url, dst, resume=True):
    if resume and os.path.exists(dst) and os.path.getsize(dst) > 0:
        return dst
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    tmp = dst + ".part"
    t = time.time()
    urllib.request.urlretrieve(url, tmp)
    os.replace(tmp, dst)
    print(f"  downloaded {os.path.getsize(dst)/1e6:.1f} MB in {time.time()-t:.1f}s")
    return dst


# ---- reprojection: tile native CRS -> scene metres -----------------------------
_TR = {}   # cache transformers by source-CRS wkt


def _transformer(src_crs):
    from pyproj import CRS, Transformer
    key = src_crs.to_wkt()
    if key not in _TR:
        horiz = src_crs.sub_crs_list[0] if src_crs.is_compound else src_crs
        _TR[key] = Transformer.from_crs(horiz, UTM16N, always_xy=True)
    return _TR[key]


def read_tile_scene(path, A, B, sample=None, seed=2019, classes=None):
    """Read a LAS/LAZ tile -> (scene Nx3 float32 [sx,sy,sz], classification u8).

    sample : keep at most this many points (random, seeded) BEFORE reprojecting, so
             the expensive pyproj step only runs on what we keep.
    classes: keep only these classification codes first (e.g. {2} for bare ground)."""
    import laspy
    las = laspy.read(path)
    x = np.asarray(las.x); y = np.asarray(las.y); z = np.asarray(las.z)
    cls = np.asarray(las.classification, np.uint8)
    if classes is not None:
        m = np.isin(cls, list(classes))
        x, y, z, cls = x[m], y[m], z[m], cls[m]
    if sample is not None and len(x) > sample:
        idx = np.random.default_rng(seed).choice(len(x), sample, replace=False)
        x, y, z, cls = x[idx], y[idx], z[idx], cls[idx]
    src = las.header.parse_crs()
    if src is None:
        from pyproj import CRS
        src = CRS.from_epsg(6473)   # KYAPED default: KY Single Zone (ftUS)
    E, N = _transformer(src).transform(x, y)
    scene = np.empty((len(E), 3), np.float32)
    scene[:, 0] = E - A                 # sx
    scene[:, 1] = z * FT_US             # sy = elevation (ftUS -> m)
    scene[:, 2] = B - N                 # sz
    return scene, cls


# ---- hypsometric colouring (KYAPED has no RGB) ---------------------------------
_RAMP = np.array([
    [38, 70, 120], [54, 110, 90], [96, 150, 70], [176, 190, 90],
    [206, 180, 120], [180, 140, 100], [200, 190, 175], [245, 245, 245],
], np.float32)


def colour_elevation(sy, lo, hi):
    t = np.clip((sy - lo) / max(hi - lo, 1e-6), 0, 1) * (len(_RAMP) - 1)
    i = np.clip(t.astype(np.int32), 0, len(_RAMP) - 2)
    f = (t - i)[:, None]
    rgb = _RAMP[i] * (1 - f) + _RAMP[i + 1] * f
    return rgb.astype(np.uint8)


# ---- write chunks in the viewer's UE-cm format ---------------------------------
_REC = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                 ('r', 'u1'), ('g', 'u1'), ('b', 'u1'), ('a', 'u1')])


def scene_to_uecm(scene, origin):
    ue = np.empty_like(scene)
    ue[:, 0] = scene[:, 0] * 100.0 + origin[0]
    ue[:, 1] = scene[:, 2] * 100.0 + origin[1]
    ue[:, 2] = scene[:, 1] * 100.0 + origin[2]
    return ue


def write_chunk(path, ue, rgb):
    n = len(ue)
    rec = np.empty(n, _REC)
    rec['x'], rec['y'], rec['z'] = ue[:, 0], ue[:, 1], ue[:, 2]
    rec['r'], rec['g'], rec['b'] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    rec['a'] = 255
    with open(path, 'wb') as f:
        f.write(struct.pack('<I', n))
        rec.tofile(f)
    return n


def render_topdown(scene, rgb, out, px=1200):
    """Quick +north-up sanity raster of the scene points coloured as given."""
    from PIL import Image
    x, z = scene[:, 0], scene[:, 2]
    x0, x1, z0, z1 = x.min(), x.max(), z.min(), z.max()
    span = max(x1 - x0, z1 - z0)
    ix = np.clip(((x - x0) / span * px).astype(np.int32), 0, px - 1)
    iz = np.clip(((z - z0) / span * px).astype(np.int32), 0, px - 1)
    lin = iz * px + ix
    img = np.zeros((px * px, 3), np.float64)
    cnt = np.bincount(lin, minlength=px * px).astype(np.float64)
    for ch in range(3):
        img[:, ch] = np.bincount(lin, weights=rgb[:, ch].astype(np.float64), minlength=px * px)
    nz = cnt > 0
    img[nz] /= cnt[nz, None]
    img = img.reshape(px, px, 3).astype(np.uint8)[::-1]   # +north up
    Image.fromarray(img, 'RGB').save(out)


def tile_path(name, info):
    ext = ".copc.laz" if info["url"].endswith(".copc.laz") else ".laz"
    return os.path.join(SCRATCH, f"{name}_{info['phase']}{ext}")


def import_one(tile, A, B, origin, target=2_000_000, render=True):
    """Download + reproject + colour + decimate one tile; return scene/rgb/cls (kept)."""
    info = tile if isinstance(tile, dict) else query_tile(tile)
    name = info["tile"]
    print(f"[{name}] phase={info['phase']} year={info['year']}")
    dst = tile_path(name, info)
    download(info["url"], dst)
    t = time.time()
    scene, cls = read_tile_scene(dst, A, B, sample=target)
    print(f"  kept {len(scene):,} pts in {time.time()-t:.1f}s  "
          f"elev[{scene[:,1].min():.0f},{scene[:,1].max():.0f}]m")
    rgb = colour_elevation(scene[:, 1], ELEV_LO, ELEV_HI)
    if render:
        os.makedirs(SCRATCH, exist_ok=True)
        render_topdown(scene, rgb, os.path.join(SCRATCH, f"{name}_topdown.png"))
        print(f"  wrote {SCRATCH}/{name}_topdown.png")
    return scene, rgb, cls


# ---- bulk download + build (point-cloud chunks + manifest) ---------------------
def download_aoi(tiles):
    """Download every AOI tile's newest .laz to SCRATCH (resumable)."""
    os.makedirs(SCRATCH, exist_ok=True)
    have = sum(1 for t in tiles if os.path.exists(tile_path(t["tile"], t)))
    print(f"downloading {len(tiles)} tiles ({have} already cached) -> {SCRATCH}")
    for i, info in enumerate(tiles, 1):
        dst = tile_path(info["tile"], info)
        if os.path.exists(dst) and os.path.getsize(dst) > 0:
            continue
        print(f"[{i}/{len(tiles)}] {info['tile']} ({info['year']}) ...", flush=True)
        try:
            download(info["url"], dst)
        except Exception as e:  # noqa: BLE001 — keep going, report at the end
            print(f"  FAILED {info['tile']}: {e}")


def build_tiles(tiles, A, B, origin, target):
    """Build a KYAPED point-cloud chunk for every tile already downloaded."""
    os.makedirs(LIDAR_DIR, exist_ok=True)
    chunks = []
    present = [t for t in tiles if os.path.exists(tile_path(t["tile"], t))]
    print(f"building {len(present)}/{len(tiles)} downloaded tiles "
          f"(target {target:,} pts/tile) ...")
    for i, info in enumerate(present, 1):
        name = info["tile"]
        try:
            scene, _ = read_tile_scene(tile_path(name, info), A, B, sample=target)
        except Exception as e:  # noqa: BLE001
            print(f"[{i}/{len(present)}] {name}: read FAILED {e}"); continue
        rgb = colour_elevation(scene[:, 1], ELEV_LO, ELEV_HI)
        ue = scene_to_uecm(scene, origin)
        n = write_chunk(os.path.join(LIDAR_DIR, f"ky_{name}.bin"), ue, rgb)
        chunks.append({"file": f"lidar/ky_{name}.bin", "count": int(n),
                       "bounds_min_cm": [float(ue[:, k].min()) for k in range(3)],
                       "bounds_max_cm": [float(ue[:, k].max()) for k in range(3)]})
        if i % 10 == 0 or i == len(present):
            print(f"[{i}/{len(present)}] {name}: {n:,} pts "
                  f"({sum(c['count'] for c in chunks):,} total)")
    return chunks


def build_ground_grid(tiles, A, B, cell=8.0, per_tile=400_000):
    """City-wide bare-ground elevation grid (scene metres) from class-2 points, so
    agents/buses can ground-snap accurately beyond the campus terrain meshes. Writes
    web/data/ground.f32 (row-major float32 [nz][nx], NaN = no data) + ground.json."""
    import math
    c = json.load(open(os.path.join(DATA, "city.json")))
    x0, z0, x1, z1 = c["bbox_scene"]                 # [xmin, zmin, xmax, zmax] scene m
    nx = int(math.ceil((x1 - x0) / cell)); nz = int(math.ceil((z1 - z0) / cell))
    grid = np.full(nx * nz, np.inf, np.float32)      # min elevation per cell (= ground)
    present = [t for t in tiles if os.path.exists(tile_path(t["tile"], t))]
    print(f"ground grid {nx}x{nz} @ {cell} m from {len(present)} tiles ...")
    for i, info in enumerate(present, 1):
        try:
            scene, _ = read_tile_scene(tile_path(info["tile"], info), A, B,
                                       sample=per_tile, classes={2})
        except Exception as e:  # noqa: BLE001
            print(f"  {info['tile']}: ground read FAILED {e}"); continue
        ix = np.clip(((scene[:, 0] - x0) / cell).astype(np.int32), 0, nx - 1)
        iz = np.clip(((scene[:, 2] - z0) / cell).astype(np.int32), 0, nz - 1)
        np.minimum.at(grid, iz * nx + ix, scene[:, 1])
        if i % 25 == 0 or i == len(present):
            print(f"  [{i}/{len(present)}] {info['tile']}")
    filled = np.isfinite(grid)
    grid[~filled] = np.nan
    grid.reshape(nz, nx).tofile(os.path.join(DATA, "ground.f32"))
    meta = {"cell": cell, "x0": x0, "z0": z0, "nx": nx, "nz": nz,
            "filled": int(filled.sum()), "total": int(grid.size),
            "elev_min": float(np.nanmin(grid)) if filled.any() else None,
            "elev_max": float(np.nanmax(grid)) if filled.any() else None,
            "note": "row-major float32 [nz][nx]; cell (ix,iz) center = "
                    "(x0+(ix+0.5)*cell, z0+(iz+0.5)*cell) scene m; value = ground elevation y; NaN=gap"}
    json.dump(meta, open(os.path.join(DATA, "ground.json"), "w"), indent=1)
    print(f"ground: {filled.sum():,}/{grid.size:,} cells filled "
          f"({100*filled.mean():.0f}%), elev[{meta['elev_min']:.0f},{meta['elev_max']:.0f}] m "
          f"-> web/data/ground.f32 + ground.json")
    return meta


def write_manifest(chunks):
    """Point the manifest's lidar layer at the KYAPED chunks (campus scans superseded)."""
    mpath = os.path.join(DATA, "manifest.json")
    m = json.load(open(mpath))
    bak = os.path.join(DATA, "manifest.campus.bak.json")
    if not os.path.exists(bak):
        json.dump(m, open(bak, "w"))      # one-time backup of the campus manifest
    m["lidar"]["offset_cm"] = [0.0, 0.0, 0.0]
    m["lidar"]["source"] = ("KyFromAbove/KYAPED 2025 Phase 3, reprojected "
                            "NAD83 KY Single Zone ftUS -> UTM16N (supersedes campus scans)")
    m["lidar"]["chunks"] = chunks
    json.dump(m, open(mpath, "w"))
    print(f"manifest updated: {len(chunks)} KYAPED chunks, "
          f"{sum(c['count'] for c in chunks):,} points (backup: manifest.campus.bak.json)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--list", action="store_true", help="list tiles over the city AOI")
    ap.add_argument("--tile", help="download + import+render a single tile (e.g. N091E300)")
    ap.add_argument("--download-aoi", action="store_true", help="download every AOI tile (resumable)")
    ap.add_argument("--build", action="store_true", help="build chunks+manifest from downloaded tiles")
    ap.add_argument("--heightmap", action="store_true", help="build the city ground-elevation grid")
    ap.add_argument("--limit", type=int, default=None, help="only the first N AOI tiles (prototyping)")
    ap.add_argument("--target", type=int, default=150_000, help="kept points per tile")
    args = ap.parse_args()

    A, B, origin = georef()
    if args.tile:
        import_one(args.tile, A, B, origin, target=max(args.target, 2_000_000))
        return

    c = json.load(open(os.path.join(DATA, "city.json")))
    tiles = query_aoi(c["bbox_lonlat"])
    if args.limit:
        tiles = tiles[:args.limit]

    if args.list:
        yrs = {}
        for t in tiles:
            yrs[t["year"]] = yrs.get(t["year"], 0) + 1
        print(f"{len(tiles)} tiles over the city AOI; by year: {dict(sorted(yrs.items()))}")
        for t in tiles[:12]:
            print(f"  {t['tile']}  {t['phase']} {t['year']}")
        return
    if args.download_aoi:
        download_aoi(tiles)
    if args.build:
        chunks = build_tiles(tiles, A, B, origin, args.target)
        if chunks:
            write_manifest(chunks)
    if args.heightmap:
        build_ground_grid(tiles, A, B)
    if not (args.download_aoi or args.build or args.heightmap):
        ap.print_help()


if __name__ == "__main__":
    main()
