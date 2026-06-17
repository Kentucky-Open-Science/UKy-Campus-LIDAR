#!/usr/bin/env python
"""Build a city-wide OSM street context for the whole Lextran service area.

The campus LiDAR/terrain only covers ~2x3 km, but the Lextran bus network spans
~18x16 km of Lexington. To "complete the digital twin" so every route has streets
and ground under it, this fetches OpenStreetMap highways for the FULL transit bbox,
projects them through the same UTM-16N -> scene map the campus uses
(`tools/transit_common.Projector`), and lays them FLAT on a single city ground
plane just below the detailed campus terrain (we have no city-wide DEM, so a flat
plane is the honest choice; the campus stays a raised island of real elevation).

Output `web/data/city.json` (consumed by web/city.js):
    { "groundY": <scene m>, "bbox_scene": [xmin,zmin,xmax,zmax],
      "bbox_lonlat": [w,s,e,n],
      "roads": [ { "pts": [[x,z], ...], "class": "primary", "name": "..." }, ... ] }

Points are 2-D [x,z] (flat at groundY) and rounded to the metre — city context
doesn't need cm. Run BEFORE tools/lextran_gtfs (the transit baker reads groundY
from here so off-campus stops/routes sit on the same plane):

    python -m tools.osm_city                         # fetch OSM + GTFS live
    python -m tools.osm_city --osm-cache osm.json --gtfs-zip tools/_transit_samples/google_transit.zip

GTFS (c) Lextran; map data (c) OpenStreetMap contributors.
"""
import argparse
import csv
import io
import json
import math
import os
import urllib.request
import zipfile

import numpy as np

from tools.extract_roads import DATA, load_mesh, build_heightmap
from tools.transit_common import Projector

OVERPASS = ["https://overpass-api.de/api/interpreter",
            "https://overpass.kumi.systems/api/interpreter"]
GTFS_ZIP_URL = "http://mystop.lextran.com/InfoPoint/gtfs-zip.ashx"
MPP = 50.0
# OSM highway classes worth drawing as city context (skip footways/cycleways/service)
CLASS_W = {
    "motorway": 3.0, "trunk": 2.6, "primary": 2.2, "secondary": 1.8,
    "tertiary": 1.4, "residential": 1.0, "unclassified": 1.0, "living_street": 0.9,
    "motorway_link": 1.4, "trunk_link": 1.4, "primary_link": 1.2,
    "secondary_link": 1.1, "tertiary_link": 1.0,
}
RESAMPLE_M = 12.0
UA = {"User-Agent": "uky-campus-viewer/1.0 (+osm city context)"}


def terrain_extent(manifest):
    gx0 = gz0 = 1e18
    gx1 = gz1 = -1e18
    for t in manifest["terrain"]["tiles"]:
        mp = os.path.join(DATA, t["mesh"])
        if not os.path.exists(mp):
            continue
        pts, _ = load_mesh(mp)
        if len(pts) < 3:
            continue
        gx0 = min(gx0, pts[:, 0].min()); gx1 = max(gx1, pts[:, 0].max())
        gz0 = min(gz0, pts[:, 2].min()); gz1 = max(gz1, pts[:, 2].max())
    return gx0, gz0, gx1, gz1


def gtfs_lonlat_bbox(zip_bytes, margin_deg=0.01):
    z = zipfile.ZipFile(io.BytesIO(zip_bytes))
    lon0 = lat0 = 1e9
    lon1 = lat1 = -1e9
    for name, la, lo in (("stops.txt", "stop_lat", "stop_lon"),
                         ("shapes.txt", "shape_pt_lat", "shape_pt_lon")):
        with z.open(name) as f:
            for r in csv.DictReader(io.TextIOWrapper(f, encoding="utf-8-sig")):
                try:
                    lat = float(r[la]); lon = float(r[lo])
                except (KeyError, ValueError):
                    continue
                lon0 = min(lon0, lon); lon1 = max(lon1, lon)
                lat0 = min(lat0, lat); lat1 = max(lat1, lat)
    return (lon0 - margin_deg, lat0 - margin_deg, lon1 + margin_deg, lat1 + margin_deg)


def fetch_osm(s, w, n, e):
    types = "|".join(sorted(CLASS_W))
    q = (f"[out:json][timeout:180];"
         f'way["highway"~"^({types})$"]({s},{w},{n},{e});(._;>;);out body;')
    last = None
    for url in OVERPASS:
        try:
            req = urllib.request.Request(url, data=q.encode("utf-8"), headers=UA)
            with urllib.request.urlopen(req, timeout=240) as r:
                return json.load(r)
        except Exception as ex:  # noqa: BLE001 — try the next mirror
            last = ex
            print(f"   overpass {url} failed ({ex}); trying next mirror")
    raise last


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--osm-cache", help="use/write this OSM json (skip Overpass if it exists)")
    ap.add_argument("--gtfs-zip", help="local GTFS zip for the bbox (else fetch live)")
    ap.add_argument("--out", default=os.path.join(DATA, "city.json"))
    args = ap.parse_args()

    manifest = json.load(open(os.path.join(DATA, "manifest.json")))
    proj = Projector(manifest)
    A, B = proj.A, proj.B

    print("[1/5] campus terrain extent + reference elevation ...")
    gx0, gz0, gx1, gz1 = terrain_extent(manifest)
    w = int(math.ceil((gx1 - gx0) / MPP)); h = int(math.ceil((gz1 - gz0) / MPP))
    elev = build_heightmap(manifest, (gx0, gz0, MPP, w, h))
    # sample the campus heightmap to find its MIN elevation; the flat city plane
    # sits just under that so the detailed terrain never sinks below the city.
    sxmin, sxmax = gx0 / 100, gx1 / 100
    szmin, szmax = -gz1 / 100, -gz0 / 100
    ys = []
    for i in range(60):
        for j in range(60):
            sx = sxmin + (sxmax - sxmin) * i / 59
            sz = szmin + (szmax - szmin) * j / 59
            ys.append(float(elev(sx * 100.0, -sz * 100.0)) / 100.0)
    # sit just below the TRUE campus minimum so the detailed terrain never pokes
    # below the flat city plane anywhere along the tile edges.
    city_y = round(min(ys) - 2.0, 2)
    print(f"       campus elevation ~[{min(ys):.0f},{max(ys):.0f}] m  ->  city plane at {city_y} m")

    print("[2/5] transit bbox ...")
    if args.gtfs_zip:
        zip_bytes = open(args.gtfs_zip, "rb").read()
    else:
        print(f"       fetching {GTFS_ZIP_URL}")
        zip_bytes = urllib.request.urlopen(
            urllib.request.Request(GTFS_ZIP_URL, headers=UA), timeout=120).read()
    wll, sll, ell, nll = gtfs_lonlat_bbox(zip_bytes)
    print(f"       bbox lon/lat ({sll:.4f},{wll:.4f},{nll:.4f},{ell:.4f})")

    print("[3/5] fetching OSM highways (this can take ~30-90 s) ...")
    if args.osm_cache and os.path.exists(args.osm_cache):
        osm = json.load(open(args.osm_cache))
        print(f"       using cache {args.osm_cache}")
    else:
        osm = fetch_osm(sll, wll, nll, ell)
        if args.osm_cache:
            json.dump(osm, open(args.osm_cache, "w"))

    els = osm["elements"]
    nodes = {x["id"]: (x["lon"], x["lat"]) for x in els if x["type"] == "node"}
    ways = [x for x in els if x["type"] == "way" and x.get("tags", {}).get("highway") in CLASS_W]

    def resample(coords, step):
        if len(coords) < 2:
            return coords
        out = [coords[0]]; carry = step
        for i in range(len(coords) - 1):
            ax, az = coords[i]; bx, bz = coords[i + 1]
            seg = math.hypot(bx - ax, bz - az)
            if seg < 1e-6:
                continue
            d = carry
            while d < seg:
                tt = d / seg
                out.append((ax + (bx - ax) * tt, az + (bz - az) * tt)); d += step
            carry = d - seg
        out.append(coords[-1])
        return out

    print("[4/5] projecting %d ways -> scene (flat) ..." % len(ways))
    xmin = zmin = 1e18; xmax = zmax = -1e18
    roads_out = []
    for wy in ways:
        hw = wy["tags"]["highway"]
        pts = []
        for nid in wy["nodes"]:
            if nid not in nodes:
                continue
            lon, lat = nodes[nid]
            sx, sz = proj(lon, lat)
            pts.append((sx, sz))
        if len(pts) < 2:
            continue
        pts = resample(pts, RESAMPLE_M)
        ipts = [[int(round(x)), int(round(z))] for x, z in pts]
        # drop consecutive duplicates created by integer rounding
        ded = [ipts[0]]
        for p in ipts[1:]:
            if p != ded[-1]:
                ded.append(p)
        if len(ded) < 2:
            continue
        for x, z in ded:
            xmin = min(xmin, x); xmax = max(xmax, x)
            zmin = min(zmin, z); zmax = max(zmax, z)
        rec = {"pts": ded, "class": hw}
        name = wy["tags"].get("name")
        if name:
            rec["name"] = name
        roads_out.append(rec)

    out = {
        "note": "OSM city street context for the full Lextran service area, projected "
                "UTM16N->scene metres and laid flat on the city ground plane; points are "
                "[x,z] scene metres at y=groundY. Map data (c) OpenStreetMap contributors.",
        "source": "openstreetmap",
        "groundY": city_y,
        "bbox_scene": [xmin, zmin, xmax, zmax],
        "bbox_lonlat": [round(wll, 6), round(sll, 6), round(ell, 6), round(nll, 6)],
        "roads": roads_out,
    }
    with open(args.out, "w") as f:
        json.dump(out, f, separators=(",", ":"))
    seg = sum(len(r["pts"]) - 1 for r in roads_out)
    print(f"[5/5] wrote {args.out}: {len(roads_out)} ways ({seg} segments), "
          f"scene X[{xmin},{xmax}] Z[{zmin},{zmax}], groundY {city_y}")


if __name__ == "__main__":
    main()
