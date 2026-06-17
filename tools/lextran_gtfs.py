#!/usr/bin/env python
"""Bake the Lextran (Lexington, KY) static GTFS feed into web/data/transit.json.

This is the *static* half of the transit integration: the route line geometry and
the bus stops, which never move and so are baked offline (the *live* half — moving
buses, predicted arrivals, and service alerts — is proxied at runtime by
`tools/serve.py`). We fetch Lextran's published GTFS zip, keep only what overlaps
the campus viewport, project every shape point and stop from lon/lat through the
SAME UTM-16N -> scene map the road network uses (`tools/transit_common.Projector`,
i.e. `tools/osm_roads.py`'s transform), drape them onto the terrain heightmap, clip
shapes to the terrain footprint, and emit scene-metre polylines + stop points — the
contract `web/transit.js` reads.

Usage:
    python -m tools.lextran_gtfs                 # fetch the live GTFS zip
    python -m tools.lextran_gtfs --gtfs-zip PATH # use a local zip (offline / CI)
    python -m tools.lextran_gtfs --pad 120       # widen the keep-margin (m)

(Run from the repo root as a module so `tools/inspect.py` doesn't shadow the stdlib
`inspect`, exactly like the other tools.)

GTFS (c) Lextran (Transit Authority of Lexington); see their developer terms.
"""
import argparse
import csv
import io
import json
import math
import os
import urllib.request
import zipfile

from tools.extract_roads import DATA, load_mesh, build_heightmap
from tools.transit_common import Projector

GTFS_ZIP_URL = "http://mystop.lextran.com/InfoPoint/gtfs-zip.ashx"
MPP = 50.0          # heightmap metres-per-pixel (matches osm_roads)
DEDUP_M = 1.0       # drop shape points closer than this (m) after projection
UA = {"User-Agent": "uky-campus-viewer/1.0 (+transit baker)"}


# --- terrain footprint, identical to tools/osm_roads.terrain_extent ---------
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
        gx0 = min(gx0, pts[:, 0].min())
        gx1 = max(gx1, pts[:, 0].max())
        gz0 = min(gz0, pts[:, 2].min())
        gz1 = max(gz1, pts[:, 2].max())
    return gx0, gz0, gx1, gz1


def fetch_zip(url):
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=120) as r:
        return r.read()


def read_table(z, name):
    """Yield each row of a GTFS .txt as a dict (utf-8-sig handles the BOM)."""
    if name not in z.namelist():
        return
    with z.open(name) as f:
        for row in csv.DictReader(io.TextIOWrapper(f, encoding="utf-8-sig")):
            yield row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gtfs-zip", help="use this local GTFS zip instead of fetching")
    ap.add_argument("--pad", type=float, default=80.0,
                    help="keep-margin around the terrain footprint, metres")
    ap.add_argument("--campus-only", action="store_true",
                    help="clip to the campus tiles (legacy) instead of the full network")
    ap.add_argument("--out", default=os.path.join(DATA, "transit.json"))
    args = ap.parse_args()
    full = not args.campus_only

    manifest = json.load(open(os.path.join(DATA, "manifest.json")))
    proj = Projector(manifest)
    A, B = proj.A, proj.B

    print("[1/6] terrain extent + elevation heightmap ...")
    gx0, gz0, gx1, gz1 = terrain_extent(manifest)
    w = int(math.ceil((gx1 - gx0) / MPP))
    h = int(math.ceil((gz1 - gz0) / MPP))
    elev = build_heightmap(manifest, (gx0, gz0, MPP, w, h))
    pad = args.pad
    sxmin, sxmax = gx0 / 100 - pad, gx1 / 100 + pad
    szmin, szmax = -gz1 / 100 - pad, -gz0 / 100 + pad

    def inb(x, z):
        return sxmin <= x <= sxmax and szmin <= z <= szmax

    # full-network draping: inside the campus tiles -> real terrain; everywhere else
    # -> the flat city ground plane (web/data/city.json from tools.osm_city), so the
    # whole bus network rides on the same surface the city streets do.
    city_y = 281.0
    city_path = os.path.join(DATA, "city.json")
    if os.path.exists(city_path):
        try:
            city_y = float(json.load(open(city_path)).get("groundY", city_y))
        except Exception:  # noqa: BLE001
            pass
    if full and not os.path.exists(city_path):
        print(f"       (no city.json yet — off-campus draped flat at {city_y} m; "
              f"run tools.osm_city for a matched ground plane)")

    def drape(sx, sz):
        if inb(sx, sz):
            return float(elev(sx * 100.0, -sz * 100.0)) / 100.0
        return city_y

    # campus lon/lat bbox (for the manifest header / proxy reuse)
    import pyproj
    to_ll = pyproj.Transformer.from_crs(32616, 4326, always_xy=True)
    corners = [(A + sx, B - sz) for sx in (sxmin, sxmax) for sz in (szmin, szmax)]
    lls = [to_ll.transform(e, n) for e, n in corners]
    lon_lo = min(p[0] for p in lls); lon_hi = max(p[0] for p in lls)
    lat_lo = min(p[1] for p in lls); lat_hi = max(p[1] for p in lls)

    print("[2/6] loading GTFS ...")
    if args.gtfs_zip:
        raw = open(args.gtfs_zip, "rb").read()
    else:
        print(f"       fetching {GTFS_ZIP_URL}")
        raw = fetch_zip(GTFS_ZIP_URL)
    z = zipfile.ZipFile(io.BytesIO(raw))

    # routes ------------------------------------------------------------------
    routes = {}
    for r in read_table(z, "routes.txt"):
        rid = r["route_id"]
        routes[rid] = {
            "id": rid,
            "shortName": (r.get("route_short_name") or "").strip(),
            "longName": (r.get("route_long_name") or "").strip(),
            "color": (r.get("route_color") or "").strip() or "3b82c4",
            "textColor": (r.get("route_text_color") or "").strip() or "FFFFFF",
            "sortOrder": int(r["route_sort_order"]) if (r.get("route_sort_order") or "").strip().isdigit() else 9999,
            "shapes": [],
        }

    # trips: route -> shape_ids, and trip_id -> route_id -----------------------
    route_shapes = {}
    trip_route = {}
    for t in read_table(z, "trips.txt"):
        rid, sid, tid = t.get("route_id"), t.get("shape_id"), t.get("trip_id")
        if tid:
            trip_route[tid] = rid
        if rid and sid:
            route_shapes.setdefault(rid, set()).add(sid)

    # shapes: shape_id -> ordered [(lon, lat)] --------------------------------
    print("[3/6] reading shapes ...")
    shape_pts = {}
    for s in read_table(z, "shapes.txt"):
        sid = s["shape_id"]
        try:
            seq = int(s["shape_pt_sequence"])
            lat = float(s["shape_pt_lat"]); lon = float(s["shape_pt_lon"])
        except (KeyError, ValueError):
            continue
        shape_pts.setdefault(sid, []).append((seq, lon, lat))
    for sid in shape_pts:
        shape_pts[sid].sort(key=lambda q: q[0])

    def project_clip(lonlat):
        """lon/lat polyline -> list of scene polylines [[x,y,z], ...].

        Full-network (default): one draped polyline spanning the whole city.
        Campus-only (legacy): split into contiguous in-bounds runs so nothing
        floats past the campus tiles."""
        xz = [proj(lon, lat) for lon, lat in lonlat]
        if full:
            runs = [xz]
        else:
            runs, run = [], []
            for sx, sz in xz:
                if inb(sx, sz):
                    run.append((sx, sz))
                elif len(run) >= 2:
                    runs.append(run); run = []
                else:
                    run = []
            if len(run) >= 2:
                runs.append(run)
        out = []
        for run in runs:
            ded = [run[0]]
            for p in run[1:]:
                if math.hypot(p[0] - ded[-1][0], p[1] - ded[-1][1]) > DEDUP_M:
                    ded.append(p)
            if len(ded) >= 2:
                out.append([[round(sx, 2), round(drape(sx, sz), 2), round(sz, 2)] for sx, sz in ded])
        return out

    # attach clipped shapes to their routes (dedup near-identical variants) ----
    print("[4/6] projecting + draping route shapes ...")
    n_polylines = 0
    for rid, sids in route_shapes.items():
        route = routes.get(rid)
        if not route:
            continue
        seen = set()
        for sid in sids:
            pts = shape_pts.get(sid)
            if not pts:
                continue
            for poly in project_clip([(lon, lat) for _, lon, lat in pts]):
                # coarse signature so inbound/outbound duplicates collapse
                sig = (len(poly), round(poly[0][0]), round(poly[0][2]),
                       round(poly[-1][0]), round(poly[-1][2]))
                if sig in seen:
                    continue
                seen.add(sig)
                route["shapes"].append(poly)
                n_polylines += 1

    # stops: keep those inside the viewport, project + drape ------------------
    print("[5/6] projecting stops ...")
    stops = {}
    for s in read_table(z, "stops.txt"):
        if (s.get("location_type") or "0").strip() not in ("", "0"):
            continue  # skip stations / entrances; keep boardable stops
        try:
            lat = float(s["stop_lat"]); lon = float(s["stop_lon"])
        except (KeyError, ValueError):
            continue
        sx, sz = proj(lon, lat)
        if not full and not inb(sx, sz):
            continue
        sid = s["stop_id"]
        stops[sid] = {
            "id": sid,
            "code": (s.get("stop_code") or "").strip(),
            "name": (s.get("stop_name") or "").strip(),
            "lat": round(lat, 6), "lon": round(lon, 6),
            "pos": [round(sx, 2), round(drape(sx, sz), 2), round(sz, 2)],
            "routes": [],
        }

    # routes-serving-each-stop via stop_times (only for kept stops) -----------
    if stops:
        stop_routes = {sid: set() for sid in stops}
        for st in read_table(z, "stop_times.txt"):
            sid = st.get("stop_id")
            if sid in stop_routes:
                rid = trip_route.get(st.get("trip_id"))
                if rid:
                    stop_routes[sid].add(rid)
        for sid, rset in stop_routes.items():
            stops[sid]["routes"] = sorted(
                rset, key=lambda r: routes.get(r, {}).get("sortOrder", 9999))

    # keep only routes that actually have geometry in-view, sorted -------------
    out_routes = [r for r in routes.values() if r["shapes"]]
    out_routes.sort(key=lambda r: r["sortOrder"])

    out = {
        "note": "Lextran static GTFS (routes + stops), projected UTM16N->scene metres; "
                "points are [x,y,z] scene metres, draped on campus terrain where it "
                "exists and on the flat city ground plane (city.json) elsewhere. Live "
                "buses/arrivals/alerts come from tools/serve.py at runtime. (c) Lextran.",
        "source": "lextran-gtfs-static",
        "scope": "campus" if args.campus_only else "full-network",
        "georef": {"A": A, "B": B, "epsg": 32616, "utmZone": "16N"},
        "cityGroundY": city_y,
        "bbox_lonlat": [round(lon_lo, 6), round(lat_lo, 6), round(lon_hi, 6), round(lat_hi, 6)],
        "routes": out_routes,
        "stops": list(stops.values()),
    }
    with open(args.out, "w") as f:
        json.dump(out, f)
    scope = "campus only" if args.campus_only else "full network"
    print(f"[6/6] wrote {args.out}: {len(out_routes)} routes "
          f"({n_polylines} polylines), {len(stops)} stops ({scope})")


if __name__ == "__main__":
    main()
