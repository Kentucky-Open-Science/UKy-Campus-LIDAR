#!/usr/bin/env python
"""Bake the Lexington traffic-camera -> twin-intersection mapping into
web/data/cameras.json.

This is the *static* half of the traffic-camera integration: the camera positions
and the twin intersection each one watches, which never change and so are baked
offline. The *live* half — the tokenized HLS stream URLs, which the city re-signs
every ~15 minutes — is proxied at runtime by `tools/twin_server.py` (/api/cameras/*),
exactly like the transit layer splits baked routes/stops (transit.json) from live
buses (/api/transit/*).

For each camera we project its published lon/lat through the SAME UTM-16N -> scene
map the road network and transit layers use (`tools/transit_common.Projector`) and
snap it to the nearest intersection in `web/data/signals.json`. If that intersection
is within `--snap-meters` (default 75 m) the camera "sits on" a real twin junction
(its marker is placed at the junction centre and labelled with both names); the ~9
outliers beyond that — mostly highway interchanges whose nearest OSM junction node is
genuinely far — keep their own projected GPS position and are flagged unmatched. Either
way every camera gets a scene position, so the viewer can always place a marker.

The token-free `still` thumbnail URL is carried through as a no-proxy fallback (it is
a live snapshot that needs no signing), so the viewer shows *something* real even when
the twin server isn't running the live proxy.

Usage:
    python -m tools.lex_cameras                       # read the cached cam_data.json
    python -m tools.lex_cameras --scrape              # re-scrape the city map first
    python -m tools.lex_cameras --cam-data PATH       # use a specific cam_data.json
    python -m tools.lex_cameras --snap-meters 60      # tighten the junction match

(Run from the repo root as a module so `tools/inspect.py` doesn't shadow the stdlib
`inspect`, exactly like the other tools.)

Camera feeds (c) City of Lexington, KY public traffic map
(https://trafficvid.lexingtonky.gov/publicmap/) — for personal and educational use;
respect the city's terms of service.
"""
import argparse
import json
import math
import os
import urllib.request

from tools.transit_common import DATA, Projector

# Default location of the TrafficStream camera cache (the sibling repo that scrapes +
# runs YOLO; the twin only needs its camera list). Override with --cam-data.
DEFAULT_CAM_DATA = os.environ.get(
    "TRAFFICSTREAM_CAM_DATA", r"Y:\WORK\TrafficStream\data\cam_data.json"
)
CITY_MAP_URL = "https://trafficvid.lexingtonky.gov/publicmap/"
UA = {"User-Agent": "uky-campus-viewer/1.0 (+camera baker)"}


def scrape_cameras(url=CITY_MAP_URL):
    """Pull the live camera list out of the city's public map HTML.

    The page embeds a `camMarker = [ {...}, ... ]` JS literal; the city uses single
    quotes, so (as TrafficStream's check_cam_online does) we swap quotes and json-load
    the array. Returns the raw list of camera dicts.
    """
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=30) as r:
        html = r.read().decode("utf-8", "replace")
    html = html.replace("'", '"')
    sub = html[html.find("camMarker"):]
    arr = sub[sub.find("["):sub.find("]")] + "]"
    return json.loads(arr)


def load_cam_data(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def load_intersections(data_dir=DATA):
    """[(id, x, y, z)] from web/data/signals.json (scene metres, already projected)."""
    with open(os.path.join(data_dir, "signals.json"), encoding="utf-8") as f:
        sig = json.load(f)
    out = []
    for s in sig.get("intersections", []):
        c = s.get("center")
        if c and len(c) >= 3:
            out.append((s["id"], c[0], c[1], c[2]))
    return out


def nearest_intersection(x, z, ints):
    """Nearest (id, x, y, z) intersection to (x,z) and the planar distance to it."""
    best, bd = None, float("inf")
    for rec in ints:
        dx, dz = rec[1] - x, rec[3] - z
        d = dx * dx + dz * dz
        if d < bd:
            bd, best = d, rec
    return best, math.sqrt(bd)


def bake(cams, ints, snap_meters):
    out, matched = [], 0
    for c in cams:
        try:
            lon, lat = float(c["lng"]), float(c["lat"])
        except (KeyError, ValueError, TypeError):
            continue
        x, z = PROJ(lon, lat)
        inter, dist = nearest_intersection(x, z, ints) if ints else (None, float("inf"))
        is_match = inter is not None and dist <= snap_meters
        if is_match:
            matched += 1
            pos = [round(inter[1], 2), round(inter[2], 2), round(inter[3], 2)]
        else:
            # no twin junction nearby (highway ramp etc.) — keep the camera's own spot,
            # borrow the nearest intersection's elevation as a ground hint (the viewer
            # re-drapes per-frame anyway).
            y = inter[2] if inter is not None else 280.0
            pos = [round(x, 2), round(y, 2), round(z, 2)]
        out.append({
            "id": c.get("camera"),
            "name": c.get("description") or c.get("camera"),
            "lat": round(lat, 6), "lon": round(lon, 6),
            "pos": pos,
            "camPos": [round(x, 2), round(z, 2)],
            "intersection": inter[0] if inter is not None else None,
            "snapDist": round(dist, 1) if math.isfinite(dist) else None,
            "matched": is_match,
            "still": c.get("still"),
        })
    return out, matched


def main():
    global PROJ
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cam-data", default=DEFAULT_CAM_DATA,
                    help="path to TrafficStream's cam_data.json (default: %(default)s)")
    ap.add_argument("--scrape", action="store_true",
                    help="re-scrape the live city map instead of reading --cam-data")
    ap.add_argument("--snap-meters", type=float, default=75.0,
                    help="max distance to call a camera 'on' a twin intersection (default 75)")
    ap.add_argument("--out", default=os.path.join(DATA, "cameras.json"))
    args = ap.parse_args()

    if args.scrape:
        print(f"scraping {CITY_MAP_URL} ...")
        cams = scrape_cameras()
    else:
        print(f"reading {args.cam_data} ...")
        cams = load_cam_data(args.cam_data)
    print(f"  {len(cams)} cameras")

    PROJ = Projector()
    ints = load_intersections()
    print(f"  {len(ints)} twin intersections (signals.json)")

    cameras, matched = bake(cams, ints, args.snap_meters)

    payload = {
        "version": 1,
        "note": ("Camera -> intersection mapping for the Lexington traffic-camera "
                 "layer. Static geometry only; the live tokenized HLS URLs are served "
                 "by tools/twin_server.py (/api/cameras/*). Regenerate with "
                 "python -m tools.lex_cameras."),
        "source": "City of Lexington public traffic map (trafficvid.lexingtonky.gov)",
        "snapMeters": args.snap_meters,
        "count": len(cameras),
        "matched": matched,
        "cameras": cameras,
    }
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(payload, f, separators=(",", ":"))

    # --- match report (mirrors the spec analysis) ---
    far = sorted((c for c in cameras if not c["matched"]),
                 key=lambda c: -(c["snapDist"] or 0))
    dists = sorted(c["snapDist"] for c in cameras if c["snapDist"] is not None)
    print(f"\nwrote {args.out}")
    print(f"  matched {matched}/{len(cameras)} cameras within {args.snap_meters:.0f} m of a twin intersection")
    if dists:
        med = dists[len(dists) // 2]
        p90 = dists[min(len(dists) - 1, int(len(dists) * 0.9))]
        print(f"  snap distance  median {med:.1f} m   p90 {p90:.1f} m   max {dists[-1]:.1f} m")
    if far:
        print(f"  {len(far)} unmatched (placed at their own GPS position):")
        for c in far:
            print(f"    {c['id']:<13} {c['name']:<30} {c['snapDist']:>7.1f} m")


if __name__ == "__main__":
    main()
