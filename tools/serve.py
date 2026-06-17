#!/usr/bin/env python
"""Static file server for web/ + a live Lextran GTFS-Realtime proxy.

This is a drop-in replacement for `python -m http.server` that ALSO exposes the
live half of the transit layer the viewer can't fetch itself (the upstream feed is
plain HTTP with no CORS headers, so a browser on http://localhost can't read it
directly). The proxy fetches Lextran's GTFS-Realtime "debug" feeds (which serialise
as JSON — no protobuf runtime needed), projects every vehicle's lon/lat into the
viewer's scene metres with the SAME georef the road network uses
(`tools/transit_common.Projector`), joins each bus to its route colour/name from the
baked `web/data/transit.json`, and re-serves it as compact same-origin JSON with a
short cache so we never hammer the agency.

Endpoints (all JSON, CORS-open):
    GET /api/transit/vehicles   live bus positions, projected to scene [x,_,z]
    GET /api/transit/trips      predicted arrivals, indexed by stop and by trip
    GET /api/transit/alerts     service alerts (decoded cause/effect)
    GET /api/transit/meta       proxy status (mode, cache ages, counts, georef)
    everything else             served as a static file from web/

Run:
    python -m tools.serve                 # live feed, http://localhost:8000/
    python -m tools.serve --port 8000
    python -m tools.serve --mock          # replay tools/_transit_samples (offline)
    python -m tools.serve --selftest      # start, hit every endpoint, exit 0/1

The viewer (web/transit.js) auto-detects the proxy and falls back to static-only
(routes + stops, no live buses) when it isn't running, so plain http.server still
works — you just don't get moving buses.

Transit data (c) Lextran (Transit Authority of Lexington).
"""
import argparse
import functools
import http.server
import json
import math
import os
import threading
import time
import urllib.request

from tools.transit_common import DATA, Projector

_HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.abspath(os.path.join(_HERE, "..", "web"))
SAMPLES = os.path.join(_HERE, "_transit_samples")

FEED_BASE = "http://mystop.lextran.com/InfoPoint/GTFS-Realtime.ashx"
FEED_QUERY = {
    "vehicles": "?&Type=VehiclePosition&serverid=0&debug=true",
    "trips": "?&Type=TripUpdate&debug=true",
    "alerts": "?&Type=Alert&debug=true",
}
FIXTURE = {
    "vehicles": "VehiclePosition.json",
    "trips": "TripUpdate.json",
    "alerts": "Alert.json",
}
UA = {"User-Agent": "uky-campus-viewer/1.0 (+transit proxy)"}

# GTFS-Realtime enums -> readable strings
VEHICLE_STATUS = {0: "incoming_at", 1: "stopped_at", 2: "in_transit_to"}
OCCUPANCY = {0: "empty", 1: "many_seats", 2: "few_seats", 3: "standing_room",
             4: "crushed_standing", 5: "full", 6: "not_accepting"}
ALERT_CAUSE = {1: "unknown", 2: "other", 3: "technical_problem", 4: "strike",
               5: "demonstration", 6: "accident", 7: "holiday", 8: "weather",
               9: "maintenance", 10: "construction", 11: "police_activity",
               12: "medical_emergency"}
ALERT_EFFECT = {1: "no_service", 2: "reduced_service", 3: "significant_delays",
                4: "detour", 5: "additional_service", 6: "modified_service",
                7: "other", 8: "unknown", 9: "stop_moved", 10: "no_effect",
                11: "accessibility_issue"}


class TransitProxy:
    """Fetches, caches, and projects the three realtime feeds."""

    def __init__(self, projector, route_map, mock=False, cache_seconds=5.0):
        self.proj = projector
        self.routes = route_map            # routeId -> {shortName,color,longName}
        self.mock = mock
        self.cache_seconds = cache_seconds
        self._cache = {}                   # kind -> (fetched_at, payload)
        self._lock = threading.Lock()
        self._t0 = time.time()

    # ---- upstream fetch (or fixture replay) ----
    def _raw(self, kind):
        if self.mock:
            path = os.path.join(SAMPLES, FIXTURE[kind])
            with open(path, "rb") as f:
                return json.loads(f.read())
        req = urllib.request.Request(FEED_BASE + FEED_QUERY[kind], headers=UA)
        with urllib.request.urlopen(req, timeout=20) as r:
            return json.loads(r.read())

    def get(self, kind):
        """Return the projected payload for a feed kind, honouring the cache."""
        now = time.time()
        with self._lock:
            hit = self._cache.get(kind)
            if hit and now - hit[0] < self.cache_seconds:
                return hit[1]
        try:
            raw = self._raw(kind)
            payload = getattr(self, "_build_" + kind)(raw)
            payload["mode"] = "mock" if self.mock else "live"
        except Exception as e:  # noqa: BLE001 — never let the proxy 500 the viewer
            with self._lock:
                stale = self._cache.get(kind)
            payload = (stale[1] if stale else {kind: [], "count": 0}).copy()
            payload["error"] = str(e)
            payload["stale"] = bool(stale)
            return payload
        with self._lock:
            self._cache[kind] = (now, payload)
        return payload

    # ---- per-feed projection / decoding ----
    def _route_of(self, route_id):
        r = self.routes.get(str(route_id)) if route_id is not None else None
        if not r:
            return None
        return {"id": r["id"], "shortName": r["shortName"], "color": r["color"],
                "name": r["longName"]}

    def _mock_nudge(self, lat, lon, bearing, speed):
        """In mock mode, crawl each bus along its bearing so motion is visible.

        Live mode never calls this (real GTFS positions are used as-is). Parked
        campus buses report ~0.45 m/s, so we floor the crawl speed to keep the
        offline demo/test lively and deterministic."""
        if not self.mock:
            return lat, lon
        eff = max(abs(speed or 0.0), 4.0)                # floor so parked buses drift
        d = (eff * (time.time() - self._t0)) % 150.0     # 0..150 m, then wrap
        th = math.radians(bearing or 0.0)
        lat2 = lat + (d * math.cos(th)) / 111320.0
        lon2 = lon + (d * math.sin(th)) / (111320.0 * max(0.2, math.cos(math.radians(lat))))
        return lat2, lon2

    def _build_vehicles(self, raw):
        out, header = [], raw.get("Header") or {}
        for e in raw.get("Entities") or []:
            v = e.get("Vehicle")
            if not v:
                continue
            pos = v.get("Position") or {}
            lat, lon = pos.get("Latitude"), pos.get("Longitude")
            if lat is None or lon is None:
                continue
            bearing = pos.get("Bearing")
            speed = pos.get("Speed")
            lat, lon = self._mock_nudge(lat, lon, bearing, speed)
            x, z = self.proj(lon, lat)
            trip = v.get("Trip") or {}
            veh = v.get("Vehicle") or {}
            rid = trip.get("RouteId")
            out.append({
                "id": str(veh.get("Id") or e.get("Id") or ""),
                "label": veh.get("Label"),
                "routeId": str(rid) if rid is not None else None,
                "route": self._route_of(rid),
                "tripId": trip.get("TripId"),
                "stopId": str(v.get("StopId")) if v.get("StopId") is not None else None,
                "seq": v.get("CurrentStopSequence"),
                "status": VEHICLE_STATUS.get(v.get("CurrentStatus")),
                "occupancy": OCCUPANCY.get(v.get("occupancy_status")),
                "lat": round(lat, 6), "lon": round(lon, 6),
                "bearing": bearing, "speed": speed,
                "x": round(x, 2), "z": round(z, 2),
                "vts": v.get("Timestamp"),
            })
        return {"ts": header.get("Timestamp") or int(time.time()),
                "count": len(out), "vehicles": out}

    def _build_trips(self, raw):
        by_stop, by_trip, header = {}, {}, raw.get("Header") or {}
        for e in raw.get("Entities") or []:
            tu = e.get("TripUpdate")
            if not tu:
                continue
            trip = tu.get("Trip") or {}
            rid = trip.get("RouteId")
            tid = trip.get("TripId")
            row_trip = []
            for stu in tu.get("StopTimeUpdates") or []:
                arr = stu.get("Arrival") or {}
                dep = stu.get("Departure") or {}
                sid = str(stu.get("StopId")) if stu.get("StopId") is not None else None
                rec = {"routeId": str(rid) if rid is not None else None, "tripId": tid,
                       "stopId": sid, "seq": stu.get("StopSequence"),
                       "arrival": arr.get("Time"), "departure": dep.get("Time"),
                       "delay": arr.get("Delay") if arr.get("Delay") is not None else dep.get("Delay")}
                row_trip.append(rec)
                if sid is not None:
                    by_stop.setdefault(sid, []).append(rec)
            if tid:
                by_trip[tid] = row_trip
        for sid in by_stop:
            by_stop[sid].sort(key=lambda r: (r["arrival"] is None, r["arrival"] or 0))
        return {"ts": header.get("Timestamp") or int(time.time()),
                "count": len(by_trip), "byStop": by_stop, "byTrip": by_trip}

    def _build_alerts(self, raw):
        out, header = [], raw.get("Header") or {}
        for e in raw.get("Entities") or []:
            al = e.get("Alert")
            if not al:
                continue

            def _txt(block):
                tr = (block or {}).get("Translations") or []
                for t in tr:
                    if (t.get("Language") or "en").startswith("en"):
                        return t.get("Text")
                return tr[0].get("Text") if tr else None

            ents = al.get("InformedEntities") or []
            periods = al.get("ActivePeriods") or [{}]
            out.append({
                "id": str(e.get("Id") or ""),
                "header": _txt(al.get("HeaderText")),
                "description": _txt(al.get("DescriptionText")),
                "cause": ALERT_CAUSE.get(al.get("cause")),
                "effect": ALERT_EFFECT.get(al.get("effect")),
                "routes": sorted({str(x.get("RouteId")) for x in ents if x.get("RouteId") is not None}),
                "stops": sorted({str(x.get("StopId")) for x in ents if x.get("StopId") is not None}),
                "start": periods[0].get("Start"), "end": periods[0].get("End"),
                "url": (_txt(al.get("Url")) if isinstance(al.get("Url"), dict) else al.get("Url")),
            })
        return {"ts": header.get("Timestamp") or int(time.time()),
                "count": len(out), "alerts": out}

    def meta(self):
        with self._lock:
            ages = {k: round(time.time() - v[0], 1) for k, v in self._cache.items()}
        return {"mode": "mock" if self.mock else "live", "feedBase": FEED_BASE,
                "cacheSeconds": self.cache_seconds, "cacheAges": ages,
                "routesKnown": len(self.routes),
                "georef": {"A": self.proj.A, "B": self.proj.B, "utmZone": "16N"},
                "endpoints": ["/api/transit/vehicles", "/api/transit/trips",
                              "/api/transit/alerts", "/api/transit/meta"]}


# module-level context so functools.partial(directory=...) keeps working
CTX = {"proxy": None}


class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send_json(self, obj, status=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.end_headers()

    def do_GET(self):
        path = self.path.split("?", 1)[0].rstrip("/")
        if path == "/api/transit" or path == "/api/transit/meta":
            return self._send_json(CTX["proxy"].meta())
        for kind in ("vehicles", "trips", "alerts"):
            if path == "/api/transit/" + kind:
                return self._send_json(CTX["proxy"].get(kind))
        if path.startswith("/api/transit"):
            return self._send_json({"error": "unknown endpoint", "path": self.path}, 404)
        return super().do_GET()

    def copyfile(self, source, outputfile):
        # the viewer drops connections mid-stream while tiles/chunks stream in;
        # swallow the reset rather than dumping a traceback (same as verify_agents).
        try:
            super().copyfile(source, outputfile)
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
            pass


def load_route_map(data_dir=DATA):
    """routeId -> {id,shortName,color,longName} from the baked transit.json."""
    path = os.path.join(data_dir, "transit.json")
    if not os.path.exists(path):
        return {}
    try:
        t = json.load(open(path))
    except Exception:  # noqa: BLE001
        return {}
    return {r["id"]: {"id": r["id"], "shortName": r.get("shortName") or r["id"],
                      "color": r.get("color") or "3b82c4",
                      "longName": r.get("longName") or ""}
            for r in t.get("routes", [])}


def build_proxy(mock=False, cache_seconds=None):
    # short cache in mock so the synthetic crawl is continuous; a real feed only
    # updates every ~15-30 s, so a 5 s cache there is plenty and spares the agency.
    if cache_seconds is None:
        cache_seconds = 0.5 if mock else 5.0
    return TransitProxy(Projector(), load_route_map(), mock=mock, cache_seconds=cache_seconds)


def make_server(port, directory=WEB):
    handler = functools.partial(Handler, directory=directory)
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    httpd.daemon_threads = True
    return httpd


def selftest(mock):
    """Start the server on an ephemeral port, hit every endpoint, validate."""
    import urllib.request as u
    CTX["proxy"] = build_proxy(mock=mock)
    httpd = make_server(0)
    port = httpd.server_address[1]
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{port}"
    ok, notes = True, []

    def fail(m):
        nonlocal ok
        ok = False
        notes.append(m)

    try:
        veh = json.loads(u.urlopen(base + "/api/transit/vehicles", timeout=25).read())
        notes.append(f"vehicles: {veh.get('count')} ({veh.get('mode')})")
        if not veh.get("vehicles"):
            fail("no vehicles returned")
        else:
            inb = sum(1 for v in veh["vehicles"] if -1400 < v["x"] < 1000 and -1200 < v["z"] < 2700)
            notes.append(f"  {inb}/{len(veh['vehicles'])} project inside the campus viewport")
            if inb == 0:
                fail("no vehicle projected in-bounds (georef wrong?)")
            v0 = veh["vehicles"][0]
            for key in ("id", "x", "z", "lat", "lon"):
                if key not in v0:
                    fail(f"vehicle missing '{key}'")
        trips = json.loads(u.urlopen(base + "/api/transit/trips", timeout=25).read())
        notes.append(f"trips: {trips.get('count')} trips, {len(trips.get('byStop', {}))} stops with arrivals")
        alerts = json.loads(u.urlopen(base + "/api/transit/alerts", timeout=25).read())
        notes.append(f"alerts: {alerts.get('count')}")
        meta = json.loads(u.urlopen(base + "/api/transit/meta", timeout=25).read())
        if not meta.get("georef"):
            fail("meta missing georef")
        # CORS header present?
        resp = u.urlopen(base + "/api/transit/vehicles", timeout=25)
        if resp.headers.get("Access-Control-Allow-Origin") != "*":
            fail("missing CORS header")
        else:
            notes.append("CORS: ok")
        # static still served?
        man = u.urlopen(base + "/data/manifest.json", timeout=25)
        if man.status != 200:
            fail("static manifest.json not served")
        else:
            notes.append("static files: ok")
    except Exception as e:  # noqa: BLE001
        fail(f"selftest threw: {e}")
    finally:
        httpd.shutdown()

    print("=== transit proxy selftest ===")
    for n in notes:
        print("  " + n)
    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--mock", action="store_true", help="replay tools/_transit_samples (offline)")
    ap.add_argument("--cache-seconds", type=float, default=5.0)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        raise SystemExit(selftest(args.mock))

    CTX["proxy"] = build_proxy(mock=args.mock, cache_seconds=args.cache_seconds)
    httpd = make_server(args.port)
    mode = "MOCK (fixtures)" if args.mock else "LIVE (mystop.lextran.com)"
    print(f"UKy campus viewer + Lextran transit proxy")
    print(f"  serving {WEB}")
    print(f"  http://localhost:{args.port}/")
    print(f"  transit: {mode}  ->  /api/transit/(vehicles|trips|alerts|meta)")
    print("  Ctrl-C to stop")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")
        httpd.shutdown()


if __name__ == "__main__":
    main()
