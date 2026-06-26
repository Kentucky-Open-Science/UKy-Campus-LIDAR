#!/usr/bin/env python
"""Authoritative multiplayer server for the Lexington Digital Twin.

The twin runs HERE, on its own server. Scripts (any machine) connect over a small
REST API to spawn agents, drive them, and read their sensors; browsers load the
viewer from the same server. The agent simulation is AUTHORITATIVE and SHARED — a
fixed-rate sim thread on the server owns every agent's state, so an agent spawned by
ANY client (a Python script or a browser) is visible to ALL connected clients. This
is the key difference from driving `window.__twin.agents` in a single browser: that
state is private to one tab; this state is shared by everyone.

Physics (ackermann / differential / holonomic-drone kinematics, ground snapping,
collision detection) is ported from web/agents.js. Ground elevation comes from the
campus terrain heightmap (and the flat city plane beyond it); collisions use the
baked per-building AABBs (web/data/buildings.pack.json) plus agent-vs-agent.

This single server also carries the **live Lextran transit proxy** (folded in from the
old tools/serve.py): it fetches the agency's GTFS-Realtime feeds, projects each bus into
scene metres with the same georef the world uses, and re-serves them same-origin. So one
`python -m tools.twin_server` gives the viewer BOTH the shared agents and the moving buses
— including the headless `--render` browser, whose first-person frames then show traffic.

API (JSON, CORS-open; see client/twin.py for a Python wrapper):
    GET    /api/world/state                  -> { t, agents:[ ... ] }   (everyone's agents)
    GET    /api/world/agents/<id>            -> one agent's full sensor state
    GET    /api/world/meta                   -> types, bounds, georef
    POST   /api/world/spawn                  { type, position?, heading?, color?, name?, owner?, kinematic?, source? } -> { id, ... }
    POST   /api/world/agents/<id>/controls   { throttle/brake/steer/reverse | move | thrust/climb/yawRate }
    POST   /api/world/agents/<id>/driveTo    { x, z, y?, speed?, arriveRadius?, stop? }
    POST   /api/world/agents/<id>/pose       { x, z, y?, heading? }   (kinematic agents: set pose directly)
    POST   /api/world/agents/<id>/stop
    DELETE /api/world/agents/<id>
        (kinematic agents carry no physics and auto-despawn after World.kinematic_ttl s without a pose update)
    GET    /docs                             -> human-readable agent spawn/control API reference (HTML)
    GET    /api/transit/vehicles             -> live bus positions, projected to scene [x,_,z]
    GET    /api/transit/trips                -> predicted arrivals, indexed by stop and trip
    GET    /api/transit/alerts               -> service alerts (decoded cause/effect)
    GET    /api/transit/meta                 -> transit proxy status (mode, cache ages, georef)
    GET    /api/cameras/streams             -> fresh tokenized HLS URLs per camera id
    GET    /api/cameras/meta                -> camera proxy status (mode, cache age, count)
    GET/POST /api/cameras/calib             -> camera->scene homographies (calibration/cameras.json)
    GET/POST /api/cameras/detections        -> live detection boxes per camera (detector -> PiP overlay)
    GET/POST /api/cameras/active            -> which camera is being viewed (per-active-camera bounding)
    GET/POST /api/cameras/detect            -> start/stop the in-process YOLO detector per camera
                                               (POST {camera, on}; needs ultralytics+opencv in this venv)
    (anything else) -> served as a static file from web/

Run:  python -m tools.twin_server [--port 8000] [--hz 50] [--render]
      python -m tools.twin_server --mock        # replay tools/_transit_samples (offline buses)
      python -m tools.twin_server --no-transit   # world only, no bus proxy

Transit data (c) Lextran (Transit Authority of Lexington).
"""
import argparse
import base64
import functools
import hashlib
import http.server
import json
import math
import os
import queue
import re
import subprocess
import sys
import threading
import time
import urllib.request
from urllib.parse import urlparse, parse_qs, urlencode

import numpy as np

from tools.extract_roads import DATA, load_mesh, build_heightmap
from tools.transit_common import Projector

_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
# Operational cache for photorealistic tiles (gitignored; see README on Google's terms).
TILECACHE = os.path.join(DATA, "tilecache")
# Effectively never expire (override with TILECACHE_TTL_DAYS for a finite cache).
TILECACHE_TTL = float(os.environ.get("TILECACHE_TTL_DAYS", "3650")) * 24 * 3600
# Soft cap on total cache size. A long-running server whose viewers fly around the
# whole city would otherwise accumulate tiles forever (thousands of LOD nodes, each a
# blob + .ct sidecar). When the cache exceeds this, oldest-mtime files are pruned.
# 0 = no cap. Override with TILECACHE_MAX_GB.
TILECACHE_MAX_BYTES = float(os.environ.get("TILECACHE_MAX_GB", "4")) * (1 << 30)
_TILECACHE_TRIM_LOCK = threading.Lock()
_TILECACHE_LAST_TRIM = 0.0


def save_key_to_env(key, var="GOOGLE_MAPS_API_KEY", path=None):
    """Persist an API key into the repo-root .env (gitignored), preserving other lines.
    Updates the existing var line in place, or appends it. Returns the .env path."""
    path = path or os.path.join(_REPO_ROOT, ".env")
    try:
        with open(path, encoding="utf-8") as f:
            lines = f.read().splitlines()
    except FileNotFoundError:
        lines = ["# Local secrets — NEVER commit (gitignored)."]
    out, found = [], False
    for ln in lines:
        if ln.strip().startswith(var + "="):
            out.append(var + "=" + key)
            found = True
        else:
            out.append(ln)
    if not found:
        out.append(var + "=" + key)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")
    return path


def load_dotenv(path=None):
    """Minimal .env loader (no dependency): populate os.environ from repo-root .env,
    without overriding values already set in the real environment."""
    path = path or os.path.join(_REPO_ROOT, ".env")
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                k, v = k.strip(), v.strip().strip('"').strip("'")
                if k and k not in os.environ:
                    os.environ[k] = v
    except FileNotFoundError:
        pass

SAMPLES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_transit_samples")

_HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.abspath(os.path.join(_HERE, "..", "web"))
# Camera->scene homography calibration: authored config (NOT pipeline output), so it
# lives OUTSIDE the gitignored web/data/ and is version-controlled (constitution P1).
CALIB_PATH = os.path.abspath(os.path.join(_HERE, "..", "calibration", "cameras.json"))
# Reentrant so a read-modify-write (_calib_post) can hold it across load+mutate+write
# without self-deadlocking when save_calib re-acquires it (fixes a lost-update race).
CALIB_LOCK = threading.RLock()

# Live detection relay (Phase 3): the detector (tools/camera_detect.py) publishes the
# image-space boxes it's spawning cars from; the viewer's PiP overlay polls + draws them.
# Plus an "active camera" signal so a detector can idle when nobody is viewing its camera
# (per-active-camera perf bounding). All in-memory + TTL'd; never persisted.
DET_LOCK = threading.Lock()
DETECTIONS = {}                 # camera_id -> {ts, frame, dets:[...]}
ACTIVE = {}                     # camera_id -> last-seen ts (per-camera "is being viewed")
DET_TTL = 6.0                   # serve detections younger than this (s)
ACTIVE_TTL = 12.0               # an active-camera signal is valid this long (s)

# In-process YOLO detector control. So the viewer can start/stop detection per-camera
# (a PiP checkbox) and `twin_server` runs the camera_detect loop itself in a daemon
# thread, instead of a separate `python -m tools.camera_detect`. ultralytics/opencv are
# imported lazily by the detector, so the server still starts without them — detection
# just can't be enabled until the server runs in a venv that has them (e.g. the GPU one).
DETECTORS = {}                  # camera_id -> {stop, thread, started, model, alive, error}
DET_MGR_LOCK = threading.Lock()
DETECT_MODEL = "yolo26x.pt"     # overridden by --detect-model in main()
DETECT_MAX_FPS = 8.0            # overridden by --detect-max-fps (legacy cap)
DETECT_INTERVAL = 0.0          # min s between YOLO inferences; 0 = freshest frame at model speed
DETECT_CONF = 0.30              # overridden by --detect-conf
SERVER_BASE = None              # "http://127.0.0.1:<port>", set in main() (in-process client)

# Hard cap on a POST body. Every API endpoint takes a tiny JSON object; a payload
# larger than this is never legitimate and an uncapped Content-Length would let a
# client force the server to buffer an arbitrary blob (OOM on a shared box).
MAX_BODY_BYTES = 1 << 20        # 1 MiB


class BodyTooLarge(Exception):
    """Raised by Handler._body when a POST's Content-Length exceeds MAX_BODY_BYTES.
    Caught at the top of do_POST so a 413 is sent cleanly instead of the handler
    proceeding with an empty body and writing a second response on the connection."""


def _reject_constant(_c):
    # json.loads accepts NaN/Infinity/-Infinity by default (mapping to float). The JSON
    # spec forbids them and they would bypass finite() at the value level, so reject at
    # the parse boundary. Used as json.loads(..., parse_constant=_reject_constant).
    raise ValueError("NaN/Infinity are not valid JSON")


def _maybe_trim_tilecache():
    """Best-effort LRU trim of the photoreal tile cache so it can't grow unbounded on
    a long-running server. Throttled to at most once per 5 min (the scan is a single
    os.scandir + stat over the cache dir; cheap, but no reason to do it per tile).
    Runs on the calling thread but returns fast under the throttle guard; failures are
    swallowed — a trim error must never break tile serving."""
    global _TILECACHE_LAST_TRIM
    if not TILECACHE_MAX_BYTES:
        return
    now = time.time()
    if not _TILECACHE_TRIM_LOCK.acquire(blocking=False):
        return
    try:
        if now - _TILECACHE_LAST_TRIM < 300:        # at most one trim per 5 min
            return
        _TILECACHE_LAST_TRIM = now
        entries = []                                  # (mtime, size, path)
        total = 0
        for name in os.scandir(TILECACHE):
            try:
                st = name.stat()
            except OSError:
                continue
            entries.append((st.st_mtime, st.st_size, name.path))
            total += st.st_size
        if total <= TILECACHE_MAX_BYTES:
            return
        # evict oldest-mtime first (the .ct sidecar sits next to its blob; both are
        # keyed by the same hash so both get pruned as they age together)
        entries.sort(key=lambda e: e[0])
        for mtime, size, path in entries:
            if total <= TILECACHE_MAX_BYTES:
                break
            try:
                os.remove(path)
                total -= size
            except OSError:
                pass
    except OSError:
        pass
    finally:
        _TILECACHE_TRIM_LOCK.release()


def load_calib():
    try:
        with open(CALIB_PATH, encoding="utf-8") as f:
            return json.load(f)
    except Exception:  # noqa: BLE001 — missing/blank is a valid empty calibration
        return {"version": 1, "cameras": {}}


def save_calib(obj):
    os.makedirs(os.path.dirname(CALIB_PATH), exist_ok=True)
    with CALIB_LOCK:
        with open(CALIB_PATH, "w", encoding="utf-8") as f:
            json.dump(obj, f, indent=2)


def finite(*vals):
    """True iff every value is a finite real number (rejects NaN/inf from the network)."""
    try:
        return all(math.isfinite(float(v)) for v in vals)
    except (TypeError, ValueError):
        return False


# ---- in-process detector manager (start/stop the YOLO loop per camera) -------------
def _have_detector_deps():
    import importlib.util
    return all(importlib.util.find_spec(m) is not None for m in ("cv2", "ultralytics"))


def _run_detector(det, stop_evt, camera):
    """Thread body: run the detector loop; record any failure for the UI; never crash
    the server. The loop checks stop_evt each frame and clears its cars on exit."""
    err = None
    try:
        det.run(max_fps=DETECT_MAX_FPS, detect_interval=DETECT_INTERVAL, stop=stop_evt.is_set)
    except (Exception, SystemExit) as e:  # noqa: BLE001 — surface to the UI, don't propagate
        err = str(e) or e.__class__.__name__
    finally:
        with DET_MGR_LOCK:
            rec = DETECTORS.get(camera)
            if rec and rec.get("thread") is threading.current_thread():
                rec["alive"] = False
                if err:
                    rec["error"] = err


# Analysis-mode detectors run alongside (never instead of) a normal detector for the same
# camera, so we key them under a distinct slot in DETECTORS. This lets the PiP "Analysis"
# button start/stop a raw image-space detector on an UNCALIBRATED camera without clobbering
# (or being clobbered by) a normal "Run YOLO" detector on that same camera.
def _det_key(camera, analysis=False):
    return (str(camera) + "::analysis") if analysis else str(camera)


def start_detector(camera, analysis=False):
    if SERVER_BASE is None:
        return {"error": "server base URL not set"}
    if not _have_detector_deps():
        return {"error": "detection needs ultralytics + opencv — start the server from a "
                         "venv that has them (e.g. the GPU detection env)"}
    from tools.camera_detect import CameraDetector, TwinClient, quad_homographies
    Hq = quad_homographies(load_calib(), camera)
    # Normal mode still needs at least one calibrated quad (nothing to map otherwise).
    # Analysis mode is exactly the bootstrap case: allow an empty Hq — the detector runs
    # YOLO on all quads and publishes raw boxes for the flow analysis, spawning nothing.
    if not Hq and not analysis:
        return {"error": f"{camera} has no calibrated quads — calibrate it in the PiP first"}
    key = _det_key(camera, analysis)
    with DET_MGR_LOCK:
        rec = DETECTORS.get(key)
        if rec and rec.get("alive") and rec["thread"].is_alive():
            return {"ok": True, "camera": camera, "running": True, "already": True,
                    "analysis": analysis}
        twin = TwinClient(SERVER_BASE)
        det = CameraDetector(twin, camera, Hq, model=DETECT_MODEL, conf=DETECT_CONF,
                             publish=True, follow_active=False, analysis=analysis)
        stop_evt = threading.Event()
        th = threading.Thread(target=_run_detector, args=(det, stop_evt, key),
                              name=f"detect-{key}", daemon=True)
        DETECTORS[key] = {"stop": stop_evt, "thread": th, "started": time.time(),
                          "model": DETECT_MODEL, "alive": True, "error": None,
                          "analysis": analysis}
        th.start()
    return {"ok": True, "camera": camera, "running": True, "model": DETECT_MODEL,
            "analysis": analysis}


def stop_detector(camera, analysis=False):
    key = _det_key(camera, analysis)
    with DET_MGR_LOCK:
        rec = DETECTORS.get(key)
        if rec:
            rec["stop"].set()
            rec["alive"] = False
    return {"ok": True, "camera": camera, "running": False, "analysis": analysis}


def stop_all_detectors():
    with DET_MGR_LOCK:
        for rec in DETECTORS.values():
            rec["stop"].set()
            rec["alive"] = False


def detector_status(camera=None):
    with DET_MGR_LOCK:
        if camera is not None:
            rec = DETECTORS.get(camera)
            alive = bool(rec and rec.get("alive") and rec["thread"].is_alive())
            return {"camera": camera, "running": alive,
                    "model": rec.get("model") if rec else DETECT_MODEL,
                    "error": rec.get("error") if rec else None,
                    "depsAvailable": _have_detector_deps()}
        running = [c for c, r in DETECTORS.items() if r.get("alive") and r["thread"].is_alive()]
        return {"running": running, "model": DETECT_MODEL, "depsAvailable": _have_detector_deps()}

# ---- per-type kinematic defs (scene metres), aligned with web/agents.js TYPES ----
# Footprints are REAL-WORLD sized so camera-detected (kinematic) vehicles render at a
# believable scale against the twin's roads/buildings. L = along local +X (forward),
# W = along +Z (width), H = up; the Agent.half / OBB footprint is derived as
# [L/2, H/2, W/2] -- the same convention as the viewer's halfExtents. The earlier table
# was ~70% scale (car 3.0x1.6, bus only 8.5 m), which rendered camera cars undersized
# and the bus far short of a real ~12 m transit coach; corrected here to match
# web/agents.js TYPES (car/truck) and realistic dimensions for the rest. The detector
# (tools/camera_detect.py) maps COCO classes onto these keys, so every road user gets
# its own honest footprint. wheelbase / trackWidth scale proportionally with the body so
# the ackermann / differential turning radius stays sane at the larger sizes.
DEFS = {
    "car":   dict(L=4.5, W=1.9, H=1.45, wheelbase=2.7, kin="ackermann", ground=True,
                  maxSpeed=25, maxAccel=6, maxSteerDeg=35),
    "truck": dict(L=8.0, W=2.5, H=3.0, wheelbase=4.8, kin="ackermann", ground=True,
                  maxSpeed=18, maxAccel=4, maxSteerDeg=28),
    # bus / motorcycle / bicycle / pedestrian: the rest of the road-user classes the
    # traffic-camera detector (tools/camera_detect.py) maps COCO detections onto, sized so
    # each renders at a believable footprint instead of a generic car box.
    "bus":   dict(L=12.0, W=2.9, H=3.3, wheelbase=6.0, kin="ackermann", ground=True,
                  maxSpeed=18, maxAccel=3, maxSteerDeg=25),
    "moto":  dict(L=2.2, W=0.9, H=1.3, wheelbase=1.5, kin="ackermann", ground=True,
                  maxSpeed=25, maxAccel=7, maxSteerDeg=40),
    "bike":  dict(L=1.8, W=0.6, H=1.2, trackWidth=0.6, kin="differential", ground=True,
                  maxSpeed=8, maxAccel=3),
    "ped":   dict(L=0.6, W=0.6, H=1.7, trackWidth=0.4, kin="differential", ground=True,
                  maxSpeed=2, maxAccel=3),
    "robot": dict(L=0.8, W=0.6, H=0.6, trackWidth=0.5, kin="differential", ground=True,
                  maxSpeed=3, maxAccel=4),
    "drone": dict(L=0.9, W=0.9, H=0.35, kin="holonomic", ground=False,
                  maxSpeed=15, maxAccel=10, maxClimb=6, maxYawRateDeg=120, minClearance=0.5),
}
DEFAULT_COLORS = {"car": 0x3577c9, "truck": 0x8e6bd0, "bus": 0xe0a83a, "moto": 0xc77f2a,
                  "bike": 0x4aa05a, "ped": 0xe25fae, "robot": 0x4aa05a, "drone": 0x9b59b6}
# first-person camera mount per type: eye height above the agent (m) + downward pitch (deg)
CAM_EYE = {"car": 1.4, "truck": 2.6, "bus": 2.8, "moto": 1.3, "bike": 1.5, "ped": 1.6,
           "robot": 0.5, "drone": 0.0}
CAM_PITCH = {"car": 6.0, "truck": 6.0, "robot": 4.0, "drone": 22.0}

D2R = math.pi / 180.0
R2D = 180.0 / math.pi


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def sign(v):
    return 1.0 if v > 0 else -1.0 if v < 0 else 0.0


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# =====================================================================
class Ground:
    """Terrain elevation + surface lookup: campus heightmap, flat city plane beyond."""

    def __init__(self):
        self.ok = False
        self.city_y = 281.0
        try:
            manifest = json.load(open(os.path.join(DATA, "manifest.json")))
            self._georef(manifest)
            self._heightmap(manifest)
            cpath = os.path.join(DATA, "city.json")
            if os.path.exists(cpath):
                self.city_y = float(json.load(open(cpath)).get("groundY", self.city_y))
            self.ok = True
        except Exception as e:  # noqa: BLE001 — degrade to a flat world
            print(f"[ground] no terrain heightmap ({e}); using a flat plane at {self.city_y} m")
        self.kygrid = None
        self._load_kygrid()

    def _load_kygrid(self):
        """Load the KyFromAbove city-wide bare-ground elevation grid (built by
        tools/ky_lidar.py --heightmap). Used as the PRIMARY ground across Lexington;
        the campus mesh heightmap + flat city plane remain fallbacks for any gaps."""
        try:
            gm = json.load(open(os.path.join(DATA, "ground.json")))
            arr = np.fromfile(os.path.join(DATA, "ground.f32"), np.float32)
            self.kygrid = (arr.reshape(gm["nz"], gm["nx"]), gm)
            print(f"[ground] KYAPED ground grid {gm['nx']}x{gm['nz']} @ {gm['cell']} m "
                  f"({100 * gm['filled'] / max(gm['total'], 1):.0f}% filled)")
        except Exception:  # noqa: BLE001 — optional layer
            self.kygrid = None

    def _georef(self, manifest):
        oc = manifest["lidar"]["original_coordinates"]
        o = manifest["origin_cm"]
        self.A = (oc[0] + o[0]) / 100.0
        self.B = -(oc[1] + o[1]) / 100.0

    def _heightmap(self, manifest):
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
        mpp = 50.0
        w = int(math.ceil((gx1 - gx0) / mpp)); h = int(math.ceil((gz1 - gz0) / mpp))
        self._elev = build_heightmap(manifest, (gx0, gz0, mpp, w, h))
        self.sxmin, self.sxmax = gx0 / 100, gx1 / 100
        self.szmin, self.szmax = -gz1 / 100, -gz0 / 100

    def height(self, sx, sz):
        if not (math.isfinite(sx) and math.isfinite(sz)):   # never int(NaN) into the grid
            return 0.0, "none"
        if self.kygrid is not None:                    # KYAPED city ground (primary)
            arr, gm = self.kygrid
            ix = int((sx - gm["x0"]) / gm["cell"]); iz = int((sz - gm["z0"]) / gm["cell"])
            if 0 <= ix < gm["nx"] and 0 <= iz < gm["nz"]:
                v = arr[iz, ix]
                if v == v:                             # finite (not NaN) -> real ground
                    return float(v), "terrain"
        if self.ok and self.sxmin <= sx <= self.sxmax and self.szmin <= sz <= self.szmax:
            return float(self._elev(sx * 100.0, -sz * 100.0)) / 100.0, "terrain"
        return self.city_y, "terrain"

    def utm(self, sx, sz):
        if not self.ok:
            return None
        return {"easting": self.A + sx, "northing": self.B - sz, "zone": "16N"}


class Buildings:
    """Per-building AABBs (from buildings.pack.json) with a coarse broad-phase grid."""

    def __init__(self):
        self.items = []
        self.cell = 64.0
        self.grid = {}
        path = os.path.join(DATA, "buildings.pack.json")
        if not os.path.exists(path):
            print("[buildings] no buildings.pack.json — agent-vs-building collisions off")
            return
        meta = json.load(open(path))
        for i, b in enumerate(meta.get("buildings", [])):
            mn, mx = b["min"], b["max"]
            it = {"id": i, "name": b["name"], "min": mn, "max": mx,
                  "cx": (mn[0] + mx[0]) / 2, "cz": (mn[2] + mx[2]) / 2}
            self.items.append(it)
            key = (int(it["cx"] // self.cell), int(it["cz"] // self.cell))
            self.grid.setdefault(key, []).append(it)
        print(f"[buildings] {len(self.items)} collision boxes loaded")

    def nearby(self, x, z):
        # Guard NaN/inf: int(NaN // cell) raises ValueError, which would propagate out of
        # World.tick() (under the lock) and kill the sim thread. A non-finite query can
        # never match a real AABB, so return nothing rather than crash.
        if not (math.isfinite(x) and math.isfinite(z)):
            return []
        gx, gz = int(x // self.cell), int(z // self.cell)
        out = []
        for dx in (-1, 0, 1):
            for dz in (-1, 0, 1):
                out.extend(self.grid.get((gx + dx, gz + dz), ()))
        return out

    def nearest(self, x, z, min_dist=8.0):
        best, bd = None, 1e18
        for it in self.items:
            d = math.hypot(it["cx"] - x, it["cz"] - z)
            if min_dist < d < bd:
                bd = d; best = it
        if not best:
            return None
        mn, mx = best["min"], best["max"]
        return {"x": best["cx"], "y": (mn[1] + mx[1]) / 2, "z": best["cz"],
                "name": best["name"], "dist": round(bd, 2)}


# =====================================================================
class Agent:
    def __init__(self, world, aid, name, typ, opts):
        d = DEFS[typ]
        self.world = world
        self.id = aid
        self.name = name
        self.type = typ
        self.owner = opts.get("owner") or "anon"
        self.color = opts.get("color", DEFAULT_COLORS[typ])
        self.d = d
        self.ground_bound = d["ground"]
        self.maxSpeed = d["maxSpeed"]
        self.maxAccel = d["maxAccel"]
        self.brakeDecel = d["maxAccel"]
        self.maxSteerRad = d.get("maxSteerDeg", 35) * D2R
        self.steerRateRad = 90 * D2R
        self.wheelbase = d.get("wheelbase", 2.5)
        self.trackWidth = d.get("trackWidth", 0.5)
        self.maxClimb = d.get("maxClimb", 6)
        self.maxYawRateRad = d.get("maxYawRateDeg", 120) * D2R
        self.minClearance = d.get("minClearance", 0.5)
        self.half = [d["L"] / 2, d["H"] / 2, d["W"] / 2]

        pos = opts.get("position")
        px = pos[0] if pos else 0.0
        pz = (pos[2] if pos and len(pos) > 2 else (pos[1] if pos else 0.0))
        self.x = float(px) if finite(px) else 0.0          # reject NaN/inf spawn coords
        self.z = float(pz) if finite(pz) else 0.0
        _hd = opts.get("heading", 0) or 0
        self.yaw = (float(_hd) if finite(_hd) else 0.0) * D2R
        self.speed = 0.0
        self.steerAngle = 0.0
        self.vel = [0.0, 0.0, 0.0]
        self.yawRate = 0.0
        self.controls = {}
        self.goal = None
        self.surface = "none"
        self.groundY = None
        self.altitudeAGL = None
        self.onGround = False
        self.offMap = True
        self.contacts = []
        self.measVel = [0.0, 0.0, 0.0]

        # initial ground placement
        gy, _ = world.ground.height(self.x, self.z)
        self.y = (pos[1] if pos and len(pos) > 2 and pos[1] is not None else None)
        if self.y is None:
            self.y = gy + (20.0 if typ == "drone" else 0.0)
        self.prev = [self.x, self.y, self.z]

        # Kinematic agents (e.g. camera-detected cars) carry no physics: their pose is set
        # directly via set_pose() and they auto-expire when updates stop (World TTL sweep).
        self.kinematic = bool(opts.get("kinematic"))
        self.source = opts.get("source")     # e.g. {"cam":.., "quad":.., "track":..}
        self.last_update = time.time()

    # ---- control inputs ----
    def set_controls(self, c):
        self.goal = None
        self.controls = self._sanitize(c or {})

    def drive_to(self, g):
        x = float(g["x"]); z = float(g["z"])
        if not finite(x, z):
            raise ValueError("driveTo x and z must be finite numbers")
        speed = float(g.get("speed", 0.6 * self.maxSpeed))
        arrive = float(g.get("arriveRadius", 2.0))
        if not finite(speed, arrive) or speed < 0 or arrive < 0:
            raise ValueError("driveTo speed and arriveRadius must be finite >= 0")
        self.goal = {
            "x": x,
            "y": (float(g["y"]) if g.get("y") is not None else None),
            "z": z,
            "speed": speed,
            "arriveRadius": arrive,
            "stop": bool(g.get("stop", True)),
        }

    def stop(self):
        self.goal = None
        self.vel = [0.0, 0.0, 0.0]
        self.controls = {"move": [0, 0, 0]} if self.type == "drone" else {"throttle": 0, "brake": 1, "steer": 0}

    def _sanitize(self, c):
        out = {}
        for k, lo, hi in (("throttle", 0, 1), ("brake", 0, 1), ("steer", -1, 1),
                          ("left", -1, 1), ("right", -1, 1), ("thrust", 0, 1), ("climb", -1, 1)):
            if c.get(k) is not None:
                v = float(c[k])
                if not math.isfinite(v):
                    continue              # drop non-finite (NaN/inf) -> integrator's default 0
                out[k] = clamp(v, lo, hi)
        if c.get("yawRate") is not None:
            yr = float(c["yawRate"])
            if math.isfinite(yr):
                out["yawRate"] = yr
        for b in ("reverse", "handbrake"):
            if c.get(b) is not None:
                out[b] = bool(c[b])
        if isinstance(c.get("move"), (list, tuple)):
            m = c["move"]
            mv = [float(m[i]) if i < len(m) and m[i] is not None else 0.0 for i in range(3)]
            out["move"] = [v if math.isfinite(v) else 0.0 for v in mv]
        return out

    # ---- goal -> controls (server-side driveTo controller, ported) ----
    def _apply_goal(self):
        g = self.goal
        if not g:
            return
        dx, dz = g["x"] - self.x, g["z"] - self.z
        dist = math.hypot(dx, dz)
        if self.type == "drone":
            dy = clamp((g["y"] - self.y), -self.maxClimb, self.maxClimb) if g["y"] is not None else 0.0
            if dist > g["arriveRadius"]:
                s = g["speed"] / max(dist, 1e-6)
                self.controls = {"move": [dx * s, dy, dz * s]}
            else:
                self.controls = {"move": [0, dy, 0]}
                if g["stop"]:
                    self.goal = None
            return
        desired = math.atan2(-dz, dx)
        err = wrap_pi(desired - self.yaw)
        steer = clamp(err / self.maxSteerRad, -1, 1)
        if dist <= g["arriveRadius"]:
            self.controls = {"throttle": 0, "brake": 1 if g["stop"] else 0, "steer": 0}
            if g["stop"]:
                self.goal = None
        else:
            self.controls = {"throttle": clamp(g["speed"] / self.maxSpeed, 0, 1), "steer": steer, "brake": 0}

    # ---- pose set directly (kinematic agents: camera cars, replays) ----
    def set_pose(self, x, z, y=None, heading=None):
        # Reject non-finite poses from a flaky detector/client: a NaN x/z would later
        # be int()'d into the ground grid and throw inside World.tick. Drop the update
        # (and do NOT bump last_update, so the TTL sweep can still reap a stuck agent).
        if not finite(x, z):
            return
        self.x = float(x)
        self.z = float(z)
        if y is not None and finite(y):
            self.y = float(y)
        if heading is not None and finite(heading):
            self.yaw = float(heading) * D2R
        self.last_update = time.time()

    # ---- integrate one tick ----
    def integrate(self, dt):
        if self.kinematic:
            return                      # pose is externally driven; no physics
        self._apply_goal()
        kin = self.d["kin"]
        if kin == "ackermann":
            self._ackermann(dt)
        elif kin == "differential":
            self._differential(dt)
        else:
            self._holonomic(dt)

    def _ackermann(self, dt):
        c = self.controls
        d = -1 if c.get("reverse") else 1
        throttle = c.get("throttle", 0)
        brake = (1 if c.get("handbrake") else 0) or c.get("brake", 0)
        drag = 0.05 * self.speed
        a = d * throttle * self.maxAccel - brake * self.brakeDecel * sign(self.speed) - drag
        self.speed = clamp(self.speed + a * dt, -0.4 * self.maxSpeed, self.maxSpeed)
        if abs(self.speed) < 0.02 and throttle == 0:
            self.speed = 0
        st = c.get("steer", 0) * self.maxSteerRad
        self.steerAngle += clamp(st - self.steerAngle, -self.steerRateRad * dt, self.steerRateRad * dt)
        self.yawRate = (self.speed / self.wheelbase) * math.tan(self.steerAngle)
        self.yaw += self.yawRate * dt
        self.x += math.cos(self.yaw) * self.speed * dt
        self.z += -math.sin(self.yaw) * self.speed * dt

    def _differential(self, dt):
        c = self.controls
        if c.get("left") is not None or c.get("right") is not None:
            vL, vR = c.get("left", 0), c.get("right", 0)
        else:
            th, st = c.get("throttle", 0), c.get("steer", 0)
            vL, vR = th - st, th + st
        if c.get("brake"):
            vL *= (1 - c["brake"]); vR *= (1 - c["brake"])
        vL = clamp(vL, -1, 1) * self.maxSpeed; vR = clamp(vR, -1, 1) * self.maxSpeed
        self.speed = (vL + vR) / 2
        self.yawRate = (vR - vL) / max(self.trackWidth, 1e-3)
        self.yaw += self.yawRate * dt
        self.x += math.cos(self.yaw) * self.speed * dt
        self.z += -math.sin(self.yaw) * self.speed * dt

    def _holonomic(self, dt):
        c = self.controls
        accel = self.maxAccel
        if isinstance(c.get("move"), list):
            mx, my, mz = c["move"]
            hor = math.hypot(mx, mz)
            if hor > self.maxSpeed:
                mx *= self.maxSpeed / hor; mz *= self.maxSpeed / hor
            my = clamp(my, -self.maxClimb, self.maxClimb)
            for i, target in enumerate((mx, my, mz)):
                dv = clamp(target - self.vel[i], -accel * dt, accel * dt)
                self.vel[i] += dv
        else:
            thrust = c.get("thrust", 0); climb = c.get("climb", 0)
            yawRate = c.get("yawRate", 0) * D2R
            fx = math.cos(self.yaw) * thrust * self.maxSpeed
            fz = -math.sin(self.yaw) * thrust * self.maxSpeed
            self.vel[0] += clamp(fx - self.vel[0], -accel * dt, accel * dt)
            self.vel[2] += clamp(fz - self.vel[2], -accel * dt, accel * dt)
            self.vel[1] += clamp(climb * self.maxClimb - self.vel[1], -accel * dt, accel * dt)
            self.yawRate = clamp(yawRate, -self.maxYawRateRad, self.maxYawRateRad)
            self.yaw += self.yawRate * dt
            self.vel = [v * 0.99 for v in self.vel]
        self.x += self.vel[0] * dt
        self.y += self.vel[1] * dt
        self.z += self.vel[2] * dt
        self.speed = math.hypot(self.vel[0], self.vel[2])

    # ---- ground snap ----
    def snap_ground(self, dt):
        gy, surf = self.world.ground.height(self.x, self.z)
        self.offMap = False
        self.groundY = gy
        self.surface = surf
        if self.ground_bound:
            # NOTE: this runs every tick for kinematic agents too, so a y passed to
            # set_pose / spawn is intentionally overridden for ground-bound types
            # (cars/trucks/robots) — they obey the one-ground-model invariant. The y?
            # field of POST /pose is therefore only effective for airborne types (drone).
            self.y = gy
            self.altitudeAGL = 0.0
            self.onGround = True
        else:
            floor = gy + self.minClearance
            if self.y < floor:
                self.y = floor
                if self.vel[1] < 0:
                    self.vel[1] = 0
            self.altitudeAGL = self.y - gy
            self.onGround = self.altitudeAGL < 0.5

    def finalize(self, dt):
        if dt > 0:
            self.measVel = [(self.x - self.prev[0]) / dt, (self.y - self.prev[1]) / dt,
                            (self.z - self.prev[2]) / dt]
        self.prev = [self.x, self.y, self.z]

    # ---- collision (axis-aligned boxes; agent-vs-building + agent-vs-agent) ----
    def detect(self):
        self.contacts = []
        # Kinematic agents (camera-detected cars) have no physics response, never read
        # self.contacts, and overlapping detections would otherwise flash them phantom-red
        # in the viewer — so they neither generate nor receive collision contacts.
        if self.kinematic:
            return
        amin = [self.x - self.half[0], self.y, self.z - self.half[2]]
        amax = [self.x + self.half[0], self.y + 2 * self.half[1], self.z + self.half[2]]
        for b in self.world.buildings.nearby(self.x, self.z):
            r = _aabb(amin, amax, b["min"], b["max"])
            if r:
                self._contact("building", b["id"], b["name"], r, (0, 0, 0))
        for o in self.world.agents.values():
            if o is self or o.kinematic:        # don't collide real agents with phantom cars
                continue
            omin = [o.x - o.half[0], o.y, o.z - o.half[2]]
            omax = [o.x + o.half[0], o.y + 2 * o.half[1], o.z + o.half[2]]
            r = _aabb(amin, amax, omin, omax)
            if r:
                self._contact("agent", o.id, o.name, r, o.measVel)

    def _contact(self, kind, oid, oname, r, ovel):
        nx, nz, pen = r
        rel = -((self.measVel[0] - ovel[0]) * nx + (self.measVel[2] - ovel[2]) * nz)
        self.contacts.append({"with": kind, "id": oid, "name": oname,
                              "normal": [nx, 0, nz], "penetration": round(pen, 3),
                              "relativeSpeed": round(rel, 3)})

    def camera_pose(self):
        """First-person camera eye position + forward unit vector (scene metres)."""
        eh = CAM_EYE.get(self.type, 1.4)
        pd = CAM_PITCH.get(self.type, 6.0) * D2R
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        cp, sp = math.cos(pd), math.sin(pd)
        return ((self.x, self.y + eh, self.z), (cy * cp, -sp, -sy * cp))

    def state(self):
        return {
            "id": self.id, "name": self.name, "type": self.type, "owner": self.owner,
            "color": self.color,
            "position": [round(self.x, 3), round(self.y, 3), round(self.z, 3)],
            "heading": round((self.yaw * R2D) % 360, 1),
            "forward": [round(math.cos(self.yaw), 4), 0, round(-math.sin(self.yaw), 4)],
            "velocity": [round(v, 3) for v in self.measVel],
            "speed": round(self.speed, 3),
            "surface": self.surface, "groundY": round(self.groundY, 3) if self.groundY is not None else None,
            "altitudeAGL": round(self.altitudeAGL, 3) if self.altitudeAGL is not None else None,
            "onGround": self.onGround, "offMap": self.offMap,
            "utm": self.world.ground.utm(self.x, self.z),
            "collisions": self.contacts,
            # so the viewer can identify camera-detected (traffic-stream) cars: these carry
            # a source with a camera id, and are kinematic (pose-driven, no physics).
            "kinematic": self.kinematic, "source": self.source,
        }


def _aabb(amin, amax, bmin, bmax):
    """XZ + Y-interval overlap test; returns (nx, nz, penetration) or None.
    Normal points from B toward A along the least-overlap horizontal axis."""
    if amax[1] <= bmin[1] or amin[1] >= bmax[1]:
        return None
    ox = min(amax[0], bmax[0]) - max(amin[0], bmin[0])
    oz = min(amax[2], bmax[2]) - max(amin[2], bmin[2])
    if ox <= 0 or oz <= 0:
        return None
    acx = (amin[0] + amax[0]) / 2; acz = (amin[2] + amax[2]) / 2
    bcx = (bmin[0] + bmax[0]) / 2; bcz = (bmin[2] + bmax[2]) / 2
    if ox < oz:
        return (1.0 if acx >= bcx else -1.0, 0.0, ox)
    return (0.0, 1.0 if acz >= bcz else -1.0, oz)


def obbObbXZ(acx, acz, aX, aZ, ahx, ahz, bcx, bcz, bX, bZ, bhx, bhz):
    """SAT overlap of two oriented boxes on the XZ plane. Line-for-line port of
    obbObbXZ() in web/agents.js: returns (nx, nz, pen) where the unit normal (nx,nz)
    points from B toward A along the least-overlap (minimum-translation) axis and `pen`
    is the penetration depth in metres, or None if the boxes are separated.

    Unlike _aabb this respects each box's yaw, so it stays correct for the small per-car
    heading jitter the camera detector produces at a stop line. aX/aZ and bX/bZ are the
    boxes' local +X / +Z axes in the XZ plane (e.g. aX=(cos,-sin), aZ=(sin,cos))."""
    axes = (aX, aZ, bX, bZ)
    min_ov = math.inf
    nx = nz = 0.0
    dx, dz = acx - bcx, acz - bcz
    for lx, lz in axes:
        length = math.hypot(lx, lz) or 1.0
        ux, uz = lx / length, lz / length
        rA = abs(ahx * (aX[0] * ux + aX[1] * uz)) + abs(ahz * (aZ[0] * ux + aZ[1] * uz))
        rB = abs(bhx * (bX[0] * ux + bX[1] * uz)) + abs(bhz * (bZ[0] * ux + bZ[1] * uz))
        dist = dx * ux + dz * uz
        ov = rA + rB - abs(dist)
        if ov <= 0:
            return None
        if ov < min_ov:
            min_ov = ov
            sgn = 1.0 if dist >= 0 else -1.0
            nx, nz = ux * sgn, uz * sgn
    return (nx, nz, min_ov)


# =====================================================================
class World:
    def __init__(self, hz=50, max_agents=64, ground=None, buildings=None):
        # ground + buildings are read-only world data; pass shared instances to avoid
        # reloading the heightmap per World (e.g. when vectorising the gym env).
        self.ground = ground if ground is not None else Ground()
        self.buildings = buildings if buildings is not None else Buildings()
        self.agents = {}
        self.traffic = None        # optional TrafficManager (NPC cars + signals); ticked first
        self.lock = threading.RLock()
        self.t = 0.0
        self.frame = 0
        self.hz = hz
        self.max_agents = max_agents
        self._next = 1
        self._names = set()
        self.kinematic_ttl = 5.0   # despawn kinematic agents un-updated for this long (s)

    def spawn(self, opts):
        typ = opts.get("type", "car")
        if typ not in DEFS:
            raise ValueError(f"unknown type '{typ}'; valid: {', '.join(DEFS)}")
        with self.lock:
            if len(self.agents) >= self.max_agents:
                raise RuntimeError(f"agent cap reached ({self.max_agents})")
            aid = self._next; self._next += 1
            name = opts.get("name") or f"{typ}_{aid}"
            if name in self._names:
                raise ValueError(f"name '{name}' already in use")
            a = Agent(self, aid, name, typ, opts)
            self.agents[aid] = a
            self._names.add(name)
            return a

    def despawn(self, aid):
        with self.lock:
            a = self.agents.pop(aid, None)
            if a:
                self._names.discard(a.name)
            return a is not None

    def get(self, aid):
        return self.agents.get(aid)

    def tick(self, dt):
        with self.lock:
            if self.traffic is not None:      # set NPC controls before integrating
                self.traffic.tick(dt)
            arr = list(self.agents.values())
            for a in arr:
                a.integrate(dt)
            for a in arr:
                a.snap_ground(dt)
                a.finalize(dt)
            # Anti-clip: push overlapping footprints apart AFTER integration/ground-snap,
            # so each rendered frame is overlap-free even though the project only does
            # collision DETECTION (no physics response). This is the one step that pulls
            # dense camera cars out of each other; detect() then reports the resolved poses.
            self._separate()
            for a in arr:
                a.detect()
            if self.kinematic_ttl:        # reap kinematic agents whose feed went silent
                now = time.time()
                for a in arr:
                    if a.kinematic and (now - a.last_update) > self.kinematic_ttl:
                        self.despawn(a.id)
            self.t += dt
            self.frame += 1

    # ---- anti-clip separation (OBB minimum-translation-vector push-apart) ----
    def _separate(self, iters=8):
        """Resolve overlapping agent footprints by nudging each overlapping pair apart
        along the OBB minimum-translation vector (XZ plane). A small fixed budget of
        iterations per tick clears a dense stop-line queue in one tick and a tightly
        packed 2D block over a few ticks (the pushes propagate outward); we don't chase
        full convergence in a single tick because the next camera pose may move the cars
        anyway -- the contract is only that the pose handed to detect() / the renderer is
        overlap-free by the time the frame is read.

        Kinematic camera cars ARE included (the bug was that they collided with nothing).
        For them there is no velocity to correct, so we simply translate x/z. TRADEOFF:
        the detector may shove a kinematic car back into an overlap on its next pose
        update, but that pose is itself re-separated on the following tick, so every
        frame the viewer renders is clean. Physics agents already integrate their own
        motion; nudging their x/z directly (without touching velocity) just removes the
        interpenetration, identical to a positional contact-resolution step.

        Footprints use each agent's full half-extents (half[0]=L/2 along +X, half[2]=W/2
        along +Z) and yaw, via the same SAT math (obbObbXZ) the detector uses. Broad-phase
        is a uniform spatial hash so this stays cheap as the camera-car count grows; with
        only a handful of agents the bucketing degenerates to the trivial all-pairs case.
        """
        arr = list(self.agents.values())
        n = len(arr)
        if n < 2:
            return
        # Precompute per-agent local axes once per tick (yaw is fixed across iterations).
        axx = [(math.cos(a.yaw), -math.sin(a.yaw)) for a in arr]   # local +X in XZ
        axz = [(math.sin(a.yaw), math.cos(a.yaw)) for a in arr]    # local +Z in XZ
        # Bucket size = a generous footprint diagonal so any overlapping pair shares, or is
        # adjacent in, the grid. Cars whose centres are further apart than this can't touch.
        cell = 0.0
        for a in arr:
            cell = max(cell, a.half[0] + a.half[2])
        cell = max(cell * 2.0, 1.0)
        for _ in range(iters):
            # rebuild the hash each iteration since positions shift as we push
            buckets = {}
            for i, a in enumerate(arr):
                key = (int(a.x // cell), int(a.z // cell))
                buckets.setdefault(key, []).append(i)
            seen = set()
            pairs = []
            for (gx, gz), idxs in buckets.items():
                neigh = []
                for dgx in (-1, 0, 1):
                    for dgz in (-1, 0, 1):
                        neigh.extend(buckets.get((gx + dgx, gz + dgz), ()))
                for i in idxs:
                    for j in neigh:
                        if j <= i:
                            continue
                        pk = (i, j)
                        if pk in seen:
                            continue
                        seen.add(pk)
                        pairs.append(pk)
            moved = False
            for i, j in pairs:
                a, b = arr[i], arr[j]
                r = obbObbXZ(a.x, a.z, axx[i], axz[i], a.half[0], a.half[2],
                             b.x, b.z, axx[j], axz[j], b.half[0], b.half[2])
                if not r:
                    continue
                nx, nz, pen = r          # unit normal points from b toward a
                if pen <= 1e-9:
                    continue
                push = 0.5 * pen          # split the penetration 50/50 between the pair
                a.x += nx * push; a.z += nz * push
                b.x -= nx * push; b.z -= nz * push
                moved = True
            if not moved:
                break

    def snapshot(self):
        with self.lock:
            return {"t": round(self.t, 3), "frame": self.frame,
                    "agents": [a.state() for a in self.agents.values()]}

    def run(self, stop_evt):
        # The authoritative world thread is the one process that must NEVER die: every
        # connected viewer and script depends on it. tick() holds the lock and touches
        # network-derived agent state, so a stray exception (e.g. a NaN that slipped
        # past an input guard) would otherwise unwind out of here and silently freeze
        # the whole world. Log + keep stepping so a transient fault is survivable.
        dt = 1.0 / self.hz
        nxt = time.time()
        while not stop_evt.is_set():
            try:
                self.tick(dt)
            except Exception as e:  # noqa: BLE001 — never let the sim thread die
                print(f"[world] tick raised, skipped frame {self.frame}: {e!r}")
            nxt += dt
            slp = nxt - time.time()
            if slp > 0:
                time.sleep(slp)
            else:
                nxt = time.time()


WORLD = None   # set in main()
RENDER = None  # RenderService when --render is on
PROXY = None   # TransitProxy when the live bus proxy is on (default; --no-transit disables)
CAMERAS = None  # CameraProxy when the live traffic-camera proxy is on (default; --no-cameras disables)


class RenderService:
    """Headless-browser render service for first-person agent cameras.

    The server has no renderer, so we drive a headless Chromium that loads our own
    viewer (which already renders terrain imagery + buildings + the shared agents)
    and call window.__renderPOV() per request. Playwright's sync API is bound to one
    thread, so all renders are serialised through a queue to a single render thread;
    HTTP handlers submit a job and block on its result. Optional (only with --render).
    """

    def __init__(self, port):
        self.port = port
        self.q = queue.Queue()
        self.ready = threading.Event()
        self.error = None
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            from playwright.sync_api import sync_playwright
        except ImportError:
            self.error = ("playwright not installed — pip install playwright && "
                          "playwright install chromium")
            print("[render] " + self.error)
            return
        try:
            with sync_playwright() as p:
                browser = p.chromium.launch(headless=True,
                                            args=["--use-gl=angle", "--ignore-gpu-blocklist"])
                page = browser.new_page(viewport={"width": 640, "height": 480})
                page.goto(f"http://127.0.0.1:{self.port}/")
                page.wait_for_function(
                    "() => window.__renderPOV && window.__viewer && "
                    "window.__viewer.state.terrain.tiles.some(t => t.status === 'loaded')",
                    timeout=60000)
                page.wait_for_timeout(1500)   # let buildings + a few tiles stream in
                self.ready.set()
                print("[render] first-person camera service ready")
                while True:
                    job = self.q.get()
                    if job is None:
                        break
                    args, fut = job
                    try:
                        fut["data"] = page.evaluate(
                            "(a) => window.__renderPOV(a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8])", args)
                    except Exception as e:  # noqa: BLE001
                        fut["err"] = str(e)
                    fut["event"].set()
        except Exception as e:  # noqa: BLE001
            self.error = f"render service failed: {e}"
            print("[render] " + self.error)

    def render(self, args, timeout=8.0):
        if self.error:
            return None, self.error
        if not self.ready.is_set() and not self.ready.wait(timeout=timeout):
            return None, self.error or "render service still starting"
        fut = {"event": threading.Event()}
        self.q.put((args, fut))
        if not fut["event"].wait(timeout=timeout):
            return None, "render timeout"
        if fut.get("err"):
            return None, fut["err"]
        return fut.get("data"), None


# =====================================================================
# Live Lextran transit proxy (folded in from the former tools/serve.py).
# The upstream feed is plain HTTP with no CORS, so a browser on localhost can't read
# it directly; we fetch the GTFS-Realtime "debug" feeds (which serialise as JSON — no
# protobuf runtime needed), project every vehicle into scene metres with the same
# georef the road network uses, join each bus to its route colour/name from the baked
# web/data/transit.json, and re-serve it as compact same-origin JSON with a short cache.
FEED_BASE = "http://mystop.lextran.com/InfoPoint/GTFS-Realtime.ashx"
FEED_QUERY = {
    "vehicles": "?&Type=VehiclePosition&serverid=0&debug=true",
    "trips": "?&Type=TripUpdate&debug=true",
    "alerts": "?&Type=Alert&debug=true",
}
FIXTURE = {"vehicles": "VehiclePosition.json", "trips": "TripUpdate.json", "alerts": "Alert.json"}
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
    """Fetches, caches, and projects the three GTFS-Realtime feeds. Never raises into
    the viewer — on upstream failure it serves the last good payload (flagged stale)."""

    def __init__(self, projector, route_map, mock=False, cache_seconds=5.0):
        self.proj = projector
        self.routes = route_map            # routeId -> {shortName,color,longName}
        self.mock = mock
        self.cache_seconds = cache_seconds
        self._cache = {}                   # kind -> (fetched_at, payload)
        self._lock = threading.Lock()
        self._t0 = time.time()

    def _raw(self, kind):
        if self.mock:
            with open(os.path.join(SAMPLES, FIXTURE[kind]), "rb") as f:
                return json.loads(f.read())
        req = urllib.request.Request(FEED_BASE + FEED_QUERY[kind], headers=UA)
        with urllib.request.urlopen(req, timeout=20) as r:
            return json.loads(r.read())

    def get(self, kind):
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

    def _route_of(self, route_id):
        r = self.routes.get(str(route_id)) if route_id is not None else None
        if not r:
            return None
        return {"id": r["id"], "shortName": r["shortName"], "color": r["color"], "name": r["longName"]}

    def _mock_nudge(self, lat, lon, bearing, speed):
        """In mock mode, crawl each bus along its bearing so motion is visible (live mode
        uses real GTFS positions as-is). Parked campus buses report ~0.45 m/s, so floor
        the crawl speed to keep the offline demo/test lively and deterministic."""
        if not self.mock:
            return lat, lon
        eff = max(abs(speed or 0.0), 4.0)
        d = (eff * (time.time() - self._t0)) % 150.0
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
            bearing, speed = pos.get("Bearing"), pos.get("Speed")
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
        return {"ts": header.get("Timestamp") or int(time.time()), "count": len(out), "vehicles": out}

    def _build_trips(self, raw):
        by_stop, by_trip, header = {}, {}, raw.get("Header") or {}
        for e in raw.get("Entities") or []:
            tu = e.get("TripUpdate")
            if not tu:
                continue
            trip = tu.get("Trip") or {}
            rid, tid = trip.get("RouteId"), trip.get("TripId")
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
        return {"ts": header.get("Timestamp") or int(time.time()), "count": len(out), "alerts": out}

    def meta(self):
        with self._lock:
            ages = {k: round(time.time() - v[0], 1) for k, v in self._cache.items()}
        return {"mode": "mock" if self.mock else "live", "feedBase": FEED_BASE,
                "cacheSeconds": self.cache_seconds, "cacheAges": ages,
                "routesKnown": len(self.routes),
                "georef": {"A": self.proj.A, "B": self.proj.B, "utmZone": "16N"},
                "endpoints": ["/api/transit/vehicles", "/api/transit/trips",
                              "/api/transit/alerts", "/api/transit/meta"]}


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
                      "color": r.get("color") or "3b82c4", "longName": r.get("longName") or ""}
            for r in t.get("routes", [])}


def build_proxy(mock=False, cache_seconds=None):
    # short cache in mock so the synthetic crawl is continuous; a real feed only updates
    # every ~15-30 s, so a 5 s cache there is plenty and spares the agency.
    if cache_seconds is None:
        cache_seconds = 0.5 if mock else 5.0
    try:
        proj = Projector()
    except FileNotFoundError as e:
        # No georef manifest yet — e.g. a fresh clone, where web/data/ is gitignored and
        # regenerated from the tools/ pipeline. The scene<->UTM offsets live in that
        # manifest; without them buses can't be placed on the map, so transit degrades
        # OFF rather than crashing the server — mirroring ground->flat-plane and
        # buildings->no-collision above. Endpoints return 503; --render still comes up.
        print(f"[transit] no georef manifest ({e}); live buses off "
              "(regenerate web/data/ from the tools/ pipeline, or pass --no-transit to silence)")
        return None
    return TransitProxy(proj, load_route_map(), mock=mock, cache_seconds=cache_seconds)


# =====================================================================
# Live traffic-camera proxy.
#
# The static camera->intersection mapping is baked offline into web/data/cameras.json
# (tools/lex_cameras.py); the ONE thing that can't be baked is the stream URL, because
# the city re-signs every HLS playlist with a ~15-minute token. So, exactly like the
# transit proxy hands the viewer fresh bus positions, this hands it fresh, un-expired
# HLS URLs — scraped from the same public map, cached, and re-served same-origin. The
# browser then plays them directly with hls.js (the Wowza origin sends
# Access-Control-Allow-Origin:*, verified, so no segment proxying is needed).
CAMERA_MAP_URL = "https://trafficvid.lexingtonky.gov/publicmap/"


class CameraProxy:
    """Scrapes the city traffic-camera map for fresh tokenized HLS URLs and re-serves
    them same-origin. Never raises into the viewer — on a scrape failure it serves the
    last good URLs (flagged stale); the tokens last ~15 min, far longer than the cache,
    so a transient failure is invisible."""

    def __init__(self, cache_seconds=60.0, url=CAMERA_MAP_URL):
        self.url = url
        self.cache_seconds = cache_seconds
        self._lock = threading.Lock()
        self._cache = None          # (fetched_at, payload)

    def _scrape(self):
        req = urllib.request.Request(self.url, headers=UA)
        with urllib.request.urlopen(req, timeout=30) as r:
            html = r.read().decode("utf-8", "replace")
        html = html.replace("'", '"')
        i = html.find("camMarker")
        start = html.find("[", i) if i >= 0 else -1
        if start < 0:
            raise ValueError("camMarker array not found in city map HTML")
        # raw_decode tolerates nested arrays / a ']' inside strings that the old
        # find("]") slice would truncate on (CameraProxy.get() serves stale on raise).
        rows, _ = json.JSONDecoder().raw_decode(html[start:])
        cams = {}
        for c in rows:
            cid = c.get("camera")
            if not cid:
                continue
            cams[cid] = {"hls": c.get("hls"), "dash": c.get("dash"),
                         "still": c.get("still"), "status": c.get("status"),
                         "override": c.get("override")}
        return cams

    def get(self):
        now = time.time()
        with self._lock:
            if self._cache and now - self._cache[0] < self.cache_seconds:
                return self._cache[1]
        try:
            cams = self._scrape()
            payload = {"ts": int(now), "count": len(cams), "mode": "live", "cams": cams}
        except Exception as e:  # noqa: BLE001 — never 500 the viewer over a flaky scrape
            with self._lock:
                stale = self._cache[1] if self._cache else None
            payload = dict(stale) if stale else {"count": 0, "cams": {}}
            payload["error"] = str(e)
            payload["stale"] = bool(stale)
            return payload
        with self._lock:
            self._cache = (now, payload)
        return payload

    def meta(self):
        with self._lock:
            age = round(time.time() - self._cache[0], 1) if self._cache else None
            n = self._cache[1]["count"] if self._cache else 0
        return {"mode": "live", "mapUrl": self.url, "cacheSeconds": self.cache_seconds,
                "cacheAge": age, "cameras": n,
                "endpoints": ["/api/cameras/streams", "/api/cameras/meta"]}


def build_camera_proxy(cache_seconds=60.0):
    return CameraProxy(cache_seconds=cache_seconds)


# =====================================================================
# Human-readable reference for the agent spawn/control API, served at GET /docs.
# The agent-types table is generated from DEFS so it can never drift from the sim.
_DOCS_TEMPLATE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Lexington Twin — Agent API</title>
<style>
 :root{--bg:#0d1117;--panel:#161b22;--line:#272e3a;--ink:#e6edf3;--mut:#9aa7b4;
   --acc:#58a6ff;--get:#3fb950;--post:#d29922;--del:#f85149;--code:#0b0f14}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--ink);
   font:15px/1.6 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif}
 code,pre{font-family:"SF Mono",ui-monospace,"JetBrains Mono",Menlo,Consolas,monospace}
 a{color:var(--acc);text-decoration:none}a:hover{text-decoration:underline}
 .wrap{max-width:880px;margin:0 auto;padding:48px 24px 96px}
 h1{font-size:30px;margin:0 0 8px;letter-spacing:-.02em}
 header p{color:var(--mut);margin:0 0 6px;font-size:14.5px}
 .base{display:inline-block;margin-top:12px;padding:7px 13px;background:var(--panel);
   border:1px solid var(--line);border-radius:8px;color:var(--mut);font-size:13px}
 .base code{color:var(--ink)}
 h2{font-size:12px;text-transform:uppercase;letter-spacing:.09em;color:var(--mut);
   margin:46px 0 14px;padding-bottom:8px;border-bottom:1px solid var(--line)}
 .ep{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:15px 18px;margin:12px 0}
 .sig{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
 .verb{font:600 11px/1 ui-monospace,monospace;letter-spacing:.04em;padding:5px 8px;border-radius:6px;color:#0d1117}
 .verb.get{background:var(--get)}.verb.post{background:var(--post)}.verb.del{background:var(--del)}
 .path{font-family:ui-monospace,monospace;font-size:14px}
 .ep p{color:var(--mut);margin:11px 0 0;font-size:14px}
 pre{background:var(--code);border:1px solid var(--line);border-radius:8px;padding:13px 15px;
   overflow:auto;margin:12px 0 0;font-size:13px;color:#c9d6e3}
 table{width:100%;border-collapse:collapse;margin-top:4px;font-size:14px}
 th,td{text-align:left;padding:8px 10px;border-bottom:1px solid var(--line)}
 th{color:var(--mut);font-weight:600;font-size:11px;text-transform:uppercase;letter-spacing:.05em}
 td code{color:var(--acc)}
 .fields{display:grid;grid-template-columns:max-content 1fr;gap:5px 16px;margin-top:12px;font-size:13.5px}
 .fields code{color:var(--acc);white-space:nowrap}.fields span{color:var(--mut)}
 .note{color:var(--mut);font-size:13.5px}em{color:var(--ink);font-style:normal;font-weight:600}
</style></head><body><div class="wrap">
<header>
 <h1>Agent API</h1>
 <p>Spawn and drive agents in the authoritative, shared Lexington digital twin.</p>
 <p class="note">Every agent lives in <em>one</em> server-side simulation, so anything you spawn or move is
 visible to all connected clients — browsers and scripts alike. JSON in and out; CORS open.</p>
 <div class="base">Base URL&nbsp;&nbsp;<code>http://&lt;host&gt;:__PORT__</code> &nbsp;·&nbsp; a Python wrapper lives in <code>client/twin.py</code></div>
</header>

<h2>Quick start</h2>
<pre># spawn a car, send it to a point, read its sensors, then despawn it
curl -sX POST http://localhost:__PORT__/api/world/spawn -H 'content-type: application/json' \\
     -d '{"type":"car","position":[120,0,-80],"heading":90,"name":"scout"}'
# -> {"id":1,"name":"scout","type":"car","position":[120,0,-80],"heading":90, ...}

curl -sX POST http://localhost:__PORT__/api/world/agents/1/driveTo \\
     -H 'content-type: application/json' -d '{"x":260,"z":-40,"speed":12}'

curl -s  http://localhost:__PORT__/api/world/agents/1            # read state
curl -sX DELETE http://localhost:__PORT__/api/world/agents/1     # despawn</pre>

<h2>Agent types</h2>
<table><thead><tr><th>type</th><th>drive model</th><th>domain</th><th>L×W×H (m)</th><th>max speed (m/s)</th></tr></thead>
<tbody>
__TYPES_ROWS__
</tbody></table>
<p class="note">The <em>drive model</em> decides which control fields apply: <code>ackermann</code> = car-like
(throttle + steering), <code>differential</code> = tank/wheel pair, <code>holonomic</code> = free-flying drone.</p>

<h2>Spawn</h2>
<div class="ep">
 <div class="sig"><span class="verb post">POST</span><span class="path">/api/world/spawn</span></div>
 <p>Create an agent. Returns its full state, including the assigned <code>id</code>. 400 on an unknown
 <code>type</code>, a duplicate <code>name</code>, or when the agent cap (<code>/meta.maxAgents</code>) is reached.</p>
 <div class="fields">
  <code>type</code><span><b>required.</b> one of the agent types above.</span>
  <code>position</code><span>[x, y, z] scene metres (default [0,0,0]). Ground agents snap to terrain; pass [x, z] for short.</span>
  <code>heading</code><span>degrees, 0–360 (default 0).</span>
  <code>color</code><span>integer RGB, e.g. 3503049 (0x3577c9). Defaults per type.</span>
  <code>name</code><span>unique label (default "&lt;type&gt;_&lt;id&gt;").</span>
  <code>owner</code><span>free-form owner tag (default "anon").</span>
  <code>kinematic</code><span>true = pose-driven, no physics; auto-despawns after ~ttl s without a /pose update.</span>
  <code>source</code><span>optional metadata (e.g. a camera id for detection-driven cars).</span>
 </div>
 <pre>POST /api/world/spawn   {"type":"drone","position":[100,40,-60],"name":"eye"}
-> {"id":3,"type":"drone","position":[100,40,-60],"heading":0,"onGround":false, ...}</pre>
</div>

<h2>Control</h2>
<div class="ep">
 <div class="sig"><span class="verb post">POST</span><span class="path">/api/world/agents/{id}/controls</span></div>
 <p>Set the agent's instantaneous inputs. They persist until you change them, and they cancel any
 active <code>driveTo</code>. Fields depend on the drive model; out-of-range values are clamped.</p>
 <div class="fields">
  <code>ackermann</code><span>{ throttle 0..1, brake 0..1, steer -1..1, reverse bool }  — car, truck, bus, moto</span>
  <code>differential</code><span>{ throttle 0..1, steer -1..1 }  or  { left -1..1, right -1..1 }  — bike, ped, robot</span>
  <code>holonomic</code><span>{ move [x,y,z] each -1..1 }  or  { thrust 0..1, climb -1..1, yawRate deg/s }  — drone</span>
 </div>
 <pre>POST /api/world/agents/1/controls   {"throttle":0.8,"steer":-0.3}</pre>
</div>
<div class="ep">
 <div class="sig"><span class="verb post">POST</span><span class="path">/api/world/agents/{id}/driveTo</span></div>
 <p>Autopilot: steer + throttle toward a target. Overrides manual controls until reached or replaced.</p>
 <div class="fields">
  <code>x</code>, <code>z</code><span><b>required.</b> target in scene metres.</span>
  <code>y</code><span>target altitude for drones (optional).</span>
  <code>speed</code><span>cruise speed m/s (default 0.6 × the type's max speed).</span>
  <code>arriveRadius</code><span>distance at which it counts as arrived (default 2 m).</span>
  <code>stop</code><span>brake to a halt on arrival (default true).</span>
 </div>
 <pre>POST /api/world/agents/1/driveTo   {"x":260,"z":-40,"speed":12,"arriveRadius":3}</pre>
</div>
<div class="ep">
 <div class="sig"><span class="verb post">POST</span><span class="path">/api/world/agents/{id}/pose</span></div>
 <p>Set position/heading directly (teleport). Intended for <code>kinematic</code> agents, which carry no
 physics — e.g. cars driven from external tracking. Each call also resets the auto-despawn timer.</p>
 <div class="fields">
  <code>x</code>, <code>z</code><span><b>required.</b> scene metres.</span>
  <code>y</code><span>elevation (optional; ground agents otherwise snap to terrain).</span>
  <code>heading</code><span>degrees (optional).</span>
 </div>
 <pre>POST /api/world/agents/7/pose   {"x":131.2,"z":-58.9,"heading":210}</pre>
</div>
<div class="ep">
 <div class="sig"><span class="verb post">POST</span><span class="path">/api/world/agents/{id}/stop</span></div>
 <p>Zero the velocity and controls and clear any <code>driveTo</code> goal. No body.</p>
</div>

<h2>Read</h2>
<div class="ep">
 <div class="sig"><span class="verb get">GET</span><span class="path">/api/world/agents/{id}</span></div>
 <p>One agent's full state.</p>
 <pre>{
  "id":1,"name":"scout","type":"car","owner":"anon","color":3503049,
  "position":[120.0,0.0,-80.0],"heading":90.0,"forward":[0,0,-1],
  "velocity":[0,0,0],"speed":0.0,"surface":"road","groundY":0.0,
  "altitudeAGL":null,"onGround":true,"offMap":false,
  "utm":{"easting":...,"northing":...,"zone":"16N"},
  "collisions":[],"kinematic":false,"source":null
}</pre>
</div>
<div class="ep">
 <div class="sig"><span class="verb get">GET</span><span class="path">/api/world/state</span></div>
 <p>Snapshot of <em>every</em> agent in the shared world: <code>{ "t": &lt;sim seconds&gt;, "agents": [ &hellip; ] }</code>.</p>
</div>
<div class="ep">
 <div class="sig"><span class="verb get">GET</span><span class="path">/api/world/meta</span></div>
 <p>World info: <code>types</code>, sim rate <code>hz</code>, <code>maxAgents</code>, live <code>agents</code> count,
 <code>ground</code>/<code>buildings</code> status, first-person <code>camera</code> availability, and the UTM-16N <code>georef</code>.</p>
</div>
<div class="ep">
 <div class="sig"><span class="verb get">GET</span><span class="path">/api/world/agents/{id}/camera</span></div>
 <p>First-person JPEG from the agent's viewpoint (eye height + pitch per type). Requires the server to
 run with <code>--render</code> (a headless browser); otherwise 503.</p>
</div>

<h2>Remove</h2>
<div class="ep">
 <div class="sig"><span class="verb del">DELETE</span><span class="path">/api/world/agents/{id}</span></div>
 <p>Despawn the agent. Returns <code>{ "ok": true }</code> (false if it was already gone).</p>
</div>

<h2>Notes</h2>
<p class="note">
 <em>Coordinates</em> are scene metres: <code>+X</code> = east, <code>+Z</code> = south, <code>+Y</code> = up.
 Each agent state also carries its UTM-16N easting/northing (<code>state.utm</code>), and <code>/meta.georef</code>
 gives the scene↔UTM offsets.<br>
 <em>Authoritative &amp; shared</em> — the sim ticks at <code>/meta.hz</code> and agents are capped at
 <code>/meta.maxAgents</code>; this is distinct from a single browser's private <code>window.__twin.agents</code>.<br>
 <em>Kinematic agents</em> ignore physics and follow <code>/pose</code>; they auto-despawn after the
 kinematic TTL with no update. Everything else is fully simulated (ground snapping + collisions).
</p>
</div></body></html>"""


def agent_api_docs_html(port=None):
    """Render the agent spawn/control API reference. The types table comes from DEFS, so
    adding a vehicle class to the sim updates these docs automatically."""
    rows = []
    for t, d in DEFS.items():
        dom = "ground" if d.get("ground") else "air"
        size = f'{d["L"]:g}×{d["W"]:g}×{d["H"]:g}'
        rows.append(f'<tr><td><code>{t}</code></td><td>{d["kin"]}</td><td>{dom}</td>'
                    f'<td>{size}</td><td>{d["maxSpeed"]:g}</td></tr>')
    return (_DOCS_TEMPLATE
            .replace("__TYPES_ROWS__", "\n".join(rows))
            .replace("__PORT__", str(port if port is not None else 8000)))


# =====================================================================
class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def end_headers(self):
        # Dev caching policy: never cache the viewer's static assets, so live edits to
        # web/*.js / *.html show up on a normal reload. ES modules cache aggressively, which
        # otherwise pins the page to a stale build (e.g. flat-mode/drape edits not taking
        # effect). API / tile / JPEG responses set their own Cache-Control, so skip those.
        try:
            if not self.path.startswith("/api/"):
                self.send_header("Cache-Control", "no-store")
        except Exception:
            pass
        super().end_headers()

    def _json(self, obj, status=200):
        # allow_nan=False: a stray NaN/inf would serialize to a bare `NaN` token that
        # browser JSON.parse rejects, silently breaking the viewer's poll. Refuse it
        # here (and never throw out of the handler) rather than emit invalid JSON.
        try:
            body = json.dumps(obj, allow_nan=False).encode("utf-8")
        except (ValueError, TypeError):
            body = b'{"error":"non-serializable response"}'
            status = 500
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _html(self, html, status=200):
        body = html.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        # Cap the body size: every endpoint here takes a small JSON object (spawn,
        # controls, pose, calib, detections) — a multi-MB POST is never legitimate,
        # and an uncapped Content-Length would let a client force the server to buffer
        # an arbitrary blob (OOM). Reject overflows with 413 before allocating.
        n = int(self.headers.get("Content-Length") or 0)
        if not n:
            return {}
        if n > MAX_BODY_BYTES:
            raise BodyTooLarge()
        try:
            # parse_constant rejects the NaN/Infinity/-Infinity tokens that the JSON
            # spec forbids but Python's json.loads ACCEPTS by default. Those would
            # otherwise sail past finite() at the value level and poison the sim.
            return json.loads(self.rfile.read(n) or b"{}",
                              parse_constant=_reject_constant)
        except BodyTooLarge:
            raise
        except Exception:
            return {}

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def _agent_id(self, path):
        # /api/world/agents/<id>[/action]
        parts = path.split("/")
        try:
            return int(parts[4]), (parts[5] if len(parts) > 5 else None)
        except (IndexError, ValueError):
            return None, None

    def _transit(self, path):
        if PROXY is None:
            return self._json({"error": "transit proxy off (no georef manifest, or --no-transit)"}, 503)
        if path in ("/api/transit", "/api/transit/meta"):
            return self._json(PROXY.meta())
        for kind in ("vehicles", "trips", "alerts"):
            if path == "/api/transit/" + kind:
                return self._json(PROXY.get(kind))
        return self._json({"error": "unknown endpoint", "path": self.path}, 404)

    def _cameras(self, path):
        if CAMERAS is None:
            return self._json({"error": "camera proxy off (started with --no-cameras)"}, 503)
        if path in ("/api/cameras", "/api/cameras/meta"):
            return self._json(CAMERAS.meta())
        if path == "/api/cameras/streams":
            return self._json(CAMERAS.get())
        return self._json({"error": "unknown endpoint", "path": self.path}, 404)

    def _send_bytes(self, body, status=200, ctype="application/octet-stream", cached=False):
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("X-Tile-Cache", "hit" if cached else "miss")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _gtile(self):
        """Operational disk cache for Google Photorealistic 3D Tiles. The viewer routes
        tile-content fetches here (preprocessURL in tiles3d.js) so repeated local sessions
        reuse already-downloaded tiles instead of re-fetching. Strictly a temporary
        performance cache for googleapis.com ONLY — NOT a redistributable offline mirror;
        respect Google Maps Platform terms. root.json is never cached (keeps the session
        fresh); the cache key drops the volatile key/session params so it hits across
        sessions."""
        q = parse_qs(urlparse(self.path).query)
        u = (q.get("u") or [None])[0]
        if not u:
            return self._send_bytes(b"missing u", 400, "text/plain")
        p = urlparse(u)
        # SSRF defense: every user-derived part of the request URL is allow-listed before
        # use, then the URL is rebuilt on a HARDCODED scheme+host. The re.fullmatch() guards
        # are both the security check and the taint barrier: the path must be a Google tile
        # path and the query may contain only url-safe chars (no ':', '/', '@'), so neither
        # the host, the path, nor the query can redirect the request off Google's tile host.
        if (p.scheme != "https" or p.hostname != "tile.googleapis.com"
                or re.fullmatch(r"/v1/3dtiles/[\w./-]+", p.path) is None
                or re.fullmatch(r"[\w.=&%+-]*", p.query) is None):
            return self._send_bytes(b"forbidden", 403, "text/plain")
        fetch_url = "https://tile.googleapis.com" + p.path + (("?" + p.query) if p.query else "")

        qd = parse_qs(p.query)
        qd.pop("key", None)
        qd.pop("session", None)
        sig = p.path + "?" + urlencode(sorted(qd.items()), doseq=True)
        cacheable = not p.path.endswith("/root.json")
        base = os.path.join(TILECACHE, hashlib.sha1(sig.encode("utf-8")).hexdigest())

        if cacheable:
            try:
                age = time.time() - os.path.getmtime(base)
                if age < TILECACHE_TTL:
                    with open(base, "rb") as f:
                        body = f.read()
                    ctype = "application/octet-stream"
                    if os.path.exists(base + ".ct"):
                        with open(base + ".ct") as f:
                            ctype = f.read().strip() or ctype
                    return self._send_bytes(body, 200, ctype, cached=True)
            except OSError:
                pass

        try:
            # fetch_url has a constant, validated scheme+host (built above) — not raw `u`.
            req = urllib.request.Request(fetch_url, headers={"User-Agent": "uky-twin/1.0"})
            with urllib.request.urlopen(req, timeout=30) as r:
                body = r.read()
                ctype = r.headers.get("Content-Type", "application/octet-stream")
        except Exception as e:  # noqa: BLE001 - surface any fetch failure to the client
            return self._send_bytes(("tile fetch failed: " + str(e)).encode(), 502, "text/plain")

        if cacheable:
            try:
                os.makedirs(TILECACHE, exist_ok=True)
                with open(base, "wb") as f:
                    f.write(body)
                with open(base + ".ct", "w") as f:
                    f.write(ctype)
            except OSError:
                pass
            _maybe_trim_tilecache()   # throttle-guarded; never throws
        return self._send_bytes(body, 200, ctype, cached=False)

    def do_GET(self):
        path = self.path.split("?", 1)[0].rstrip("/")
        if path == "/docs":           # agent spawn/control API reference (human-readable)
            return self._html(agent_api_docs_html(self.server.server_address[1]))
        if path == "/api/photoreal":
            # Photorealistic-basemap key, supplied via the GOOGLE_MAPS_API_KEY env var (or
            # a gitignored .env) so it never lives in the repo. Returns {"key": null} when
            # unset (no 404 noise); the viewer falls back to its in-browser key entry. Set
            # PHOTOREAL_PROVIDER=ion to use a Cesium ion token instead. `cache:true` tells
            # the viewer it can route tile fetches through the on-disk cache proxy below.
            # Tiles are always streamed LIVE from the Map Tiles API (no offline copy); the
            # /api/gtile proxy is just a bounded, transient performance cache.
            return self._json({
                "key": os.environ.get("GOOGLE_MAPS_API_KEY")
                or os.environ.get("PHOTOREAL_KEY") or None,
                "provider": os.environ.get("PHOTOREAL_PROVIDER", "google"),
                "cache": True,
            })
        if path == "/api/gtile":
            return self._gtile()
        if path.startswith("/api/transit"):
            return self._transit(path)
        if path == "/api/cameras/calib":     # works without the live camera proxy
            return self._json(load_calib())
        if path == "/api/cameras/detections":
            return self._json(self._get_detections())
        if path == "/api/cameras/active":
            return self._json(self._get_active())
        if path == "/api/cameras/detect":
            cam = parse_qs(urlparse(self.path).query).get("camera", [None])[0]
            return self._json(detector_status(cam))
        if path.startswith("/api/cameras"):
            return self._cameras(path)
        if path.startswith("/api/world"):
            if WORLD is None:
                return self._json({"error": "world not running"}, 503)
            if path == "/api/world/state":
                return self._json(WORLD.snapshot())
            if path == "/api/world/nearest_building":
                q = parse_qs(urlparse(self.path).query)
                try:
                    x = float(q.get("x", [0])[0]); z = float(q.get("z", [0])[0])
                except (TypeError, ValueError):
                    return self._json({"error": "x and z must be numbers"}, 400)
                return self._json(WORLD.buildings.nearest(x, z) or {})
            if path == "/api/world/meta":
                return self._json({
                    "types": list(DEFS), "hz": WORLD.hz, "maxAgents": WORLD.max_agents,
                    "agents": len(WORLD.agents),   # live count, so cap pressure is visible
                    "ground": WORLD.ground.ok, "buildings": len(WORLD.buildings.items),
                    "camera": RENDER is not None and RENDER.error is None,
                    "cameraReady": RENDER is not None and RENDER.ready.is_set(),
                    "transit": ("mock" if PROXY.mock else "live") if PROXY else None,
                    "georef": {"A": WORLD.ground.A, "B": WORLD.ground.B, "zone": "16N"} if WORLD.ground.ok else None,
                })
            if path.startswith("/api/world/agents"):
                aid, action = self._agent_id(path)
                # Hold the world lock across lookup + state read: the sim thread mutates
                # agent fields every tick, so reading them unlocked (the old code) is a
                # data race that can tear a pose mid-update. For the camera action we
                # capture the pose under the lock, then release before the blocking
                # RENDER call (don't hold the lock across a headless-browser round-trip).
                with WORLD.lock:
                    a = WORLD.get(aid) if aid is not None else None
                    if not a:
                        return self._json({"error": "no such agent"}, 404)
                    if action == "camera":
                        cam = a.camera_pose()
                    else:
                        st = a.state()
                if action == "camera":
                    return self._send_camera(a, cam)
                return self._json(st)
            return self._json({"error": "unknown endpoint", "path": self.path}, 404)
        return super().do_GET()

    def _send_camera(self, a, cam=None):
        if RENDER is None:
            return self._json({"error": "camera feed off — start the server with --render"}, 503)
        q = parse_qs(urlparse(self.path).query)
        try:
            w = int(float(q.get("w", [320])[0])); h = int(float(q.get("h", [240])[0]))
        except (TypeError, ValueError):
            return self._json({"error": "w and h must be numbers"}, 400)
        # cam (eye/forward) is captured under the world lock by the caller; only fall
        # back to an unlocked read if called directly (defensive — no current caller).
        (ex, ey, ez), (fx, fy, fz) = cam if cam is not None else a.camera_pose()
        data, err = RENDER.render([a.id, ex, ey, ez, fx, fy, fz, w, h])
        if err or not data:
            return self._json({"error": err or "no frame"}, 503)
        try:
            img = base64.b64decode(data.split(",", 1)[1])
        except Exception:
            return self._json({"error": "bad frame data"}, 502)
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(img)))
        self.end_headers()
        self.wfile.write(img)

    def _calib_post(self, b):
        """Upsert one (cameraId, quad) homography into calibration/cameras.json, or
        replace the whole document with {full:{...}}."""
        if isinstance(b.get("full"), dict):
            full = b["full"]
            # The `full` branch overwrites the git-tracked calibration/cameras.json
            # verbatim, so sanity-check the shape before clobbering authored config:
            # it must be a dict with a 'cameras' dict. Rejects an accidental {} or a
            # malformed payload that would otherwise wipe everyone's calibration.
            if not isinstance(full.get("cameras"), dict):
                return self._json({"error": "full must have a 'cameras' object"}, 400)
            save_calib(full)
            return self._json({"ok": True, "mode": "full"})
        cid, quad = b.get("cameraId"), b.get("quad")
        if not cid or quad is None:
            return self._json({"error": "need cameraId and quad (or full)"}, 400)
        # Hold the lock across the whole read-modify-write so two concurrent calib POSTs
        # can't both load the same baseline and clobber each other's quad (lost update).
        # CALIB_LOCK is reentrant, so save_calib re-acquiring it inside is safe.
        with CALIB_LOCK:
            calib = load_calib()
            cam = calib.setdefault("cameras", {}).setdefault(str(cid), {})
            if b.get("intersection") is not None:
                cam["intersection"] = b["intersection"]
            # keep reprojError so a quad's calibration quality survives reload (triage)
            entry = {k: b[k] for k in ("covers", "imgW", "imgH", "H", "points", "reprojError") if k in b}
            cam.setdefault("quads", {})[str(quad)] = entry
            save_calib(calib)
        return self._json({"ok": True, "cameraId": cid, "quad": quad})

    # ---- live detection relay + active-camera signal (Phase 3) ----
    def _get_detections(self):
        cam = parse_qs(urlparse(self.path).query).get("camera", [None])[0]
        now = time.time()
        with DET_LOCK:
            rec = DETECTIONS.get(cam) if cam else None
        if not rec or (now - rec["ts"]) > DET_TTL:
            return {"camera": cam, "dets": [], "stale": True}
        return {"camera": cam, "ts": rec["ts"], "age": round(now - rec["ts"], 2),
                "frame": rec.get("frame"), "dets": rec["dets"]}

    def _post_detections(self, b):
        cam = b.get("camera")
        if not cam:
            return self._json({"error": "need camera"}, 400)
        now = time.time()
        with DET_LOCK:
            DETECTIONS[str(cam)] = {"ts": now, "frame": b.get("frame"),
                                    "dets": b.get("dets") or []}
            # sweep stale entries so DETECTIONS can't grow unbounded over a long run
            # (arbitrary camera ids POSTed over time would otherwise accumulate keys).
            for k in [k for k, v in DETECTIONS.items() if now - v["ts"] > DET_TTL]:
                del DETECTIONS[k]
        return self._json({"ok": True, "camera": cam, "count": len(b.get("dets") or [])})

    def _get_active(self):
        # Per-camera signal: report whether THE queried camera has a live viewer, so
        # multiple viewers/detectors on different cameras don't alias one global slot.
        cam = parse_qs(urlparse(self.path).query).get("camera", [None])[0]
        now = time.time()
        with DET_LOCK:
            for k in [k for k, ts in ACTIVE.items() if now - ts > ACTIVE_TTL]:
                del ACTIVE[k]   # prune expired viewers
            if cam is not None:
                ts = ACTIVE.get(cam)
                return {"camera": cam, "active": ts is not None,
                        "age": round(now - ts, 2) if ts is not None else None}
            # no camera specified -> the most-recently-active camera (back-compat)
            latest = max(ACTIVE.items(), key=lambda kv: kv[1], default=(None, 0.0))
            return {"camera": latest[0], "active": latest[0] is not None}

    def _post_active(self, b):
        cam = b.get("camera")
        with DET_LOCK:
            if cam is None:
                ACTIVE.clear()              # explicit "no camera viewed" clears all
            elif b.get("active") is False:  # this viewer stopped watching THIS camera
                ACTIVE.pop(str(cam), None)
            else:
                ACTIVE[str(cam)] = time.time()  # viewing / heartbeat
        return self._json({"ok": True, "camera": cam})

    def do_POST(self):
        try:
            return self._do_post()
        except BodyTooLarge:
            return self._json({"error": "body too large"}, 413)

    def _do_post(self):
        path = self.path.split("?", 1)[0].rstrip("/")
        if path == "/api/photoreal":
            # Persist a key entered in the web UI to the gitignored .env so it survives
            # restarts (and is reused without re-prompting). Local dev convenience.
            b = self._body()
            k = (b.get("key") or "").strip()
            if not k:
                return self._json({"error": "need key"}, 400)
            try:
                save_key_to_env(k)
                os.environ["GOOGLE_MAPS_API_KEY"] = k
                if b.get("provider"):
                    save_key_to_env(b["provider"], var="PHOTOREAL_PROVIDER")
                    os.environ["PHOTOREAL_PROVIDER"] = b["provider"]
                return self._json({"ok": True, "persisted": ".env"})
            except OSError as e:
                return self._json({"error": "could not write .env: " + str(e)}, 500)
        if path == "/api/cameras/calib":
            return self._calib_post(self._body())
        if path == "/api/cameras/detections":
            return self._post_detections(self._body())
        if path == "/api/cameras/active":
            return self._post_active(self._body())
        if path == "/api/cameras/detect":     # start/stop the in-process YOLO detector
            b = self._body(); cam = b.get("camera")
            if not cam:
                return self._json({"error": "need camera"}, 400)
            # analysis=true -> raw image-space detector for the PiP Analysis bootstrap (runs
            # on an uncalibrated camera, alongside any normal detector for the same camera).
            ana = bool(b.get("analysis"))
            return self._json(start_detector(cam, analysis=ana) if b.get("on")
                              else stop_detector(cam, analysis=ana))
        if WORLD is None and path.startswith("/api/world"):
            return self._json({"error": "world not running"}, 503)
        if path == "/api/world/spawn":
            try:
                a = WORLD.spawn(self._body())
                return self._json(a.state())
            except (ValueError, RuntimeError) as e:
                return self._json({"error": str(e)}, 400)
        if path.startswith("/api/world/agents"):
            aid, action = self._agent_id(path)
            b = self._body()
            # Validate the action-specific payload BEFORE taking the world lock, so a
            # malformed body returns a clean 400 (matching /pose and /spawn) instead of
            # raising KeyError/ValueError out of the mutator, which would close the
            # connection without an HTTP response.
            if action in ("driveTo", "pose"):
                if b.get("x") is None or b.get("z") is None:
                    return self._json({"error": f"{action} needs x and z"}, 400)
                if not finite(b["x"], b["z"]):
                    return self._json({"error": f"{action} x and z must be finite"}, 400)
            elif action not in ("controls", "stop", None):
                return self._json({"error": "unknown action"}, 404)
            # Hold the world lock across lookup+mutate so a pose/controls update can't
            # interleave mid-tick (integrate -> snap_ground) and tear an agent's state.
            # WORLD.lock is reentrant, so the mutators acquiring nothing extra is fine.
            try:
                with WORLD.lock:
                    a = WORLD.get(aid) if aid is not None else None
                    if not a:
                        return self._json({"error": "no such agent"}, 404)
                    if action == "controls":
                        a.set_controls(b)
                    elif action == "driveTo":
                        a.drive_to(b)
                    elif action == "stop":
                        a.stop()
                    elif action == "pose":
                        a.set_pose(b["x"], b["z"], b.get("y"), b.get("heading"))
            except (KeyError, ValueError, TypeError) as e:
                return self._json({"error": "bad body: " + str(e)}, 400)
            return self._json({"ok": True})
        return self._json({"error": "unknown endpoint"}, 404)

    def do_DELETE(self):
        path = self.path.split("?", 1)[0].rstrip("/")
        if WORLD is None and path.startswith("/api/world"):
            return self._json({"error": "world not running"}, 503)
        if path.startswith("/api/world/agents"):
            aid, _ = self._agent_id(path)
            return self._json({"ok": WORLD.despawn(aid) if aid is not None else False})
        return self._json({"error": "unknown endpoint"}, 404)

    def copyfile(self, source, outputfile):
        # the viewer drops connections mid-stream while tiles/chunks stream in; swallow
        # the reset rather than dumping a traceback (same as the old serve.py).
        try:
            super().copyfile(source, outputfile)
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
            pass


def _lan_ip():
    """Best-effort primary LAN IPv4 — the address other devices use to reach us.
    Opens a UDP socket toward a public IP and reads the chosen source address; no
    packets are actually sent. Returns None if it can't be determined (e.g. offline)."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


def make_server(port, directory=WEB, host="0.0.0.0"):
    """ThreadingHTTPServer on `host:port` with the combined world+transit+static Handler.
    `host` defaults to 0.0.0.0 (all interfaces) so the viewer is reachable from other
    devices on the LAN; pass 127.0.0.1 for localhost-only. The listen socket is open on
    return, so a --render browser can connect before serve_forever() starts accepting.
    Shared by main() and tools/verify_transit.py."""
    handler = functools.partial(Handler, directory=directory)
    httpd = http.server.ThreadingHTTPServer((host, port), handler)
    httpd.daemon_threads = True
    return httpd


def world_data_present():
    """True once the full city is built. The single-command bootstrap is meant to produce
    ALL of Lexington, so gate on three reliable markers — manifest.json (georef + terrain),
    ground.f32 (KyFromAbove citywide elevation), and buildings.pack.json (packed buildings).
    If any is missing — e.g. only a partial campus base exists (manifest but no citywide
    LiDAR/buildings) — the bootstrap offers to build the rest. Best-effort layers (cameras/
    transit) are deliberately NOT gated on, so a flaky scrape can't cause a reprompt loop."""
    return all(os.path.exists(os.path.join(DATA, f))
               for f in ("manifest.json", "ground.f32", "buildings.pack.json"))


def maybe_bootstrap_world_data(mode="prompt"):
    """Populate an empty web/data/ on first run.

    web/data/ is gitignored and built locally, so a fresh clone has no world. The georef
    anchor comes from this box's UE assets (no public download substitutes), and every
    other layer reads that manifest — so this runs `build_all.py --citywide`, which after
    the campus georef base downloads ALL of Lexington: ~114k buildings, roads, traffic
    lights, intersections, crosswalks, cameras, buses, and the ~8 GB KYAPED LiDAR.

    mode: 'prompt' (ask, but only when stdin is a TTY — a headless --render must never
    block on input()), 'yes' (build without asking, for scripted setup), or 'off' (never
    build, just print guidance). Returns True iff the world data is present afterward.
    """
    if world_data_present():
        return True

    build = [sys.executable, os.path.join(_REPO_ROOT, "tools", "build_all.py"), "--citywide"]
    shown = "python tools/build_all.py --citywide"

    def _guidance():
        print(f"  [bootstrap] no web/data/ yet (gitignored, built locally). This box has the UE "
              f"assets, so build the whole city with:\n                {shown}")
        print("              downloads all of Lexington — buildings, roads, traffic lights, "
              "intersections,\n              crosswalks, cameras, buses + the ~8 GB KYAPED LiDAR "
              "(needs laspy[lazrs]/shapely/scikit-image),\n              then restart the server.")

    if mode == "off":
        _guidance()
        return False

    interactive = bool(getattr(sys.stdin, "isatty", lambda: False)())
    if mode == "prompt" and not interactive:
        # Non-TTY guard: never hang a headless/automated run waiting on a prompt.
        print("  [bootstrap] world data missing and no interactive terminal to prompt on — skipping.")
        _guidance()
        return False

    if mode == "prompt":
        print("\n  World data is missing or incomplete (web/data/ is gitignored; built locally).")
        print(f"  This box has the UE assets, so I can build ALL of Lexington now:  {shown}")
        print("  Downloads everything: terrain + ~114k buildings, roads, traffic lights,")
        print("  intersections, crosswalks, cameras, and buses — plus the ~8 GB KYAPED LiDAR.")
        print("  Big job: needs network + laspy[lazrs]/shapely/scikit-image, and can take a while.")
        try:
            ans = input("  Build it now? [y/N] ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            ans = ""
        if ans not in ("y", "yes"):
            print("  Skipping the build — serving a flat/empty world for now.")
            _guidance()
            return False

    print(f"\n  [bootstrap] running: {shown}")
    print("  (this can take a while — output streams below)\n")
    try:
        subprocess.run(build, cwd=_REPO_ROOT, check=True)
    except (subprocess.CalledProcessError, OSError) as e:
        print(f"\n  [bootstrap] build failed ({e}) — serving a flat/empty world.")
        _guidance()
        return False

    if world_data_present():
        print("\n  [bootstrap] world data ready.\n")
        return True
    print("\n  [bootstrap] build finished but the city still looks incomplete "
          "(need manifest.json + ground.f32 + buildings.pack.json) — check the output above.\n")
    return False


def main():
    global WORLD, RENDER, PROXY, CAMERAS, DETECT_MODEL, DETECT_MAX_FPS, DETECT_INTERVAL, DETECT_CONF, SERVER_BASE
    load_dotenv()   # pick up GOOGLE_MAPS_API_KEY etc. from a gitignored .env
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--host", default="0.0.0.0",
                    help="interface to bind (default 0.0.0.0 = all, reachable on the LAN; "
                         "use 127.0.0.1 for localhost-only)")
    ap.add_argument("--hz", type=int, default=50, help="simulation tick rate")
    ap.add_argument("--render", action="store_true",
                    help="enable first-person agent cameras (drives a headless browser; needs playwright)")
    ap.add_argument("--bootstrap", action="store_true",
                    help="if web/data/ is empty, build it from the UE assets (tools/build_all.py "
                         "--with-city --with-transit) WITHOUT prompting — for scripted/headless setup")
    ap.add_argument("--no-bootstrap", action="store_true",
                    help="never offer to build a missing web/data/ (default: prompt when run in a terminal)")
    ap.add_argument("--mock", action="store_true",
                    help="replay tools/_transit_samples instead of the live Lextran feed (offline buses)")
    ap.add_argument("--no-transit", action="store_true", help="disable the live bus proxy")
    ap.add_argument("--transit-cache-seconds", type=float, default=None,
                    help="seconds to cache each transit feed (default 5 s live / 0.5 s mock)")
    ap.add_argument("--no-cameras", action="store_true",
                    help="disable the live traffic-camera URL proxy")
    ap.add_argument("--camera-cache-seconds", type=float, default=60.0,
                    help="seconds to cache the scraped camera URLs (tokens last ~900 s; default 60)")
    ap.add_argument("--kinematic-ttl", type=float, default=5.0,
                    help="despawn kinematic agents (e.g. camera cars) after this many seconds "
                         "without a pose update (default 5; 0 disables the sweep)")
    ap.add_argument("--detect-model", default="yolo26x.pt",
                    help="YOLO weights for in-process per-camera detection (POST /api/cameras/detect). "
                         "Default yolo26x (largest/most accurate); needs ultralytics+opencv in the server's venv.")
    ap.add_argument("--detect-max-fps", type=float, default=8.0,
                    help="legacy per-camera pace cap (used only when --detect-interval < 0)")
    ap.add_argument("--detect-interval", type=float, default=0.0,
                    help="min seconds between YOLO inferences per camera (default 0 = run on the "
                         "freshest frame at model speed; stale frames are dropped so the twin never "
                         "lags). Set e.g. 1.0 to cap GPU load.")
    ap.add_argument("--detect-conf", type=float, default=0.30,
                    help="detection confidence threshold")
    args = ap.parse_args()
    DETECT_MODEL = args.detect_model
    DETECT_MAX_FPS = args.detect_max_fps
    DETECT_INTERVAL = args.detect_interval
    DETECT_CONF = args.detect_conf
    SERVER_BASE = f"http://127.0.0.1:{args.port}"   # in-process detector talks to ourselves

    # First-run bootstrap: an empty web/data/ (fresh clone) can't be served meaningfully —
    # offer to build it from the UE assets before the world loads, so World() picks it up.
    boot_mode = "off" if args.no_bootstrap else ("yes" if args.bootstrap else "prompt")
    maybe_bootstrap_world_data(boot_mode)

    print("loading world (terrain heightmap + building boxes) ...")
    WORLD = World(hz=args.hz)
    WORLD.kinematic_ttl = args.kinematic_ttl
    stop_evt = threading.Event()
    threading.Thread(target=WORLD.run, args=(stop_evt,), daemon=True).start()

    if not args.no_transit:
        PROXY = build_proxy(mock=args.mock, cache_seconds=args.transit_cache_seconds)

    if not args.no_cameras:
        CAMERAS = build_camera_proxy(cache_seconds=args.camera_cache_seconds)

    httpd = make_server(args.port, host=args.host)
    if args.render:
        RENDER = RenderService(args.port)

    transit = ("off (--no-transit)" if args.no_transit else
               "off (no georef manifest)" if PROXY is None else
               f"{'MOCK (fixtures)' if args.mock else 'LIVE (mystop.lextran.com)'}"
               "  ->  /api/transit/(vehicles|trips|alerts|meta)")
    cameras = ("off (--no-cameras)" if args.no_cameras else
               "LIVE (trafficvid.lexingtonky.gov)  ->  /api/cameras/(streams|meta)")
    if args.host in ("0.0.0.0", "::"):
        lan = _lan_ip()
    elif args.host in ("127.0.0.1", "localhost", "::1"):
        lan = None
    else:
        lan = args.host
    print(f"\nLexington Digital Twin server — authoritative shared world + live transit + traffic cameras")
    print(f"  viewer:  http://localhost:{args.port}/")
    if lan:
        print(f"           http://{lan}:{args.port}/   (LAN — reachable from other devices; bound on {args.host})")
    print(f"  world:   http://localhost:{args.port}/api/world/(state|meta|spawn|agents/<id>/...)")
    print(f"  docs:    http://localhost:{args.port}/docs   (agent spawn/control API reference)")
    print(f"  transit: {transit}")
    print(f"  cameras: {cameras}")
    print(f"  sim:     {args.hz} Hz   ground:{'on' if WORLD.ground.ok else 'flat'}   "
          f"buildings:{len(WORLD.buildings.items)}")
    print(f"  camera:  {'ON (first-person /agents/<id>/camera; starting headless renderer…)' if args.render else 'off (use --render)'}")
    print(f"  detect:  {'ready' if _have_detector_deps() else 'deps missing (ultralytics+opencv) — install in this venv to enable'}"
          f"  ->  POST /api/cameras/detect (per-camera YOLO, model {DETECT_MODEL}); start it from the camera PiP")
    print("  Ctrl-C to stop")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")
        stop_all_detectors()
        stop_evt.set()
        httpd.shutdown()


if __name__ == "__main__":
    main()
