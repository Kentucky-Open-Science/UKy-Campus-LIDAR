#!/usr/bin/env python
"""Authoritative multiplayer server for the UKy campus digital twin.

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
    GET    /api/transit/vehicles             -> live bus positions, projected to scene [x,_,z]
    GET    /api/transit/trips                -> predicted arrivals, indexed by stop and trip
    GET    /api/transit/alerts               -> service alerts (decoded cause/effect)
    GET    /api/transit/meta                 -> transit proxy status (mode, cache ages, georef)
    GET    /api/cameras/streams             -> fresh tokenized HLS URLs per camera id
    GET    /api/cameras/meta                -> camera proxy status (mode, cache age, count)
    (anything else) -> served as a static file from web/

Run:  python -m tools.twin_server [--port 8000] [--hz 50] [--render]
      python -m tools.twin_server --mock        # replay tools/_transit_samples (offline buses)
      python -m tools.twin_server --no-transit   # world only, no bus proxy

Transit data (c) Lextran (Transit Authority of Lexington).
"""
import argparse
import base64
import functools
import http.server
import json
import math
import os
import queue
import threading
import time
import urllib.request
from urllib.parse import urlparse, parse_qs

import numpy as np

from tools.extract_roads import DATA, load_mesh, build_heightmap
from tools.transit_common import Projector

SAMPLES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_transit_samples")

_HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.abspath(os.path.join(_HERE, "..", "web"))
# Camera->scene homography calibration: authored config (NOT pipeline output), so it
# lives OUTSIDE the gitignored web/data/ and is version-controlled (constitution P1).
CALIB_PATH = os.path.abspath(os.path.join(_HERE, "..", "calibration", "cameras.json"))
CALIB_LOCK = threading.Lock()


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

# ---- per-type kinematic defs (scene metres), ported from web/agents.js TYPES ----
DEFS = {
    "car":   dict(L=4.3, W=1.9, H=1.45, wheelbase=2.6, kin="ackermann", ground=True,
                  maxSpeed=25, maxAccel=6, maxSteerDeg=35),
    "truck": dict(L=8.5, W=2.5, H=3.2, wheelbase=5.0, kin="ackermann", ground=True,
                  maxSpeed=18, maxAccel=4, maxSteerDeg=28),
    "robot": dict(L=0.8, W=0.6, H=0.6, trackWidth=0.5, kin="differential", ground=True,
                  maxSpeed=3, maxAccel=4),
    "drone": dict(L=0.9, W=0.9, H=0.35, kin="holonomic", ground=False,
                  maxSpeed=15, maxAccel=10, maxClimb=6, maxYawRateDeg=120, minClearance=0.5),
}
DEFAULT_COLORS = {"car": 0x3577c9, "truck": 0xc7702a, "robot": 0x4aa05a, "drone": 0x9b59b6}
# first-person camera mount per type: eye height above the agent (m) + downward pitch (deg)
CAM_EYE = {"car": 1.4, "truck": 2.6, "robot": 0.5, "drone": 0.0}
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
        self.x = float(pos[0]) if pos else 0.0
        self.z = float(pos[2]) if pos and len(pos) > 2 else (float(pos[1]) if pos else 0.0)
        self.yaw = (opts.get("heading", 0) or 0) * D2R
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
        self.goal = {
            "x": float(g["x"]),
            "y": (float(g["y"]) if g.get("y") is not None else None),
            "z": float(g["z"]),
            "speed": float(g.get("speed", 0.6 * self.maxSpeed)),
            "arriveRadius": float(g.get("arriveRadius", 2.0)),
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
                out[k] = clamp(float(c[k]), lo, hi)
        if c.get("yawRate") is not None:
            out["yawRate"] = float(c["yawRate"])
        for b in ("reverse", "handbrake"):
            if c.get(b) is not None:
                out[b] = bool(c[b])
        if isinstance(c.get("move"), (list, tuple)):
            m = c["move"]
            out["move"] = [float(m[0] or 0), float(m[1] or 0), float(m[2] or 0)]
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
        self.x = float(x)
        self.z = float(z)
        if y is not None:
            self.y = float(y)
        if heading is not None:
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
        amin = [self.x - self.half[0], self.y, self.z - self.half[2]]
        amax = [self.x + self.half[0], self.y + 2 * self.half[1], self.z + self.half[2]]
        for b in self.world.buildings.nearby(self.x, self.z):
            r = _aabb(amin, amax, b["min"], b["max"])
            if r:
                self._contact("building", b["id"], b["name"], r, (0, 0, 0))
        for o in self.world.agents.values():
            if o is self:
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
            for a in arr:
                a.detect()
            if self.kinematic_ttl:        # reap kinematic agents whose feed went silent
                now = time.time()
                for a in arr:
                    if a.kinematic and (now - a.last_update) > self.kinematic_ttl:
                        self.despawn(a.id)
            self.t += dt
            self.frame += 1

    def snapshot(self):
        with self.lock:
            return {"t": round(self.t, 3), "frame": self.frame,
                    "agents": [a.state() for a in self.agents.values()]}

    def run(self, stop_evt):
        dt = 1.0 / self.hz
        nxt = time.time()
        while not stop_evt.is_set():
            self.tick(dt)
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
    return TransitProxy(Projector(), load_route_map(), mock=mock, cache_seconds=cache_seconds)


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
        sub = html[html.find("camMarker"):]
        arr = sub[sub.find("["):sub.find("]")] + "]"
        rows = json.loads(arr)
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
class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, status=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if not n:
            return {}
        try:
            return json.loads(self.rfile.read(n) or b"{}")
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
            return self._json({"error": "transit proxy off (started with --no-transit)"}, 503)
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

    def do_GET(self):
        path = self.path.split("?", 1)[0].rstrip("/")
        if path.startswith("/api/transit"):
            return self._transit(path)
        if path == "/api/cameras/calib":     # works without the live camera proxy
            return self._json(load_calib())
        if path.startswith("/api/cameras"):
            return self._cameras(path)
        if path.startswith("/api/world"):
            if WORLD is None:
                return self._json({"error": "world not running"}, 503)
            if path == "/api/world/state":
                return self._json(WORLD.snapshot())
            if path == "/api/world/nearest_building":
                q = parse_qs(urlparse(self.path).query)
                x = float(q.get("x", [0])[0]); z = float(q.get("z", [0])[0])
                return self._json(WORLD.buildings.nearest(x, z) or {})
            if path == "/api/world/meta":
                return self._json({
                    "types": list(DEFS), "hz": WORLD.hz, "maxAgents": WORLD.max_agents,
                    "ground": WORLD.ground.ok, "buildings": len(WORLD.buildings.items),
                    "camera": RENDER is not None and RENDER.error is None,
                    "cameraReady": RENDER is not None and RENDER.ready.is_set(),
                    "transit": ("mock" if PROXY.mock else "live") if PROXY else None,
                    "georef": {"A": WORLD.ground.A, "B": WORLD.ground.B, "zone": "16N"} if WORLD.ground.ok else None,
                })
            if path.startswith("/api/world/agents"):
                aid, action = self._agent_id(path)
                a = WORLD.get(aid) if aid is not None else None
                if not a:
                    return self._json({"error": "no such agent"}, 404)
                if action == "camera":
                    return self._send_camera(a)
                return self._json(a.state())
            return self._json({"error": "unknown endpoint", "path": self.path}, 404)
        return super().do_GET()

    def _send_camera(self, a):
        if RENDER is None:
            return self._json({"error": "camera feed off — start the server with --render"}, 503)
        q = parse_qs(urlparse(self.path).query)
        w = int(float(q.get("w", [320])[0])); h = int(float(q.get("h", [240])[0]))
        (ex, ey, ez), (fx, fy, fz) = a.camera_pose()
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
            save_calib(b["full"])
            return self._json({"ok": True, "mode": "full"})
        cid, quad = b.get("cameraId"), b.get("quad")
        if not cid or quad is None:
            return self._json({"error": "need cameraId and quad (or full)"}, 400)
        calib = load_calib()
        cam = calib.setdefault("cameras", {}).setdefault(str(cid), {})
        if b.get("intersection") is not None:
            cam["intersection"] = b["intersection"]
        entry = {k: b[k] for k in ("covers", "imgW", "imgH", "H", "points") if k in b}
        cam.setdefault("quads", {})[str(quad)] = entry
        save_calib(calib)
        return self._json({"ok": True, "cameraId": cid, "quad": quad})

    def do_POST(self):
        path = self.path.split("?", 1)[0].rstrip("/")
        if path == "/api/cameras/calib":
            return self._calib_post(self._body())
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
            a = WORLD.get(aid) if aid is not None else None
            if not a:
                return self._json({"error": "no such agent"}, 404)
            b = self._body()
            if action == "controls":
                a.set_controls(b)
            elif action == "driveTo":
                a.drive_to(b)
            elif action == "stop":
                a.stop()
            elif action == "pose":
                if b.get("x") is None or b.get("z") is None:
                    return self._json({"error": "pose needs x and z"}, 400)
                a.set_pose(b["x"], b["z"], b.get("y"), b.get("heading"))
            else:
                return self._json({"error": "unknown action"}, 404)
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


def make_server(port, directory=WEB):
    """ThreadingHTTPServer on `port` with the combined world+transit+static Handler.
    The listen socket is open on return, so a --render browser can connect before
    serve_forever() starts accepting. Shared by main() and tools/verify_transit.py."""
    handler = functools.partial(Handler, directory=directory)
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    httpd.daemon_threads = True
    return httpd


def main():
    global WORLD, RENDER, PROXY, CAMERAS
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--hz", type=int, default=50, help="simulation tick rate")
    ap.add_argument("--render", action="store_true",
                    help="enable first-person agent cameras (drives a headless browser; needs playwright)")
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
    args = ap.parse_args()

    print("loading world (terrain heightmap + building boxes) ...")
    WORLD = World(hz=args.hz)
    WORLD.kinematic_ttl = args.kinematic_ttl
    stop_evt = threading.Event()
    threading.Thread(target=WORLD.run, args=(stop_evt,), daemon=True).start()

    if not args.no_transit:
        PROXY = build_proxy(mock=args.mock, cache_seconds=args.transit_cache_seconds)

    if not args.no_cameras:
        CAMERAS = build_camera_proxy(cache_seconds=args.camera_cache_seconds)

    httpd = make_server(args.port)
    if args.render:
        RENDER = RenderService(args.port)

    transit = ("off (--no-transit)" if args.no_transit else
               f"{'MOCK (fixtures)' if args.mock else 'LIVE (mystop.lextran.com)'}"
               "  ->  /api/transit/(vehicles|trips|alerts|meta)")
    cameras = ("off (--no-cameras)" if args.no_cameras else
               "LIVE (trafficvid.lexingtonky.gov)  ->  /api/cameras/(streams|meta)")
    print(f"\nUKy campus twin server — authoritative shared world + live transit + traffic cameras")
    print(f"  viewer:  http://localhost:{args.port}/")
    print(f"  world:   http://localhost:{args.port}/api/world/(state|meta|spawn|agents/<id>/...)")
    print(f"  transit: {transit}")
    print(f"  cameras: {cameras}")
    print(f"  sim:     {args.hz} Hz   ground:{'on' if WORLD.ground.ok else 'flat'}   "
          f"buildings:{len(WORLD.buildings.items)}")
    print(f"  camera:  {'ON (first-person /agents/<id>/camera; starting headless renderer…)' if args.render else 'off (use --render)'}")
    print("  Ctrl-C to stop")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")
        stop_evt.set()
        httpd.shutdown()


if __name__ == "__main__":
    main()
