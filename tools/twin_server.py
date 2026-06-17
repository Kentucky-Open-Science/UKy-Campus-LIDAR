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

API (JSON, CORS-open; see client/twin.py for a Python wrapper):
    GET    /api/world/state                  -> { t, agents:[ ... ] }   (everyone's agents)
    GET    /api/world/agents/<id>            -> one agent's full sensor state
    GET    /api/world/meta                   -> types, bounds, georef
    POST   /api/world/spawn                  { type, position?, heading?, color?, name?, owner? } -> { id, ... }
    POST   /api/world/agents/<id>/controls   { throttle/brake/steer/reverse | move | thrust/climb/yawRate }
    POST   /api/world/agents/<id>/driveTo    { x, z, y?, speed?, arriveRadius?, stop? }
    POST   /api/world/agents/<id>/stop
    DELETE /api/world/agents/<id>
    (anything else) -> served as a static file from web/

Run:  python -m tools.twin_server [--port 8000] [--hz 50]
"""
import argparse
import functools
import http.server
import json
import math
import os
import threading
import time
from urllib.parse import urlparse, parse_qs

import numpy as np

from tools.extract_roads import DATA, load_mesh, build_heightmap

_HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.abspath(os.path.join(_HERE, "..", "web"))

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

    # ---- integrate one tick ----
    def integrate(self, dt):
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
    def __init__(self, hz=50, max_agents=64):
        self.ground = Ground()
        self.buildings = Buildings()
        self.agents = {}
        self.lock = threading.RLock()
        self.t = 0.0
        self.frame = 0
        self.hz = hz
        self.max_agents = max_agents
        self._next = 1
        self._names = set()

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
            arr = list(self.agents.values())
            for a in arr:
                a.integrate(dt)
            for a in arr:
                a.snap_ground(dt)
                a.finalize(dt)
            for a in arr:
                a.detect()
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


WORLD = None  # set in main()


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

    def do_GET(self):
        path = self.path.split("?", 1)[0].rstrip("/")
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
                "georef": {"A": WORLD.ground.A, "B": WORLD.ground.B, "zone": "16N"} if WORLD.ground.ok else None,
            })
        if path.startswith("/api/world/agents"):
            aid, _ = self._agent_id(path)
            a = WORLD.get(aid) if aid is not None else None
            return self._json(a.state() if a else {"error": "no such agent"}, 200 if a else 404)
        if path.startswith("/api/world"):
            return self._json({"error": "unknown endpoint", "path": self.path}, 404)
        return super().do_GET()

    def do_POST(self):
        path = self.path.split("?", 1)[0].rstrip("/")
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
            else:
                return self._json({"error": "unknown action"}, 404)
            return self._json({"ok": True})
        return self._json({"error": "unknown endpoint"}, 404)

    def do_DELETE(self):
        path = self.path.split("?", 1)[0].rstrip("/")
        if path.startswith("/api/world/agents"):
            aid, _ = self._agent_id(path)
            return self._json({"ok": WORLD.despawn(aid) if aid is not None else False})
        return self._json({"error": "unknown endpoint"}, 404)


def main():
    global WORLD
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--hz", type=int, default=50, help="simulation tick rate")
    args = ap.parse_args()

    print("loading world (terrain heightmap + building boxes) ...")
    WORLD = World(hz=args.hz)
    stop_evt = threading.Event()
    threading.Thread(target=WORLD.run, args=(stop_evt,), daemon=True).start()

    handler = functools.partial(Handler, directory=WEB)
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", args.port), handler)
    httpd.daemon_threads = True
    print(f"\nUKy campus twin server — authoritative shared world")
    print(f"  viewer:  http://localhost:{args.port}/")
    print(f"  API:     http://localhost:{args.port}/api/world/(state|meta|spawn|agents/<id>/...)")
    print(f"  sim:     {args.hz} Hz   ground:{'on' if WORLD.ground.ok else 'flat'}   "
          f"buildings:{len(WORLD.buildings.items)}")
    print("  Ctrl-C to stop")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")
        stop_evt.set()
        httpd.shutdown()


if __name__ == "__main__":
    main()
