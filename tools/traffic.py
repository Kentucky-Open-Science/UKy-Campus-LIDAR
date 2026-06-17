"""Server-side traffic: deterministic signals + NPC vehicles, for the shared world & gym.

The viewer already simulates traffic signals and renders transit buses, but that lives
in the BROWSER (window.__twin) — a server-side / gym agent couldn't perceive it. This
ports the signal phase machine to Python (from web/data/signals.json) and adds NPC cars
that drive the real road network (web/data/roads.json) with simple IDM car-following and
red-light stopping. Both become first-class parts of the authoritative `World`, so any
agent (gym, scripted, or human) can sense and must yield to them. Deterministic given
the world seed.
"""
import json
import math
import os

from tools.twin_server import DATA
from tools.roadnet import shared_graph

D2R = math.pi / 180.0


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Signals:
    """Deterministic fixed-time signal controller (port of roads.js statesAt)."""

    def __init__(self, data_dir=None):
        self.legs = []          # flat: {x, z, plan, group}
        self.ok = False
        path = os.path.join(data_dir or DATA, "signals.json")
        if not os.path.exists(path):
            return
        try:
            model = json.load(open(path))
        except Exception:  # noqa: BLE001
            return
        for it in model.get("intersections", []):
            if it.get("control") != "signal" or not it.get("phasePlan"):
                continue
            plan = it["phasePlan"]
            for leg in it.get("legs", []):
                sp, grp = leg.get("stopPoint"), leg.get("signalGroup")
                if sp and grp:
                    self.legs.append({"x": sp[0], "z": sp[2], "plan": plan, "group": grp})
        self.ok = bool(self.legs)

    @staticmethod
    def _state(plan, t):
        cyc = plan["cycleSec"]
        local = (t + plan["offsetSec"]) % cyc
        acc = 0.0
        for ph in plan["phases"]:
            if local < acc + ph["durSec"]:
                return ph["groupStates"]
            acc += ph["durSec"]
        return plan["phases"][-1]["groupStates"]

    def ahead(self, x, z, yaw, t, max_dist=22.0, lane=4.0):
        """Nearest signal stop-line the agent is approaching: (distance, state) or
        (None, None). state is 'green'/'yellow'/'red'."""
        fx, fz = math.cos(yaw), -math.sin(yaw)
        best_d, best_state = None, None
        for lg in self.legs:
            rx, rz = lg["x"] - x, lg["z"] - z
            fwd = rx * fx + rz * fz
            if fwd <= 0 or fwd > max_dist:
                continue
            if abs(-rx * fz + rz * fx) > lane:          # outside our lane width
                continue
            if best_d is None or fwd < best_d:
                best_d, best_state = fwd, self._state(lg["plan"], t).get(lg["group"], "red")
        return best_d, best_state


class TrafficManager:
    """NPC cars that drive the road graph with IDM car-following + red-light stops."""

    NPC_SPEED = 9.0          # target cruise speed (m/s)
    GAP = 12.0               # IDM following gap (m)
    STOP_DIST = 10.0         # start braking this far from a red stop-line

    def __init__(self, world, rng, count=12, signals=None, region=None):
        self.world = world
        self.rng = rng
        self.graph = shared_graph()
        self.signals = signals if signals is not None else Signals()
        self.npcs = []        # [{agent, route, idx}]
        # restrict NPC spawns to road nodes in `region`=(cx,cz,half) so they cluster
        # where the agent actually is (otherwise a few NPCs are lost on a big campus)
        self._nodes = list(range(len(self.graph.nodes))) if self.graph.ok else []
        if region and self._nodes:
            cx, cz, half = region
            inreg = [i for i in self._nodes
                     if abs(self.graph.nodes[i][0] - cx) <= half and abs(self.graph.nodes[i][1] - cz) <= half]
            if inreg:
                self._nodes = inreg
        for _ in range(count if self._nodes else 0):
            self._spawn()

    def _spawn(self):
        g = self.graph
        n = int(self._nodes[int(self.rng.integers(len(self._nodes)))])
        x, z = g.nodes[n]
        route = g.random_route(self.rng, start_node=n)
        if len(route) < 2:
            return
        heading = math.degrees(math.atan2(-(route[1][1] - z), route[1][0] - x))
        try:
            a = self.world.spawn({"type": "car", "position": [x, None, z],
                                  "heading": heading, "owner": "npc"})
        except Exception:  # noqa: BLE001 (cap reached / name clash)
            return
        self.npcs.append({"agent": a, "route": route, "idx": 1})

    def _leader_gap(self, npc):
        a = npc["agent"]
        fx, fz = math.cos(a.yaw), -math.sin(a.yaw)
        best = 1e9
        for o in self.world.agents.values():
            if o is a:
                continue
            rx, rz = o.x - a.x, o.z - a.z
            fwd = rx * fx + rz * fz
            if 0 < fwd < best and abs(-rx * fz + rz * fx) < 2.6:
                best = fwd
        return best

    def tick(self, dt):
        t = self.world.t
        for npc in self.npcs:
            a = npc["agent"]
            route = npc["route"]
            # advance along the route; re-roam at the end
            while npc["idx"] < len(route) and math.dist((a.x, a.z), route[npc["idx"]]) < 6.0:
                npc["idx"] += 1
            if npc["idx"] >= len(route):
                npc["route"] = self.graph.random_route(self.rng, hops=14) or [(a.x, a.z)]
                npc["idx"] = min(1, len(npc["route"]) - 1)
                route = npc["route"]
            tx, tz = route[min(npc["idx"], len(route) - 1)]

            desired = math.atan2(-(tz - a.z), tx - a.x)
            steer = _clamp(_wrap(desired - a.yaw) / a.maxSteerRad, -1, 1)
            throttle = _clamp(self.NPC_SPEED / a.maxSpeed, 0, 1)
            brake = 0.0

            gap = self._leader_gap(npc)               # IDM car-following
            if gap < self.GAP:
                throttle = 0.0
                brake = _clamp((self.GAP - gap) / self.GAP, 0, 1)

            sd, st = self.signals.ahead(a.x, a.z, a.yaw, t)   # red-light stopping
            if sd is not None and st in ("red", "yellow") and sd < self.STOP_DIST:
                throttle = 0.0
                brake = max(brake, _clamp((self.STOP_DIST - sd) / self.STOP_DIST + 0.2, 0, 1))

            a.set_controls({"throttle": throttle, "brake": brake, "steer": steer})
