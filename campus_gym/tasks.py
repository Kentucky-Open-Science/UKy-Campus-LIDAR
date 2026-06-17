"""Named, language-conditioned navigation goals (Tier 3 — the agentic layer).

Turns the twin's REAL named entities into tasks specified in natural language: the
Lextran bus stops (`transit.json`, e.g. "Transit Center", "West Main @ 707") and the
named campus streets (`roads.json`, e.g. "Pennsylvania Avenue"). A task is then
"drive to the Transit Center" — a language-conditioned goal grounded in a real campus
coordinate. This is what makes the env *agentic* (a task you specify in words) rather
than just an RL reward over a random point.

`CampusNavEnv` is a `CampusEnv` whose reset picks a named place, spawns the agent
nearby, and puts the instruction + place name in `info` (so an LLM/VLM agent can read
`info["instruction"]`). Success = reach the named place; SPL is computed from the
straight-line optimal distance (`info["optimal_dist"]`) vs. the path taken.
"""
import json
import math
import os

import gymnasium as gym

from tools.twin_server import DATA
from tools.roadnet import shared_graph
from .env import CampusEnv, build_observation, free_point

_TEMPLATES = ["drive to {name}", "navigate to {name}", "go to {name}", "head over to {name}"]


class NamedGoals:
    """Registry of named campus destinations: (name, kind, pos=(x,z))."""

    def __init__(self, data_dir=None):
        d = data_dir or DATA
        self.places = []
        self._load_stops(d)
        self._load_streets(d)
        seen, uniq = set(), []
        for p in self.places:                      # dedup by name, keep first
            if p["name"] and p["name"] not in seen:
                seen.add(p["name"]); uniq.append(p)
        self.places = uniq

    def _load_stops(self, d):
        path = os.path.join(d, "transit.json")
        if not os.path.exists(path):
            return
        try:
            t = json.load(open(path))
        except Exception:
            return
        for s in t.get("stops", []):
            nm = (s.get("name") or "").strip()
            if nm and s.get("pos"):
                self.places.append({"name": nm, "kind": "stop", "pos": (s["pos"][0], s["pos"][2])})

    def _load_streets(self, d):
        path = os.path.join(d, "roads.json")
        if not os.path.exists(path):
            return
        try:
            r = json.load(open(path))
        except Exception:
            return
        best = {}                                  # name -> (length, midpoint)
        for road in r.get("roads", []):
            nm = (road.get("name") or "").strip()
            pts = road.get("pts") or []
            if not nm or len(pts) < 2:
                continue
            length = sum(math.dist((pts[i][0], pts[i][2]), (pts[i + 1][0], pts[i + 1][2]))
                         for i in range(len(pts) - 1))
            if nm not in best or length > best[nm][0]:
                mid = pts[len(pts) // 2]
                best[nm] = (length, (mid[0], mid[2]))
        for nm, (_, mid) in best.items():
            self.places.append({"name": nm, "kind": "street", "pos": mid})

    def within(self, bounds, margin=0.0):
        x0, z0, x1, z1 = bounds
        return [p for p in self.places
                if x0 - margin <= p["pos"][0] <= x1 + margin and z0 - margin <= p["pos"][1] <= z1 + margin]

    def instruction(self, place, rng=None):
        tmpl = _TEMPLATES[int(rng.integers(len(_TEMPLATES)))] if rng is not None else _TEMPLATES[0]
        name = place["name"]
        if place["kind"] == "stop" and "stop" not in name.lower():
            name = f"the {name} stop"
        elif place["kind"] == "street":
            name = name if name[0].isupper() else f"{name}"
        return tmpl.format(name=name)


class CampusNavEnv(CampusEnv):
    """Language-conditioned navigation to a named campus place. `info['instruction']`
    is the natural-language goal; `info['goal_name']` the destination."""

    def __init__(self, agent_type="car", spawn_radius=130.0, goal_radius=12.0, **kwargs):
        kwargs.pop("goal", None); kwargs.pop("region", None); kwargs.pop("goal_radius", None)
        super().__init__(agent_type=agent_type, goal=True, goal_radius=goal_radius, **kwargs)
        self.spawn_radius = spawn_radius
        self._goals = NamedGoals()
        g = self.ground
        self._places = (self._goals.within((g.sxmin, g.szmin, g.sxmax, g.szmax), margin=-20.0)
                        if getattr(g, "ok", False) else self._goals.places) or self._goals.places
        if not self._places:
            raise RuntimeError("no named destinations (run tools.lextran_gtfs / tools.osm_roads first)")

    def reset(self, *, seed=None, options=None):
        gym.Env.reset(self, seed=seed)             # seed self.np_random
        place = self._places[int(self.np_random.integers(len(self._places)))]
        gx, gz = place["pos"]
        self.goal_name = place["name"]
        self.instruction = self._goals.instruction(place, self.np_random)
        self.region = (gx, gz, self.spawn_radius)
        self.world = self._make_world()
        for _ in range(12):
            sx, sz = free_point(self.world, self.np_random, self.region, min_from=(gx, gz, 40.0))
            heading = float(self.np_random.uniform(0, 360))
            self.agent = self.world.spawn({"type": self.agent_type, "position": [sx, None, sz],
                                           "heading": heading, "owner": "gym"})
            self.world.tick(self.dt)
            if not self.agent.contacts:
                break
            self.world.despawn(self.agent.id)
        self.goal = (gx, gz)
        self._prev_d = math.hypot(gx - self.agent.x, gz - self.agent.z)
        self._begin_episode()
        # SPL optimal = shortest navigable route over the road graph (not straight line)
        _, self._opt_dist = shared_graph().route((self.agent.x, self.agent.z), (gx, gz))
        return build_observation(self.agent, self.world, self.goal), self._info()
