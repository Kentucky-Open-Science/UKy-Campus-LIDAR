"""Single-agent Gymnasium environment over the campus twin (Tier 0 gym contract).

Wraps the authoritative simulation core (`tools/twin_server.World`/`Agent`) as a
synchronous, clock-decoupled `gymnasium.Env`: it advances ONLY inside `step()`, so it
runs headless and faster-than-real-time (unlike the real-time REST server, which is
kept for interactive/multi-client use). The read-only world data (terrain heightmap +
building AABBs) is loaded once and shared across env instances, so this is cheap to
vectorise later.

Task: navigate the agent to a (seeded) goal point on campus without crashing.
  observation : ego kinematics + nearest-building (ego frame) + goal (ego frame)   [Box(13)]
  action      : ground = [accel/brake, steer] in [-1,1]; drone = [vx,vy,vz] in [-1,1]
  reward      : progress toward goal - collision/off-map penalties - small time cost
  terminated  : reached goal | crashed into a building | drove off the map
  truncated   : hit max_episode_steps
"""
import math
import os
import sys

import numpy as np
import gymnasium as gym
from gymnasium import spaces

# import the sim core from the tools package (run from the repo root)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from tools.twin_server import World, Ground, Buildings, DEFS  # noqa: E402

OBS_DIM = 18          # ego(7) + building(3) + goal(3) + vehicle(3) + signal(2)
OBS_RANGE = 60.0      # normaliser for nearest-building distance (m)
GOAL_RANGE = 200.0    # normaliser for goal distance (m)
VEH_RANGE = 40.0      # normaliser for nearest-vehicle distance (m)
SIG_RANGE = 25.0      # normaliser for signal-ahead distance (m)
GOAL_RADIUS = 8.0     # within this of the goal counts as reached (m)

_SHARED = None        # (Ground, Buildings) loaded once, shared read-only
_SIGNALS = None       # shared deterministic signal controller


def shared_world_data():
    global _SHARED
    if _SHARED is None:
        _SHARED = (Ground(), Buildings())
    return _SHARED


def shared_signals():
    global _SIGNALS
    if _SIGNALS is None:
        from tools.traffic import Signals
        _SIGNALS = Signals()
    return _SIGNALS


# ----------------------------------------------------------------- spaces ---
def obs_space():
    return spaces.Box(-np.inf, np.inf, (OBS_DIM,), dtype=np.float32)


def action_space(agent_type):
    n = 3 if agent_type == "drone" else 2
    return spaces.Box(-1.0, 1.0, (n,), dtype=np.float32)


# ------------------------------------------------------ shared mechanics ---
def apply_action(agent, action):
    """Map a normalised action vector onto the agent's controls."""
    a = np.asarray(action, dtype=np.float32).ravel()
    if agent.type == "drone":
        agent.set_controls({"move": [float(a[0]) * agent.maxSpeed,
                                     float(a[1]) * agent.maxClimb,
                                     float(a[2]) * agent.maxSpeed]})
    else:
        throttle, steer = float(a[0]), float(a[1])
        if throttle >= 0:
            agent.set_controls({"throttle": throttle, "brake": 0.0, "steer": steer})
        else:
            agent.set_controls({"throttle": 0.0, "brake": -throttle, "steer": steer})


def build_observation(agent, world, goal):
    """Ego-frame observation vector (see module docstring)."""
    yaw = agent.yaw
    cy, sy = math.cos(yaw), math.sin(yaw)
    ms = max(agent.maxSpeed, 1e-3)

    def ego(dx, dz):                     # world XZ offset -> ego (forward, lateral)
        return dx * cy - dz * sy, dx * sy + dz * cy

    fwd_v, lat_v = ego(agent.measVel[0], agent.measVel[2])

    # nearest building, ego frame
    odx = odz = odist = 0.0
    best, bd = None, 1e18
    for b in world.buildings.nearby(agent.x, agent.z):
        d = (b["cx"] - agent.x) ** 2 + (b["cz"] - agent.z) ** 2
        if d < bd:
            bd, best = d, b
    if best is not None:
        ex, ez = ego(best["cx"] - agent.x, best["cz"] - agent.z)
        odx, odz, odist = ex / OBS_RANGE, ez / OBS_RANGE, math.sqrt(bd) / OBS_RANGE

    # goal, ego frame
    gdx = gdz = gdist = 0.0
    if goal is not None:
        gx, gz = ego(goal[0] - agent.x, goal[1] - agent.z)
        gdx, gdz = gx / GOAL_RANGE, gz / GOAL_RANGE
        gdist = math.hypot(goal[0] - agent.x, goal[1] - agent.z) / GOAL_RANGE

    # nearest OTHER vehicle (NPC or another agent), ego frame
    vdx = vdz = vdist = 0.0
    vbest, vbd = None, VEH_RANGE * VEH_RANGE
    for o in world.agents.values():
        if o is agent:
            continue
        d = (o.x - agent.x) ** 2 + (o.z - agent.z) ** 2
        if d < vbd:
            vbd, vbest = d, o
    if vbest is not None:
        ex, ez = ego(vbest.x - agent.x, vbest.z - agent.z)
        vdx, vdz, vdist = ex / VEH_RANGE, ez / VEH_RANGE, math.sqrt(vbd) / VEH_RANGE

    # traffic signal ahead (1 = a red/yellow we must stop for; dist normalised, 1 = clear)
    sig_dist, sig_red = 1.0, 0.0
    sigs = getattr(world, "signals", None)
    if sigs is not None and sigs.ok:
        sd, st = sigs.ahead(agent.x, agent.z, agent.yaw, world.t)
        if sd is not None:
            sig_dist = sd / SIG_RANGE
            sig_red = 1.0 if st in ("red", "yellow") else 0.0

    agl = (agent.altitudeAGL or 0.0) / 50.0 if agent.type == "drone" else 0.0
    coll = 1.0 if agent.contacts else 0.0
    return np.array([agent.speed / ms, fwd_v / ms, lat_v / ms, cy, sy, agl, coll,
                     odx, odz, odist, gdx, gdz, gdist, vdx, vdz, vdist, sig_dist, sig_red],
                    dtype=np.float32)


# default reward as named, weighted terms (override per-env with reward_weights=...)
DEFAULT_REWARD = {
    "progress": 1.0,      # per metre of progress toward the goal
    "goal_bonus": 10.0,   # one-off on reaching the goal (terminal)
    "collision": -5.0,    # on hitting a building (terminal)
    "offmap": -5.0,       # on leaving the map (terminal)
    "time": -0.01,        # per step
    "speed": 0.05,        # per m/s (only used when there is no goal)
}


def reward_done(agent, goal, prev_goal_dist, steps, max_steps, weights=None,
                goal_radius=GOAL_RADIUS):
    """Return (reward, terminated, truncated, new_goal_dist, info). The reward is a
    sum of named terms (info['reward_terms']) so shaping is transparent + ablatable."""
    w = DEFAULT_REWARD if weights is None else {**DEFAULT_REWARD, **weights}
    crashed = any(c["with"] == "building" for c in agent.contacts)
    offmap = agent.surface == "none"
    reached = terminated = False
    new_d = prev_goal_dist
    terms = {"time": w["time"]}

    if goal is not None:
        new_d = math.hypot(goal[0] - agent.x, goal[1] - agent.z)
        terms["progress"] = w["progress"] * (prev_goal_dist - new_d)
        if new_d < goal_radius:
            terms["goal_bonus"] = w["goal_bonus"]; terminated = reached = True
    else:
        terms["speed"] = w["speed"] * agent.speed

    if crashed:
        terms["collision"] = w["collision"]; terminated = True
    if offmap:
        terms["offmap"] = w["offmap"]; terminated = True

    reward = float(sum(terms.values()))
    truncated = (steps >= max_steps) and not terminated
    return reward, terminated, truncated, new_d, {
        "reached_goal": reached, "crashed": crashed, "offmap": offmap, "reward_terms": terms}


def _inside_building(world, x, z, margin=3.0):
    for b in world.buildings.nearby(x, z):
        if (b["min"][0] - margin <= x <= b["max"][0] + margin and
                b["min"][2] - margin <= z <= b["max"][2] + margin):
            return True
    return False


def free_point(world, rng, region, min_from=None, tries=60):
    """Sample a point in `region`=(cx,cz,half) that isn't inside a building."""
    cx, cz, half = region
    for _ in range(tries):
        x = cx + float(rng.uniform(-half, half))
        z = cz + float(rng.uniform(-half, half))
        if _inside_building(world, x, z):
            continue
        if min_from and math.hypot(x - min_from[0], z - min_from[1]) < min_from[2]:
            continue
        return x, z
    return cx, cz


# --------------------------------------------------------------- the env ---
class CampusEnv(gym.Env):
    metadata = {"render_modes": [], "render_fps": 50}

    def __init__(self, agent_type="car", max_episode_steps=1000, dt=None,
                 region=(0.0, 0.0, 200.0), goal=True, reward_weights=None,
                 goal_radius=GOAL_RADIUS, npc_traffic=0, signals=False,
                 domain_random=False, render_mode=None):
        super().__init__()
        if agent_type not in DEFS:
            raise ValueError(f"unknown agent_type '{agent_type}'; valid: {', '.join(DEFS)}")
        self.agent_type = agent_type
        self.max_episode_steps = max_episode_steps
        self.dt = dt if dt else 1.0 / 50
        self.region = region
        self.use_goal = goal
        self.reward_weights = reward_weights
        self.goal_radius = goal_radius
        self.npc_traffic = int(npc_traffic)         # background NPC cars on the roads
        self.signals_on = bool(signals or npc_traffic)   # deterministic traffic lights
        self.domain_random = bool(domain_random)    # per-reset dynamics randomization (sim-to-real)
        self.render_mode = render_mode
        self.ground, self.buildings = shared_world_data()
        self.observation_space = obs_space()
        self.action_space = action_space(agent_type)
        self.world = self.agent = self.goal = None
        self.instruction = self.goal_name = None      # set by the named-goal subclass
        self._steps = 0
        self._prev_d = self._opt_dist = self._path_len = 0.0
        self._last_xz = (0.0, 0.0)

    def _make_world(self):
        """Fresh World, optionally with deterministic signals + NPC traffic attached
        (seeded by self.np_random). Shared by the random- and named-goal resets."""
        w = World(ground=self.ground, buildings=self.buildings)
        if self.signals_on:
            w.signals = shared_signals()
        if self.npc_traffic > 0:
            from tools.traffic import TrafficManager
            w.traffic = TrafficManager(w, self.np_random, count=self.npc_traffic,
                                       signals=w.signals, region=self.region)
        return w

    def _spawn_agent_in(self, region, min_from=None):
        """Spawn the controlled agent clear of buildings (re-sampling on contact), then
        apply per-reset dynamics randomization if enabled. Used by both reset paths."""
        a = None
        for _ in range(12):
            sx, sz = free_point(self.world, self.np_random, region, min_from=min_from)
            heading = float(self.np_random.uniform(0, 360))
            a = self.world.spawn({"type": self.agent_type, "position": [sx, None, sz],
                                  "heading": heading, "owner": "gym"})
            self.world.tick(self.dt)
            if not a.contacts:
                break
            self.world.despawn(a.id)
        self.agent = a
        if self.domain_random:                       # jitter dynamics +/-15%
            jit = lambda: float(self.np_random.uniform(0.85, 1.15))
            a.maxSpeed *= jit(); a.maxAccel *= jit()
            a.brakeDecel = a.maxAccel; a.maxSteerRad *= jit()
        return a

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)          # seeds self.np_random
        self.world = self._make_world()
        self._spawn_agent_in(self.region)
        self.goal = None
        if self.use_goal:
            gx, gz = free_point(self.world, self.np_random, self.region,
                                min_from=(self.agent.x, self.agent.z, 60.0))
            self.goal = (gx, gz)
            self._prev_d = math.hypot(gx - self.agent.x, gz - self.agent.z)
        self._begin_episode()
        return build_observation(self.agent, self.world, self.goal), self._info()

    def _begin_episode(self):
        """Reset the per-episode bookkeeping (path length, optimal distance for SPL)."""
        self._steps = 0
        self._opt_dist = self._prev_d
        self._path_len = 0.0
        self._last_xz = (self.agent.x, self.agent.z)

    def step(self, action):
        apply_action(self.agent, action)
        self.world.tick(self.dt)
        self._steps += 1
        self._path_len += math.hypot(self.agent.x - self._last_xz[0], self.agent.z - self._last_xz[1])
        self._last_xz = (self.agent.x, self.agent.z)
        reward, terminated, truncated, new_d, einfo = reward_done(
            self.agent, self.goal, self._prev_d, self._steps, self.max_episode_steps,
            weights=self.reward_weights, goal_radius=self.goal_radius)
        self._prev_d = new_d
        info = self._info(); info.update(einfo)
        return build_observation(self.agent, self.world, self.goal), reward, terminated, truncated, info

    def _info(self):
        return {"position": [round(self.agent.x, 2), round(self.agent.y, 2), round(self.agent.z, 2)],
                "goal": self.goal, "steps": self._steps,
                "goal_dist": round(self._prev_d, 2) if self.goal else None,
                "path_len": round(self._path_len, 2), "optimal_dist": round(self._opt_dist, 2),
                "instruction": self.instruction, "goal_name": self.goal_name}

    def render(self):
        # headless by design; first-person pixels are available from the live server
        # (tools/twin_server.py --render -> /api/world/agents/<id>/camera).
        return None

    def close(self):
        self.world = self.agent = None
