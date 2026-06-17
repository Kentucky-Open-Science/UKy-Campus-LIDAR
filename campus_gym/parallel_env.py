"""Multi-agent (PettingZoo Parallel) environment over the campus twin.

Same task and mechanics as the single-agent `CampusEnv`, but every agent acts each
step and obs/reward/terminated/truncated/info are dicts keyed by agent id — the
natural fit for the twin's simultaneous, real-time-style world. Agents that crash,
reach their goal, or time out leave `self.agents`; the episode ends when it's empty.
Agents collide with each other as well as with buildings.
"""
import functools
import math

from gymnasium.utils import seeding
from pettingzoo import ParallelEnv

from tools.twin_server import World, DEFS
from .env import (shared_world_data, obs_space, action_space, apply_action,
                  build_observation, reward_done, free_point)


class CampusParallelEnv(ParallelEnv):
    metadata = {"name": "campus_parallel_v0", "render_modes": []}

    def __init__(self, agent_types=("car", "car", "drone"), max_episode_steps=1000,
                 dt=None, region=(0.0, 0.0, 200.0), goal=True, render_mode=None):
        for t in agent_types:
            if t not in DEFS:
                raise ValueError(f"unknown agent_type '{t}'; valid: {', '.join(DEFS)}")
        self.agent_types = list(agent_types)
        self.possible_agents = [f"{t}_{i}" for i, t in enumerate(self.agent_types)]
        self._type = dict(zip(self.possible_agents, self.agent_types))
        self.max_episode_steps = max_episode_steps
        self.dt = dt if dt else 1.0 / 50
        self.region = region
        self.use_goal = goal
        self.render_mode = render_mode
        self.ground, self.buildings = shared_world_data()
        self._np = None
        self.world = None
        self.agents = []
        self._handle = {}     # agent name -> Agent
        self._goal = {}
        self._prev = {}
        self._steps = 0

    @functools.lru_cache(maxsize=None)
    def observation_space(self, agent):
        return obs_space()

    @functools.lru_cache(maxsize=None)
    def action_space(self, agent):
        return action_space(self._type[agent])

    def reset(self, seed=None, options=None):
        if self._np is None or seed is not None:
            self._np, _ = seeding.np_random(seed)
        self.world = World(ground=self.ground, buildings=self.buildings)
        self.agents = list(self.possible_agents)
        self._handle, self._goal, self._prev = {}, {}, {}
        self._steps = 0
        for name in self.agents:
            for _ in range(8):
                sx, sz = free_point(self.world, self._np, self.region)
                heading = float(self._np.uniform(0, 360))
                a = self.world.spawn({"type": self._type[name], "position": [sx, None, sz],
                                      "heading": heading, "name": name, "owner": "gym"})
                self.world.tick(self.dt)
                if not a.contacts:
                    break
                self.world.despawn(a.id)
            self._handle[name] = a
        self.world.tick(self.dt)
        for name in self.agents:
            a = self._handle[name]
            if self.use_goal:
                gx, gz = free_point(self.world, self._np, self.region,
                                    min_from=(a.x, a.z, 60.0))
                self._goal[name] = (gx, gz)
                self._prev[name] = math.hypot(gx - a.x, gz - a.z)
            else:
                self._goal[name] = None
                self._prev[name] = 0.0
        obs = {n: build_observation(self._handle[n], self.world, self._goal[n]) for n in self.agents}
        infos = {n: {} for n in self.agents}
        return obs, infos

    def step(self, actions):
        for name, act in actions.items():
            h = self._handle.get(name)
            if h is not None:
                apply_action(h, act)
        self.world.tick(self.dt)
        self._steps += 1

        obs, rewards, terms, truncs, infos = {}, {}, {}, {}, {}
        for name in self.agents:
            a = self._handle[name]
            r, term, trunc, new_d, einfo = reward_done(
                a, self._goal[name], self._prev[name], self._steps, self.max_episode_steps)
            self._prev[name] = new_d
            obs[name] = build_observation(a, self.world, self._goal[name])
            rewards[name], terms[name], truncs[name], infos[name] = r, term, trunc, einfo

        # agents that finished this step leave the world for subsequent steps
        for name in list(self.agents):
            if terms[name] or truncs[name]:
                self.world.despawn(self._handle[name].id)
                self.agents.remove(name)
        return obs, rewards, terms, truncs, infos

    def render(self):
        return None

    def close(self):
        self.world = None
