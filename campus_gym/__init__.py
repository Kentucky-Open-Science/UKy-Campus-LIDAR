"""Campus digital twin as a gym environment (Tier 0 of the agentic-gym roadmap).

Synchronous, headless, seedable wrappers over the authoritative sim core
(`tools/twin_server.World`/`Agent`):

  * `CampusEnv` — single-agent `gymnasium.Env`. Registered ids: `Campus-v0` (car) and
    `Campus{Car,Truck,Robot,Drone}-v0`.
  * `CampusParallelEnv` — multi-agent PettingZoo Parallel env.

    import campus_gym, gymnasium as gym
    env = gym.make("Campus-v0")
    obs, info = env.reset(seed=0)
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())

The real-time REST server (`tools/twin_server.py`) is unchanged and complementary —
use it for interactive/multi-client demos; use this for training/evaluation.
"""
import os
import sys

# make the `tools` package importable when this package is imported from anywhere
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from gymnasium.envs.registration import register  # noqa: E402

from .env import CampusEnv, DEFAULT_REWARD  # noqa: E402
from .parallel_env import CampusParallelEnv  # noqa: E402
from .tasks import CampusNavEnv, NamedGoals  # noqa: E402

__all__ = ["CampusEnv", "CampusParallelEnv", "CampusNavEnv", "NamedGoals", "DEFAULT_REWARD"]

# random-point navigation (Campus*-v0) and named/language-goal navigation (CampusNav*-v0)
register(id="Campus-v0", entry_point="campus_gym.env:CampusEnv", max_episode_steps=1000)
register(id="CampusNav-v0", entry_point="campus_gym.tasks:CampusNavEnv", max_episode_steps=1500)
for _t in ("car", "truck", "robot", "drone"):
    register(id=f"Campus{_t.capitalize()}-v0", entry_point="campus_gym.env:CampusEnv",
             max_episode_steps=1000, kwargs={"agent_type": _t})
    register(id=f"CampusNav{_t.capitalize()}-v0", entry_point="campus_gym.tasks:CampusNavEnv",
             max_episode_steps=1500, kwargs={"agent_type": _t})
# traffic variants: NPC cars on the real roads + deterministic signals to sense/yield to
register(id="CampusTraffic-v0", entry_point="campus_gym.env:CampusEnv", max_episode_steps=1500,
         kwargs={"agent_type": "car", "npc_traffic": 12, "signals": True})
register(id="CampusNavTraffic-v0", entry_point="campus_gym.tasks:CampusNavEnv", max_episode_steps=1500,
         kwargs={"agent_type": "car", "npc_traffic": 12})
