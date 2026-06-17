#!/usr/bin/env python3
"""Minimal usage of the campus gym environment (Tier 0).

Unlike the other examples, this needs NO running server — the gym wraps the sim core
in-process and steps synchronously (headless, faster than real time). Task: drive the
agent to a seeded goal on campus without crashing.

    pip install gymnasium pettingzoo
    python examples/gym_random.py                 # single-agent, random policy
    python examples/gym_random.py --type drone
    python examples/gym_random.py --multi          # multi-agent (PettingZoo)
"""
import argparse
import os
import sys

# run from anywhere: put the repo root on the path, not the tools/ dir
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import gymnasium as gym
import campus_gym  # noqa: F401  registers Campus-v0 etc.


def single(agent_type, episodes):
    env = gym.make(f"Campus{agent_type.capitalize()}-v0")
    print(f"obs space: {env.observation_space}\nact space: {env.action_space}\n")
    for ep in range(episodes):
        obs, info = env.reset(seed=ep)
        total, steps = 0.0, 0
        done = False
        while not done:
            obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
            total += reward; steps += 1
            done = terminated or truncated
        why = ("reached goal" if info.get("reached_goal") else "crashed" if info.get("crashed")
               else "off map" if info.get("offmap") else "time limit")
        print(f"episode {ep}: {steps:3d} steps, return {total:6.1f}, ended: {why}")
    env.close()


def multi(episodes):
    from campus_gym import CampusParallelEnv
    env = CampusParallelEnv(agent_types=("car", "truck", "drone", "robot"))
    for ep in range(episodes):
        obs, infos = env.reset(seed=ep)
        steps = 0
        while env.agents:
            actions = {a: env.action_space(a).sample() for a in env.agents}
            obs, rewards, terms, truncs, infos = env.step(actions)
            steps += 1
        print(f"episode {ep}: ran {steps} steps over {len(env.possible_agents)} agents")
    env.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--type", default="car", choices=["car", "truck", "robot", "drone"])
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--multi", action="store_true", help="run the multi-agent (PettingZoo) env")
    args = ap.parse_args()
    (multi(args.episodes) if args.multi else single(args.type, args.episodes))


if __name__ == "__main__":
    main()
