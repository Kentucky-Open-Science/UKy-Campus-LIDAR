#!/usr/bin/env python3
"""Language-conditioned navigation + evaluation on the campus gym (Tiers 2-3).

Each episode gives the agent a natural-language instruction grounded in a REAL named
campus place ("drive to the Transit Center", "navigate to Pennsylvania Avenue"); the
agent must reach it. This runs a weak greedy baseline, prints the per-episode result,
the aggregate metrics (success rate, SPL, collision rate), and records one episode for
deterministic replay. No server needed — the gym steps the sim core in-process.

    pip install gymnasium pettingzoo
    python examples/gym_nav_eval.py --type car --episodes 10
"""
import argparse
import os
import sys
import tempfile

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from campus_gym import CampusNavEnv
from campus_gym.eval import evaluate, greedy_goal_policy
from campus_gym.record import record_episode, replay


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--type", default="car", choices=["car", "truck", "robot", "drone"])
    ap.add_argument("--episodes", type=int, default=10)
    args = ap.parse_args()

    env = CampusNavEnv(args.type, max_episode_steps=800)
    policy = greedy_goal_policy(env)            # ignores obstacles — a baseline, not a solver

    print(f"language-conditioned navigation — {args.type}, {args.episodes} episodes\n")
    agg, rows = evaluate(env, policy, episodes=args.episodes)
    for r in rows:
        outcome = "REACHED" if r["success"] else "crashed" if r["crashed"] else "off-map" if r["offmap"] else "timeout"
        print(f"  seed {r['seed']:2d}: {outcome:8s}  spl={r['spl']:.2f}  return={r['return']:6.1f}  \"{r['instruction']}\"")

    print(f"\n  metrics over {agg['episodes']} episodes:")
    print(f"    success rate   : {agg['success_rate']:.0%}")
    print(f"    SPL            : {agg['spl']:.3f}   (success weighted by path efficiency)")
    print(f"    collision rate : {agg['collision_rate']:.0%}")
    print(f"    mean return    : {agg['mean_return']:.1f}")

    # record one episode and verify it replays deterministically
    path = os.path.join(tempfile.gettempdir(), "campus_episode.jsonl")
    meta = record_episode(env, policy, seed=0, path=path)
    rep = replay(env, path)
    print(f"\n  recorded seed 0 ({meta['steps']} steps) -> {path}")
    print(f"  deterministic replay: ok={rep['ok']}  (reward mismatches={rep['mismatches']})")
    env.close()


if __name__ == "__main__":
    main()
