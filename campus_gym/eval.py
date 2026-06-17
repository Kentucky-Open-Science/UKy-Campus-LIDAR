"""Evaluation harness + metrics for campus_gym: success rate, SPL, collision rate.

SPL (Success weighted by Path Length) is the standard embodied-navigation metric —
it rewards *reaching* the goal *efficiently*, using the straight-line optimal distance
(`info['optimal_dist']`) over the path actually travelled (`info['path_len']`).

    from campus_gym import CampusNavEnv
    from campus_gym.eval import evaluate, greedy_goal_policy
    agg, rows = evaluate(CampusNavEnv("car"), greedy_goal_policy, episodes=20)
    print(agg["success_rate"], agg["spl"])
"""
import math

import numpy as np


def spl(success, optimal, path):
    if not success:
        return 0.0
    return float(optimal / max(path, optimal, 1e-6))


def evaluate(env, policy, episodes=20, seeds=None, max_steps=None):
    """Roll out a `policy(obs, info) -> action` over seeded episodes.
    Returns (aggregate_metrics, per_episode_rows)."""
    rows = []
    for i in range(episodes):
        seed = int(seeds[i]) if seeds is not None else i
        obs, info = env.reset(seed=seed)
        total, steps, done = 0.0, 0, False
        while not done:
            obs, r, term, trunc, info = env.step(policy(obs, info))
            total += r; steps += 1
            done = term or trunc or (max_steps is not None and steps >= max_steps)
        success = bool(info.get("reached_goal"))
        rows.append({
            "seed": seed, "success": success, "return": total, "steps": steps,
            "crashed": bool(info.get("crashed")), "offmap": bool(info.get("offmap")),
            "spl": spl(success, info.get("optimal_dist", 0.0), info.get("path_len", 0.0)),
            "instruction": info.get("instruction"),
        })

    def mean(k):
        return float(np.mean([r[k] for r in rows])) if rows else 0.0

    agg = {
        "episodes": episodes,
        "success_rate": mean("success"), "spl": mean("spl"),
        "collision_rate": mean("crashed"), "offmap_rate": mean("offmap"),
        "mean_return": mean("return"), "mean_steps": mean("steps"),
    }
    return agg, rows


# ---- baseline policies (for sanity-checking the harness; not strong agents) ----
def random_policy(env):
    return lambda obs, info: env.action_space.sample()


def greedy_goal_policy(env):
    """A weak baseline that steers straight at the goal using the ego-frame goal vector
    in the observation (obs[10]=forward, obs[11]=lateral; obs[3]=cos yaw, obs[4]=sin yaw).
    It ignores obstacles (bumps into buildings) — just enough to exercise the harness."""
    is_drone = env.action_space.shape[0] == 3

    def policy(obs, info):
        gfwd, glat = float(obs[10]), float(obs[11])
        if is_drone:                          # 3-D move: ego goal -> world velocity
            cy, sy = float(obs[3]), float(obs[4])
            wdx, wdz = gfwd * cy + glat * sy, -gfwd * sy + glat * cy
            n = math.hypot(wdx, wdz) + 1e-6
            return np.array([wdx / n, 0.0, wdz / n], dtype=np.float32)
        ang = math.atan2(glat, abs(gfwd) + 1e-3)   # >0 => goal to the right
        return np.array([0.5, float(np.clip(-1.5 * ang, -1, 1))], dtype=np.float32)

    return policy
