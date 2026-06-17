"""Trajectory recording + deterministic replay for campus_gym episodes.

Records each episode as JSONL — a metadata header (seed, agent type, the language
instruction, goal, optimal distance, return) followed by one line per step (action +
reward + done flags). Because the env is deterministic given (seed, action sequence),
`replay()` re-runs the saved seed + actions and checks the reward stream matches —
exact, reproducible replay, which is what makes results auditable.

    from campus_gym import CampusNavEnv
    from campus_gym.record import record_episode, replay
    from campus_gym.eval import greedy_goal_policy
    env = CampusNavEnv("car")
    record_episode(env, greedy_goal_policy(env), seed=0, path="ep0.jsonl")
    print(replay(env, "ep0.jsonl"))     # {'ok': True, 'mismatches': 0, ...}
"""
import json

import numpy as np


def record_episode(env, policy, seed, path, max_steps=3000):
    obs, info = env.reset(seed=seed)
    meta = {
        "seed": int(seed),
        "agent_type": getattr(env, "agent_type", None),
        "instruction": info.get("instruction"),
        "goal_name": info.get("goal_name"),
        "goal": info.get("goal"),
        "optimal_dist": info.get("optimal_dist"),
    }
    steps, done, t = [], False, 0
    while not done and t < max_steps:
        a = np.asarray(policy(obs, info), dtype=np.float32).ravel()
        obs, r, term, trunc, info = env.step(a)
        steps.append({"action": [round(float(x), 6) for x in a],
                      "reward": round(float(r), 6),
                      "terminated": bool(term), "truncated": bool(trunc)})
        done = term or trunc
        t += 1
    meta.update(steps=len(steps), success=bool(info.get("reached_goal")),
                ret=round(sum(s["reward"] for s in steps), 4),
                path_len=info.get("path_len"))
    with open(path, "w") as f:
        f.write(json.dumps({"meta": meta}) + "\n")
        for s in steps:
            f.write(json.dumps(s) + "\n")
    return meta


def replay(env, path, atol=1e-4):
    """Re-run a recorded episode (same seed + actions) and verify the reward stream."""
    with open(path) as f:
        lines = [json.loads(line) for line in f if line.strip()]
    meta, recorded = lines[0]["meta"], lines[1:]
    env.reset(seed=meta["seed"])
    mismatches = 0
    for rec in recorded:
        _, r, _, _, _ = env.step(np.array(rec["action"], dtype=np.float32))
        if abs(r - rec["reward"]) > atol:
            mismatches += 1
    return {"ok": mismatches == 0, "steps": len(recorded), "mismatches": mismatches,
            "instruction": meta.get("instruction")}
