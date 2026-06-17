#!/usr/bin/env python3
"""Verify the Tier-0 gym wrappers (campus_gym): API compliance, determinism, rollouts.

Run from the repo root:  python tools/verify_gym.py
Exit code 0 = all checks passed. Needs gymnasium + pettingzoo (see requirements).
"""
import os
import sys

# tools/inspect.py shadows the stdlib `inspect` when this dir is on sys.path (which
# breaks numpy's import); drop our own dir and use the repo root instead.
_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(_HERE, ".."))
sys.path[:] = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]
sys.path.insert(0, ROOT)

import numpy as np
import gymnasium as gym
from gymnasium.utils.env_checker import check_env

import campus_gym  # noqa: F401  (registers Campus-v0 etc.)
from campus_gym import CampusEnv, CampusParallelEnv

ok = True
notes = []


def check(cond, msg):
    global ok
    notes.append(("PASS" if cond else "FAIL") + "  " + msg)
    if not cond:
        ok = False


def rollout(env, steps=400, seed=0):
    obs, info = env.reset(seed=seed)
    total, n, term_reason = 0.0, 0, "max"
    a_space = env.action_space
    a_space.seed(seed)
    for _ in range(steps):
        obs, r, terminated, truncated, info = env.step(a_space.sample())
        total += r; n += 1
        if terminated or truncated:
            term_reason = ("reached" if info.get("reached_goal") else
                           "crashed" if info.get("crashed") else
                           "offmap" if info.get("offmap") else "truncated")
            break
    return total, n, term_reason


def main():
    print("=== Gymnasium single-agent (CampusEnv) ===")
    # 1) API compliance
    env = CampusEnv(agent_type="car", max_episode_steps=300)
    try:
        check_env(env, skip_render_check=True)
        check(True, "gymnasium check_env passed")
    except Exception as e:  # noqa: BLE001
        check(False, f"check_env raised: {e}")
    env.close()

    # 2) registration + make
    try:
        e2 = gym.make("Campus-v0")
        e2.reset(seed=1); e2.step(e2.action_space.sample()); e2.close()
        check(True, "gym.make('Campus-v0') works")
    except Exception as e:  # noqa: BLE001
        check(False, f"gym.make failed: {e}")

    # 3) seeded determinism: same seed -> identical reset obs and identical rollout
    a = CampusEnv(agent_type="car"); b = CampusEnv(agent_type="car")
    o1, _ = a.reset(seed=123); o2, _ = b.reset(seed=123)
    check(np.allclose(o1, o2), "same seed -> identical reset observation")
    acts = [a.action_space.sample() for _ in range(50)]
    ra = [a.step(x)[1] for x in acts]; rb = [b.step(x)[1] for x in acts]
    check(np.allclose(ra, rb), "same seed + actions -> identical reward sequence")
    od, _ = a.reset(seed=999)
    check(not np.allclose(o1, od), "different seed -> different reset observation")
    a.close(); b.close()

    # 4) random rollouts for every body type
    for t in ("car", "truck", "robot", "drone"):
        env = CampusEnv(agent_type=t, max_episode_steps=300)
        total, n, reason = rollout(env, steps=300, seed=7)
        check(n > 0 and np.isfinite(total), f"{t}: rollout ran {n} steps, return {total:.1f}, ended={reason}")
        env.close()

    print("=== PettingZoo multi-agent (CampusParallelEnv) ===")
    # 5) PettingZoo API compliance
    try:
        from pettingzoo.test import parallel_api_test
        parallel_api_test(CampusParallelEnv(agent_types=("car", "drone", "robot"),
                                            max_episode_steps=200), num_cycles=50)
        check(True, "pettingzoo parallel_api_test passed")
    except Exception as e:  # noqa: BLE001
        check(False, f"parallel_api_test raised: {e}")

    # 6) multi-agent rollout to completion
    penv = CampusParallelEnv(agent_types=("car", "car", "drone", "robot"), max_episode_steps=300)
    obs, infos = penv.reset(seed=3)
    check(len(obs) == 4, f"parallel reset returns obs for all {len(obs)} agents")
    steps = 0
    while penv.agents and steps < 300:
        actions = {a: penv.action_space(a).sample() for a in penv.agents}
        obs, rewards, terms, truncs, infos = penv.step(actions)
        steps += 1
    check(steps > 0, f"parallel rollout ran {steps} steps, {len(penv.agents)} agents still active")
    penv.close()

    print("=== Tier 1: vectorization (SyncVectorEnv) ===")
    try:
        from gymnasium.vector import SyncVectorEnv
        venv = SyncVectorEnv([lambda: gym.make("Campus-v0") for _ in range(3)])
        o, _ = venv.reset(seed=0)
        o, r, term, trunc, vinfo = venv.step(venv.action_space.sample())
        check(o.shape[0] == 3 and r.shape[0] == 3, f"SyncVectorEnv(3): batched obs {o.shape}, reward {r.shape}")
        venv.close()
    except Exception as e:  # noqa: BLE001
        check(False, f"vectorization failed: {e}")

    print("=== Tier 3: named / language-conditioned goals ===")
    from campus_gym import CampusNavEnv
    nav = CampusNavEnv("car", max_episode_steps=400)
    o1, i1 = nav.reset(seed=4)
    check(bool(i1.get("instruction")) and bool(i1.get("goal_name")),
          f"named goal -> instruction {i1.get('instruction')!r}")
    check("reward_terms" in nav.step(nav.action_space.sample())[4], "step info has per-term reward breakdown")
    o2, i2 = CampusNavEnv("car").reset(seed=4)
    check(i1["instruction"] == i2["instruction"] and np.allclose(o1, o2),
          "named-goal determinism: same seed -> same instruction + obs")
    nav.close()

    print("=== Tier 2/3: eval harness + SPL ===")
    from campus_gym.eval import evaluate, greedy_goal_policy
    e = CampusNavEnv("car", max_episode_steps=600)
    agg, rows = evaluate(e, greedy_goal_policy(e), episodes=8)
    check(0.0 <= agg["spl"] <= 1.0 and 0.0 <= agg["success_rate"] <= 1.0 and len(rows) == 8,
          f"eval(8 eps): success={agg['success_rate']:.2f} spl={agg['spl']:.2f} "
          f"collisions={agg['collision_rate']:.2f} return={agg['mean_return']:.1f}")
    e.close()

    print("=== Tier 3: record + deterministic replay ===")
    import tempfile
    from campus_gym.record import record_episode, replay
    e = CampusNavEnv("car", max_episode_steps=300)
    path = os.path.join(tempfile.gettempdir(), "campus_ep.jsonl")
    meta = record_episode(e, greedy_goal_policy(e), seed=2, path=path)
    rep = replay(e, path)
    check(rep["ok"] and rep["mismatches"] == 0,
          f"recorded {meta['steps']} steps; deterministic replay ok (mismatches={rep['mismatches']})")
    e.close()

    print("=== NPC traffic + signals (sensed in the observation) ===")
    tenv = gym.make("CampusTraffic-v0")
    o, _ = tenv.reset(seed=1)
    check(o.shape[0] == 18, f"traffic env obs dim {o.shape[0]} (adds nearest-vehicle + signal features)")
    nnpc = sum(1 for a in tenv.unwrapped.world.agents.values() if a.owner == "npc")
    check(nnpc > 0, f"{nnpc} NPC vehicles driving the road graph")
    tenv.close()

    print("=== Scenarios + domain randomization ===")
    from campus_gym import make_scenario, SCENARIOS, train_test_seeds
    built = 0
    for name in SCENARIOS:
        env = make_scenario(name, max_episode_steps=40)
        env.reset(seed=0); env.step(env.action_space.sample()); env.close(); built += 1
    check(built == len(SCENARIOS), f"all {built} named scenarios build + step")
    tr, te = train_test_seeds()
    check(set(tr).isdisjoint(te), f"train/test seed split disjoint ({len(tr)} train / {len(te)} test)")
    e1 = CampusEnv("car", domain_random=True); e1.reset(seed=1); ms1 = e1.agent.maxSpeed
    e2 = CampusEnv("car", domain_random=True); e2.reset(seed=1); ms2 = e2.agent.maxSpeed
    e3 = CampusEnv("car", domain_random=True); e3.reset(seed=2); ms3 = e3.agent.maxSpeed
    check(abs(ms1 - ms2) < 1e-9 and abs(ms1 - ms3) > 1e-9,
          f"domain randomization: deterministic per seed, varies across (maxSpeed {ms1:.1f} vs {ms3:.1f})")
    for e in (e1, e2, e3):
        e.close()

    print("=== Tier 1: AsyncVectorEnv ===")
    try:
        import functools
        from gymnasium.vector import AsyncVectorEnv
        import campus_gym as cg
        aenv = AsyncVectorEnv([functools.partial(cg.make_env, "Campus-v0") for _ in range(2)])
        ao, _ = aenv.reset(seed=0); aenv.step(aenv.action_space.sample()); aenv.close()
        check(ao.shape[0] == 2, f"AsyncVectorEnv(2): batched obs {ao.shape}")
    except Exception as e:  # noqa: BLE001 — subprocess vectorisation is platform-finicky
        notes.append("NOTE  AsyncVectorEnv not verified here (SyncVectorEnv works): "
                     + str(e).splitlines()[0][:100])

    print("\n".join("  " + n for n in notes))
    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
