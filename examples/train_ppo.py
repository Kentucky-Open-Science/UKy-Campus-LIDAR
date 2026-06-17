#!/usr/bin/env python3
"""Train a PPO policy on the campus gym, and show it beat the untrained baseline.

Proves the gym actually trains end-to-end: it builds a vectorized env, runs
Stable-Baselines3 PPO, and evaluates the policy on HELD-OUT seeds before vs. after
training (return, success rate, SPL). No server needed — the gym steps in-process.

    pip install stable-baselines3        # uses the torch you already have
    python examples/train_ppo.py                       # quick demo (~50k steps)
    python examples/train_ppo.py --timesteps 300000 --n-envs 8 --save ppo_campus
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import campus_gym  # noqa: F401  registers the envs
from campus_gym.eval import evaluate


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--env-id", default="Campus-v0")
    ap.add_argument("--timesteps", type=int, default=50_000)
    ap.add_argument("--n-envs", type=int, default=4)
    ap.add_argument("--eval-episodes", type=int, default=20)
    ap.add_argument("--region", type=float, default=90.0, help="half-size of the spawn/goal box (m)")
    ap.add_argument("--save", default=None, help="path to save the trained model")
    args = ap.parse_args()

    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.env_util import make_vec_env
    except ImportError:
        raise SystemExit("Install the RL trainer:  pip install stable-baselines3")

    env_kwargs = dict(region=(0.0, 0.0, args.region), max_episode_steps=400)
    print(f"building {args.n_envs} vectorized '{args.env_id}' envs (region {args.region} m) ...")
    venv = make_vec_env(args.env_id, n_envs=args.n_envs, seed=0, env_kwargs=env_kwargs)
    model = PPO("MlpPolicy", venv, verbose=0, n_steps=512, batch_size=256, gamma=0.99)

    import gymnasium as gym
    eval_env = gym.make(args.env_id, **env_kwargs)
    test_seeds = list(range(1_000_000, 1_000_000 + args.eval_episodes))   # held out from training

    def policy(obs, info):
        return model.predict(obs, deterministic=True)[0]

    print(f"evaluating the UNTRAINED policy on {args.eval_episodes} held-out episodes ...")
    base, _ = evaluate(eval_env, policy, episodes=args.eval_episodes, seeds=test_seeds)

    print(f"training PPO for {args.timesteps:,} timesteps ...")
    model.learn(total_timesteps=args.timesteps)

    print("evaluating the TRAINED policy on the same held-out episodes ...")
    trained, _ = evaluate(eval_env, policy, episodes=args.eval_episodes, seeds=test_seeds)

    print("\n  metric          before -> after")
    print(f"  mean return   {base['mean_return']:7.1f} -> {trained['mean_return']:7.1f}")
    print(f"  success rate  {base['success_rate']:7.0%} -> {trained['success_rate']:7.0%}")
    print(f"  SPL           {base['spl']:7.2f} -> {trained['spl']:7.2f}")
    print(f"  collisions    {base['collision_rate']:7.0%} -> {trained['collision_rate']:7.0%}")
    if trained["mean_return"] > base["mean_return"]:
        print("\n  return improved -> the policy is learning to drive toward goals.")
    if trained["success_rate"] < 0.5:
        print("  (this is a short demo; reliably reaching goals + avoiding buildings needs more\n"
              "   steps -- try --timesteps 300000 --n-envs 8, ideally CampusNav-v0 with a goal route)")
    if args.save:
        model.save(args.save)
        print(f"\nsaved model -> {args.save}.zip")
    venv.close(); eval_env.close()


if __name__ == "__main__":
    main()
