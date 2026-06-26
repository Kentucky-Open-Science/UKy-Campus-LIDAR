#!/usr/bin/env python3
"""Reinforcement-learning car task: drive a FIXED real route across Lexington.

Start : Broadway & Red Mile Road        (scene ~ -1186, -181)
Goal  : Nicholasville Rd & Cooper Dr / Waller Ave   (scene ~ -754, 999)

The agent must follow the road network from the start intersection to the goal
intersection. The reward shapes exactly what was asked for:

    + progress      per metre advanced ALONG the road route toward the goal
    + goal_bonus    one-off for arriving at the target intersection   (terminal)
    + on_road       per step while on the route's road corridor
    + speed         per m/s  (keep moving -> shorter time to target)
    - off_road      per step once it strays off the road
    - stall         per step while crawling/stopped         ("bad if slow")
    - collision     per step while overlapping a building   (--strict-collision = terminal)
    - time          per step                                ("time to target")

Like the other gym examples (gym_random / train_ppo), this runs the sim IN-PROCESS
via the `campus_gym` package (no server, headless, faster than real time) — so it
imports campus_gym + tools from the repo root rather than being self-contained.

    pip install gymnasium stable-baselines3
    python examples/car_rl_route.py --baseline-only      # route-follower sanity run
    python examples/car_rl_route.py                       # baseline + PPO before/after
    python examples/car_rl_route.py --timesteps 1000000 --n-envs 8 --save ppo_route

Watch it drive live (top-down video stream in your browser; needs pillow):
    python examples/car_rl_route.py --watch              # watch the route-follower
    python examples/car_rl_route.py --watch --model ppo_route.zip   # watch a trained policy
"""
import argparse
import math
import os
import sys

# run from anywhere: put the repo root on the path (campus_gym + tools live there)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
import gymnasium as gym

from campus_gym.env import CampusEnv, build_observation, apply_action
from campus_gym.eval import spl
from tools.roadnet import shared_graph

# resolved from web/data/roads.json (these roads share exact junction vertices)
START = (-1185.7, -180.7)     # Broadway x Red Mile Road
GOAL = (-753.9, 998.9)        # Nicholasville Rd x Cooper Dr / Waller Ave

# reward weights — override a few from the CLI, or edit here for experiments
ROUTE_REWARD = {
    "progress": 1.0,            # per metre advanced along the route toward the goal
    "goal_bonus": 200.0,        # arriving at the target intersection (terminal)
    "on_road": 0.15,            # per step inside the road corridor
    "off_road": -0.6,           # per step once off the road
    "speed": 0.03,              # per m/s (encourage moving -> lower time to target)
    "stall": -0.4,              # per step while nearly stopped (bad if slow)
    "collision": -2.0,          # per step while overlapping a building (heavily bad)
    "collision_terminal": -60.0,  # one-off if --strict-collision ends the episode on a hit
    "offmap": -40.0,            # left the world entirely (terminal; rare)
    "time": -0.02,              # per step (time-to-target pressure)
}
# NOTE: buildings are collision *sensors* in the sim — a car can pass through one (it just
# flags a contact, no physics push-back). This real route threads through OSM building
# boxes for ~16% of its length, so a TERMINAL crash would make it unsolvable. By default a
# building overlap is a strong per-step penalty (you're meant to avoid it, but can power
# through the unavoidable spans); pass --strict-collision to make any hit end the episode.


# --------------------------------------------------------------- the route ---
class Route:
    """A polyline (x,z waypoints) with arc-length queries for progress + look-ahead."""

    def __init__(self, pts):
        self.pts = [(float(x), float(z)) for x, z in pts]
        self.cum = [0.0]
        for i in range(1, len(self.pts)):
            self.cum.append(self.cum[-1] + math.dist(self.pts[i - 1], self.pts[i]))
        self.total = self.cum[-1]

    def project(self, x, z):
        """Nearest point on the route -> (arc_length_s, lateral_distance)."""
        best_d2, best_s, best_lat = 1e18, 0.0, 0.0
        for i in range(len(self.pts) - 1):
            ax, az = self.pts[i]; bx, bz = self.pts[i + 1]
            dx, dz = bx - ax, bz - az
            seg2 = dx * dx + dz * dz
            t = 0.0 if seg2 == 0 else ((x - ax) * dx + (z - az) * dz) / seg2
            t = max(0.0, min(1.0, t))
            px, pz = ax + t * dx, az + t * dz
            d2 = (x - px) ** 2 + (z - pz) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_s = self.cum[i] + t * math.sqrt(seg2)
                best_lat = math.sqrt(d2)
        return best_s, best_lat

    def point_at(self, s):
        """Point on the route at arc-length `s` (clamped to the route)."""
        s = max(0.0, min(self.total, s))
        for i in range(len(self.pts) - 1):
            if self.cum[i + 1] >= s:
                seg = self.cum[i + 1] - self.cum[i]
                t = 0.0 if seg == 0 else (s - self.cum[i]) / seg
                ax, az = self.pts[i]; bx, bz = self.pts[i + 1]
                return (ax + t * (bx - ax), az + t * (bz - az))
        return self.pts[-1]


_ROUTE = None


def shared_route():
    """Shortest navigable road route START->GOAL, built once over the road graph."""
    global _ROUTE
    if _ROUTE is None:
        pts, total = shared_graph().route(START, GOAL)
        _ROUTE = Route(pts)
    return _ROUTE


# ----------------------------------------------------------------- the env ---
class RoadRouteEnv(CampusEnv):
    """Drive the car from Broadway/Red Mile to Nicholasville/Cooper-Waller, on-road.

    Reuses CampusEnv's world/observation/action; overrides reset() (fixed start +
    route) and step() (route-following reward). The observation's "goal" is a moving
    look-ahead point on the route, so the policy always sees the next road heading.
    """

    def __init__(self, max_episode_steps=6000, corridor=8.0, lookahead=25.0,
                 goal_radius=12.0, stall_speed=1.0, spawn_jitter=4.0,
                 terminate_on_crash=False, reward_weights=None, dt=None, **kwargs):
        kwargs.pop("goal", None); kwargs.pop("region", None); kwargs.pop("goal_radius", None)
        super().__init__(agent_type="car", goal=True, goal_radius=goal_radius,
                         max_episode_steps=max_episode_steps, dt=dt, **kwargs)
        self.corridor = float(corridor)
        self.lookahead = float(lookahead)
        self.stall_speed = float(stall_speed)
        self.spawn_jitter = float(spawn_jitter)
        self.terminate_on_crash = bool(terminate_on_crash)
        self.rw = {**ROUTE_REWARD, **(reward_weights or {})}
        self.route = shared_route()
        self.final_goal = GOAL
        self.instruction = ("drive from Broadway & Red Mile to "
                            "Nicholasville Rd & Cooper Dr / Waller Ave")
        self.goal_name = "Nicholasville Rd & Cooper Dr / Waller Ave"
        # center the (optional) NPC-traffic region on the route
        self.region = ((START[0] + GOAL[0]) / 2, (START[1] + GOAL[1]) / 2,
                       max(abs(START[0] - GOAL[0]), abs(START[1] - GOAL[1])) / 2 + 120.0)
        self._s = 0.0
        self._remaining = self.route.total
        self._on_road_steps = 0

    def _lookahead_point(self):
        return self.route.point_at(self._s + self.lookahead)

    def reset(self, *, seed=None, options=None):
        gym.Env.reset(self, seed=seed)            # seeds self.np_random
        self.world = self._make_world()
        sx, sz = START
        # heading: face an early point on the route so the car starts pointed correctly
        ax, az = self.route.point_at(min(self.route.total, 15.0))
        heading = math.degrees(math.atan2(-(az - sz), ax - sx))
        a = None
        for _ in range(8):                        # re-sample a touch if we spawn in a wall
            jx = sx + (self.np_random.uniform(-self.spawn_jitter, self.spawn_jitter)
                       if self.spawn_jitter else 0.0)
            jz = sz + (self.np_random.uniform(-self.spawn_jitter, self.spawn_jitter)
                       if self.spawn_jitter else 0.0)
            a = self.world.spawn({"type": self.agent_type, "position": [jx, None, jz],
                                  "heading": heading, "owner": "gym"})
            self.world.tick(self.dt)
            if not a.contacts:
                break
            self.world.despawn(a.id)
        self.agent = a
        if self.domain_random:                    # jitter dynamics +/-15% (sim-to-real)
            jit = lambda: float(self.np_random.uniform(0.85, 1.15))
            a.maxSpeed *= jit(); a.maxAccel *= jit()
            a.brakeDecel = a.maxAccel; a.maxSteerRad *= jit()

        self._s, _ = self.route.project(a.x, a.z)
        self._remaining = self.route.total - self._s
        self._on_road_steps = 0
        self._crash_steps = 0
        self.goal = self._lookahead_point()       # obs goal = look-ahead on the route
        self._begin_episode()
        self._opt_dist = self.route.total         # SPL optimal = road-route length
        return build_observation(self.agent, self.world, self.goal), self._info()

    def step(self, action):
        apply_action(self.agent, action)
        self.world.tick(self.dt)
        self._steps += 1
        self._path_len += math.hypot(self.agent.x - self._last_xz[0],
                                     self.agent.z - self._last_xz[1])
        self._last_xz = (self.agent.x, self.agent.z)

        s, lateral = self.route.project(self.agent.x, self.agent.z)
        progress = self._remaining - (self.route.total - s)   # >0 when remaining shrinks
        self._remaining = self.route.total - s
        self._s = s

        on_road = lateral <= self.corridor
        if on_road:
            self._on_road_steps += 1
        crashed = any(c["with"] == "building" for c in self.agent.contacts)
        offmap = self.agent.surface == "none"
        dist_goal = math.hypot(self.final_goal[0] - self.agent.x,
                               self.final_goal[1] - self.agent.z)
        reached = dist_goal <= self.goal_radius

        w = self.rw
        terms = {"time": w["time"], "progress": w["progress"] * progress,
                 "speed": w["speed"] * self.agent.speed}
        terms["on_road" if on_road else "off_road"] = w["on_road"] if on_road else w["off_road"]
        if self.agent.speed < self.stall_speed:
            terms["stall"] = w["stall"]
        terminated = False
        if crashed:                                    # buildings are sensors, not walls
            self._crash_steps += 1
            terms["collision"] = w["collision"]        # strong per-step penalty
            if self.terminate_on_crash:
                terms["collision"] = w["collision_terminal"]; terminated = True
        if reached:
            terms["goal_bonus"] = w["goal_bonus"]; terminated = True
        if offmap:
            terms["offmap"] = w["offmap"]; terminated = True

        reward = float(sum(terms.values()))
        truncated = (self._steps >= self.max_episode_steps) and not terminated
        self.goal = self._lookahead_point()
        info = self._info()
        info.update({"reached_goal": reached, "crashed": crashed, "offmap": offmap,
                     "on_road": on_road, "lateral": round(lateral, 2),
                     "route_remaining": round(self._remaining, 1),
                     "on_road_frac": round(self._on_road_steps / max(self._steps, 1), 3),
                     "crash_frac": round(self._crash_steps / max(self._steps, 1), 3),
                     "reward_terms": terms})
        return build_observation(self.agent, self.world, self.goal), reward, terminated, truncated, info


# ------------------------------------------------------------- baseline pol ---
def route_follow_policy(env):
    """A non-RL controller: steer toward the look-ahead point (obs goal, ego frame),
    easing the throttle when it has to turn hard. Proves the route + reward are sane
    and gives PPO something to beat."""
    def policy(obs, info):
        gfwd, glat = float(obs[10]), float(obs[11])      # ego-frame look-ahead direction
        ang = math.atan2(glat, abs(gfwd) + 1e-3)         # >0 => target to the right
        steer = float(np.clip(-1.8 * ang, -1.0, 1.0))
        throttle = float(np.clip(0.75 - 0.6 * abs(steer), 0.3, 0.75))  # slow into turns
        return np.array([throttle, steer], dtype=np.float32)
    return policy


# ------------------------------------------------------------- eval + main ---
def run_episodes(env, policy, episodes, seeds=None):
    """Roll out policy(obs, info)->action; return rich per-episode rows."""
    dt = env.dt
    rows = []
    for i in range(episodes):
        seed = int(seeds[i]) if seeds is not None else i
        obs, info = env.reset(seed=seed)
        total, steps, done = 0.0, 0, False
        while not done:
            obs, r, term, trunc, info = env.step(policy(obs, info))
            total += r; steps += 1
            done = term or trunc
        success = bool(info.get("reached_goal"))
        rows.append({
            "seed": seed, "success": success, "return": total, "steps": steps,
            "sim_s": steps * dt, "crashed": bool(info.get("crashed")),
            "on_road_frac": float(info.get("on_road_frac", 0.0)),
            "crash_frac": float(info.get("crash_frac", 0.0)),
            "remaining": float(info.get("route_remaining", 0.0)),
            "spl": spl(success, info.get("optimal_dist", 0.0), info.get("path_len", 0.0)),
        })
    return rows


def summarize(label, rows):
    m = lambda k: float(np.mean([r[k] for r in rows])) if rows else 0.0
    print(f"  {label:9s} success={m('success'):4.0%}  spl={m('spl'):.2f}  "
          f"on-road={m('on_road_frac'):4.0%}  in-bldg={m('crash_frac'):4.0%}  "
          f"time={m('sim_s'):5.0f}s  return={m('return'):8.1f}  remaining={m('remaining'):5.0f}m")


def watch_session(args):
    """Spawn a live top-down video stream and drive the car so you can watch it.

    Loops episodes forever (Ctrl-C to stop). Policy is the route-follower, or a saved
    PPO model with --model. Playback is paced to --watch-speed x real time."""
    import time
    try:
        from route_view import StreamServer, TopDownRenderer
    except ImportError as e:
        raise SystemExit(f"the viewer needs pillow:  pip install pillow   ({e})")

    env = RoadRouteEnv(max_episode_steps=args.max_steps, corridor=args.corridor,
                       terminate_on_crash=args.strict_collision, spawn_jitter=0.0)
    if args.model:
        from stable_baselines3 import PPO
        model = PPO.load(args.model)
        def policy(obs, info):
            return model.predict(obs, deterministic=True)[0]
        label = f"PPO: {os.path.basename(args.model)}"
    else:
        policy = route_follow_policy(env)
        label = "route-follower"

    srv = StreamServer()
    url = srv.start(args.watch_host, args.watch_port)
    rend = TopDownRenderer(env.route.pts, env.buildings, GOAL, corridor=args.corridor)

    print("\n" + "=" * 64)
    print(f"  watching the car ({label}) — open this in your browser:\n\n      {url}\n")
    print(f"  route {env.route.total:.0f} m, playback {args.watch_speed:g}x real time."
          "  Ctrl-C to stop.")
    print("=" * 64 + "\n", flush=True)      # show the URL immediately, before the run loop

    step_dt = env.dt / max(args.watch_speed, 1e-6)
    frame_dt = 1.0 / 30.0
    ep = 0
    try:
        while True:
            obs, info = env.reset(seed=ep)
            ep += 1
            ret, done, last_frame = 0.0, False, 0.0
            while not done:
                t0 = time.perf_counter()
                obs, r, term, trunc, info = env.step(policy(obs, info))
                ret += r
                done = term or trunc
                now = time.perf_counter()
                if now - last_frame >= frame_dt or done:
                    a = env.agent
                    hud = {"title": f"{label}  (episode {ep})",
                           "progress": 1.0 - info["route_remaining"] / max(env.route.total, 1.0),
                           "remaining": info["route_remaining"], "speed": a.speed,
                           "sim_s": info["steps"] * env.dt, "on_road": info["on_road"],
                           "in_bldg": info["crashed"], "reached": info["reached_goal"], "ret": ret}
                    srv.publish(rend.render_jpeg({"x": a.x, "z": a.z, "yaw": a.yaw}, hud))
                    last_frame = now
                sleep = step_dt - (time.perf_counter() - t0)
                if sleep > 0:
                    time.sleep(sleep)
            outcome = ("ARRIVED" if info["reached_goal"] else "crashed-out"
                       if info["crashed"] and env.terminate_on_crash else "timed out")
            print(f"  episode {ep}: {outcome}  return={ret:.0f}  "
                  f"time={info['steps'] * env.dt:.0f}s  remaining={info['route_remaining']:.0f} m")
            time.sleep(1.2)                    # hold the final frame a beat before resetting
    except KeyboardInterrupt:
        print("\nstopping the stream.")
    finally:
        srv.close(); env.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--timesteps", type=int, default=300_000)
    ap.add_argument("--n-envs", type=int, default=4)
    ap.add_argument("--eval-episodes", type=int, default=5)
    ap.add_argument("--max-steps", type=int, default=6000, help="step budget per episode")
    ap.add_argument("--corridor", type=float, default=8.0, help="on-road half-width (m)")
    ap.add_argument("--strict-collision", action="store_true",
                    help="end the episode on any building contact (else a per-step penalty)")
    ap.add_argument("--baseline-only", action="store_true",
                    help="just run the route-follower baseline (no training)")
    ap.add_argument("--save", default=None, help="path to save the trained PPO model")
    ap.add_argument("--watch", action="store_true",
                    help="spawn a live top-down video stream and drive the car (no training)")
    ap.add_argument("--model", default=None, help="saved PPO .zip to drive with --watch")
    ap.add_argument("--watch-host", default="127.0.0.1", help="stream bind host")
    ap.add_argument("--watch-port", type=int, default=8009, help="stream port")
    ap.add_argument("--watch-speed", type=float, default=3.0, help="playback speed x real time")
    args = ap.parse_args()

    if args.watch:
        watch_session(args); return

    def make_env():
        return RoadRouteEnv(max_episode_steps=args.max_steps, corridor=args.corridor,
                            terminate_on_crash=args.strict_collision)

    env = make_env()
    print(f"task: {env.instruction}")
    print(f"  start {START} -> goal {GOAL}")
    print(f"  on-road route length: {env.route.total:.0f} m  "
          f"(corridor +/-{args.corridor:.0f} m, budget {args.max_steps} steps "
          f"= {args.max_steps * env.dt:.0f}s)\n")

    print(f"route-follower baseline ({args.eval_episodes} episodes):")
    summarize("baseline", run_episodes(env, route_follow_policy(env), args.eval_episodes))
    if args.baseline_only:
        env.close(); return

    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.env_util import make_vec_env
    except ImportError:
        raise SystemExit("Install the RL trainer:  pip install stable-baselines3")

    print(f"\nbuilding {args.n_envs} vectorized envs + PPO ...")
    venv = make_vec_env(make_env, n_envs=args.n_envs, seed=0)
    model = PPO("MlpPolicy", venv, verbose=0, n_steps=1024, batch_size=256, gamma=0.999)
    eval_env = make_env()
    test_seeds = list(range(1_000_000, 1_000_000 + args.eval_episodes))  # held out

    def ppo_policy(obs, info):
        return model.predict(obs, deterministic=True)[0]

    print(f"PPO before/after training ({args.timesteps:,} timesteps, held-out seeds):")
    summarize("untrained", run_episodes(eval_env, ppo_policy, args.eval_episodes, test_seeds))
    model.learn(total_timesteps=args.timesteps)
    summarize("trained", run_episodes(eval_env, ppo_policy, args.eval_episodes, test_seeds))
    print("\n  (a 1.2 km on-road route is a long-horizon task — the route-follower is the\n"
          "   strong reference; PPO needs many timesteps, larger --n-envs, and reward\n"
          "   tuning to approach it. The reward terms are in ROUTE_REWARD.)")
    if args.save:
        model.save(args.save)
        print(f"\nsaved model -> {args.save}.zip")
    venv.close(); eval_env.close()


if __name__ == "__main__":
    main()
