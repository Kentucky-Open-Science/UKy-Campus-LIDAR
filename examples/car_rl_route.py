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

Watch it in the REAL 3-D digital twin (spawns the car in a running twin_server and
mirrors it as you train + validate — open the viewer to watch):
    python -m tools.twin_server                          # in another terminal (viewer on :8000)
    python examples/car_rl_route.py --twin http://localhost:8000              # baseline -> train -> validate, in the twin
    python examples/car_rl_route.py --twin http://localhost:8000 --baseline-only   # just watch the route-follower
    python examples/car_rl_route.py --twin http://localhost:8000 --model ppo_route.zip  # watch a trained policy

Also pull the FIRST-PERSON camera the car receives from the server (what it sees) and
re-serve it as a video stream — needs the twin started with --render:
    python -m tools.twin_server --render                 # camera feed on
    python examples/car_rl_route.py --twin http://localhost:8000 --watch   # opens a stream at :8009
"""
import argparse
import math
import os
import sys
import threading

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


def _parse_size(s):
    try:
        w, h = str(s).lower().split("x")
        return int(w), int(h)
    except Exception:  # noqa: BLE001
        return 640, 480


class CameraRelay(threading.Thread):
    """Pull the mirrored car's FIRST-PERSON camera from the twin_server and republish it
    to a local MJPEG StreamServer — the stream the car receives from the server. The
    headless renderer is the bottleneck (~1-3 fps); we relay whatever it produces. Needs
    the twin started with --render (else the camera endpoint 503s).

    Uses its OWN twin client with a longer timeout so a slow render never stalls the fast
    pose-mirroring loop (which uses a short timeout)."""

    def __init__(self, mirror, stream, size=(640, 480), timeout=10):
        super().__init__(daemon=True)
        from twin import Twin
        self.mirror = mirror
        self.stream = stream
        self.w, self.h = size
        self._twin = Twin(mirror.twin.base, timeout=timeout)
        self._stop = False
        self.frames = 0
        self._warned = False

    def run(self):
        import time
        while not self._stop:
            a = self.mirror.agent
            if a is None:
                time.sleep(0.1); continue
            try:
                self.stream.publish(self._twin.get(a.id).camera(self.w, self.h))
                self.frames += 1
            except Exception as e:  # noqa: BLE001 — keep relaying through transient errors
                if "no such agent" in str(e):
                    time.sleep(0.3)     # ghost TTL-reaped during a training pause; it'll respawn
                    continue
                if not self._warned:    # a genuine camera-off (no --render) or other fault
                    print(f"[watch] camera unavailable: {e}\n"
                          f"        is the twin started with --render?", flush=True)
                    self._warned = True
                time.sleep(0.5)

    def stop(self):
        self._stop = True


# ----------------------------------------------------- watch in the real twin ---
class MirrorTwin:
    """Mirror the gym car's pose into a running twin_server as a kinematic 'ghost', so
    you watch it drive in the real 3-D digital-twin viewer. The gym and the twin share
    the same scene coordinates, so the ghost lands on the right roads. It carries no
    physics (pose-driven) and auto-despawns ~5 s after updates stop, so we re-spawn it
    if it was reaped (e.g. during a training pause)."""

    def __init__(self, url, color=0xffd54a, owner="rl-route", timeout=3):
        from twin import Twin, TwinError       # vendored client in this directory
        self._Twin, self._TwinError = Twin, TwinError
        # short timeout so a slow/hung server can't stall the real-time pose loop for long
        # (transport timeouts now surface as TwinError, see twin.py _req)
        self.twin = Twin(url, owner=owner, timeout=timeout)
        self.color = color
        self.agent = None
        self._fail = 0
        try:
            self.twin.meta()                    # fail fast with a clear message if it's down
        except TwinError as e:
            raise SystemExit(f"can't reach the twin at {url} — start it first with\n"
                             f"    python -m tools.twin_server\n({e})")
        self.url = self.twin.base

    def _spawn(self, x, y, z, heading_deg):
        self.agent = self.twin.spawn("car", position=[x, y, z], heading=heading_deg,
                                     color=self.color, kinematic=True,
                                     source={"sim": "campus_gym", "task": "rl-route"})

    def update(self, x, y, z, heading_deg):
        """Push the current pose; (re)spawn the ghost if it's missing or was TTL-reaped.
        Swallows transport/HTTP errors (a momentarily slow/hung server mustn't crash the
        run or the SB3 training callback); warns the first time and periodically after."""
        try:
            if self.agent is None:
                self._spawn(x, y, z, heading_deg)
            else:
                try:
                    self.agent.pose(x, z, y=y, heading=heading_deg)
                except (self._TwinError, OSError):
                    # likely a TTL reap (or a hiccup): drop the stale handle (best-effort
                    # despawn so we don't leave a duplicate) and respawn a fresh ghost.
                    stale, self.agent = self.agent, None
                    try:
                        stale.despawn()
                    except (self._TwinError, OSError):
                        pass
                    self._spawn(x, y, z, heading_deg)
            self._fail = 0
        except (self._TwinError, OSError) as e:
            self.agent = None
            self._fail += 1
            if self._fail == 1 or self._fail % 60 == 0:
                print(f"[twin] mirror update failed ({e}); retrying ...", flush=True)

    def close(self):
        try:
            if self.agent is not None:
                self.agent.despawn()
        except (self._TwinError, OSError):
            pass
        self.agent = None


def mirror_rollout(env, policy, mirror, speed=1.0, episodes=1, loop=False,
                   step_cap=None, label="drive", push_hz=30.0):
    """Roll out policy(obs, info) and mirror each pose into the twin, paced to
    `speed` x real time so it looks natural in the viewer."""
    import time
    step_dt = env.dt / max(speed, 1e-6)
    push_dt = 1.0 / max(push_hz, 1e-6)
    ep = 0
    while True:
        obs, info = env.reset(seed=ep)
        ep += 1
        done, last_push, steps = False, 0.0, 0
        next_t = time.perf_counter()
        while not done:
            obs, r, term, trunc, info = env.step(policy(obs, info))
            steps += 1
            done = term or trunc or (step_cap is not None and steps >= step_cap)
            now = time.perf_counter()
            if now - last_push >= push_dt or done:
                a = env.agent
                mirror.update(a.x, a.y, a.z, math.degrees(a.yaw))
                last_push = now
            # pace against an absolute schedule so a slow push is amortized, not compounded
            next_t += step_dt
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            elif sleep < -step_dt:
                next_t = time.perf_counter()    # fell far behind; reset rather than sprint
        print(f"  [{label}] ep {ep}: reached={info['reached_goal']}  "
              f"time={info['steps'] * env.dt:.0f}s  remaining={info['route_remaining']:.0f} m  "
              f"on-road={info['on_road_frac']:.0%}")
        if not loop and ep >= episodes:
            return


def _twin_eval_callback(eval_env, mirror, speed, freq, step_cap):
    """SB3 callback: every `freq` timesteps, mirror one eval episode of the CURRENT
    policy into the twin so you watch the agent improve while it trains."""
    from stable_baselines3.common.callbacks import BaseCallback

    class _CB(BaseCallback):
        def __init__(self):
            super().__init__()
            self._next = freq

        def _on_step(self):
            if self.num_timesteps >= self._next:
                self._next += freq
                pol = lambda obs, info: self.model.predict(obs, deterministic=True)[0]
                print(f"\n[twin] mirroring the policy at {self.num_timesteps:,} steps "
                      f"— watch the viewer ...", flush=True)
                mirror_rollout(eval_env, pol, mirror, speed=speed, episodes=1,
                               step_cap=step_cap, label=f"train@{self.num_timesteps}")
            return True

    return _CB()


def twin_session(args):
    """Spawn the car in the REAL digital twin and drive it there: watch a policy
    (validation) or train with periodic mirrored evals — all visible in the 3-D viewer.
    With --watch, also re-serve the car's first-person camera (what it sees) as a stream."""
    mirror = MirrorTwin(args.twin)
    env = RoadRouteEnv(max_episode_steps=args.max_steps, corridor=args.corridor,
                       terminate_on_crash=args.strict_collision, spawn_jitter=0.0)
    venv = eval_env = model = stream = relay = None

    cam_url = None
    if args.watch:                              # pull the car's first-person camera feed
        from route_view import StreamServer
        if not mirror.twin.meta().get("camera"):
            raise SystemExit("the twin has no first-person camera feed — restart it with:\n"
                             "    python -m tools.twin_server --render")
        stream = StreamServer()
        try:
            cam_url = stream.start(args.watch_host, args.watch_port)
        except OSError as e:
            raise SystemExit(f"could not bind {args.watch_host}:{args.watch_port} ({e}); "
                             f"pick another with --watch-port")
        relay = CameraRelay(mirror, stream, _parse_size(args.watch_size))
        relay.start()

    print("\n" + "=" * 64)
    print(f"  the car is driving in the digital twin. Open:\n")
    print(f"    3-D viewer (third person) : {mirror.url}/")
    if cam_url:
        print(f"    car camera (first person) : {cam_url}")
    print(f"\n  route {env.route.total:.0f} m, playback {args.twin_speed:g}x real time."
          "  Ctrl-C to stop.")
    print("=" * 64 + "\n", flush=True)
    try:
        if args.model:
            m = _load_ppo(args.model)
            pol = lambda obs, info: m.predict(obs, deterministic=True)[0]
            print(f"watching {os.path.basename(args.model)} in the twin (loops; Ctrl-C to stop)\n")
            mirror_rollout(env, pol, mirror, speed=args.twin_speed, loop=True,
                           label=os.path.basename(args.model))
            return
        if args.baseline_only:
            print("watching the route-follower in the twin (loops; Ctrl-C to stop)\n")
            mirror_rollout(env, route_follow_policy(env), mirror, speed=args.twin_speed,
                           loop=True, label="route-follower")
            return

        try:
            from stable_baselines3 import PPO
            from stable_baselines3.common.env_util import make_vec_env
        except ImportError:
            raise SystemExit("Install the RL trainer:  pip install stable-baselines3")

        print("1/3  baseline route-follower (validation) — watch it in the twin\n")
        mirror_rollout(env, route_follow_policy(env), mirror, speed=args.twin_speed,
                       episodes=1, label="baseline")

        def make_env():
            return RoadRouteEnv(max_episode_steps=args.max_steps, corridor=args.corridor,
                                terminate_on_crash=args.strict_collision)
        print(f"\n2/3  training PPO ({args.timesteps:,} steps); mirroring the policy "
              f"every {args.twin_eval_freq:,} steps\n", flush=True)
        venv = make_vec_env(make_env, n_envs=args.n_envs, seed=0)
        model = PPO("MlpPolicy", venv, verbose=0, n_steps=1024, batch_size=256, gamma=0.999)
        eval_env = RoadRouteEnv(max_episode_steps=args.max_steps, corridor=args.corridor,
                                terminate_on_crash=args.strict_collision, spawn_jitter=0.0)
        cb = _twin_eval_callback(eval_env, mirror, args.twin_speed, args.twin_eval_freq,
                                 step_cap=min(args.max_steps, 2500))
        model.learn(total_timesteps=args.timesteps, callback=cb)

        print("\n3/3  trained policy (validation) — watch it in the twin\n")
        pol = lambda obs, info: model.predict(obs, deterministic=True)[0]
        mirror_rollout(eval_env, pol, mirror, speed=args.twin_speed, episodes=2, label="trained")
    except KeyboardInterrupt:
        print("\nstopping.")
    finally:
        # save the (possibly partially-) trained model even on Ctrl-C, if requested
        if args.save and model is not None:
            try:
                model.save(args.save)
                print(f"saved model -> {args.save}.zip", flush=True)
            except Exception as e:  # noqa: BLE001
                print(f"could not save model: {e}", flush=True)
        if relay is not None:
            relay.stop()
        if stream is not None:
            stream.close()
        for closeable in (venv, eval_env):
            if closeable is not None:
                try:
                    closeable.close()
                except Exception:  # noqa: BLE001
                    pass
        mirror.close(); env.close()


def _load_ppo(path):
    try:
        from stable_baselines3 import PPO
    except ImportError:
        raise SystemExit("--model needs the RL trainer:  pip install stable-baselines3")
    if not (os.path.exists(path) or os.path.exists(path + ".zip")):
        raise SystemExit(f"no such model file: {path}")
    return PPO.load(path)


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
    ap.add_argument("--twin", default=None, metavar="URL",
                    help="watch in the REAL 3-D digital twin: mirror the car into a running "
                         "twin_server (e.g. http://localhost:8000). Trains + validates there.")
    ap.add_argument("--watch", action="store_true",
                    help="with --twin: also stream the car's FIRST-PERSON camera (what it sees "
                         "from the server; needs the twin started with --render)")
    ap.add_argument("--model", default=None, help="saved PPO .zip to drive with --twin")
    ap.add_argument("--twin-speed", type=float, default=1.0, help="twin playback speed x real time")
    ap.add_argument("--twin-eval-freq", type=int, default=20_000,
                    help="during training, mirror an eval episode into the twin every N steps")
    ap.add_argument("--watch-host", default="127.0.0.1", help="first-person stream bind host")
    ap.add_argument("--watch-port", type=int, default=8009, help="first-person stream port")
    ap.add_argument("--watch-size", default="640x480", help="first-person stream frame size WxH")
    args = ap.parse_args()

    if args.watch and not args.twin:
        raise SystemExit("--watch streams the car's camera from the twin server; also pass "
                         "--twin http://HOST:8000 (started with --render).")
    if args.twin:
        twin_session(args); return

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
