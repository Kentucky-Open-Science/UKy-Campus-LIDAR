#!/usr/bin/env python
"""Anti-clip validator for the camera -> YOLO -> twin-cars pipeline.

WHAT THIS CHECKS
----------------
The Phase-2/3 traffic feature spawns one twin car per detected vehicle (through the
authoritative server in tools/twin_server.py — the /api/world/spawn API, the World/Agent
classes, and the per-type DEFS dimension table). At a stop-line several detections land
within less than a car length of each other, so their footprints can overlap. This tool
answers the concrete question, against the REAL server code (not a re-implementation):

    "If we densely spawn N kinematic 'car' agents in a tight queue/grid and run the
     server's World.tick() a few times, do any two car footprints still overlap?"

Two historical bugs made the answer 'yes':
  (A) SCALE — the DEFS table was ~70% scale (car 3.0×1.6 m), so camera cars rendered
      undersized and a 'bus' was ~8.5 m instead of a real ~12 m coach.
  (B) ANTI-CLIP — the project did collision DETECTION ONLY (no separation/resolution),
      and kinematic camera cars were exempt from even detection, so dense detections
      overlapped with nothing pushing them apart.

Both are now fixed in tools/twin_server.py: DEFS is real-world sized, and World.tick()
runs an OBB minimum-translation-vector push-apart pass (World._separate) over ALL agents
including the kinematic camera cars. This validator imports the REAL World / Agent / DEFS
and proves the live tick() drives a dense overlapping queue to ~0 penetration.

Run:  python -m tools.qa_cars
      python -m tools.qa_cars --n 8 --spacing 2.4 --ticks 8 --yaw-jitter-deg 8
"""
from __future__ import annotations

import argparse
import math

# Import the REAL simulation: same World/Agent/DEFS the server runs, and the same OBB SAT
# helper the detector/separation pass uses, so the overlap numbers here are exactly what
# the live sim would compute. (cv2/numpy/etc. used elsewhere in twin_server are fine to
# import here; torch/ultralytics are NOT needed — no YOLO is involved in this test.)
from tools.twin_server import World, Ground, Buildings, DEFS, obbObbXZ


def car_footprint():
    """(L, W, hx, hz) for a 'car' from the live DEFS, matching Agent.half = [L/2, H/2, W/2]:
    hx = L/2 (along local +X / forward), hz = W/2 (along local +Z / width)."""
    d = DEFS["car"]
    return d["L"], d["W"], d["L"] / 2.0, d["W"] / 2.0


def axis_unit(yaw):
    """Local +X and +Z axes of a body at heading `yaw` (rad) in the XZ plane.
    Mirrors twin_server / agents.js: axX=(cos,-sin), axZ=(sin,cos)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (c, -s), (s, c)


def max_penetration(cars):
    """Largest car-vs-car OBB penetration depth (m) over all pairs (0.0 if none overlap).
    `cars` is any iterable of objects exposing .x, .z, .yaw and .half (the live Agents do).
    Uses each agent's own half-extents so it stays honest if DEFS changes."""
    items = list(cars)
    worst = 0.0
    worst_pair = None
    for i in range(len(items)):
        a = items[i]
        aX, aZ = axis_unit(a.yaw)
        ahx, ahz = a.half[0], a.half[2]
        for j in range(i + 1, len(items)):
            b = items[j]
            bX, bZ = axis_unit(b.yaw)
            r = obbObbXZ(a.x, a.z, aX, aZ, ahx, ahz,
                         b.x, b.z, bX, bZ, b.half[0], b.half[2])
            if r and r[2] > worst:
                worst = r[2]
                worst_pair = (getattr(a, "id", i), getattr(b, "id", j))
    return worst, worst_pair


def spawn_dense_queue(world, n, spacing, jitter_yaw):
    """Spawn n KINEMATIC 'car' agents nose-to-tail along +X with the given centre spacing
    (m). spacing < car length guarantees initial overlap. Alternate cars get a small yaw
    so the OBB test exercises rotated boxes (not just the axis-aligned case). Returns the
    spawned Agent list. Kinematic = camera-detected cars: pose-driven, no physics."""
    agents = []
    for i in range(n):
        yaw_deg = (jitter_yaw if i % 2 else -jitter_yaw)
        a = world.spawn({
            "type": "car",
            "position": [i * spacing, 0.0, 0.0],   # [x, y, z]; queue runs along +X at z=0
            "heading": yaw_deg,
            "kinematic": True,
            "name": f"qa_car_{i}",
            "source": {"cam": "qa", "track": i},
        })
        agents.append(a)
    return agents


def spawn_dense_grid(world, rows, cols, spacing, jitter_yaw, x0=200.0, z0=200.0):
    """A tighter stress case: a rows×cols block of kinematic cars on a grid whose pitch is
    well under a car length, so every car overlaps its four neighbours at once. Offset away
    from the origin to exercise the spatial-hash broad-phase at non-zero coordinates."""
    agents = []
    k = 0
    for r in range(rows):
        for c in range(cols):
            yaw_deg = (jitter_yaw if (r + c) % 2 else -jitter_yaw)
            a = world.spawn({
                "type": "car",
                "position": [x0 + c * spacing, 0.0, z0 + r * spacing],
                "heading": yaw_deg,
                "kinematic": True,
                "name": f"qa_grid_{k}",
                "source": {"cam": "qa", "track": 1000 + k},
            })
            agents.append(a)
            k += 1
    return agents


def run_case(label, spawn_fn, ticks, hz):
    """Spawn a scenario into a fresh World, measure penetration before/after ticking, and
    return (pen_before, pen_after, n). Shares Ground/Buildings so the heightmap isn't
    reloaded; disables the kinematic TTL reaper so the cars persist across ticks."""
    ground = run_case._ground
    buildings = run_case._buildings
    world = World(hz=hz, max_agents=512, ground=ground, buildings=buildings)
    world.kinematic_ttl = 0          # don't auto-despawn kinematic cars during the test
    cars = spawn_fn(world)
    n = len(cars)

    pen_before, pair_before = max_penetration(cars)
    dt = 1.0 / hz
    for _ in range(ticks):
        world.tick(dt)               # runs integrate -> snap_ground -> _separate -> detect
    pen_after, pair_after = max_penetration(cars)

    print(f"[{label}] {n} cars")
    print(f"    initial max penetration (as the twin would DETECT it): {pen_before:.4f} m"
          + (f"  (worst pair {pair_before})" if pair_before else ""))
    print(f"    after {ticks} World.tick() separation passes:          {pen_after:.6f} m"
          + (f"  (worst pair {pair_after})" if pair_after else ""))
    # sanity that cars actually opened up (not merely that nothing overlapped to begin with)
    min_gap = min((math.hypot(a.x - b.x, a.z - b.z)
                   for i, a in enumerate(cars) for b in cars[i + 1:]),
                  default=float("inf"))
    print(f"    min centre-to-centre gap after settle:                 {min_gap:.4f} m")
    return pen_before, pen_after, n


def main():
    ap = argparse.ArgumentParser(description="Anti-clip validator for twin cars (real World).")
    ap.add_argument("--n", type=int, default=8, help="cars in the single-file queue (default 8)")
    ap.add_argument("--spacing", type=float, default=2.4,
                    help="centre spacing in m (default 2.4; < car length forces overlap)")
    ap.add_argument("--ticks", type=int, default=12,
                    help="World.tick() calls (default 12; the dense 2D grid needs a few "
                         "ticks for the push-apart to propagate from the centre outward)")
    ap.add_argument("--hz", type=float, default=50.0, help="sim rate for dt (default 50)")
    ap.add_argument("--yaw-jitter-deg", type=float, default=8.0,
                    help="alternate-car yaw so the OBB test exercises rotated boxes")
    ap.add_argument("--grid", type=int, default=4,
                    help="side of the NxN dense grid stress case (default 4; 0 to skip)")
    args = ap.parse_args()

    L, W, _, _ = car_footprint()
    print(f"DIMS: car footprint from live DEFS = {L:.2f} x {W:.2f} m "
          f"(half-extents {L / 2:.2f} x {W / 2:.2f}); "
          f"bus {DEFS['bus']['L']:.1f} m, truck {DEFS['truck']['L']:.1f} m\n")
    jit = math.radians(args.yaw_jitter_deg)
    print(f"scenario: queue of {args.n} cars at centre spacing {args.spacing} m "
          f"(< {L:.2f} m car length), yaw jitter +/-{args.yaw_jitter_deg} deg; "
          f"plus a {args.grid}x{args.grid} dense grid at {args.spacing} m pitch\n")

    # Load the heavy world data once and share across cases.
    run_case._ground = Ground()
    run_case._buildings = Buildings()

    cases = [("queue", lambda w: spawn_dense_queue(w, args.n, args.spacing, jit))]
    if args.grid and args.grid >= 2:
        cases.append((f"grid{args.grid}x{args.grid}",
                      lambda w: spawn_dense_grid(w, args.grid, args.grid, args.spacing, jit)))

    TOL = 1e-3
    worst_before = 0.0
    worst_after = 0.0
    for label, fn in cases:
        pb, pa, _ = run_case(label, fn, args.ticks, args.hz)
        worst_before = max(worst_before, pb)
        worst_after = max(worst_after, pa)
        print()

    ok = worst_after <= TOL and worst_before > TOL   # must have actually stressed it AND cleared it
    print("=" * 64)
    print(f"max penetration BEFORE separation: {worst_before:.4f} m  (overlapping queue, as designed)")
    print(f"max penetration AFTER  separation: {worst_after:.6f} m  (target 0, tol {TOL} m)")
    print()
    print(f"RESULT: {'PASS' if ok else 'FAIL'} — the live World.tick() separation pass drives "
          f"dense kinematic camera cars to ~0 overlap")
    if not ok and worst_before <= TOL:
        print("  (note: nothing overlapped initially — pick a tighter --spacing to stress it)")

    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
