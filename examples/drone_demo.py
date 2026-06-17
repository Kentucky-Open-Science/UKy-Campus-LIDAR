#!/usr/bin/env python3
"""Spawn a drone in the campus digital twin, fly it around, detect a collision, stop.

This talks to the twin SERVER (tools/twin_server.py) over its REST API using the
`client/twin.py` wrapper — so it runs anywhere, the twin lives on its own server, and
the drone it spawns is part of the SHARED world: anyone else connected (another script
or a browser viewing http://<server>:8000/) sees this drone fly and crash in real time.

First, on the server machine:
    python -m tools.twin_server                 # serves the viewer + the world API on :8000

Then run this from any machine:
    python examples/drone_demo.py                       # talks to localhost:8000
    python examples/drone_demo.py --url http://HOST:8000
    python examples/drone_demo.py --owner alice

Open http://<server>:8000/ in a browser while it runs to watch the drone in 3-D.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "client")))
from twin import Twin, TwinError  # noqa: E402


def watch(agent, seconds, label, target=None, arrive=6.0, until_hit=False):
    """Poll the agent for a while, printing telemetry; stop early on arrival/collision."""
    deadline = time.time() + seconds
    last = None
    while time.time() < deadline:
        s = agent.state()
        last = s
        hits = s.get("collisions", [])
        p = s["position"]
        print(f"  [{label}] pos=({p[0]:7.1f},{p[1]:6.1f},{p[2]:7.1f}) "
              f"AGL={s.get('altitudeAGL') or 0:5.1f}m  speed={s['speed']:4.1f}m/s  "
              f"surface={s['surface']:<8} contacts={len(hits)}")
        if until_hit and hits:
            return s
        if target is not None:
            dx, dz = target[0] - p[0], target[2] - p[2]
            if (dx * dx + dz * dz) ** 0.5 < arrive:
                return s
        time.sleep(0.4)
    return last


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--url", default="http://localhost:8000", help="twin server URL")
    ap.add_argument("--owner", default="drone-demo", help="tag agents with this owner")
    args = ap.parse_args()

    twin = Twin(args.url, owner=args.owner)
    try:
        meta = twin.meta()
    except TwinError as e:
        print("ERROR:", e); return 1
    print(f"connected to twin at {args.url}  (sim {meta['hz']} Hz, "
          f"ground {'on' if meta['ground'] else 'flat'}, {meta['buildings']} buildings)")

    # who else is in the shared world right now?
    others = twin.agents()
    if others:
        print(f"shared world already has {len(others)} agent(s):")
        for o in others:
            print(f"  - #{o['id']} {o['type']} '{o['name']}' (owner {o['owner']}) at "
                  f"{[round(v,1) for v in o['position']]}")
    else:
        print("shared world is empty — your drone will be the first agent in it.")

    # 1. spawn -------------------------------------------------------------
    drone = twin.spawn("drone", position=[-127.0, 303.6, 1224.1], color=0xff3344)
    s = drone.state()
    sx, sy, sz = s["position"]
    utm = s.get("utm")
    print(f"\nspawned drone #{drone.id} at scene ({sx:.1f},{sy:.1f},{sz:.1f})"
          + (f"  UTM-16N {round(utm['easting'])},{round(utm['northing'])}" if utm else ""))

    # 2. fly a square circuit at hover altitude ----------------------------
    print("\nflying a square circuit ...")
    for i, (wx, wz) in enumerate([(sx + 70, sz), (sx + 70, sz - 70),
                                  (sx, sz - 70), (sx, sz)], 1):
        drone.drive_to(wx, wz, y=sy, speed=10, arrive_radius=6)
        watch(drone, 7, f"leg {i}/4", target=(wx, sy, wz), arrive=7)

    # 3. drop onto the nearest building and fly into it --------------------
    b = twin.nearest_building(sx, sz)
    if b:
        print(f"\nnearest building: {b['name']} ({b['dist']} m away) — flying into it "
              f"at roof height to trigger a collision ...")
        drone.drive_to(b["x"], b["z"], y=b["y"], speed=9, arrive_radius=4)
    else:
        print("\nno buildings nearby — flying forward at low altitude until something is hit ...")
        drone.drive_to(sx + 250, sz, y=sy - 12, speed=9)
    s = watch(drone, 16, "approach", until_hit=True)

    # 4. report + stop -----------------------------------------------------
    hits = (s or {}).get("collisions", [])
    if hits:
        c = hits[0]
        print(f"\nCOLLISION DETECTED — hit {c['with']} '{c['name']}' "
              f"(penetration {c['penetration']} m, closing speed {c['relativeSpeed']} m/s)")
    else:
        print("\nno collision within the time budget.")

    print("stopping the drone ...")
    drone.stop()
    final = watch(drone, 2, "stopping")
    print(f"\ndone — drone at rest, speed {final['speed']} m/s. It stays in the shared "
          f"world (despawning); other clients saw the whole flight.")
    drone.despawn()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
