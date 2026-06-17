#!/usr/bin/env python3
"""Drive a TRUCK autonomously by vision: first-person camera -> YOLO -> controls.

The truck spawns into the SHARED twin world and navigates using only what its camera
sees (the smallest YOLO model picks out obstacles; it steers around them and cruises
otherwise). Everyone connected — other scripts and any browser — sees it drive.

Start the server with the camera feed on, then run this:
    python -m tools.twin_server --render            # twin + first-person cameras on :8000
    python examples/truck_demo.py                    # this script (any machine: --url HOST:8000)
    # open http://localhost:8000/ to watch
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "client")))
sys.path.insert(0, os.path.dirname(__file__))
from twin import Twin, TwinError           # noqa: E402
from yolo_drive import load_model, navigate  # noqa: E402

TYPE, COLOR, START = "truck", 0xc7702a, [60, None, 10]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--url", default="http://localhost:8000")
    ap.add_argument("--owner", default=f"{TYPE}-demo")
    ap.add_argument("--seconds", type=int, default=60)
    args = ap.parse_args()

    twin = Twin(args.url, owner=args.owner)
    try:
        meta = twin.meta()
    except TwinError as e:
        print("ERROR:", e); return 1
    if not meta.get("camera"):
        print("ERROR: this server has no first-person camera feed.\n"
              "Restart it with:  python -m tools.twin_server --render")
        return 1

    print("loading the smallest YOLO model (yolov8n) ...")
    model = load_model()
    agent = twin.spawn(TYPE, position=START, color=COLOR)
    print(f"spawned {TYPE} #{agent.id} — navigating by vision for {args.seconds}s "
          f"(open {args.url}/ to watch)\n")
    navigate(agent, model, seconds=args.seconds, name=f"{TYPE}#{agent.id}")
    print("\ndone — stopping and despawning.")
    agent.despawn()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
