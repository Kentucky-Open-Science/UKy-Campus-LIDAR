#!/usr/bin/env python3
"""Drive a TRUCK autonomously by vision: first-person camera -> YOLO -> controls.

The truck spawns into the SHARED twin world and navigates using only what its camera
sees (a YOLO model picks out obstacles; it steers around them and cruises otherwise).
Everyone connected — other scripts and any browser — sees it drive.

Start the server with the camera feed on, then run this:
    python -m tools.twin_server --render            # twin + first-person cameras on :8000
    python examples/truck_demo.py                    # this script (any machine: --url HOST:8000)
    python examples/truck_demo.py --port 9000 --model yolov8x.pt
    # open http://localhost:8000/ to watch

Runs standalone from this directory (twin.py + yolo_drive.py) plus third-party deps
(ultralytics, pillow). Press Ctrl-C any time — the truck is despawned from the server.
"""
import argparse

from twin import Twin, TwinError, with_port
from yolo_drive import load_model, navigate

TYPE, COLOR, START = "truck", 0xc7702a, [-127.0, None, 1224.1]  # x,z on the map; None height auto-snaps onto the terrain


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--url", default="http://localhost:8000", help="twin server base URL")
    ap.add_argument("--port", type=int, default=None, help="override the port in --url")
    ap.add_argument("--owner", default=f"{TYPE}-demo")
    ap.add_argument("--seconds", type=int, default=60)
    ap.add_argument("--model", default="yolov8n.pt",
                    help="YOLO model id/path (e.g. yolov8n.pt .. yolov8x.pt)")
    args = ap.parse_args()

    url = with_port(args.url, args.port)
    twin = Twin(url, owner=args.owner)
    try:
        meta = twin.meta()
    except TwinError as e:
        print("ERROR:", e); return 1
    if not meta.get("camera"):
        print("ERROR: this server has no first-person camera feed.\n"
              "Restart it with:  python -m tools.twin_server --render")
        return 1

    print(f"loading YOLO model {args.model} ...")
    model = load_model(args.model)
    agent = twin.spawn(TYPE, position=START, color=COLOR)
    print(f"spawned {TYPE} #{agent.id} — navigating by vision for {args.seconds}s "
          f"(open {url}/ to watch)\n")
    try:
        navigate(agent, model, seconds=args.seconds, name=f"{TYPE}#{agent.id}")
        print("\ndone — stopping and despawning.")
    except KeyboardInterrupt:
        print("\ninterrupted (Ctrl-C) — despawning.")
    finally:
        try:
            agent.despawn()
        except Exception as e:  # noqa: BLE001
            print("  warning: despawn failed:", e)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
