"""Vision-based control shared by the car / truck / robot demos.

Pulls an agent's first-person camera from the twin server, runs a YOLO model on each
frame, and steers to avoid whatever it sees — a minimal "drive by what the camera
shows" loop. Each demo picks the model with --model (default yolo26n.pt, the smallest
of the YOLO26 family). The twin server must be started with `--render` so the camera feed exists
(`python -m tools.twin_server --render`).

YOLO is pretrained on COCO, so it flags things like cars, trucks, and people — e.g.
vehicles in the aerial ground imagery and other agents that cross the view. Where it
sees nothing, the agent just cruises. Install once: `pip install ultralytics`.
"""
import os
import time

_MODEL = None
_MODEL_NAME = None


def load_model(name="yolo26n.pt"):
    """Load a YOLO model once and cache it (weights auto-download on first use).

    `name` is any Ultralytics model id/path (e.g. "yolo26n.pt" .. "yolo26x.pt"). A bare
    filename that sits next to this script is used directly, so the demos find their
    bundled weights no matter which directory you launch them from.
    """
    global _MODEL, _MODEL_NAME
    if _MODEL is not None and _MODEL_NAME == name:
        return _MODEL
    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit(
            "YOLO not installed. Install the vision model with:\n"
            "    pip install ultralytics\n"
            "(it pulls in torch; the weights download automatically on first run)")
    resolved = name
    local = os.path.join(os.path.dirname(os.path.abspath(__file__)), name)
    if not os.path.dirname(name) and os.path.exists(local):
        resolved = local          # prefer weights bundled in this directory
    _MODEL = YOLO(resolved)
    _MODEL_NAME = name
    return _MODEL


def detect(model, pil_img, conf=0.25):
    """Run YOLO on a PIL image -> list of {cls, conf, cx, cy, area} (normalised 0..1)."""
    res = model.predict(pil_img, conf=conf, verbose=False)[0]
    w, h = pil_img.size
    out = []
    for b in res.boxes:
        x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
        out.append({
            "cls": model.names[int(b.cls[0])], "conf": float(b.conf[0]),
            "cx": (x1 + x2) / 2 / w, "cy": (y1 + y2) / 2 / h,
            "area": (x2 - x1) * (y2 - y1) / (w * h),
        })
    return out


def steer_from(dets, area_thresh=0.05):
    """Turn detections into (throttle, steer). Avoid the biggest object dead ahead;
    cruise when the path looks clear. steer>0 turns left (away from a right-side object)."""
    ahead = [d for d in dets if 0.25 < d["cx"] < 0.75 and d["area"] > area_thresh]
    if not ahead:
        return 0.45, 0.0                                  # clear -> cruise (eased; YOLO caps the loop rate)
    big = max(ahead, key=lambda d: d["area"])
    steer = 1.0 if big["cx"] >= 0.5 else -1.0             # steer away from its side
    throttle = 0.2 if big["area"] > 0.25 else 0.35        # ease off if it's very close
    return throttle, steer


def navigate(agent, model, seconds=60, fps=4, frame=(416, 320), name="agent"):
    """Camera -> YOLO -> controls loop. Recovers (turns away) on a collision."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        try:
            img = agent.camera_image(*frame)
        except Exception as e:  # noqa: BLE001
            print("  camera unavailable:", e, "(start the server with --render)")
            break
        dets = detect(model, img)
        throttle, steer = steer_from(dets)
        agent.set_controls(throttle=throttle, steer=steer)

        s = agent.state()
        p = s["position"]
        labels = ", ".join(sorted({d["cls"] for d in dets})) or "(none)"
        line = (f"  {name} pos=({p[0]:6.0f},{p[2]:6.0f}) speed={s['speed']:4.1f} "
                f"throttle={throttle:.2f} steer={steer:+.0f}  YOLO[{len(dets):2d}]: {labels}")
        if s["collisions"]:
            line += f"  COLLISION x{len(s['collisions'])}"
        print(line)

        # stuck against something -> turn hard for a moment to break free
        if s["collisions"] and s["speed"] < 0.6:
            agent.set_controls(throttle=0.35, steer=1.0)
            time.sleep(0.6)
        time.sleep(max(0.0, 1.0 / fps))
    agent.stop()
