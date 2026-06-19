#!/usr/bin/env python
"""Phase 2: detect vehicles in a traffic camera and spawn kinematic twin cars.

A decoupled world client (same pattern as client/twin.py): it pulls ONE camera's HLS
stream, splits the 2x2 quad, runs a YOLO model per *calibrated* sub-view, maps each
vehicle's tire point (bbox bottom-centre) through that sub-view's calibrated homography
to a scene (x,z), and drives kinematic shared-world cars (spawn / pose / despawn) so
every connected viewer sees real traffic appear, move, and leave with the vehicles.

It talks to the twin only over HTTP, so it needs no campus deps and runs wherever
ultralytics + OpenCV live (e.g. TrafficStream's GPU venv). The geometry + track
lifecycle (SceneTracker) is dependency-free and unit-tested separately; ultralytics/cv2
are imported lazily, only when actually streaming.

Prereqs: the twin server is running (python -m tools.twin_server) and the target camera
has at least one quad calibrated via the PiP tool (calibration/cameras.json).

Usage (from a venv with ultralytics+opencv, e.g. TrafficStream's):
    python -m tools.camera_detect --camera LEX-CAM-052
    python -m tools.camera_detect --camera LEX-CAM-052 --twin http://127.0.0.1:8000 \
        --model yolo26n.pt --conf 0.35 --imgsz 640 --max-fps 8

Camera feeds (c) City of Lexington, KY — personal/educational use; respect their terms.
"""
import argparse
import json
import math
import time
import urllib.request

# COCO vehicle classes (matches TrafficStream)
VEHICLE_CLASSES = {1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus"}
CLASS_COLOR = {1: 0x4aa05a, 2: 0x35c4c4, 3: 0xc77f2a, 5: 0xe0a83a}


# ---- homography apply (pure python; mirrors web/homography.js applyHomography) ----
def apply_h(H, u, v):
    x = H[0] * u + H[1] * v + H[2]
    y = H[3] * u + H[4] * v + H[5]
    w = H[6] * u + H[7] * v + H[8]
    if not math.isfinite(w) or abs(w) < 1e-18:
        return None
    return (x / w, y / w)


def heading_from_motion(dx, dz):
    """Scene motion (dx,dz) -> world heading degrees. The world's forward is
    (cos yaw, 0, -sin yaw); yaw = atan2(-dz, dx) makes forward == the motion vector."""
    return math.degrees(math.atan2(-dz, dx))


# ---- twin world client (stdlib urllib; works from any python) ----
class TwinClient:
    def __init__(self, base="http://127.0.0.1:8000"):
        self.base = base.rstrip("/")

    def _req(self, method, path, body=None):
        data = json.dumps(body).encode() if body is not None else None
        r = urllib.request.Request(self.base + path, data=data, method=method,
                                   headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(r, timeout=10) as resp:
            return json.loads(resp.read() or b"{}")

    def spawn(self, x, z, heading, source, color=0x35c4c4):
        try:
            a = self._req("POST", "/api/world/spawn", {
                "type": "car", "kinematic": True, "position": [x, None, z],
                "heading": heading, "color": color, "source": source})
            return a.get("id")
        except Exception:  # noqa: BLE001 — a flaky twin must not kill the detector
            return None

    def pose(self, aid, x, z, heading):
        try:
            self._req("POST", f"/api/world/agents/{aid}/pose", {"x": x, "z": z, "heading": heading})
        except Exception:  # noqa: BLE001
            pass

    def despawn(self, aid):
        try:
            self._req("DELETE", f"/api/world/agents/{aid}")
        except Exception:  # noqa: BLE001
            pass

    def streams(self):
        try:
            return self._req("GET", "/api/cameras/streams").get("cams", {})
        except Exception:  # noqa: BLE001
            return {}

    def publish_detections(self, camera, frame, dets):
        """Relay the image-space boxes (the viewer's PiP overlay draws these)."""
        try:
            self._req("POST", "/api/cameras/detections", {"camera": camera, "frame": frame, "dets": dets})
        except Exception:  # noqa: BLE001
            pass

    def active_camera(self):
        try:
            return self._req("GET", "/api/cameras/active").get("camera")
        except Exception:  # noqa: BLE001
            return None

    def calib(self):
        try:
            return self._req("GET", "/api/cameras/calib")
        except Exception:  # noqa: BLE001
            return {"cameras": {}}


# ---- scene-space tracker: cross-quad dedup + association + spawn/pose/despawn ----
# Tracking happens in SCENE metres (not image space) so a vehicle seen in two quads
# dedups naturally and headings come straight from real-world motion.
class SceneTracker:
    def __init__(self, twin, camera_id, dedup_m=4.0, assoc_m=8.0, lost_frames=10, min_move=0.5):
        self.twin = twin
        self.cam = camera_id
        self.dedup_m = dedup_m        # merge detections this close (cross-quad overlap)
        self.assoc_m = assoc_m        # match a detection to a track within this distance
        self.lost_frames = lost_frames  # despawn after this many frames unmatched
        self.min_move = min_move      # only update heading once moved this far (m)
        self.tracks = {}              # tid -> {agent_id,x,z,heading,last_frame,quad,cls}
        self._next = 1

    def _dedup(self, dets):
        clusters = []
        for d in dets:
            for c in clusters:
                if math.hypot(c["x"] - d["x"], c["z"] - d["z"]) <= self.dedup_m:
                    n = c["n"]
                    c["x"] = (c["x"] * n + d["x"]) / (n + 1)
                    c["z"] = (c["z"] * n + d["z"]) / (n + 1)
                    c["n"] = n + 1
                    c["cls"] = c["cls"] or d.get("cls")
                    break
            else:
                clusters.append({"x": d["x"], "z": d["z"], "n": 1, "cls": d.get("cls"), "quad": d.get("quad")})
        return clusters

    def update(self, frame, dets):
        clusters = self._dedup(dets)
        free = set(self.tracks)
        matched = set()
        # greedy nearest-neighbour association (smallest distances first)
        cand = []
        for ci, c in enumerate(clusters):
            for tid in self.tracks:
                d = math.hypot(self.tracks[tid]["x"] - c["x"], self.tracks[tid]["z"] - c["z"])
                if d <= self.assoc_m:
                    cand.append((d, ci, tid))
        cand.sort()
        for d, ci, tid in cand:
            if ci in matched or tid not in free:
                continue
            c, t = clusters[ci], self.tracks[tid]
            dx, dz = c["x"] - t["x"], c["z"] - t["z"]
            if math.hypot(dx, dz) >= self.min_move:
                t["heading"] = heading_from_motion(dx, dz)
            t["x"], t["z"], t["last_frame"] = c["x"], c["z"], frame
            self.twin.pose(t["agent_id"], c["x"], c["z"], t["heading"])
            matched.add(ci); free.discard(tid)
        # unmatched clusters -> new tracks (spawn)
        for ci, c in enumerate(clusters):
            if ci in matched:
                continue
            color = CLASS_COLOR.get(c.get("cls"), 0x35c4c4)
            aid = self.twin.spawn(c["x"], c["z"], 0.0,
                                  {"cam": self.cam, "quad": c.get("quad"), "track": self._next}, color)
            if aid is not None:
                self.tracks[self._next] = {"agent_id": aid, "x": c["x"], "z": c["z"], "heading": 0.0,
                                           "last_frame": frame, "quad": c.get("quad"), "cls": c.get("cls")}
                self._next += 1
        # reap tracks unseen for too long (the twin TTL is the backstop)
        for tid in [t for t, v in self.tracks.items() if frame - v["last_frame"] > self.lost_frames]:
            self.twin.despawn(self.tracks[tid]["agent_id"])
            del self.tracks[tid]
        return {"clusters": len(clusters), "tracks": len(self.tracks)}

    def clear(self):
        for v in self.tracks.values():
            self.twin.despawn(v["agent_id"])
        self.tracks = {}


# ---- detector: stream + YOLO per calibrated quad -> scene detections ----
def quad_homographies(calib_doc, camera_id):
    """{quad_index: H(9)} for the calibrated sub-views of `camera_id`."""
    cam = (calib_doc.get("cameras") or {}).get(camera_id) or {}
    out = {}
    for q, v in (cam.get("quads") or {}).items():
        H = v.get("H")
        if isinstance(H, list) and len(H) == 9:
            out[int(q)] = H
    return out


def tire_to_scene(H, sw, sh, x1, y1, x2, y2):
    """Bbox (pixels in a quad sub-image of size sw x sh) -> scene (x,z) via H.
    Tire point = bottom-centre, normalized to [0,1] within the quad."""
    u = ((x1 + x2) / 2) / sw
    v = y2 / sh
    return apply_h(H, u, v)


class CameraDetector:
    def __init__(self, twin, camera_id, Hq, model="yolo26n.pt", conf=0.35, imgsz=640,
                 max_det=50, publish=True, follow_active=False):
        self.twin = twin
        self.cam = camera_id
        self.Hq = Hq                  # {quad: H}
        self.model_name = model
        self.conf = conf
        self.imgsz = imgsz
        self.max_det = max_det
        self.publish = publish        # relay image boxes for the PiP overlay
        self.follow_active = follow_active  # only run inference while this camera is viewed
        self.tracker = SceneTracker(twin, camera_id)

    def _stream_url(self):
        s = self.twin.streams().get(self.cam) or {}
        return s.get("hls")

    def run(self, max_fps=8.0, stop=None, on_frame=None):
        import cv2                       # lazy: only needed to actually stream
        from ultralytics import YOLO
        url = self._stream_url()
        if not url:
            raise SystemExit(f"no live HLS for {self.cam} — is the twin server running with the camera proxy?")
        print(f"detector: {self.cam}  quads={sorted(self.Hq)}  model={self.model_name}")
        model = YOLO(self.model_name)
        cap = cv2.VideoCapture(url)
        classes = list(VEHICLE_CLASSES)
        frame, period = 0, (1.0 / max_fps if max_fps else 0)
        try:
            while not (stop and stop()):
                t0 = time.time()
                ok, img = cap.read()
                if not ok or img is None:
                    cap.release(); time.sleep(1.0); cap = cv2.VideoCapture(self._stream_url() or url); continue
                # per-active-camera perf bounding: idle (no inference) while nobody views us
                if self.follow_active and self.twin.active_camera() != self.cam:
                    self.tracker.update(frame, [])      # lets tracks age out -> despawn
                    frame += 1
                    time.sleep(0.4)
                    continue
                h, w = img.shape[:2]
                dets, raw = [], []
                for q, H in self.Hq.items():
                    col, row = q % 2, q // 2
                    sub = img[row * h // 2:(row + 1) * h // 2, col * w // 2:(col + 1) * w // 2]
                    sh, sw = sub.shape[:2]
                    res = model.predict(sub, conf=self.conf, classes=classes, imgsz=self.imgsz,
                                        max_det=self.max_det, verbose=False)[0]
                    for box in res.boxes:
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        cls = int(box.cls[0])
                        raw.append({"quad": q, "box": [x1 / sw, y1 / sh, x2 / sw, y2 / sh],
                                    "cls": cls, "conf": round(float(box.conf[0]), 2)})
                        sc = tire_to_scene(H, sw, sh, x1, y1, x2, y2)
                        if sc:
                            dets.append({"x": sc[0], "z": sc[1], "quad": q, "cls": cls})
                stat = self.tracker.update(frame, dets)
                if self.publish:
                    self.twin.publish_detections(self.cam, frame, raw)
                if on_frame:
                    on_frame(frame, dets, stat, raw)
                frame += 1
                dt = time.time() - t0
                if period > dt:
                    time.sleep(period - dt)
        finally:
            self.tracker.clear()
            cap.release()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--camera", required=True, help="camera id, e.g. LEX-CAM-052")
    ap.add_argument("--twin", default="http://127.0.0.1:8000", help="twin server base URL")
    ap.add_argument("--model", default="yolo26n.pt", help="YOLO weights (nano by default for speed)")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--max-det", type=int, default=50)
    ap.add_argument("--max-fps", type=float, default=8.0, help="detection pace (0 = uncapped)")
    ap.add_argument("--no-publish", action="store_true",
                    help="don't relay image boxes for the PiP overlay (spawn cars only)")
    ap.add_argument("--follow-active", action="store_true",
                    help="only run inference while this camera is the one being viewed "
                         "(per-active-camera perf bounding)")
    args = ap.parse_args()

    twin = TwinClient(args.twin)
    Hq = quad_homographies(twin.calib(), args.camera)
    if not Hq:
        raise SystemExit(f"{args.camera} has no calibrated quads — calibrate it in the PiP tool first.")
    det = CameraDetector(twin, args.camera, Hq, model=args.model, conf=args.conf,
                         imgsz=args.imgsz, max_det=args.max_det,
                         publish=not args.no_publish, follow_active=args.follow_active)
    try:
        det.run(max_fps=args.max_fps)
    except KeyboardInterrupt:
        print("\nstopping; clearing this camera's cars")


if __name__ == "__main__":
    main()
