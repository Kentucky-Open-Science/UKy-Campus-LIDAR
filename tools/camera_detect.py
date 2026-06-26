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
        --model yolo26x.pt --conf 0.30 --imgsz 640 --max-fps 8

Camera feeds (c) City of Lexington, KY — personal/educational use; respect their terms.
"""
import argparse
import json
import math
import threading
import time
import urllib.parse
import urllib.request

# COCO classes we map into the twin: pedestrians + every road vehicle (person, bicycle,
# car, motorcycle, bus, truck). Each maps to a twin agent TYPE sized for that class
# (CLASS_TYPE) and a COLOUR that must match web/app.js CLS_COLOR (the PiP overlay) so the
# on-video box and the spawned 3D body are the same colour.
DETECT_CLASSES = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}
CLASS_COLOR = {0: 0xe25fae, 1: 0x4aa05a, 2: 0x27c4c4, 3: 0xc77f2a, 5: 0xe0a83a, 7: 0x8e6bd0}
CLASS_TYPE = {0: "ped", 1: "bike", 2: "car", 3: "moto", 5: "bus", 7: "truck"}
DEFAULT_CAR_COLOR = 0x27c4c4


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

    def spawn(self, x, z, heading, source, color=DEFAULT_CAR_COLOR, typ="car"):
        try:
            a = self._req("POST", "/api/world/spawn", {
                "type": typ, "kinematic": True, "position": [x, None, z],
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

    def is_active(self, camera):
        """True iff `camera` currently has a live viewer (per-camera signal, so two
        viewers on two cameras don't alias one global slot)."""
        try:
            r = self._req("GET", "/api/cameras/active?camera=" + urllib.parse.quote(str(camera)))
            return bool(r.get("active"))
        except Exception:  # noqa: BLE001
            return False

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
        """Merge detections of the SAME vehicle seen across DIFFERENT quads (the only
        real duplicate source — overlap near quad seams). Detections from the SAME quad
        are never merged: one quad's YOLO already de-duplicates via NMS, so merging
        within a quad would collapse two cars in adjacent lanes (< dedup_m apart) into
        one track and drop the other. Each detection joins its NEAREST eligible cluster
        (order-independent), not the first within range, and we record the full set of
        contributing quads so a seam car is distinguishable downstream."""
        clusters = []
        for d in dets:
            dq = d.get("quad")
            best, bd = None, self.dedup_m
            for c in clusters:
                if dq is not None and dq in c["quads"]:
                    continue                      # same quad -> distinct vehicle, never merge
                dist = math.hypot(c["x"] - d["x"], c["z"] - d["z"])
                if dist <= bd:
                    bd, best = dist, c
            if best is not None:
                n = best["n"]
                best["x"] = (best["x"] * n + d["x"]) / (n + 1)
                best["z"] = (best["z"] * n + d["z"]) / (n + 1)
                best["n"] = n + 1
                best["cls"] = best["cls"] or d.get("cls")
                if dq is not None:
                    best["quads"].add(dq)
            else:
                clusters.append({"x": d["x"], "z": d["z"], "n": 1, "cls": d.get("cls"),
                                 "quad": dq, "quads": {dq} if dq is not None else set()})
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
            # Heading from CUMULATIVE motion since the last commit (anchor), not the
            # single-frame step: a slow/queued car (or one detected at < min_move per
            # 125 ms frame) still accumulates enough travel to get a correct heading,
            # instead of being pinned to +X until one frame happens to jump >= min_move.
            adx, adz = c["x"] - t["ax"], c["z"] - t["az"]
            if math.hypot(adx, adz) >= self.min_move:
                t["heading"] = heading_from_motion(adx, adz)
                t["ax"], t["az"] = c["x"], c["z"]   # re-anchor at the committed point
            t["x"], t["z"], t["last_frame"] = c["x"], c["z"], frame
            self.twin.pose(t["agent_id"], c["x"], c["z"], t["heading"])
            matched.add(ci); free.discard(tid)
        # unmatched clusters -> new tracks (spawn)
        for ci, c in enumerate(clusters):
            if ci in matched:
                continue
            cls = c.get("cls")
            color = CLASS_COLOR.get(cls, DEFAULT_CAR_COLOR)
            typ = CLASS_TYPE.get(cls, "car")        # person->ped, truck->truck, bus->bus, ...
            aid = self.twin.spawn(c["x"], c["z"], 0.0,
                                  {"cam": self.cam, "quad": c.get("quad"),
                                   "quads": sorted(c["quads"]), "cls": cls, "track": self._next},
                                  color, typ)
            if aid is not None:
                self.tracks[self._next] = {"agent_id": aid, "x": c["x"], "z": c["z"],
                                           "ax": c["x"], "az": c["z"],  # heading anchor
                                           "heading": 0.0, "last_frame": frame,
                                           "quad": c.get("quad"), "cls": c.get("cls")}
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
    def __init__(self, twin, camera_id, Hq, model="yolo26x.pt", conf=0.30, imgsz=640,
                 max_det=50, publish=True, follow_active=False, analysis=False):
        self.twin = twin
        self.cam = camera_id
        self.Hq = Hq                  # {quad: H}
        self.model_name = model
        self.conf = conf
        self.imgsz = imgsz
        self.max_det = max_det
        self.publish = publish        # relay image boxes for the PiP overlay
        self.follow_active = follow_active  # only run inference while this camera is viewed
        # analysis mode: run YOLO on EVERY quad (calibrated or not) and publish the raw
        # image-space boxes for the PiP "Analysis" overlay, but only map+spawn scene cars
        # for quads that actually have a homography. This lets an UNCALIBRATED camera be
        # bootstrapped: the Analysis flow watches these raw boxes' motion to infer traffic
        # flow and propose a calibration seed, without us spawning anything bogus.
        self.analysis = analysis
        self.tracker = SceneTracker(twin, camera_id)

    def _stream_url(self):
        s = self.twin.streams().get(self.cam) or {}
        return s.get("hls")

    def run(self, max_fps=8.0, detect_interval=None, stop=None, on_frame=None):
        import cv2                       # lazy: only needed to actually stream
        from ultralytics import YOLO
        url = self._stream_url()
        if not url:
            raise SystemExit(f"no live HLS for {self.cam} — is the twin server running with the camera proxy?")
        # which quads to run inference on: in analysis mode, ALL four (so an uncalibrated
        # camera still produces boxes for the whole frame); otherwise only calibrated ones.
        quads = [0, 1, 2, 3] if self.analysis else sorted(self.Hq)
        print(f"detector: {self.cam}  quads={quads}  calib={sorted(self.Hq)}"
              f"  model={self.model_name}{'  [analysis]' if self.analysis else ''}")
        model = YOLO(self.model_name)
        classes = list(DETECT_CLASSES)
        # Always run YOLO on the FRESHEST frame and drop whatever piled up while the model was
        # busy — so the twin tracks live traffic and automatically "skips more frames" exactly
        # when the model is slower (no fixed pacing, no interpolation; the twin cars just jump
        # to each new detection). A tiny background thread decodes the stream at its native
        # rate into a single-slot latest-frame buffer (OpenCV + torch release the GIL, so it
        # keeps draining during inference); this loop takes the latest frame, runs YOLO,
        # publishes, repeats. detect_interval / max_fps, if > 0, impose an OPTIONAL minimum gap
        # between inferences; 0 (the default) = as fast as the model can go on the live edge.
        min_gap = (detect_interval if (detect_interval is not None and detect_interval >= 0)
                   else (1.0 / max_fps if max_fps else 0.0))
        latest = {"img": None, "seq": 0}
        lk = threading.Lock()
        grab_stop = threading.Event()

        def grabber():
            cap = cv2.VideoCapture(url)
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # best-effort "keep only newest" hint
            except Exception:
                pass
            seq = 0
            while not grab_stop.is_set() and not (stop and stop()):
                ok, img = cap.read()
                if not ok or img is None:
                    cap.release(); time.sleep(1.0)
                    cap = cv2.VideoCapture(self._stream_url() or url)
                    continue
                seq += 1
                with lk:
                    latest["img"], latest["seq"] = img, seq
            cap.release()

        gth = threading.Thread(target=grabber, name=f"grab-{self.cam}", daemon=True)
        gth.start()
        frame, last_seq, last_infer, was_idle = 0, 0, 0.0, False
        try:
            while not (stop and stop()):
                now = time.time()
                # per-active-camera perf bounding: idle (no inference) while nobody views us
                if self.follow_active and not self.twin.is_active(self.cam):
                    self.tracker.update(frame, [])       # lets tracks age out -> despawn
                    if self.publish and not was_idle:    # clear the PiP overlay on going idle
                        self.twin.publish_detections(self.cam, frame, [])  # else stale boxes linger up to DET_TTL
                    was_idle = True
                    frame += 1
                    time.sleep(0.4)
                    continue
                was_idle = False
                with lk:
                    seq, img = latest["seq"], latest["img"]
                # Wait for a frame NEWER than the one we already processed (and honour an
                # optional minimum gap). Skipping straight to the latest seq is what drops
                # every stale frame that arrived while the previous inference was running.
                if img is None or seq == last_seq or (min_gap and (now - last_infer) < min_gap):
                    time.sleep(0.005)
                    continue
                last_seq, last_infer = seq, now
                h, w = img.shape[:2]
                dets, raw = [], []
                for q in quads:
                    H = self.Hq.get(q)        # None in analysis mode for uncalibrated quads
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
                        # only map a tire point into the scene (and thus spawn a car) when this
                        # quad is calibrated; analysis mode publishes raw boxes but spawns nothing
                        # for uncalibrated quads (H is None).
                        if H is None:
                            continue
                        sc = tire_to_scene(H, sw, sh, x1, y1, x2, y2)
                        if sc:
                            dets.append({"x": sc[0], "z": sc[1], "quad": q, "cls": cls})
                stat = self.tracker.update(frame, dets)
                if self.publish:
                    self.twin.publish_detections(self.cam, frame, raw)
                if on_frame:
                    on_frame(frame, dets, stat, raw)
                frame += 1
        finally:
            grab_stop.set()           # signal the grabber thread to stop, then wait for it
            self.tracker.clear()
            gth.join(timeout=2.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--camera", required=True, help="camera id, e.g. LEX-CAM-052")
    ap.add_argument("--twin", default="http://127.0.0.1:8000", help="twin server base URL")
    ap.add_argument("--model", default="yolo26x.pt",
                    help="YOLO weights. Default yolo26x (largest/most accurate) for a GPU; "
                         "drop to yolo26l/m/n if you need more speed.")
    ap.add_argument("--conf", type=float, default=0.30, help="detection confidence threshold")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--max-det", type=int, default=50)
    ap.add_argument("--max-fps", type=float, default=8.0,
                    help="legacy detection pace cap (used only when --detect-interval < 0)")
    ap.add_argument("--detect-interval", type=float, default=0.0,
                    help="min seconds between YOLO inferences (default 0 = run on the freshest "
                         "frame as fast as the model allows; stale frames are dropped so the twin "
                         "never lags). Set e.g. 1.0 to cap GPU load. <0 falls back to --max-fps.")
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
        det.run(max_fps=args.max_fps, detect_interval=args.detect_interval)
    except KeyboardInterrupt:
        print("\nstopping; clearing this camera's cars")


if __name__ == "__main__":
    main()
