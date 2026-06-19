# Tasks — Phase 1: geometry + kinematic spawn

Ordered so each layer is verified before the next stacks on it.

## T1 — Homography solver (web/homography.js) + unit tests
- [ ] `solveHomography(src,dst)` (DLT; 4-pt exact, >4 least-squares; Gaussian elim; no deps)
- [ ] `applyHomography(H,[u,v])`, `invertHomography(H)`
- [ ] Node test: recover identity/known transforms; round-trip points < 1e-6; >4-pt LSQ;
      reject collinear. **Gate: tests pass before UI uses it.**

## T2 — Kinematic shared-world agents (tools/twin_server.py)
- [ ] `Agent.kinematic`, `set_pose`, `last_update`; `integrate` early-return when kinematic
- [ ] `World.spawn` passes kinematic; `World.tick` TTL sweep (default 5 s)
- [ ] `POST /api/world/agents/<id>/pose`; update docstring for the kinematic spawn flag
- [ ] API test: spawn kinematic → in state, no drift; pose moves it; DELETE; TTL despawn

## T3 — netagents flat-height fix (web/netagents.js)
- [ ] Import FLAT mode; render shared-world agents at FLAT_Y (drape) when flat
- [ ] Headless: kinematic car renders at FLAT_Y and the set x/z

## T4 — Calibration store (tools/twin_server.py + calibration/cameras.json)
- [ ] `GET/POST /api/cameras/calib` reading/merging `calibration/cameras.json`
- [ ] Seed file + `.gitignore` check (must be tracked, not under web/data/)
- [ ] API test: POST a calib → GET returns it; persists across restart

## T5 — PiP calibration UI (web/cameras.js + app.js + style.css + index.html)
- [ ] Calibrate mode: 2×2 quad grid overlay, quad select, image-point clicks (normalized)
- [ ] Ground-point pick via 3D raycast; pair points; solve + save homography
- [ ] Reproject twin reference points onto the video (H⁻¹) for judging the fit
- [ ] `window.__twin.cameras.getCalib/setCalib/imageToScene`

## T6 — Click-to-spawn test mode (web/cameras.js + app.js)
- [ ] Calibrated image click → imageToScene → POST kinematic spawn (source-tagged)
- [ ] "Clear cars" despawns click-cars
- [ ] Headless end-to-end: inject calibration → click pixel → car at expected scene point
      in a second viewer; clear removes it

## T7 — Verify + document + PR
- [ ] Run all three test tiers green; capture evidence
- [ ] README note (Traffic cameras: calibrate + click-to-spawn; Phase 2/3 to come)
- [ ] Branch `feat/camera-cars-phase1` → PR with test evidence

## Phase 2 — live detector (DONE)

`tools/camera_detect.py` — a decoupled world client (runs in a venv with ultralytics +
opencv, e.g. TrafficStream's; talks to the twin over HTTP only):

- [x] T8 — `apply_h` (pure-python homography apply; cross-checked == `web/homography.js`)
      + `heading_from_motion` (scene motion → world heading)
- [x] T9 — `TwinClient` (stdlib urllib): spawn/pose/despawn/streams/calib
- [x] T10 — `SceneTracker`: scene-space cross-quad **dedup**, greedy nearest-neighbour
      **association** (stable ids), spawn / pose (motion-heading) / despawn-when-lost.
      Unit-tested 13/13 (mock twin) + integration 7/7 (real twin world API).
- [x] T11 — `CameraDetector`: pull the camera HLS, split the 2×2 quad, run YOLO per
      **calibrated** quad, tire point (bbox bottom-centre) → homography → scene → tracker.
      Lazy ultralytics/cv2 import so the core stays testable without a GPU.
- [x] T12 — Real YOLO smoke on the live Harrodsburg/Lakespur feed: 40/40 frames detected
      vehicles (369 dets, up to 11 simultaneous tracks) → scene points → twin cars.

Run: `python -m tools.camera_detect --camera LEX-CAM-052` (twin running; quad calibrated).

## Phase 3 — detection overlay + perf bounding (DONE)

Decision: the PiP boxes are **server-published** (the detector relays the exact boxes it
spawns cars from), not an in-browser model — fully testable, reuses the accurate YOLO,
shows "this detection → this twin car". Tradeoff: ~1–2 s offset from the buffered video.

- [x] T13 — twin server relay: `GET/POST /api/cameras/detections` (per-camera, TTL'd) +
      `GET/POST /api/cameras/active` (which camera is viewed). API tested 8/8.
- [x] T14 — `camera_detect.py` publishes its image boxes each frame (`--no-publish` to
      opt out); `--follow-active` idles inference while its camera isn't being viewed.
- [x] T15 — PiP **Detect** toggle: polls the relay, draws class-coloured boxes on the
      overlay (quad-aware). Opening a PiP signals the active camera; closing clears it.
- [x] T16 — Headless overlay test 8/8 (open → active signalled → Detect → boxes polled +
      drawn → close → cleared) + real YOLO publish to the relay (9 live boxes).

Deferred (optional, not needed for the feature): ByteTrack for occlusion robustness;
fisheye undistortion for tighter positioning; an in-browser model for frame-perfect PiP
sync.
