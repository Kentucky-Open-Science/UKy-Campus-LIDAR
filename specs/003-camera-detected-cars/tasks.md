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
