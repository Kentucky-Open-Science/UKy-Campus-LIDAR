# Implementation Plan — Phase 1: geometry + kinematic spawn

Mirrors the established live-layer pattern (constitution Principle 3). No ML in Phase 1.

## Architecture

Detected cars become **kinematic shared-world agents**, which `web/netagents.js` already
renders for every connected viewer — so there is no new car-rendering layer, only a
kinematic path in the server and a height fix in the client. The Phase 2 detector will be
a decoupled world client (same pattern as `client/twin.py`) that calls the kinematic API.

## Components

### Server — `tools/twin_server.py`
- `Agent`: `self.kinematic = bool(opts.get("kinematic"))`. `integrate()` returns early
  for kinematic (no physics). Add `set_pose(x, z, y=None, heading=None)` and
  `self.last_update` timestamp. `snap_ground` still records groundY but does not move a
  kinematic car off its set pose (it may set y; the client overrides height anyway).
- `World`: accept `kinematic` through `spawn`; in `tick`, sweep kinematic agents whose
  `last_update` is older than `kinematic_ttl` (default 5 s) and despawn them.
- HTTP: `POST /api/world/agents/<id>/pose { x, z, y?, heading? }` → `set_pose`. Document
  the kinematic spawn flag on `/api/world/spawn`.
- Calibration store: `GET /api/cameras/calib` (serve the file) and
  `POST /api/cameras/calib { cameraId, quad, ... }` (merge + persist) to
  `calibration/cameras.json` (NOT under gitignored `web/data/`; this is authored config —
  constitution Principle 1).

### Client — `web/homography.js` (new, dependency-free)
- `solveHomography(srcPts, dstPts)` → 3×3 (DLT; exact for 4, least-squares via normal
  equations for >4; Gaussian elimination, no deps). `applyHomography(H, [u,v])` → [x,z].
- Unit-tested in isolation (node) before any UI.

### Client — `web/cameras.js` / PiP (extend)
- Calibration overlay on the PiP `<video>`: a 2×2 grid; select a quad; click image points
  (recorded normalized within the quad, so resolution-independent) and matching ground
  points via a 3D raycast onto the ground plane; solve + save the homography; reproject
  twin reference points (`signals.json` center/stop bars for the camera's intersection)
  back onto the video via `H⁻¹` for the user to judge.
- "Spawn from click" test mode: a calibrated image click → `applyHomography` → POST a
  kinematic spawn; "Clear cars" despawns the click-cars.
- `window.__twin.cameras` gains: `getCalib(id,quad)`, `setCalib(...)`,
  `imageToScene(id,quad,[u,v])`.

### Client — `web/netagents.js` (fix)
- Import flat mode; render shared-world agents at `FLAT_Y` when flat (drape height),
  rather than the server's terrain y. Heading/x/z unchanged.

### Calibration file — `calibration/cameras.json`
```
{ "version": 1,
  "cameras": { "LEX-CAM-0XX": { "intersection": "int_XXXX",
    "quads": { "0": { "covers": "<approach>", "imgW": .., "imgH": ..,
      "H": [h11..h33], "points": [{ "img":[u,v], "scene":[x,z] }, ...] } } } } }
```
Quad index: 0=TL, 1=TR, 2=BL, 3=BR. Image coords normalized [0,1] within the quad.

## Coordinate conventions
- Quad split: top-left/top-right/bottom-left/bottom-right of the full frame.
- Image point for a vehicle = bbox bottom-center (tire-contact), normalized within its
  quad. Calibration uses the same normalization so the Phase 2 detector is consistent.
- Homography maps normalized image (u,v) → scene (x, z). Height comes from the ground
  model at runtime (FLAT_Y in flat mode).

## Test strategy (constitution Principle 6)
1. **Unit (node)**: `solveHomography` recovers known transforms; round-trips test points
   within tolerance; handles 4 and >4 points; rejects degenerate/collinear input.
2. **API (python/curl)**: kinematic spawn → appears in state, does not drift; `pose`
   moves it; DELETE removes it; TTL auto-despawns; calib GET/POST round-trips.
3. **Headless viewer (playwright)**: netagents renders a kinematic car at FLAT_Y and the
   set x/z; programmatic calibration (inject correspondences) → save → a known pixel
   click spawns a car within tolerance of the expected scene point; clear removes it.

## Risks / mitigations (Phase 1)
- Fisheye error → calibrate over the road region; "approximate" accepted.
- Client/server height mismatch → netagents flat fix (FR-003).
- Ghost cars if a client dies → server TTL sweep (FR-002).
