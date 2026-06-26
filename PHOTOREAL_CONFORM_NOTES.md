# Photoreal conform + adaptive quality + camera-car fixes

Branch: `feature/photoreal-conform-and-quality` (uncommitted — review before committing; `main` is protected).

## What was wrong

Overlays (roads, lane markings, crosswalks, signals, trees, parked cars, buses,
shared-world/YOLO cars, agents, camera markers, LiDAR) are baked at **our** DTM/LiDAR
elevation (NAVD88). Google's photoreal mesh is a **different** surface (WGS84
photogrammetry). The old runtime drape corrected the gap with **one global vertical
offset** sampled under the camera — so the two surfaces only coincided at a single point
and everything else floated/sank, sliding vertically as you panned.

## The fix

### 1. Streaming ground-conform field — `web/drape.js` (new)
Models the residual as a **spatial field** `offset(x,z) = photorealGroundY − ourGroundY`,
sampled lazily on a ~48 m grid by raycasting both surfaces, **settled per cell** against LOD
streaming (commit only when recent samples agree — kills the bobbing), and bilinearly
interpolated. Overlays are rebased by the **locally** interpolated offset:

- **Road network** (ribbons, markings, crosswalks, intersection pads, signals, props):
  conformed **per-vertex** on the CPU, so hills/overpasses keep their relative profile and it
  stays one draw call per layer. Because the real geometry moves, the **driving-agent sim
  ground-snaps onto the conformed roads for free** (it raycasts the lifted ribbons).
- **Buses** (`transit.js`) and **shared-world / camera cars** (`netagents.js`): sample the
  field **per-object** (`drapeOffsetAt`).
- **LiDAR / labels / camera markers**: ride a single focus-sampled offset (point layers; local
  sub-metre variation is invisible).

Wiring in `web/app.js`: `createDrapeField(...)` → `registerTree(roadnet)` + `setBounds(...)` →
`updateDrape()` drives the field each frame (replaces the old single-offset code).

Pure math (`bilinearSample`, `settle`) is unit-tested without a GPU:
`cd web && node tests/drape_math.test.mjs` → **ALL PASS**.

### 2. Adaptive auto-FPS tile quality — `web/tiles3d.js`, `web/app.js`, `web/index.html`
- **Anisotropic filtering** (max) on tile textures — crisp at grazing angles.
- Bigger LRU cache + more download/parse concurrency + uncapped subdivision depth.
- **Auto-FPS controller**: nudges the tile `errorTarget` to hold ~60 fps — creeps toward the
  1 px high-fidelity floor when there's headroom, backs off when slow. Panel checkbox
  **"adaptive quality (auto-hold ~60fps)"** (on by default); dragging the detail slider is a
  manual override.

### 3. Camera-car scale + anti-clip — `tools/twin_server.py`
Camera-detected vehicles spawn server-side. The `DEFS` table was ~70 % scale and there was
**no separation pass** (collision was detect-only; kinematic cars were exempt), so dense cars
overlapped.
- Real dimensions: car **4.5×1.9×1.45**, truck 8.0×2.5×3.0, **bus 12.0×2.9×3.3**, moto/bike/ped sane.
- New **OBB MTV separation** in `World.tick` over all agents incl. kinematic camera cars.
  Proof: `python -m tools.qa_cars` → **PASS** (max car-car penetration ≈ 1.9 m → ~0).

## Run + verify locally

```sh
# from repo root, with your .env GOOGLE_MAPS_API_KEY present:
python -m tools.twin_server            # viewer + live buses + shared world, on :8000
# open http://localhost:8000/?flat=0   (real-elevation mode; photoreal on by default)
```

Verify the conform: enable **Photorealistic 3D (Google)**, fly low over downtown — roads,
signals, crosswalks and parked cars should sit ON the imagery and stay put as you pan/zoom
(no floating, no vertical sliding). Toggle the layer off/on to see the smooth ease.

QA harnesses (run with a browser / your env):

```sh
python -m tools.qa_buttons --port 8000   # clicks every UI control, asserts effect + screenshots
python -m tools.qa_cars                  # anti-clip proof (no browser needed)
python -m tools.qa_camera_pipeline       # COCO->type mapping + homography geometry (no model)
# live camera -> YOLO -> twin cars (needs the live camera + model weights):
python -m tools.camera_detect --camera <CAM-ID> --twin http://127.0.0.1:8000 --model yolo26n.pt
```

## Sandbox caveats (why some checks were static here)
This build environment had **no egress to Google tiles / city cameras** and **no browser**, so
live tile rendering, live-camera YOLO, and the Playwright button run must happen on your
machine. In-sandbox we verified: drape math (unit test), camera-car scale + anti-clip
(`qa_cars` PASS), the COCO/homography geometry (`qa_camera_pipeline`), and a full static UI
audit (50/50 controls wired, 0 broken — `extracted/qa/REPORT.md`).
