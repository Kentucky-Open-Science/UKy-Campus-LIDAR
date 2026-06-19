# Code Review — UKy / Lexington Digital Twin

**Date:** 2026-06-19
**Scope:** Whole environment, with emphasis on (a) traffic-camera vehicle detection &
rendering accuracy at intersections, (b) the user-guided camera→twin placement
(calibration) system, and (c) performance/production-readiness.
**Method:** Six independent review dimensions (detection accuracy, homography numerics,
placement/UX, server kinematics & concurrency, viewer performance, end-to-end
integration). Every candidate finding was then **adversarially verified** by a second
pass instructed to refute it. 37 candidates → **31 confirmed/kept, 6 rejected** as false
positives. The math core was independently re-verified numerically (see below).

## TL;DR

The system is well-architected and the hardest parts are *correct*: the homography
solver is numerically sound (recovers known transforms to ~1e-13, rejects collinear
input), the JS (`web/homography.js`) and Python (`tools/camera_detect.py`) implementations
are **bit-exact**, the quad/coordinate conventions (0=TL/1=TR/2=BL/3=BR, tire-point
normalization, PiP box mapping) are consistent end-to-end, the heading formula matches the
server's forward vector, and heights stay consistent under flat-world mode. The render
loop respects the "one packed buffer / one draw call per layer" constitution.

The defects are concentrated in three places: **the PiP overlay's coordinate mapping
ignores the video letterbox** (the single highest-impact bug — it silently corrupts both
calibration and detection alignment), **the live tracker's spawn-heading and dedup logic**
(cars face the wrong way at spawn; adjacent-lane cars can merge), and a cluster of
**robustness/leak/perf** issues (an unvalidated pose can crash the sim thread, a GPU leak
on agent churn, stale overlay boxes, a calibration heartbeat that never stops).

| Severity | Count |
|---|---|
| High | 2 |
| Medium | 11 |
| Low | 13 |
| Optimization | 5 |

Independently verified ground truth (re-run during this review):
- Homography solver: 8/8 numerical checks pass (identity, 4-pt exact, >4 LSQ, invert
  round-trip, collinear→null).
- JS↔Python `applyHomography`/`apply_h` parity: identical to 1e-6 on test points.

---

## HIGH

### H1 — `set_pose` accepts NaN/inf and can kill the sim thread + break every client poll
**File:** `tools/twin_server.py` (`Agent.set_pose` 375-382; `Ground.height` ~196; `_json` 997; pose endpoint 1190-1193) · **category:** robustness

`set_pose` does `float(x)/float(z)` with no finiteness guard, and request JSON is parsed
with Python's default `json.loads`, which **accepts the `NaN`/`Infinity` tokens**. A single
`POST /api/world/agents/<id>/pose {"x": NaN, "z": 0}` flows into `snap_ground →
Ground.height`, which does `int((sx - x0)/cell)` → `ValueError: cannot convert float NaN
to integer` **inside `World.tick`**, which has no try/except in `World.run`, so the sim
thread dies permanently for all clients. Even short of the crash, a non-finite value is
serialized by `json.dumps` (default `allow_nan=True`) as the bare `NaN` token, which
browser `JSON.parse` rejects — breaking the `netagents` poll. Violates the
never-throw-into-the-loop principle.

> Note: the live detector path does **not** produce this (its `apply_h` rejects
> non-finite `w` and returns finite-or-`None`); the exposure is the unvalidated public
> pose endpoint and any future caller.

**Fix:** finiteness-guard `set_pose` (drop the update if `x`/`z` non-finite; do not bump
`last_update` so the TTL can still reap), mirror in `Agent.__init__` position parsing,
make `Ground.height` tolerant of non-finite input (choke-point defense), and set
`allow_nan=False` in `_json`.

### H2 — PiP click & detection-box mapping ignore the video `object-fit: contain` letterbox
**File:** `web/style.css` 211-217 + `web/app.js` (`camCal.draw` 1335-1373; click handler 1463-1477) · **category:** correctness

The PiP `<video>` is `object-fit: contain` inside a **user-resizable** panel, so the video
content is letterboxed (pillar/letter bars) whenever the stage aspect ≠ the stream's 4:3
— which is true even at the default panel size, and always after a resize. But every image
coordinate is computed against the **full overlay rect**: calibration clicks normalize as
`u=(clientX-left)/width`, and detection/cal geometry draws at `x=(col*0.5+box*0.5)*W`.
In a letterboxed state, overlay-fraction `(u,v)` ≠ video-content `(u,v)`, so:
- the **homography is solved from misregistered correspondences** → both click-spawned and
  detector-spawned cars land at the wrong scene `(x,z)`;
- the **detection boxes float off the vehicles** on the video.

This is the one defect that corrupts *both* end-to-end paths, because the homography is the
shared contract between calibration and detection.

**Fix:** compute the displayed content rect from `video.videoWidth/Height` vs the stage
(`scale=min(sw/vw,sh/vh)`, centered `offX/offY`), map clicks into content-normalized space
(rejecting clicks in the bars), and inverse-map when drawing. Apply the transform to **all**
overlay geometry (boxes, quad grid, highlight, points, reproj crosses, pending marker), not
just the boxes. Fall back to 960×720 until `videoWidth>0`.

---

## MEDIUM

### M1 — Newly-spawned camera cars face +X until they move ≥0.5 m in a single frame
**File:** `tools/camera_detect.py` 157-172 · **accuracy.** A new track spawns with
`heading 0.0` and only updates heading when the **single-frame** displacement passes
`min_move=0.5 m`. At `--max-fps 8` that needs ≥4 m/s (~9 mph) in one 125 ms step, so
slow/queued/just-appeared cars (the typical intersection scene) and one-shot detections sit
visibly mis-rotated (pointing +X). **Fix:** accumulate displacement from a per-track anchor
since spawn/last-update and commit `atan2(-dz,dx)` once the *cumulative* move exceeds
`min_move`; hold/ suppress orientation until known.

### M2 — Greedy scene dedup (`dedup_m=4 m`) merges adjacent-lane cars and is order-dependent
**File:** `tools/camera_detect.py` 125-138 · **accuracy.** `_dedup` is single-pass
first-match clustering at 4 m — wider than a US lane (~3.7 m) — so two cars in adjacent
lanes collapse into one track placed *between* them, and the unmatched second track is
reaped after `lost_frames` (a real car disappears). The merge is also iteration-order
dependent (centroid jitter). Intent (per the code comment) was *cross-quad* dedup only.
**Fix:** only merge detections from **different** quads; effectively disable within-quad
merging (YOLO NMS already dedups a single quad); use a deterministic nearest-cluster (not
first-match) assignment; record the contributing-quad set.

### M3 — `--follow-active` idle path leaves stale PiP boxes for up to `DET_TTL` (6 s)
**File:** `tools/camera_detect.py` 240-245 · **correctness.** While idling on a non-active
camera the detector ages out/despawns its world cars but never publishes an empty detection
set, so the relay keeps serving the last boxes as "fresh" for up to 6 s — ghost boxes over
no cars. **Fix:** publish `[]` on the active→idle transition (guarded by `--publish`).

### M4 — Coincident/contradictory 4-point correspondences aren't rejected; bad H can be saved
**File:** `web/homography.js` 64-118 + `web/cameras.js` `solve`/`save` 269-291 ·
**robustness.** The 1e-15 pivot guard correctly rejects collinear input, but two coincident
image clicks mapping to different scene points produce a non-null H with a huge
`reprojError` (e.g. 69 m); nothing gates `save()` on the error, so the garbage homography is
persisted and mislocates every car in that quad. **Fix:** reject duplicate/coincident
points in `solve()` (the real misclick root cause) and gate `save()` on a reprojection
ceiling.

### M5 — After a successful Solve&Save the authoring session is never reset
**File:** `web/cameras.js` `save`/`clearSession` 267,277-291 + `web/app.js` 1422-1431 ·
**robustness.** `save()` leaves `session.quad`/`pairs` intact, so clicking in a *different*
quad errors with "point is in quad X, session is calibrating quad Y"; the only escape is
toggling Calibrate off/on. `clearSession()` exists but is never called. **Fix:** reset the
session after a successful save (capturing `quad`/`reproj` for the status first).

### M6 — Click-car heartbeat is never stopped on PiP close/switch → leaked cars + interval
**File:** `web/app.js` `reset`/`ensureHeartbeat`/close handler 1281-1287, 1385, 1450-1461 ·
**bug.** A 2 s `setInterval` re-poses click-spawned cars to outlive the 5 s TTL, but closing
the PiP (or switching cameras via the `cam-open`→`reset()` path) neither stops the interval
nor clears the cars — they keep being re-posed forever, visible to all shared-world clients,
unreachable from the (closed) UI. **Fix:** stop the heartbeat (and clear cars) in `reset()`.

### M7 — Kinematic camera cars run O(N²) collision every tick and flash phantom-red
**File:** `tools/twin_server.py` 593-616, 484-507 + `web/netagents.js` 90-92 · **performance
/ correctness.** `tick()` runs `detect()` for kinematic cars too; overlapping detections at
one intersection produce spurious agent contacts that the viewer renders red. The collision
result is never consumed (kinematic = no physics), so it's pure waste *and* visibly wrong.
**Fix:** short-circuit `detect()` for kinematic agents and skip kinematic agents inside the
detection loop (so real agents don't flash red against phantom cars).

### M8 — `GET /api/world/nearest_building` (and `_send_camera`) 500 + leak a traceback on a bad query param
**File:** `tools/twin_server.py` 1067-1070, 1094-1095 · **robustness.** `float(q.get("x"))`
with no guard raises `ValueError` on `?x=abc`; there's no top-level handler, so the
connection drops with a stderr traceback. **Fix:** wrap query-param parsing in try/except →
400; add a thin top-level guard.

### M9 — Calibration overlay reallocates its canvas + forces a reflow every 200 ms
**File:** `web/app.js` `draw` 1335-1340 + `pollDetections` 1388-1401 · **performance.**
`draw()` calls `getBoundingClientRect()` (sync reflow) and reassigns `canvas.width/height`
(always reallocates the backing store) on every 200 ms detection poll and every
ResizeObserver fire, on top of the in-panel HLS decode. **Fix:** resize only when the
measured size actually changes; drive size off the ResizeObserver.

### M10 — `netagents` leaks body + arrow geometry/material on every agent despawn
**File:** `web/netagents.js` 54-79, 94-101 · **performance.** `spawn()` makes a
`BoxGeometry`, a `ConeGeometry`, and two materials per agent; removal disposes only the body
material and sprite. The body/arrow geometry and arrow material are never freed — and
camera-detected cars churn continuously, so VRAM grows over a session. **Fix:** dispose all
owned GPU resources on removal (or, better, share cached per-type geometries + a shared arrow
material and clone only the body material).

### M11 — Active-camera perf-bounding signal is a single global → multi-viewer/detector aliasing
**File:** `tools/twin_server.py` 84,1150-1160 + `tools/camera_detect.py` 241 · **robustness.**
`ACTIVE` is one slot; with two viewers on two cameras only the last open "wins", so a
`--follow-active` detector for the other camera idles and lets its tracks despawn *while a
viewer is watching*, and one viewer closing its PiP clears the signal for everyone. **Fix:**
make it a per-camera `{id: ts}` map; the detector queries its own id; close clears only its
own entry.

---

## LOW

- **L1** — Detector vs viewer disagree on the COCO *car* color (`0x35c4c4` vs `#27c4c4`); the
  on-video box is a slightly different teal than the 3D car. `tools/camera_detect.py` 33,64,166.
  *Fix:* unify to the camera-layer canonical `0x27c4c4`.
- **L2** — Python `apply_h` returns `None` where JS returns `[NaN,NaN]` on degenerate `w`; a
  type divergence (loud-vs-silent failure) that is benign today because all callers guard
  both. `tools/camera_detect.py` 37-43. *Fix:* document the intentional divergence (do **not**
  naively unify — a `(nan,nan)` tuple is truthy and would slip past the `if sc:` guard).
- **L3** — `calib.solve()` overloads its `error` field (numeric reprojError on success, string
  on failure); safe today, a refactor trap. `web/cameras.js` 269-291. *Fix:* return
  `{H, reproj}` on success, keep `error` for strings.
- **L4** — Server drops `reprojError` from the calibration POST, so a quad's quality metric is
  unrecoverable after reload. `tools/twin_server.py` 1125 + `web/cameras.js` 282,289. *Fix:*
  add `reprojError` to the persisted whitelist and the in-memory cache.
- **L5** — `snap_ground` overwrites the `set_pose` `y` for ground-bound agents every tick, so
  the documented `pose {y?}` field is a silent no-op for cars/trucks/robots. `tools/twin_server.py`
  385-396,459-476. *Fix:* document it (the snap is the correct default under one-ground-model).
- **L6** — `DETECTIONS` dict grows unbounded — stale camera keys are TTL-filtered on read but
  never evicted; the open POST lets arbitrary `camera` strings accumulate keys.
  `tools/twin_server.py` 82-86,1131-1148. *Fix:* sweep stale entries on write.
- **L7** — The `camMarker` scrape (`html.find("[")…find("]")`) is fragile; `CameraProxy` catches
  and serves stale, but the identical code in `lex_cameras.scrape_cameras` is unguarded and
  crashes the offline bake. `tools/twin_server.py` 942-958 + `tools/lex_cameras.py` 55-68.
  *Fix:* bail if the token is absent, parse with `json.JSONDecoder().raw_decode`, guard the bake.
- **L8** — `save_calib` is read-modify-write but the lock covers only the write, so concurrent
  calib POSTs can lose updates. `tools/twin_server.py` 97-101,1112-1128. *Fix:* hold the lock
  across the whole RMW (use an RLock / non-locking write helper to avoid self-deadlock).
- **L9** — The global `max_agents=64` cap is shared; a busy intersection can silently starve
  camera cars with no telemetry. `tools/twin_server.py` 551,571-573. *Fix:* expose the live
  agent count in `/api/world/meta` (optional per-source budget).
- **L10** — A matched camera's marker is drawn at the snapped intersection *centre* (up to 75 m
  away), but cars land per-homography, so an operator validating detections against the marker
  can misread the geometry. `tools/lex_cameras.py` 108-117 + `web/cameras.js` 161-168. *Fix:*
  surface the snap distance in the PiP/calibration status.
- **L11** — Cross-quad dedup keeps only the first cluster's quad/cls, biasing track metadata and
  causing heading flicker on seam-straddling cars. `tools/camera_detect.py` 125-138 (same root as
  M2). *Fix:* record the contributing-quad set; add heading hysteresis (folded into the M2 fix).

## OPTIMIZATION (and verified-correct notes)

- **O1** — `A^TA` assembly is symmetric; the inner loop could be halved. **Not worth changing**
  (8×8 solved a handful of times during hand calibration). No action.
- **O2** — `groundPointFromRay` sign/ground-model logic verified **correct**; add a small UX
  message when a calibration scene-click lands off the ground (currently swallowed silently).
  `web/app.js` 1907-1936.
- **O3** — `World.get` reads `self.agents` outside the lock; harmless under the GIL but a
  control/pose update can interleave mid-tick (one-tick torn pose). Wrap the handler's
  lookup+mutate in `WORLD.lock` (the RLock makes nesting safe). `tools/twin_server.py` 590-591,1178-1196.
- **O4** — The buildings color-mode toggle walks ~114k buildings with per-vertex `setXYZ`
  (visible hitch). Write directly into the `Float32Array` (the same pattern used at load time).
  `web/app.js` 1506-1518.
- **O5** — The detection overlay repaints unconditionally 5×/s; gate `draw()` on a dets
  content-signature change (the `_camSig`/`_busSig` pattern already used in this file).
  `web/app.js` 1388-1401.

---

## Rejected (false positives, after adversarial verification)

1. *"`invertHomography` threshold / denormalization can silently produce a bad H"* — No: every
   singular step bails to `null` and `det(T)` for real data is ~13 orders above the floor.
2. *"Spawn/relay fetches bypass the `sameOriginPath` SSRF guard"* — No taint: the user-controlled
   value is `?data=` (which **is** guarded); `proxyBase` is a hardcoded `''` constant.
3. *"Single global `ACTIVE` leaves previous-camera cars lingering"* — That's bounded graceful
   degradation, not a defect (the multi-viewer aliasing *is* captured separately as M11).
4. *"Normal-equations condition-squaring is a precision risk"* — No: Hartley normalization keeps
   `cond(A^TA)` ~57–140; re-verified numerically (max reproj ~1e-13 m).
5. *"Reproj crosses vs detection boxes use inconsistent quad offset"* — Verified **identical**
   (`col*0.5+qu*0.5`), round-trips with `quadOf`.
6. *"TTL sweep mutates `self.agents` while iterating"* — Safe: it iterates a `list(...)` copy
   under a reentrant lock.

---

## Fix plan

All confirmed findings except **O1** (deliberately skipped) are addressed in the
accompanying change set. Order: H1, H2 first (correctness/accuracy of the two target
subsystems), then the medium tracker/UX/robustness/perf items, then the low/robustness
hardening and optimizations.

---

## Resolution & verification (applied)

Files changed: `tools/twin_server.py`, `tools/camera_detect.py`, `tools/lex_cameras.py`,
`web/app.js`, `web/cameras.js`, `web/netagents.js` (+330/−101). Changes left in the working
tree (not committed — per the repo's PR-only workflow).

**What was fixed**

- **H1** — `set_pose` and `Agent.__init__` now reject non-finite coords (new `finite()`
  helper); `Ground.height` early-returns on non-finite input; `_json` serializes with
  `allow_nan=False` and never throws. NaN can no longer kill the sim thread or break the
  client poll.
- **H2** — new `contentBox()`/`toPx()`/`clickToContent()` map every PiP click and all
  overlay geometry (boxes, quad grid, highlight, points, reproj crosses, pending marker)
  through the real `object-fit:contain` content box; clicks in the letterbox bars are
  rejected. Calibration and detection now align with the visible video at any panel size.
- **M1** — heading is committed from *cumulative* displacement since a per-track anchor, so
  slow/queued cars get a correct orientation instead of facing +X.
- **M2 / L11** — `_dedup` merges only across *different* quads, joins the *nearest* eligible
  cluster (order-independent), and records the contributing-quad set. Adjacent-lane cars in
  one quad stay distinct.
- **M3** — the `--follow-active` idle path publishes an empty detection set on the
  active→idle transition, clearing stale PiP boxes immediately.
- **M4** — `solve()` rejects coincident image points; `save()` refuses to persist a fit
  with reprojection error above a ceiling.
- **M5** — `save()` resets the session, so the next quad calibrates without an off/on toggle.
- **M6** — `reset()` stops the heartbeat and despawns click-cars on PiP close / camera switch.
- **M7** — kinematic cars are excluded from collision (no O(N²) work, no phantom-red flashes).
- **M8** — query-param parsing returns 400 instead of a 500 + traceback.
- **M9 / O5** — overlay canvas resizes only on real size change; the detection redraw is
  gated on a content-change signature.
- **M10** — `netagents` shares per-type geometries + the arrow material and disposes only
  per-agent resources, eliminating the GPU leak on agent churn.
- **M11** — the active-camera signal is per-camera (`{id: ts}`), refreshed by a client
  heartbeat while a PiP is open, and cleared per-camera on close.
- **L1–L10, O2–O4** — color unified to `0x27c4c4`; `apply_h`/`applyHomography` divergence
  documented; `solve()` returns `{H, reproj}` (no overloaded field); `reprojError` persisted;
  `snap_ground` y-override documented; `DETECTIONS` swept on write; the camMarker scrape uses
  `raw_decode` + bails cleanly (both server and bake); calibration write is lock-atomic; live
  agent count exposed in `/api/world/meta`; off-ground calibration clicks give feedback; the
  buildings color toggle writes the `Float32Array` directly; `World.get`+mutate is lock-wrapped.

**Verification performed**

- Homography solver: 8/8 numeric checks; JS↔Python `apply` parity exact.
- Letterbox `contentBox` math: pillarbox detection, center→(0.5,0.5), in-bar rejection,
  px↔content round-trip — all pass.
- `SceneTracker`: 8/8 — same-quad adjacent lanes stay distinct, cross-quad overlap merges
  (records both quads, order-independent), heading accumulates across sub-`min_move` frames
  with the correct sign.
- Server logic: 15/15 — `finite()`, `Ground.height(NaN)` safe, kinematic integrate no-op,
  `set_pose` NaN/inf rejection, `tick()` never raises, kinematic collision skipped, snapshot
  is NaN-free JSON.
- Live server (`--no-cameras`, port 8077): `/api/world/meta` reports `agents`;
  `nearest_building?x=abc` → 400 (no traceback); calib POST round-trips with `reprojError`;
  per-camera active A/B isolation + per-camera clear; a NaN pose is dropped while the server
  stays up and world state stays valid JSON; valid pose applied. Calibration file restored.
- `py_compile` (3 modules) + `node --check` (4 ES modules): all clean.
