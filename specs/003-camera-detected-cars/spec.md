# Feature Specification: Camera-Detected Cars in the Twin

**Feature Branch**: `feat/camera-cars-phase1`

**Created**: 2026-06-18

**Status**: Phase 1 (geometry + kinematic spawn)

**Input**: User description: "use the traffic cameras to detect cars in the intersection
and then spawn a digital twin car into the scene for the duration of when the car is in
view of the camera. … all the traffic cameras have 4 video feeds in 1 stream so you will
have to use best judgement on the orientation of the camera in respect to the
intersection. use a yolo model to find the cars and their positions. … if you add
something onto the PiP of the traffic camera I can attempt to help orientate the
intersection to make it reality."

## Clarifications

### Session 2026-06-18

- Q: Where does detection run? → A: Server-side YOLO (reuse TrafficStream / GPU) drives
  the spawned cars; a light in-browser detector draws PiP boxes synced to the displayed
  video.
- Q: What do detected cars spawn into? → A: The shared authoritative world
  (`/api/world/*`), so every connected viewer sees them; cars are kinematic (pose set
  from detections, no physics).
- Q: First target intersection? → A: Harrodsburg/Lakespur (highest-resolution quad, most
  traffic, clear lane geometry, tight twin match); Tates Creek/Rockbridge is the clean
  fallback.
- Q: Accuracy bar for v1? → A: Approximate — a manual homography per sub-view; cars land
  in roughly the right lane (fisheye distortion ignored). Good enough to look real.

### Established physical reality (from sampling live frames)

- Every camera stream is a **2×2 quad of four independent wide-angle cameras**, not one
  scene from four angles. Per-view resolution varies (480×360 to 1280×960).
- City intersections show the four approaches of ONE junction; interchanges show four
  different roads across a large area.
- Lenses are fisheye (visible barrel distortion); some views have heavy mast-arm
  occlusion. → A single homography is locally (road-region) accurate, not globally.

## Scope

This spec covers the whole feature; **Phase 1** (this branch) delivers the geometry and
spawn substrate WITHOUT machine learning, so the hardest part — mapping a camera pixel
to a scene position and getting a car to appear there for everyone — is proven first.

- **Phase 1 (in scope here)**: kinematic shared-world cars; a per-(camera, quad)
  homography; a PiP calibration tool to author it by clicking image↔twin point pairs;
  and a manual "spawn from image click" that proves a calibrated pixel maps to the right
  scene position and a car appears there for all viewers, then despawns.
- **Phase 2 (future)**: server YOLO + ByteTrack detector that feeds the kinematic spawn
  loop from real detections, with the spawn/track/despawn lifecycle.
- **Phase 3 (future)**: in-browser detection boxes on the PiP, motion-derived heading,
  smoothing, class colors, cross-quad dedup, and per-active-camera perf bounding.

## User Scenarios & Testing *(Phase 1)*

### User Story 1 - Kinematic cars are shared and pose-driven (Priority: P1)

A script (and later the detector) spawns a car into the shared world with a directly set
pose (x, z, heading), updates that pose over time, and despawns it. The car is kinematic
— it does not drive under physics — and is visible to every connected viewer, sitting on
the ground at the correct height regardless of the active ground model.

**Why this priority**: this is the substrate everything else feeds. If a pose-driven car
can't be placed and seen by all viewers at the right height, nothing downstream works.

**Independent Test**: via the world API, spawn a kinematic car at a known (x, z); confirm
it appears in `/api/world/state` and renders in a headless viewer at that x/z and on the
ground plane; update its pose and confirm it moves; despawn and confirm it disappears;
leave it un-updated past the TTL and confirm it auto-despawns.

**Acceptance Scenarios**:

1. **Given** the twin server is running, **When** a kinematic car is spawned at (x,z,θ),
   **Then** `/api/world/state` reports it at that pose and it does not move on its own.
2. **Given** a kinematic car exists, **When** its pose is set to a new (x,z,θ), **Then**
   the next world state and every viewer show it at the new pose.
3. **Given** the flat-world viewer, **When** a kinematic car renders, **Then** it sits on
   the flat ground (FLAT_Y), not at the server's terrain elevation.
4. **Given** a kinematic car that stops receiving pose updates, **When** the TTL elapses,
   **Then** the world auto-despawns it (no ghost cars if the detector dies).

### User Story 2 - Calibrate a camera sub-view to the twin (Priority: P1)

From the camera PiP, the user enters calibration mode, sees the 2×2 quad grid, picks a
sub-view, then clicks ≥4 points in the camera image and the matching ground points in the
3D twin. The system solves a homography (image px → scene x,z) for that sub-view, shows
the fit by reprojecting twin reference geometry back onto the video, and saves it.

**Why this priority**: the homography is the bridge from pixels to the world; without it
no detection can place a car. The user explicitly offered to do this orientation.

**Independent Test**: feed a known set of image↔scene correspondences to the solver and
confirm it recovers the homography (round-trips test points within tolerance). In the
viewer, drive the calibration flow programmatically (inject point pairs), save, reload,
and confirm the stored homography maps a test pixel to the expected scene point.

**Acceptance Scenarios**:

1. **Given** ≥4 non-collinear image↔scene correspondences, **When** the solver runs,
   **Then** it returns a homography that maps each source point to its target within a
   small tolerance.
2. **Given** a calibrated sub-view, **When** the user saves, **Then** the homography is
   persisted to a version-controlled calibration file keyed by (camera id, quad) and
   reloads on next session.
3. **Given** a saved calibration, **When** the reprojection overlay is shown, **Then**
   twin reference points (intersection center / stop bars) draw at plausible positions on
   the video for the user to judge and refine.

### User Story 3 - Click a pixel, a car appears in the twin (Priority: P1)

With a sub-view calibrated, the user clicks a point in the camera image (where a car's
tires would be). A kinematic car appears in the twin at the mapped scene position,
visible to all viewers, and can be cleared.

**Why this priority**: this is the end-to-end Phase 1 proof that the geometry + spawn
pipeline works, standing in for the detector that Phase 2 will add.

**Independent Test**: with a calibrated sub-view, programmatically click a known image
point; confirm a kinematic car spawns within tolerance of the expected scene position and
appears in a second viewer; clear it and confirm removal.

**Acceptance Scenarios**:

1. **Given** a calibrated sub-view, **When** the user clicks an image point, **Then** a
   kinematic car spawns at `H · pixel` and is reported in world state.
2. **Given** a spawned click-car, **When** the user clicks "clear", **Then** the car
   despawns for all viewers.

## Requirements *(Phase 1)*

- **FR-001**: The world MUST support kinematic agents: spawn with a kinematic flag, set
  pose directly, never integrate physics for them.
- **FR-002**: The world MUST expose a pose-set action and MUST auto-despawn kinematic
  agents that go un-updated past a configurable TTL.
- **FR-003**: The viewer MUST render shared-world cars on the active ground model height
  (FLAT_Y in flat mode), not the server's elevation.
- **FR-004**: A homography solver MUST compute an image→scene transform from ≥4
  correspondences (least-squares for >4) with no external runtime dependency.
- **FR-005**: Calibration MUST be authored from the PiP (quad grid + click pairs),
  persisted keyed by (camera id, quad) to a version-controlled file, and served/saved via
  the twin server.
- **FR-006**: The system MUST reproject twin reference geometry onto the PiP so the user
  can judge and refine the fit.
- **FR-007**: A calibrated image click MUST spawn a kinematic car at the mapped scene
  position, visible to all viewers, and be clearable.

### Key Entities

- **Kinematic car**: a shared-world agent with pose (x, z, heading), a source (camera id,
  quad, track id later), a TTL, no physics.
- **Sub-view calibration**: per (camera id, quad) a 3×3 homography (image px → scene
  x,z), the correspondence points used, and the source image dimensions.

## Out of Scope (Phase 1)

YOLO/detection, tracking, the in-browser PiP detection boxes, motion heading, cross-quad
dedup, multi-camera perf bounding, fisheye undistortion. (Phases 2–3.)

## Success Criteria

- A calibrated pixel click places a shared, ground-sitting kinematic car within a few
  meters of the intended scene point, visible to all viewers, that clears on command and
  auto-expires if abandoned — proven by unit tests (solver), API tests (kinematic
  lifecycle), and headless viewer tests (calibration round-trip + click-to-spawn).
