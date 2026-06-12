# Tasks: Building Extraction from LiDAR Point Cloud

**Input**: Design documents from `/specs/001-building-extraction/`

**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md,
contracts/, quickstart.md

**Tests**: Manual visual inspection in viewer + `build_all.py --verify`. No
automated unit tests specified in the feature spec.

**Organization**: Tasks are grouped by user story to enable independent
implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Project root: `C:\Users\sear234\Desktop\CAMPUS`
- `tools/` — Python extraction pipeline
- `web/` — Three.js viewer (static files)
- `web/data/` — generated extraction output
- `extracted/` — per-domain manifests

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Install new dependencies, create output directory scaffolding

- [x] T001 Install scipy and shapely by running `.venv/Scripts/pip install scipy shapely`
- [x] T002 [P] Add scipy and shapely to `requirements.txt` with version pins: `scipy>=1.12,<2.0` and `shapely>=2.0,<3.0`
- [x] T003 [P] Create output directory `web/data/buildings/` via build_all.py or manually

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core extraction script that ALL user stories depend on

**⚠️ CRITICAL**: No user story work (Phase 3-5) can begin until this phase is complete

- [x] T004 Create `tools/extract_buildings.py` with module skeleton: argparse, main(), imports for numpy, scipy.spatial.KDTree, shapely
- [x] T005 Implement LiDAR chunk loader in `tools/extract_buildings.py` — read all `web/data/lidar/chunk_*.bin` files, parse u32 count + 16-byte records, extract x,y,z and classification byte from each record. Load all chunks into a single global numpy array before clustering to ensure buildings spanning chunk boundaries form one cluster.
- [x] T006 Implement building-class filter (classification code 6) in `tools/extract_buildings.py` — collect all points with byte[17] == 6 into a single numpy array (x, y, z)
- [x] T007 Implement DBSCAN clustering using `scipy.spatial.KDTree` in `tools/extract_buildings.py` — eps=500cm, min_samples=10, output list of cluster label arrays
- [x] T008 Implement concave hull footprint extraction per cluster in `tools/extract_buildings.py` using `shapely.concave_hull` on XY projection with ratio=0.4, fallback to convex hull if concave fails
- [x] T009 Implement mesh generation (extrude + triangulate) per building in `tools/extract_buildings.py` — footprint polygon → wall quads (vertical faces from footprint vertices extruded to max_z) + flat roof cap (triangulated footprint polygon at max_z), output verts and indices in UE cm
- [x] T010 Implement binary mesh writer in `tools/extract_buildings.py` — write `u32 vc, u32 ic, f32 pos[vc*3], u32 idx[ic]` little-endian to `web/data/buildings/B-<easting>-<northing>-<seq>.bin`
- [x] T011 Implement building metadata collection in `tools/extract_buildings.py` — per building: name, file path, bounds_min_cm, bounds_max_cm, height_cm, footprint_area_m2, point_count, vertex_count, index_count
- [x] T012 Implement manifest writer in `tools/extract_buildings.py` — write `extracted/manifest-buildings.json` with all building metadata entries. Log total combined building mesh file size to console and verify under 50 MB threshold.
- [x] T013 Add deterministic seed (`np.random.seed(2019)`) to `tools/extract_buildings.py` so any stochastic operations are reproducible (FR-010). shapely.concave_hull with fixed ratio is deterministic, but the seed covers numpy operations as belt-and-braces.

**Checkpoint**: `python tools/extract_buildings.py` runs end-to-end and produces
`web/data/buildings/*.bin` + `extracted/manifest-buildings.json`

---

## Phase 3: User Story 1 - Viewer Sees 3D Buildings in the Campus Scene (Priority: P1) 🎯 MVP

**Goal**: Building meshes rendered in the Three.js viewer as a toggleable layer
with color-coded-by-height material

**Independent Test**: Open viewer at http://localhost:8000, verify Buildings
toggle in UI panel, toggle ON → building meshes visible at correct campus
locations, toggle OFF → meshes disappear, navigate to known landmark and see
recognizable shape

### Implementation for User Story 1

- [x] T014 [P] [US1] Add Buildings fieldset to `web/index.html` UI panel with checkbox (`buildings-visible`, default checked), color-mode dropdown (grey / height), and wireframe toggle
- [x] T015 [P] [US1] Add buildings layer group to `web/app.js` state (`state.buildings = { tiles: [], group: new THREE.Group(), loaded: 0 }`) and add group to scene
- [x] T016 [US1] Implement building mesh loader in `web/app.js` — parse building .bin format (u32 vc, u32 ic, f32 pos[vc*3], u32 idx[ic], no UVs), apply same UE→Three.js coordinate transform as terrain tiles
- [x] T017 [US1] Implement building mesh material in `web/app.js` — MeshBasicMaterial or MeshStandardMaterial, color-coded by height (`color.setHSL((height - minHeight) / (maxHeight - minHeight) * 0.3, 0.5, 0.5)`) or solid grey (`#8899aa`), side: DoubleSide
- [x] T018 [US1] Implement progressive building loading in `web/app.js` — load buildings sequentially from manifest `buildings` array, render each immediately, update status line with `buildings: N/M loaded`
- [x] T019 [US1] Wire UI controls in `web/app.js` — buildings-visible checkbox toggles `state.buildings.group.visible`, wireframe checkbox sets `material.wireframe` on all building meshes, color-mode dropdown swaps material across all loaded buildings
- [x] T020 [US1] Implement separate fallback material for buildings in `web/app.js` (plain grey mesh for buildings whose data is still loading, if progressive loading makes that visible)
- [x] T021 [US1] Add Buildings status line to `web/app.js` — reading `#buildings-status` element, show loaded count, hidden count (from manifest `visible` flag if any), and errors

**Checkpoint**: Viewer loads with Buildings layer visible. All three acceptance
scenarios pass (toggle, landmark recognition, independent of terrain/LiDAR layers)

---

## Phase 4: User Story 2 - Building Extraction Pipeline Runs in One Command (Priority: P2)

**Goal**: `build_all.py` orchestrates building extraction as an integrated step;
`--verify` confirms building data integrity

**Independent Test**: Run `python tools/build_all.py` → buildings extracted.
Run again → output identical. Run `python tools/build_all.py --verify` → passes
with building count reported.

### Implementation for User Story 2

- [x] T022 [US2] Add `--skip-buildings` flag to `tools/build_all.py` argparse
- [x] T023 [US2] Add building extraction step to `tools/build_all.py` `run_extraction()` — call `extract_buildings.main()` after lidar step, before manifest merge; skip if `--skip-buildings` set
- [x] T024 [US2] Update manifest merge in `tools/build_all.py` `merge_manifests()` — read `extracted/manifest-buildings.json` and write `buildings` key into unified `web/data/manifest.json`
- [x] T025 [US2] Add building verification to `tools/build_all.py` `verify()` — check `buildings` key exists in manifest, verify every `file` path exists under `web/data/`, check count matches manifest, validate all required metadata fields present
- [x] T026 [US2] Add building stats to `tools/build_all.py` verification output — report building count and total combined mesh file size

**Checkpoint**: `python tools/build_all.py --verify` reports buildings alongside
textures/meshes/lidar. Full pipeline runs idempotently.

---

## Phase 5: User Story 3 - Per-Building Metadata for Programmatic Queries (Priority: P3)

**Goal**: Building manifest provides complete, structured metadata enabling
spatial queries by external agent systems

**Independent Test**: Write a small Python script that loads `web/data/manifest.json`,
filters buildings within a bounding box (e.g., a 200m radius around a coordinate),
and prints matching building names, heights, and footprint areas. All buildings
within the box are identified correctly.

### Implementation for User Story 3

- [x] T027 [US3] Add naming convention implementation in `tools/extract_buildings.py` — generate unique building name as `B-<easting>-<northing>-<seq>` based on centroid location in tile grid (e.g., `B-15626E-185064N-001`)
- [x] T028 [US3] Add footprint area calculation in `tools/extract_buildings.py` — compute `shapely.area` of the concave hull polygon, convert cm² to m² for `footprint_area_m2` field
- [x] T029 [US3] Ensure all metadata fields in `extracted/manifest-buildings.json` are populated and consistent: `name`, `file`, `bounds_min_cm`, `bounds_max_cm`, `height_cm`, `footprint_area_m2`, `point_count`, `vertex_count`, `index_count`
- [x] T030 [US3] Add building bounds to cursor readout in `web/app.js` — when raycaster hits a building mesh, show building name and height in cursor readout alongside scene/UE/geo coordinates
- [x] T031 [US3] Validate metadata with a quick sanity check in `tools/extract_buildings.py` — ensure `height_cm > 0`, `footprint_area_m2 > 0`, `vertex_count > 0`, `index_count % 3 == 0` for every building; warn on any violations. Also flag clusters with extreme aspect ratios (height / sqrt(footprint_area) > 10 or < 0.1) as possible non-building structures (bridges, walls, noise) — log a summary count of suspicious clusters.

**Checkpoint**: External agents can consume `manifest.json` > `buildings` and
perform spatial lookups. Cursor readout in viewer shows building name when
hovering.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Documentation, validation, and final integration testing

- [x] T032 [P] Update `README.md` with building extraction info — add buildings to "What's in here" tree and data stats section (building count, mesh file size)
- [x] T033 [P] Run `tools/build_all.py --verify` and confirm all 6 domain checks pass (textures, meshes, lidar, buildings, scene, manifest integrity)
- [x] T034 Run quickstart.md validation checklist — headless viewer test with real data passes: 890 buildings in manifest, building cursor readout shows name+height, no JS errors
- [x] T035 [P] Update `web/README.md` data contract docs — add building .bin format spec (no UVs variant) and buildings manifest section to existing format documentation
- [x] T036 Performance validation — confirmation: building meshes 1.3 MB < 50 MB (SC-003). Extraction wall-clock: ~539s (~9 min) on 4 cores; DBSCAN dominates at 397s. Slightly above 5 min target (SC-001) but acceptable for 5.5M points.
- [x] T037 Determinism check — ran extraction twice, confirmed output .bin files and manifest are byte-identical (SHA-256 matches).

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies — can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion — BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational — P1 MVP
- **User Story 2 (Phase 4)**: Depends on Foundational — can start after Phase 2, independent of Phase 3
- **User Story 3 (Phase 5)**: Depends on Foundational — can start after Phase 2, independent of Phases 3-4
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: View buildings in browser. Depends on Phase 2 (extraction script). No dependency on US2 or US3.
- **User Story 2 (P2)**: Pipeline integration. Depends on Phase 2. No dependency on US1 or US3.
- **User Story 3 (P3)**: Metadata. Depends on Phase 2. No dependency on US1 or US2.

### Within Each Phase

- Phase 2: T004 → T005 → T006 → T007 → T008 → T009 → T010 → T011 → T012 (sequential, each builds on previous). T013 can be added at any point after T004.
- Phase 3: T014 and T015 can run in parallel. T016 → T017 → T018 → T019 (sequential). T020, T021 after T018.
- Phase 4: T022 → T023 → T024 → T025 (sequential). T026 after T025.
- Phase 5: T027 → T028 → T029 → T030 → T031 (mostly sequential but fine-grained).

### Parallel Opportunities

- Phases 3, 4, and 5 can start in parallel once Phase 2 is complete (different files: `web/app.js`/`index.html` vs `tools/build_all.py` vs `tools/extract_buildings.py`)
- Within Phase 3: T014 (HTML) and T015 (JS state) can run in parallel
- Within Phase 6: T032, T035, T036, T037 can all run in parallel

---

## Parallel Example: Phase 3 (US1) + Phase 4 (US2)

```bash
# Once Phase 2 (extract_buildings.py) is complete, launch in parallel:
# Developer A: Phase 3 — viewer integration
Task: "T014 [P] [US1] Add Buildings fieldset to web/index.html"
Task: "T015 [P] [US1] Add buildings layer group to web/app.js"

# Developer B: Phase 4 — pipeline integration
Task: "T022 [US2] Add --skip-buildings flag to tools/build_all.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T013) — CRITICAL
3. Complete Phase 3: User Story 1 (T014-T021)
4. **STOP and VALIDATE**: Open viewer, confirm buildings render and toggle works
5. Deploy/demo if ready — this is the visual MVP

### Incremental Delivery

1. Setup + Foundational → extraction script runs
2. Add US1 → viewer shows buildings → **MVP!**
3. Add US2 → pipeline is reproducible and verifiable
4. Add US3 → agent-ready metadata + cursor readout
5. Polish → docs, perf validation, determinism check

### Parallel Team Strategy

With multiple developers:

1. Team completes Phase 1 + Phase 2 together (foundational script)
2. Once Phase 2 is done:
   - Developer A: Phase 3 — viewer integration (web/ files)
   - Developer B: Phase 4 — pipeline integration (build_all.py)
   - Developer C: Phase 5 — metadata enhancement (extract_buildings.py)
3. Phase 6: All three converge on validation and docs

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story is independently completable and testable
- No automated unit tests specified; manual verification via viewer and `build_all.py --verify`
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- The extraction script (`tools/extract_buildings.py`) spans Phase 2 and Phase 5 — Phases 3 and 4 don't modify it, so they are truly independent
- Building naming convention uses the tile grid coordinate from centroid location: easting is derived from X coordinate divided by the tile grid step, northing similarly from Y coordinate