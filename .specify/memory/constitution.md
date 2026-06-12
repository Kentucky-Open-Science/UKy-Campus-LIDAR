<!--
  Sync Impact Report
  ==================
  Version change: none → 1.0.0 (initial ratification)
  Modified principles: N/A (initial creation)
  Added sections:
    - Preamble (Project Identity)
    - Principle I: Open Formats, No Lock-In
    - Principle II: Browser-Native Rendering
    - Principle III: Reproducible Extraction Pipeline
    - Principle IV: Geospatial Accuracy
    - Principle V: Agent-Ready World Model
    - Principle VI: Progressive Performance
    - Governance
  Removed sections: none
  Templates requiring updates:
    - .specify/templates/plan-template.md (not yet created) - N/A
    - .specify/templates/spec-template.md (not yet created) - N/A
    - .specify/templates/tasks-template.md (not yet created) - N/A
  Follow-up TODOs: none
-->

# UKy Campus Digital Twin — Project Constitution

**Ratification Date:** 2026-06-12
**Last Amended:** 2026-06-12
**Version:** 1.0.0

---

## Preamble

This project creates an interactive, browser-native 3D digital twin of the
University of Kentucky campus from UE 4.24.3 editor assets (LiDAR point cloud,
DTM terrain tiles, aerial imagery). The extraction pipeline runs entirely
outside Unreal Engine and produces open-format, self-describing data served as
static files.

**Long-term vision:** The 3D environment serves as a world model where agentic
AI agents and autonomous robots can navigate, reason about, and interact with a
spatially accurate digital representation of the physical campus.

---

## Principle I — Open Formats, No Lock-In

All extracted data MUST be stored in open, self-describing binary formats with
an accompanying JSON manifest. No viewer or agent that consumes this data
SHALL require Unreal Engine, any proprietary plugin, or any commercial license.

**Rules:**
- Terrain meshes: little-endian `u32 vc, u32 ic, f32[pos*3], f32[uv*2], u32[idx*3]`.
- Textures: standard JPEG, quality 82, max 4096px dimension.
- Point cloud: little-endian `u32 count` header + `{f32 x,y,z, u8 r,g,b,a}` records per chunk.
- A single `manifest.json` at the data root declares coordinate space, tile
  inventory, lidar chunk inventory, and origin offset.

**Rationale:** The digital twin is a long-lived asset. Proprietary formats rot.
Open binary contracts ensure any language, engine, or agent runtime can consume
the data without reverse-engineering.

---

## Principle II — Browser-Native Rendering

The primary interactive viewer MUST run in a web browser with zero install,
zero build step, and zero server-side computation. All rendering is client-side
WebGL via Three.js.

**Rules:**
- Three.js vendored at a pinned version (currently 0.160.0).
- OrbitControls vendored; no CDN dependency at runtime.
- JavaScript ES modules with importmap (`"three": "./lib/three.module.js"`).
- Static file serving only (`python -m http.server` or equivalent).
- UI: layer toggles, opacity, point budget, wireframe, camera reset, WASD fly.
- FPS/triangle/point counter and georeferenced cursor readout.

**Rationale:** A digital twin's value is proportional to its accessibility.
Browser-native delivery removes platform barriers and enables instant sharing.
Vendored dependencies guarantee the viewer works offline and survives upstream
breaking changes.

---

## Principle III — Reproducible Extraction Pipeline

The full pipeline from `.uasset` sources to `web/data/` MUST be reproducible
with a single command and a pinned Python environment.

**Rules:**
- `tools/build_all.py` orchestrates all extraction steps.
- `requirements.txt` pins `numpy`, `pillow`, `playwright` versions.
- A `.venv/` with those packages is the canonical runtime.
- Every extraction tool MUST be idempotent (safe to re-run over existing output).
- `tools/build_all.py --verify` MUST confirm data integrity (all referenced
  files exist, counts match manifest) without re-extracting.

**Rationale:** Reproducibility is non-negotiable for trust. Any contributor
must be able to regenerate the entire dataset from the original `.uasset`
sources and confirm the output is byte-identical to the published data.

---

## Principle IV — Geospatial Accuracy

The coordinate system and transform chain from UE4 world space to Three.js
scene space MUST be documented and testable. Every asset that carries a
georeference (LiDAR `OriginalCoordinates`, State Plane tile naming) MUST
preserve that information through the pipeline.

**Rules:**
- `manifest.json` declares `origin_cm` and per-tile `translation_cm`,
  `rotation_deg`, `scale`.
- The transform chain UE cm → Three.js meters is documented in `web/README.md`
  and implemented in `app.js` (`ueCmToScene`, `ueRotationMatrix`).
- LiDAR `original_coordinates` (double vector, KY State Plane cm) MUST be in
  the manifest.
- Cursor readout SHALL report scene meters, UE cm, and georeferenced
  coordinates when `original_coordinates` is available.

**Rationale:** A world model useful to autonomous agents MUST be metrically
accurate. Coordinates that drift or have undocumented transforms make
navigation, localization, and sensor fusion impossible.

---

## Principle V — Agent-Ready World Model

The data contract, coordinate system, and scene graph MUST be designed so that
an external agent runtime (AI agent, robot controller, simulation engine) can
consume the environment without HTML/JS dependencies.

**Rules (current phase):**
- The binary data contract (`manifest.json` + `.bin` mesh/lidar chunks) is
  language-agnostic and fully documented in `web/README.md`.
- The coordinate frame (UE cm, Z-up) and origin offset are explicit in the
  manifest.
- Terrain geometry includes per-tile bounds so an agent can determine which
  tiles to load for a given spatial query.

**Rules (future phase — agent integration):**
- The data directory SHALL serve a REST or WebSocket endpoint exposing mesh
  bounds, point cloud spatial index, and collision proxies.
- A navigation mesh (NavMesh) export from the terrain DTM SHALL be added.
- An agent SDK (Python or TypeScript) SHALL provide `get_tile(x, y)`,
  `query_lidar(bbox)`, `raycast(origin, direction)` over the static data.
- Autonomous robot simulation in the environment MUST use real-world units
  (meters) and respect the documented coordinate transform.

**Rationale:** The campus digital twin is not just a visualization tool — it is
infrastructure for embodied AI research. The data format and access patterns
must anticipate programmatic consumers, not just human viewers.

---

## Principle VI — Progressive Performance

The viewer MUST load and display data progressively so that a usable scene
appears within seconds, even on modest hardware, while the full dataset
continues loading.

**Rules:**
- Terrain tiles load sequentially, each rendered immediately upon arrival.
- LiDAR chunks load in manifest order until the point budget is met; additional
  chunks are hidden, not dropped (user can adjust budget slider).
- A loading overlay shows current operation (tile name, chunk number).
- Default point budget is the full dataset (all chunks visible).
- Fallback checkerboard textures appear for tiles whose ortho imagery is still
  loading.
- Frame time budget: target 30+ fps on integrated GPU with full terrain +
  point cloud visible.

**Rationale:** The full dataset is ~300 MB. Blocking until all data arrives
creates a poor experience and fails on slow connections. Progressive loading
with visible partial results keeps the viewer responsive.

---

## Governance

### Amendment Procedure

1. Propose changes via pull request to this file (`.specify/memory/constitution.md`).
2. Changes that alter principles (add, remove, redefine) require a MAJOR or
   MINOR version bump per semantic versioning.
3. All amendments MUST include a Sync Impact Report as an HTML comment at the
   top of this file.
4. Template files under `.specify/templates/` and runtime guidance docs
   (`README.md`, `web/README.md`, `PLAN.md`) MUST be reviewed for consistency
   with any amended principle.

### Versioning Policy

- **MAJOR**: Backward-incompatible governance or principle removals/redefinitions.
- **MINOR**: New principle added, new section added, materially expanded guidance.
- **PATCH**: Clarifications, wording improvements, typo fixes.

### Compliance Review

- Every feature spec and task list MUST reference the principles it touches.
- The `build_all.py --verify` command serves as an automated compliance gate
  for Principles I and III.
- Before merging any change that touches `web/data/` output or manifest
  structure, run `build_all.py --verify` and confirm the viewer loads without
  console errors.