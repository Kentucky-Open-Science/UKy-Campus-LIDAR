# Implementation Plan: Building Extraction from LiDAR Point Cloud

**Branch**: `001-building-extraction` | **Date**: 2026-06-12 | **Spec**: [spec.md](./spec.md)

**Input**: Feature specification from `/specs/001-building-extraction/spec.md`

## Summary

Extract 3D building meshes from the ~5.7M building-class (code 6) LiDAR points
already present in `web/data/lidar/chunk_*.bin`. Cluster points into individual
structures via DBSCAN, compute concave-hull footprints from the XY projection,
extrude to per-building max height, and output simplified meshes in the existing
binary format. Integrate into the Three.js viewer as a new layer and into
`build_all.py` as a pipeline step.

## Technical Context

**Language/Version**: Python 3.13 (project `.venv`)

**Primary Dependencies**:
- `numpy` 2.4.6 — point cloud loading, array operations
- `scipy` (new) — spatial KDTree for DBSCAN neighbor queries, `ConvexHull`
- `shapely` (new) — concave hull (alpha shape) computation, polygon area
- `Pillow` 11.3.0 — already in requirements.txt
- Three.js 0.160.0 — already vendored in `web/lib/`

**Storage**: Static binary files (`.bin`) served via HTTP, JSON manifest.
No database. Output to `web/data/buildings/`.

**Testing**: Manual visual inspection in viewer. `build_all.py --verify` for
automated integrity check. `playwright` (already installed) for headless
viewer testing.

**Target Platform**: Python batch pipeline (Windows, local). Viewer: any modern
browser with WebGL.

**Project Type**: Single-project CLI pipeline + static web viewer.

**Performance Goals**:
- Extraction pipeline completes in <5 minutes (SC-001)
- Building mesh files combined <50 MB (SC-003)
- Viewer renders buildings at 30+ fps alongside terrain + LiDAR (SC-004)

**Constraints**:
- 16 GB RAM, 4 CPU cores (standard workstation)
- No Unreal Engine dependency (open formats only)
- Deterministic output (same input → same output, FR-010)

**Scale/Scope**: ~200 campus buildings from 24.9M LiDAR points (5.7M building-class).

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I — Open Formats | PASS | `.bin` format is the same documented binary contract as terrain meshes. No proprietary dependencies. |
| II — Browser-Native | PASS | Buildings are served as static `.bin` files; viewer renders them client-side. No build step, no server compute. |
| III — Reproducible | PASS | `tools/extract_buildings.py` is idempotent and integrated into `build_all.py`. `--verify` covers it. |
| IV — Geospatial Accuracy | PASS | Building coordinates are in the same UE cm frame as terrain/lidar. `manifest.json` includes bounds per building. |
| V — Agent-Ready | PASS | Manifest includes per-building metadata (bounds, height, footprint area). Binary data is language-agnostic. |
| VI — Progressive Performance | PASS | Buildings are a separate loadable layer; viewer can display terrain/textures before buildings arrive. |

**Gate result**: PASS — all principles satisfied without violations.

**Post-design re-check**: Confirmed after Phase 1. No violations introduced.
Bin format unchanged. New dependencies `scipy` and `shapely` are MIT/BSD-licensed
open-source, compatible with the project's open-format philosophy.

## Project Structure

### Documentation (this feature)

```
specs/001-building-extraction/
├── plan.md              # This file
├── research.md          # Phase 0 output — algorithm selection & parameter tuning
├── data-model.md        # Phase 1 output — entity schema, file format spec
├── quickstart.md        # Phase 1 output — runnable validation scenarios
├── contracts/           # Phase 1 output (N/A — all interfaces are file-based)
└── tasks.md             # Phase 2 output (NOT created by /speckit.plan)
```

### Source Code (repository root — new/modified files)

```
tools/
├── extract_buildings.py    # NEW: main extraction script
├── build_all.py            # MODIFIED: add --skip-buildings flag, building step

web/
├── app.js                  # MODIFIED: add Buildings layer, toggle, loading
├── index.html              # MODIFIED: add Buildings fieldset to UI panel
└── data/
    └── buildings/          # NEW: output directory for *.bin building meshes

requirements.txt            # MODIFIED: add scipy, shapely
```

## Complexity Tracking

> No violations to justify. All constitution gates pass cleanly.