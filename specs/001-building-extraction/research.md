# Research: Building Extraction from LiDAR Point Cloud

**Feature**: 001-building-extraction | **Phase**: 0 | **Date**: 2026-06-12

## Research Tasks

### 1. Alpha Shape / Concave Hull Algorithm for Footprint Extraction

**Decision**: Use `shapely.concave_hull` with ratio parameter.

**Rationale**:
- Shapely's `concave_hull(ratio)` produces footprint polygons directly from
  2D point arrays. The `ratio` parameter (0 = convex hull, 1 = maximally
  concave) controls how tightly the hull follows the point cloud outline.
- For campus buildings, a ratio of 0.3–0.5 balances capturing non-rectangular
  footprints (L-shaped wings, curved walls) against noise from sparse edges.
- Shapely is BSD-licensed, actively maintained, and compatible with the
  project's Python 3.13 `.venv`.

**Alternatives considered**:
- `alphashape` library (pure Python): Slower on large point sets, less
  mature than shapely.
- Manual Delaunay triangulation + edge filtering: More code, harder to
  tune, no benefit over shapely for this use case.
- Open3D: Heavy dependency (C++ bindings), overkill for 2D footprint work.

### 2. DBSCAN Parameter Tuning for Campus-Scale Building Clustering

**Decision**: `eps=500` (5m in UE cm), `min_samples=10`.

**Rationale**:
- Campus buildings are typically spaced 10-100m apart. Eps=500cm ensures
  adjacent buildings (walls within 5m) are separated into distinct clusters.
- Min_samples=10 filters out noise clusters with fewer than 10 points (a
  5m×5m area with LiDAR sampling of ~1 pt/m² would yield ~25 points; 10 is
  conservative).
- At 5.7M points, `scipy.spatial.KDTree` provides O(n log n) query performance
  suitable for the 5-minute time budget.
- These parameters are baked as constants; FR-010 (deterministic output)
  guarantees reproducibility.

**Alternatives considered**:
- HDBSCAN: Better at variable cluster density but significantly slower on
  5.7M points; requires cluster tree building. Not needed since campus
  building density is relatively uniform (not mixed urban/rural).
- Grid-based clustering: Faster but requires fixed grid cell size; produces
  artifacts at grid boundaries for buildings that span cells.

### 3. Mesh Generation Strategy from Footprint + Point Heights

**Decision**: Extrude concave hull footprint polygon, cap with triangulated
high-point roof surface.

**Rationale**:
- Floor: footprint polygon vertices at base elevation (min Z of cluster).
- Walls: each footprint edge becomes two triangles (wall quad), duplicated
  at max Z (roof level). Total verts = 2 × hull_vertices.
- Roof: triangulate the hull polygon at max Z elevation using ear-clipping
  (shapely's built-in triangulation of polygon exterior).
- Simplification: if a building is tall and thin (aspect ratio > 5:1),
  keep the full hull. Otherwise, decimate hull vertices to keep per-building
  vertex count under ~500.
- Output matches the project binary format: `u32 vc, u32 ic, f32 pos[vc*3],
  u32 idx[ic]` (no UVs — buildings are untextured in v1).

**File size estimate**: ~200 buildings × ~300 verts/building × 12 bytes/vert
(positions) + ~600 indices × 4 bytes = ~200 × (3600 + 2400) ≈ 1.2 MB raw.
Plus index overhead, header. Ballpark ~2-5 MB total, well under the 50 MB cap.

### 4. DBSCAN Performance Scaling

**Decision**: Load points in batches, build a single global KDTree.

**Rationale**:
- The 64 existing lidar chunks total ~12M points (decimated). Filtering to
  building-class (code 6) yields ~5.7M points. This fits in memory: 5.7M ×
  12 bytes (3×f32) = 68 MB.
- Build one `scipy.spatial.KDTree` over all building XY points. DBSCAN via
  `KDTree.query_ball_point` is O(n log n), well within 5-minute budget.
- Memory usage: 68 MB (points) + ~200 MB (KDTree internal) ≈ 270 MB.
  Within 16 GB RAM constraint with room to spare.

### 5. New Dependencies

**Decision**: Add `scipy` and `shapely` to `requirements.txt`.

**Rationale**:
- `scipy` (BSD): provides `KDTree` for fast DBSCAN neighbor search and
  `ConvexHull` for geometry operations.
- `shapely` (BSD): provides `concave_hull` for footprint generation and
  polygon area computation.
- Both are well-established scientific Python libraries compatible with
  numpy 2.x and Python 3.13.
- No additional C/C++ compilers needed; both have prebuilt Windows wheels.

**Updated requirements.txt**:
```
numpy>=2.0,<3.0
pillow>=10.0,<12.0
playwright>=1.40,<2.0
scipy>=1.12,<2.0
shapely>=2.0,<3.0
```