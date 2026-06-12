# Feature Specification: Building Extraction from LiDAR Point Cloud

**Feature Branch**: `001-building-extraction`

**Created**: 2026-06-12

**Status**: Draft

**Input**: User description: "We have the textures and point cloud loaded (which works great), now I need you to generate 3d structures based off the point cloud. basically i want you to create a digital twin"

## Clarifications

### Session 2026-06-12

- Q: How should building geometry be constructed from clustered points? → A: Alpha shape / concave hull footprint + height-based extrusion. Render priority: buildings first, then streets/intersections, then terrain, then cars and people.
- Q: Minimum building size filter for mesh generation? → A: No minimum — generate meshes for every cluster regardless of size. No filtering by footprint area or point count.
- Q: Render priority and feature phasing for non-building structures? → A: Buildings only in v1. Streets/intersections, cars, and people are planned follow-ups to be defined in separate feature specs.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Viewer Sees 3D Buildings in the Campus Scene (Priority: P1)

A user opens the web viewer and sees recognizable 3D building geometries
overlaid on the terrain, alongside the existing LiDAR point cloud and aerial
imagery. Buildings appear at their real-world locations with approximate
height, footprint, and roof shape derived from the classified building-class
LiDAR points.

**Why this priority**: This is the core deliverable — transforming the raw
point cloud into a recognizable digital twin with structures. Without visible
buildings, the experience is just a terrain map with dots.

**Independent Test**: Open the viewer, observe that building meshes appear at
locations matching the underlying point cloud's building-class points. Verify
building count matches known UKy campus structures (roughly 200+ buildings).
Toggle terrain and LiDAR layers independently to confirm buildings render as a
separate layer.

**Acceptance Scenarios**:

1. **Given** the viewer has loaded terrain and LiDAR, **When** the new building
   extraction pipeline has been run, **Then** building 3D meshes appear in the
   scene at positions consistent with the LiDAR building-class points.
2. **Given** the viewer with buildings visible, **When** the user toggles a
   new "Buildings" layer checkbox off, **Then** all building meshes disappear.
3. **Given** the viewer, **When** the user navigates to a known campus landmark
   (e.g., a stadium or tower), **Then** a recognizable 3D approximation of
   that structure is visible.

---

### User Story 2 - Building Extraction Pipeline Runs in One Command (Priority: P2)

A developer or operator runs a single command that processes the LiDAR
building-class points, clusters them into individual structures, generates
simplified 3D mesh geometry per building, and writes the output into the
existing `web/data/` directory. The pipeline integrates with the existing
`build_all.py` orchestrator.

**Why this priority**: Reproducibility is a core project principle (III).
The extraction must be automated and idempotent, not a manual modeling effort.

**Independent Test**: Run the extraction command on the LiDAR dataset. Verify
it produces building mesh files under `web/data/buildings/` and updates
`web/data/manifest.json` with a `buildings` section. Re-run the command and
confirm output is identical (idempotent).

**Acceptance Scenarios**:

1. **Given** the project `.uasset` LiDAR source and the existing extracted
   chunk files, **When** the building extraction script is run, **Then**
   building mesh files appear under `web/data/buildings/` in the agreed binary
   format.
2. **Given** building extraction has completed, **When** `build_all.py --verify`
   is run, **Then** building data integrity is confirmed alongside terrain,
   textures, and lidar.
3. **Given** building extraction was already run once, **When** it is run a
   second time, **Then** output is byte-identical to the first run.

---

### User Story 3 - Buildings Have Per-Building Metadata and Can Be Queried Programmatically (Priority: P3)

The manifest includes per-building metadata (bounding box, height, footprint
area, building class/type if inferable) so that future agentic AI or robot
systems can query and reason about individual structures.

**Why this priority**: Supports Principle V (Agent-Ready World Model).
Without structured metadata, buildings are just anonymous geometry blobs.

**Independent Test**: Load the manifest's `buildings` array in a Python script.
For each building, verify the metadata fields are populated and consistent
with the corresponding mesh file. Write a simple spatial query (e.g., "find
buildings within 100m of coordinate X,Y") using the bounds in the manifest.

**Acceptance Scenarios**:

1. **Given** building extraction has completed, **When** a program reads
   `manifest.json`, **Then** the `buildings` array contains entries with
   `name`, `file`, `bounds_min_cm`, `bounds_max_cm`, `height_cm`, and
   `footprint_area_m2` fields.
2. **Given** the buildings manifest, **When** a spatial query is performed for
   buildings within a bounding box, **Then** the result correctly identifies
   only buildings whose bounds intersect that box.

---

### Edge Cases

- What happens when building-class points span across chunk boundaries?
  The extraction algorithm must handle cross-chunk clusters.
- How does the system handle buildings that are partially outside the
  terrain tile coverage area? Those buildings are extracted but may not
  have corresponding terrain underneath.
- What about structures that are not buildings (e.g., bridges, overpasses,
  walls) that may be included in the building classification? The pipeline
  should separate by geometry: tall, compact clusters are buildings; flat,
  elongated clusters may be flagged differently.
- Point density varies: areas near campus center have denser returns than
  periphery. The clustering and mesh generation must handle sparse data
  gracefully (produce coarser geometry rather than nothing).

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a Python extraction tool (`tools/extract_buildings.py`)
  that reads the LiDAR point cloud chunks and produces simplified 3D building meshes.
- **FR-002**: The extraction tool MUST filter for building-class LiDAR points
  (classification code 6) and exclude ground, vegetation, and other classes.
- **FR-003**: System MUST cluster building-class points into individual
  structures using spatial proximity (DBSCAN or similar density-based
  clustering) with parameters tuned for the campus building layout.
  All clusters are included — no minimum size filter is applied.
- **FR-004**: System MUST generate a simplified 3D mesh per building via Alpha shape
  (concave hull) of the footprint XY projection, extruded vertically to each
  structure's maximum height. Roof surface is derived from the top-most points
  within the footprint. This produces recognizable non-rectangular silhouettes
  with controlled vertex counts.
- **FR-005**: Building meshes MUST be stored in an open binary format matching
  the existing data contract: little-endian `u32 vert_count, u32 index_count,
  f32 positions[v*3], u32 indices[i]` (positions in UE cm, same coordinate
  frame as terrain tiles).
- **FR-006**: System MUST write per-building metadata to `extracted/manifest-buildings.json`
  and integrate entries into the unified `web/data/manifest.json` under a
  `buildings` key containing an array of `{name, file, bounds_min_cm,
  bounds_max_cm, height_cm, footprint_area_m2, point_count, vertex_count,
  index_count}`.
- **FR-007**: The `build_all.py` orchestrator MUST include building extraction
  as a step (after lidar, before manifest merge), with support for
  `--skip-buildings` flag.
- **FR-008**: The Three.js viewer MUST load building meshes as a new layer
  group, with a "Buildings" toggle checkbox in the UI panel, rendered with
  a neutral material (solid color or wireframe, since buildings lack
  individual texture data).
- **FR-009**: Building meshes MUST use a solid grey or color-coded-by-height
  material in the viewer, with the ability to toggle wireframe mode.
- **FR-010**: The extraction MUST be deterministic: given the same input
  point cloud, the output meshes and metadata are identical across runs.

### Key Entities

- **Building**: A single campus structure identified from clustered
  building-class LiDAR points. Attributes: unique ID (name), bounding box
  (min/max in UE cm), height (cm), footprint area (m²), vertex/triangle counts,
  associated mesh file path, source point count.
- **Building Mesh**: A binary file containing vertex positions and triangle
  indices representing a simplified 3D model of one building. Same format
  convention as terrain meshes (UE cm coordinates, little-endian).
- **Building Manifest**: JSON document listing all extracted buildings with
  metadata, integrated into the top-level `manifest.json`.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The extraction pipeline processes all 24.9M LiDAR points and
  produces building meshes in under 5 minutes on a standard workstation
  (16 GB RAM, 4 cores).
- **SC-002**: At least 80% of known UKy campus buildings (approximately 200
  structures) are represented as distinct 3D meshes recognizable by a human
  observer comparing the viewer to satellite imagery.
- **SC-003**: Building mesh files total under 50 MB (combined) so the viewer
  load time remains acceptable when added to existing terrain and point cloud
  data.
- **SC-004**: The viewer renders all building meshes at 30+ fps on integrated
  GPU when terrain and point cloud are also visible.
- **SC-005**: A first-time user can open the viewer and identify at least 5
  specific campus buildings by shape alone within 30 seconds of exploration.
- **SC-006**: `build_all.py --verify` passes with building data present,
  confirming files exist and metadata is consistent.

## Assumptions

- Building-class points (classification 6) from the LiDAR provide sufficient
  spatial coverage to extract recognizable building geometry without
  additional manual modeling.
- The DBSCAN parameters will be tuned once based on campus building layout
  (approximate building spacing 10-100m, heights 3-80m) and baked into the
  extraction script as constants.
- Building meshes are untextured in v1; aerial imagery already provides
  visual context. Future iterations may drape aerial textures onto building
  roofs.
- The extraction runs as a batch preprocessing step, not in real-time in the
  browser. Building meshes are static assets served alongside terrain tiles.
- Buildings outside the terrain mesh coverage area are still extracted and
  included (they may float without terrain beneath them, but the point cloud
  provides visual context).
- The viewer's buildings layer uses a uniform material (MeshBasicMaterial or
  MeshStandardMaterial with color-by-height) rather than per-building
  textures, since the source .uasset materials are terrain-only.
- Coordinate system is the same UE world centimeters as all other assets;
  the shared origin transform from `manifest.json` applies consistently.

## Future Phases (out of scope for v1)

The following digital twin layers are planned as separate feature specs to be
developed after building extraction is complete and validated:

| Phase | Layer | Priority | Notes |
|-------|-------|----------|-------|
| P4 | Streets & intersections | High | Extract from ground-class LiDAR points (class 2) + terrain DTM. Generate road centerlines, intersection zones, and pedestrian paths. |
| P5 | Vehicles (static) | Medium | Detect car-sized clusters from unclassified/low-vegetation points; place simplified vehicle proxy meshes in parking lots and along streets. |
| P5 | Pedestrians / people | Low | Animated or static human-scale proxy objects for campus life simulation. Likely requires external data or procedural generation rather than LiDAR extraction. |