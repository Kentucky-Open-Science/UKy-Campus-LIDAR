# Data Model: Building Extraction

**Feature**: 001-building-extraction | **Phase**: 1 | **Date**: 2026-06-12

## Entities

### Building

A single campus structure identified from clustered building-class LiDAR points.

| Field | Type | Description |
|-------|------|-------------|
| name | string | Unique identifier, derived from grid coordinate (e.g., `B-15626E-185064N-001`) |
| file | string | Path relative to `web/data/`: `buildings/<name>.bin` |
| bounds_min_cm | float[3] | Minimum X,Y,Z of the mesh in UE centimeters |
| bounds_max_cm | float[3] | Maximum X,Y,Z of the mesh in UE centimeters |
| height_cm | float | `bounds_max_cm[2] - bounds_min_cm[2]` |
| footprint_area_m2 | float | Area of the concave-hull footprint polygon in square meters |
| point_count | int | Number of LiDAR points (class 6) that formed this cluster |
| vertex_count | int | Number of vertices in the output mesh |
| index_count | int | Number of indices in the output mesh (triangles × 3) |

**Naming convention**: `B-<easting>-<northing>-<seq>`
- `<easting>`: 5-digit easting from tile grid (e.g., `15626`)
- `<northing>`: 6-digit northing from tile grid (e.g., `185064`)
- `<seq>`: zero-padded 3-digit sequence within that tile (001, 002, ...)
- This guarantees uniqueness and ties each building to its spatial region.

### Building Mesh File (.bin)

Binary file format — identical structure to existing terrain mesh files,
but without UV data.

| Offset | Type | Field | Description |
|--------|------|-------|-------------|
| 0 | u32 | vert_count | Number of vertices |
| 4 | u32 | index_count | Number of indices (triangles × 3) |
| 8 | f32[vert_count×3] | positions | Vertex positions in UE cm (X, Y, Z) |
| 8 + vc×12 | u32[index_count] | indices | Triangle indices (winding order as generated) |

Total file size: `8 + vert_count × 12 + index_count × 4` bytes.

### Building Manifest Entry (in manifest.json)

```json
{
  "buildings": {
    "format": "u32 vc, u32 ic, f32 pos[vc*3], u32 idx[ic] -- UE cm, no UVs",
    "tiles": [
      {
        "name": "B-15626E-185064N-001",
        "file": "buildings/B-15626E-185064N-001.bin",
        "bounds_min_cm": [-91525.0, -170396.5, 0.0],
        "bounds_max_cm": [-89500.0, -169800.0, 1500.0],
        "height_cm": 1500.0,
        "footprint_area_m2": 425.3,
        "point_count": 3450,
        "vertex_count": 128,
        "index_count": 384
      }
    ]
  }
}
```

## Relationships

```
LiDAR chunks (*.bin)
  └── class-6 filter ──┐
                        ├── DBSCAN clustering ──► Building clusters
                        │
Footprint extraction ◄──┘  (shapely concave_hull on XY projection)
  └── extrude + triangulate ──► Building Mesh (*.bin)

Building Mesh  +  Building metadata  ──► manifest.json (buildings section)

manifest.json
  ├── terrain.tiles[]   -- existing, unchanged
  ├── lidar.chunks[]    -- existing, unchanged
  └── buildings[]       -- NEW for this feature
```

## State Transitions

No runtime state. Building data is generated once in batch and consumed
statically. The only transition is the extraction pipeline:

```
[Pending] ── run extract_buildings.py ──► [Extracted] ── build_all.py --verify ──► [Verified]
```

- **Pending**: No `web/data/buildings/` directory, no `buildings` key in
  manifest.json.
- **Extracted**: Files written, manifest updated.
- **Verified**: `build_all.py --verify` confirms all files exist and metadata
  is consistent.

## Validation Rules

- Every building MUST have a unique `name`.
- Every `file` reference MUST resolve to an existing `.bin` file.
- `bounds_min_cm[i] < bounds_max_cm[i]` for i ∈ {0,1,2}.
- `height_cm > 0`.
- `footprint_area_m2 > 0`.
- `point_count >= 10` (implied by DBSCAN min_samples, but verified).
- `vertex_count > 0`, `index_count > 0`, `index_count % 3 == 0`.
- All coordinates are in UE centimeters (same frame as terrain and lidar).