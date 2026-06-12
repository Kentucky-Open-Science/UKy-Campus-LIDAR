# Contract: Building Mesh Binary Format

**Version**: 1.0
**Feature**: 001-building-extraction
**Spec**: [data-model.md](../data-model.md)

## Overview

Building meshes use the same little-endian binary layout as terrain tiles
(defined in `web/README.md`), with one difference: **no UV data** — buildings
are untextured in v1.

## Format (little-endian)

| Offset | Size | Type | Field |
|--------|------|------|-------|
| 0 | 4 | u32 | `vert_count` |
| 4 | 4 | u32 | `index_count` |
| 8 | `vert_count × 12` | f32[3] | `positions` — UE centimeters |
| 8 + vc×12 | `index_count × 4` | u32[] | `indices` — triangle list |

## Constraints

- `vert_count > 0`
- `index_count > 0`
- `index_count % 3 == 0` (complete triangles)
- Positions in same UE world centimeter coordinate frame as `manifest.json`
  `origin_cm` and terrain tile positions.

## Consumer Notes

To load a building mesh in Three.js:

```js
const dv = new DataView(buffer);
const vc = dv.getUint32(0, true);
const ic = dv.getUint32(4, true);
const pos = new Float32Array(buffer, 8, vc * 3);
const idx = new Uint32Array(buffer, 8 + vc * 12, ic);
// Apply same transform as terrain tiles:
// three.(x,y,z) = (ue.x - originCm[0], ue.z - originCm[2], ue.y - originCm[1]) * 0.01
```

## Versioning

If a future version adds UVs, normals, or other attributes, the format will
be versioned by incrementing this document and adding a distinguishing field
to the manifest entry (e.g., `"format_version": 2`). The viewer checks the
`format` key in `manifest.json > buildings` to select a loader path.

## Manifest Contract

The `manifest.json` contract extension for buildings:

```json
{
  "buildings": {
    "format": "u32 vc, u32 ic, f32 pos[vc*3], u32 idx[ic] -- UE cm, no UVs",
    "tiles": [
      {
        "name": "<unique-id>",
        "file": "buildings/<name>.bin",
        "bounds_min_cm": [x, y, z],
        "bounds_max_cm": [x, y, z],
        "height_cm": <float>,
        "footprint_area_m2": <float>,
        "point_count": <int>,
        "vertex_count": <int>,
        "index_count": <int>
      }
    ]
  }
}
```

All fields are required for every building entry.