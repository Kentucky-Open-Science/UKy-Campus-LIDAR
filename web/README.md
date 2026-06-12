# UKy Campus — web viewer

Three.js viewer for the terrain tiles + LiDAR point cloud extracted from the
UE 4.24 packages. Pure static files; no build step.

## Run

```sh
cd web
python -m http.server 8000
# then open http://localhost:8000/
```

- Real data: viewer loads `web/data/manifest.json` (produced by the
  integration agent).
- Synthetic test data: run `python tools/make_test_data.py` once (writes
  `web/data_test/`), then open `http://localhost:8000/?data=data_test`.

Three.js 0.160.0 is vendored in `web/lib/` (no internet needed at runtime).
`index.html` maps the bare specifier `three` to `./lib/three.module.js` via an
importmap, so `lib/OrbitControls.js` is unmodified upstream.

## Data contract (what app.js parses)

All geometry stays in UE world coordinates: **centimeters, Z-up, left-handed**.
The viewer converts per-vertex: `three.(x,y,z) = (ue.x, ue.z, ue.y)`, subtracts
`origin_cm` (in JS doubles, before the Float32 cast), scales cm→m (×0.01), and
flips triangle winding (the axis swap mirrors handedness). Materials are also
DoubleSide as belt-and-braces.

### `data/manifest.json`

```jsonc
{
  "origin_cm": [x, y, z],            // UE cm; subtracted from everything
  "terrain": {
    "tiles": [
      {
        "name": "15626E185064N",
        "mesh": "meshes/15626E185064N.bin",      // relative to data dir
        "texture": "textures/15626E185064N.jpg",
        "translation_cm": [0, 0, 0]              // optional, default 0
      }
    ]
  },
  "lidar": {
    "offset_cm": [x, y, z],          // added to chunk-relative point coords
    "original_coordinates": [X, Y, Z], // georeference doubles (optional)
    "chunks": [
      { "file": "lidar/chunk_000.bin", "count": 1000000 }  // or plain strings
    ]
  }
}
```

If `origin_cm` is missing the viewer falls back to `lidar.offset_cm`, then the
first tile's `translation_cm`, then `[0,0,0]`.

### `data/meshes/<NAME>.bin` (little-endian)

```
u32 vert_count
u32 index_count
f32 positions[vert_count*3]   // UE cm, exactly as stored in the asset
f32 uvs[vert_count*2]         // UV channel 0, raw
u32 indices[index_count]      // triangle list
```

### `data/lidar/chunk_NNN.bin` (little-endian)

```
u32 count
count * { f32 x, f32 y, f32 z, u8 r, u8 g, u8 b, u8 a }   // 16-byte stride
// xyz in UE cm relative to manifest lidar.offset_cm
```

## UI

- Layer toggles (terrain / LiDAR), terrain opacity, wireframe.
- **UV V-flip** checkbox (default ON: `v = 1 - v`, textures loaded with
  `flipY = false`). Toggle it if imagery appears mirrored north/south.
- Point size + point budget (M points) sliders; chunks load progressively in
  manifest order until the budget is met, extra loaded chunks are hidden.
- Camera reset, FPS/triangle/point counter.
- Cursor readout (raycast on terrain): scene meters, original UE cm, and
  georeferenced coords (`lidar.original_coordinates + UE cm`) when available.
- OrbitControls + WASD fly, Q/E down/up, Shift = 4× speed.

Missing files are reported per-layer in the panel; the viewer starts fine with
no data at all (shows a placeholder grid).
