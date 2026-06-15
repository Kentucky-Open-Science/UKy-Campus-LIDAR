# UKy Campus — Interactive 3D Viewer

Extract UE 4.24.3 UKy Campus LiDAR point cloud + DTM terrain tiles + aerial
imagery into open formats and view them in an interactive Three.js web viewer
— **no Unreal Engine required**.

## Quick start

```sh
cd web
python -m http.server 8000
# Open http://localhost:8000/
```

The viewer loads `web/data/manifest.json` and streams terrain tiles + LiDAR
point chunks. All data is pre-extracted — the server just serves static files.

## What's in here

```
CAMPUS/
├── LIDAR/                    # UE4 .uasset source (POINT_CLOUD_2019, 448 MB)
├── MESHES/DTM_GRID/          # UE4 .uasset sources (16 meshes, 18 textures, 17 materials)
├── tools/                    # Python extraction tools
│   ├── uasset.py             # Core UE4 package parser (v518 / 4.24.3)
│   ├── inspect.py            # Export/property dumper
│   ├── extract_texture.py    # Texture2D -> PNG -> JPEG
│   ├── extract_mesh.py       # StaticMesh -> .bin (positions, UVs, indices)
│   ├── extract_lidar.py      # LidarPointCloud -> decimated chunked .bin
│   ├── extract_scene.py      # Blueprint scene assembly (transforms, materials)
│   ├── extract_buildings.py  # LiDAR building-class → 3D mesh (legacy, DBSCAN)
│   ├── extract_buildings_hybrid.py  # OSM footprints split LiDAR + give height
│   ├── verify_buildings_osm.py      # verify footprints vs OSM ground truth
│   ├── build_all.py          # Full pipeline orchestrator
│   └── verify_viewer.py      # Headless viewer test (requires playwright)
├── web/                      # Three.js viewer (static)
│   ├── index.html
│   ├── app.js
│   ├── style.css
│   ├── lib/                  # Vendored Three.js 0.160 + OrbitControls
│   └── data/                 # Generated extraction output
│       ├── manifest.json     # Unified scene manifest
│       ├── meshes/*.bin      # 16 terrain tiles (verts, UVs, indices)
│       ├── textures/*.jpg    # 18 aerial imagery textures
│       ├── lidar/chunk_*.bin # 64 decimated point-cloud chunks
│       └── buildings/*.bin   # ~890 extracted building meshes
└── extracted/                # Per-domain manifests + reports
```

## Re-extracting from source

All data is already extracted and under `web/data/`. If you need to regenerate:

```sh
pip install pillow numpy
python tools/build_all.py
# Or selectively:
python tools/build_all.py --skip-textures --skip-meshes
```

## Viewer controls

| Control | Action |
|---------|--------|
| Left mouse | Orbit |
| Right mouse | Pan |
| Scroll | Zoom |
| WASD | Fly |
| Q / E | Down / Up |
| Shift | 4x speed |

UI panel: layer toggles, terrain opacity, UV V-flip, point cloud budget slider,
wireframe mode, camera reset.

## Data stats

- 16 terrain tiles (804 m square grid, half-mile spacing)
- 18 ortho imagery textures (4096x4096, ~74 MB total JPEG)
- ~24.9M LiDAR points (decimated to ~12M in 64 chunks, ~183 MB)
- 3,109 building meshes (2,346 OSM-split + LiDAR-shaped, 763 LiDAR-only) from
  ~5.5M building-class LiDAR points + OpenStreetMap footprints (~3.9 MB total).
  Median footprint IoU vs OSM 0.93; verify with `python -m tools.verify_buildings_osm`
- Viewport: ~1.8 km x 3.4 km area centered on UKy Lexington campus