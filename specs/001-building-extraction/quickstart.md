# Quickstart: Building Extraction from LiDAR

**Feature**: 001-building-extraction | **Phase**: 1

## Prerequisites

- Python 3.13 `.venv` with all requirements installed:
  ```sh
  .venv/Scripts/pip install -r requirements.txt
  ```
- Existing `web/data/` populated with terrain meshes, textures, and lidar
  chunks (run `python tools/build_all.py` first if not yet extracted).

## Installation (new dependencies)

The building extraction adds `scipy` and `shapely`:

```sh
.venv/Scripts/pip install scipy shapely
```

## Run Extraction

```sh
# Full pipeline including buildings
python tools/build_all.py

# Or buildings-only
python tools/extract_buildings.py
```

Expected output:
- `web/data/buildings/` directory created with ~150-250 `.bin` files
- `web/data/manifest.json` updated with `buildings` key
- Console reports building count and total mesh size

## Verify

```sh
python tools/build_all.py --verify
```

Expected output:
```
verification OK — 18 textures, 16 meshes, 12.0M lidar pts in 64 chunks,
XX buildings
```

## Run the Viewer

```sh
cd web
python -m http.server 8000
# Open http://localhost:8000/
```

**Validation checklist**:

1. [ ] Buildings layer toggle appears in UI panel under "Terrain"
2. [ ] Toggle buildings ON — ~150-250 colored meshes appear at campus
       building locations
3. [ ] Toggle buildings OFF — meshes disappear, terrain + LiDAR remain
4. [ ] Navigate to a known landmark (stadium, tower) — recognizable
       3D shape is visible
5. [ ] Wireframe toggle works for buildings (viewer FR-009)
6. [ ] FPS counter stays ≥30 with all layers visible
7. [ ] Cursor readout shows correct scene coords when hovering over
       buildings

## Expected Artifacts

| Artifact | Location | Approx Size |
|----------|----------|-------------|
| Building meshes | `web/data/buildings/*.bin` | 2-5 MB total |
| Building manifest | `web/data/manifest.json` > `buildings` key | ~50 KB |
| Per-building metadata | `extracted/manifest-buildings.json` | ~100 KB |