# UKy Campus — Interactive render OUTSIDE Unreal Engine

**Goal:** Extract the UE 4.24.3 editor assets in this folder (LiDAR point cloud +
DTM terrain tiles + aerial imagery) into open formats and build an interactive
web viewer (Three.js) — no Unreal Engine required.

**Status legend:** [x] done · [~] in progress · [ ] todo

---

## Asset inventory

| Path | Contents |
|---|---|
| `LIDAR/POINT_CLOUD_2019.uasset` | 448 MB `LidarPointCloud` (plugin `/Script/LidarPointCloudRuntime`), 2019 flight colored with 2021 basemap |
| `MESHES/DTM_GRID/Meshes/*.uasset` | 16 StaticMesh terrain tiles (FBX-imported TINs from ArcGIS) |
| `MESHES/DTM_GRID/Textures/*.uasset` | 18 Texture2D aerial imagery tiles (PNG source payloads, up to ~220 MB) |
| `MESHES/DTM_GRID/Materials/*.uasset` | 18 MaterialInstanceConstant (1:1 name match mesh↔texture↔material) |
| `MESHES/DTM_GRID/GRID_DTM_COMBINED.uasset` | Blueprint that places all tiles (tile placement may also be derivable from mesh bounds/names) |

Tile naming = Kentucky State Plane-ish grid: `<EEEEE>E<NNNNNN>N` e.g. `15626E185064N`.
Easting steps of 26, Northing steps of 2640 (so easting likely in units of 100 ft → 2640 ft = half-mile square tiles).
Original imagery source: `.../ArcGIS/Projects/CampusDigitalTwin/GRID TIFs PNG/15626E185064N.png` (from AssetImportData).

## Format findings (verified so far)

### Package container (all files)
- UE4 uncooked editor packages, magic `0x9E2A83C1`, FileVersionUE4=**518** (4.24.3, CL 11590370), PackageFlags=0.
- **Working parser: `tools/uasset.py`** — parses summary, names, imports, exports,
  tagged properties, FByteBulkData. `tools/inspect.py` dumps any package.
- Summary quirk at v518: after `Guid` come `PersistentGuid` + `OwnerPersistentGuid` (16 B each) because !PKG_FilterEditorOnly.
- After each export's tagged-property list ("None" terminator) there is an int32 "has object guid" flag (usually 0) before native serialization.
- `bulk_data_start_offset` in summary = where end-of-file bulk payloads begin; bulk `offset` fields are relative to it (add it). File ends with 4-byte tail magic `c1832a9e`.

### Textures — SOLVED, trivial
- Texture2D export tagged props → `Source` (TextureSource struct): SizeX, SizeY (e.g. 4096×4096), NumMips=1, Format=`TSF_BGRA8`, **bPNGCompressed=True**.
- The single bulk payload = a complete **PNG file**: bytes `[bulk_data_start_offset : file_size-4]` (verified PNG magic `89504e47`).
- Extraction: slice bytes → write `.png` → downscale to web-friendly JPEG (e.g. 2048/4096) with Pillow.

### Meshes — container SOLVED, payload = FMeshDescription
- The end-of-file bulk region `[bulk_data_start_offset : file-4]` is ONE
  FArchive::SerializeCompressed blob. Layout (all i64): {magic 0x9E2A83C1,
  chunk_size 0x20000}, {comp_total, uncomp_total}, then n=ceil(uncomp/chunk)
  pairs {comp_i, uncomp_i}, then zlib streams. `decompress_chunked` in
  tools/uasset.py handles it (verified: 303,893 B → 1,449,553 B).
- Decompressed payload = **FMeshDescription** (NOT legacy FRawMesh):
  starts `02 0d 00 00` = 3330 then TBitArray of 0xFF = TSparseArray
  allocation mask (3330 verts, small tile). Need 4.24 MeshDescription.cpp
  serialization: VertexArray, VertexInstanceArray, EdgeArray, PolygonArray,
  PolygonGroupArray (TMeshElementArray=TSparseArray each), then
  TAttributesSet maps (FName-keyed: 'Position' f32x3 on vertices,
  'TextureCoordinate' f32x2 on vertex instances, etc.).
- StaticMesh export props include: SourceModels (1 LOD), StaticMaterials (slot "Default OBJ"), ExtendedBounds Origin ≈ (-89579, 31355, -245589) [note: huge offsets — mesh verts are in absolute world-ish coords], AssetImportData → FBX.
- After props + 4-byte guid flag: FStripDataFlags(2) + bCooked(4)=0 + BodySetup ref(4) + NavCollision ref(4) + 8 unknown bytes + LightingGuid(16) + Sockets count(4)=0, then per-source-model bulk data:
  - 4.24 stores RawMesh and/or MeshDescription bulk (FByteBulkData hdr: u32 flags, i32 count, i32 sizeOnDisk, i64 offset; then FGuid + bool bGuidIsHash). Flags bit 0x01=PayloadAtEndOfFile, 0x02=zlib (FArchive::SerializeCompressed chunked format — `decompress_chunked` in uasset.py), 0x40=inline.
  - Observed candidates in 15626E185064N: values 1449553 (uncompressed?) / 304917 (compressed?); payload region is 303,893 B. Exact field alignment TBD — brute-force parse the 142-byte native region.
  - If payload is FRawMesh (likely for legacy FBX import path, ImportVersion=1): serialized as versioned struct: i32 Version, i32 LicenseeVersion, then TArrays: FaceMaterialIndices(i32), FaceSmoothingMasks(u32), VertexPositions(FVector=3×f32), WedgeIndices(i32), WedgeTangentX/Y/Z(FVector), WedgeTexCoords[8](FVector2D), WedgeColors(FColor). Empty arrays = count 0.
  - If MeshDescription instead: bulkdata contains serialized FMeshDescription (name-indexed attribute maps; more complex — decode attribute arrays `Position`, `VertexInstance` UVs, triangle arrays).
- Fallback if mesh decode stalls: tiles are DTM grids — could regenerate terrain from extracted PNG ortho + LiDAR ground points. **Prefer real mesh decode.**

### LiDAR — native data observations (decode TODO)
- All 448 MB stored INLINE in export 1 (no end-of-file bulk; bulk region empty).
- After props (end 1771): 8 zero bytes, FString source path
  "...\LIDAR\points_colorized.las", then u32=1, u32=0xD0001, u32=0xD0001,
  u32=1, then 6×f32 octree box: min(-91525, -170396.5, -16872.5)
  max(+91525, +170396.5, +16872.5) [cm], then u32=1, zeros...
  Points likely TArray<FLidarPointCloudPoint>{FVector pos, FColor bgra,
  flag byte(s)} per octree node, ~17-18 B/pt × ~26M points.
- OriginalCoordinates: X=71898655.0, Y=<read from asset>, Z=<...> (doubles)
  = georeference offset (KY single-zone state plane related, likely cm).

### (old notes)
- Single export `POINT_CLOUD_2019` (448,663,319 B). Tagged props seen in names: OriginalCoordinates (DoubleVector = 3×f64), ClassificationsImported, SourcePath.
- Custom version GUID `50000000-43000000-50000000-46000000` ("P","C","P","F") version **15** = LidarPointCloud plugin's FLidarPointCloudFileVersion... (4.24-era plugin = "LidarPointCloudRuntime" built-in beta).
- Plan: dump native data after props; expect FLidarPointCloudOctree: bounds + node tree, each node has FByteBulkData or inline arrays of points. 4.24-era FLidarPointCloudPoint ≈ FVector Location (3×f32) + FColor (4×u8, BGRA) + packed flags ≈ 16–18 B/point. 448 MB ≈ ~26M points.
- Cross-check: UE 4.24 plugin source on GitHub (EpicGames/UnrealEngine, Engine/Plugins/Enterprise/LidarPointCloud) — also LIDAR plugin marketplace docs.
- Output: decimated binary chunks (e.g. Float32 XYZ + u8 RGB) for Three.js; full-res optional via spatial chunking; consider potree-style or simple grid chunks + density slider.

## Pipeline architecture

```
tools/uasset.py          core package parser (DONE)
tools/inspect.py         export/property dumper (DONE)
tools/extract_texture.py uasset -> PNG -> resized JPEG       [ ]
tools/extract_mesh.py    uasset -> positions/UV/tris -> GLB or .bin [ ]
tools/extract_lidar.py   uasset -> chunked point binaries     [ ]
tools/extract_blueprint.py GRID_DTM_COMBINED -> tile transforms JSON [ ]
tools/build_all.py       run everything -> web/data/          [ ]
web/index.html + app.js  Three.js viewer                      [ ]
web/data/                generated: tiles GLB/JPG, points/*.bin, manifest.json
```

### Viewer plan (web/)
- Three.js (pin version, e.g. 0.160 via CDN or vendored), OrbitControls + WASD fly mode.
- Terrain: 16 GLB tiles (or raw indexed BufferGeometry .bin + manifest), each textured with its JPEG ortho. KY State Plane feet → scene units; recenter to local origin (subtract centroid; UE coords were cm).
- Point cloud: THREE.Points with chunked binary attribute loads, point budget slider, size attenuation, classification/elevation/RGB color modes if available.
- UI: layer toggles (basemap / LiDAR), opacity, stats. Serve via `python -m http.server` from web/.

## Workflow run (2026-06-12)

Multi-agent workflow `campus-extract-render` launched: run ID `wf_d6589f13-21e`,
script at `C:\Users\sear234\.claude\projects\C--Users-sear234-Desktop-CAMPUS\f4fb5404-63be-426e-b5a4-8080e79647fa\workflows\scripts\campus-extract-render-wf_d6589f13-21e.js`.
Phases: Extract (4 decoders + viewer skeleton in parallel) → Integrate (merge
manifests → web/data/manifest.json) → Verify (data integrity / viewer code /
offline visual renders). Agents write reports to `extracted/REPORT-*.md` and
partial manifests to `extracted/manifest-*.json`. If interrupted, resume with
Workflow({scriptPath, resumeFromRunId: "wf_d6589f13-21e"}) — completed agents
are cached. Pillow 12.2 + numpy 2.4.6 installed. `.gitignore` created.

## Execution checklist

- [x] Core uasset parser + inspector
- [x] Texture format identified (PNG payload)
- [~] Mesh native region decode (brute-force the 142-byte tail; then RawMesh vs MeshDescription)
- [ ] LiDAR octree reverse-engineering (biggest unknown; check engine plugin source)
- [ ] extract_texture.py + run on 18 textures (needs Pillow: `pip install pillow`)
- [ ] extract_mesh.py + run on 16 meshes; sanity: tile XY extents ≈ 2640 ft × 2640 ft, contiguous neighbors
- [ ] extract_lidar.py + decimation/chunking
- [ ] Blueprint transforms (or derive placement from mesh names/bounds if Blueprint parse is hard)
- [ ] web viewer; verify: tiles seamless, textures not flipped (check V flip!), LiDAR registers onto basemap (same coordinate frame; LiDAR was colored FROM basemap so XY should match)
- [ ] .gitignore (exclude web/data, __pycache__, extracted intermediates; keep *.uasset out or LFS — they're 3.6 GB)
- [ ] README with how-to-run

## Notes / gotchas
- Python 3.13 available. Windows; prefer forward-slash paths in bash tool.
- UE units = cm; UE is left-handed Z-up; Three.js right-handed Y-up → transform: three(x, y, z) = (ue.x, ue.z, ue.y) (mirrors handedness, check winding/normals).
- Texture V coordinate likely needs flip (UE vs glTF/three UV origin).
- ExtendedBounds origin ≈ (-895m, 313m, -2455m in m if cm) — tiles are NOT at origin; mesh verts may be in world coords already (check first decoded tile, compare across two tiles to learn layout; Blueprint may add offsets).
- A `.git` repo already exists at CAMPUS root (only .uasset committed? check `git status` before committing).
- LiDAR OriginalCoordinates (double vector) = georeference offset — keep it in manifest.json for georegistration.
