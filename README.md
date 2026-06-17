# UKy Campus — Interactive 3D Viewer

Extract UE 4.24.3 UKy Campus LiDAR point cloud + DTM terrain tiles + aerial
imagery into open formats and view them in an interactive Three.js web viewer
— **no Unreal Engine required**.

## Quick start

```sh
python -m tools.serve          # static files + LIVE Lextran bus feed, on :8000
# Open http://localhost:8000/
```

`tools/serve.py` is a drop-in for `python -m http.server` that also proxies the
live Lexington (Lextran) GTFS-Realtime feed so you get **moving buses** on the map.
No buses needed? Plain `cd web && python -m http.server 8000` still works (routes +
stops render; live buses just stay off). Add `python -m tools.serve --mock` to
replay a recorded feed offline.

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
│   ├── osm_roads.py          # OpenStreetMap → campus road network (roads.json)
│   ├── osm_city.py           # OpenStreetMap → city-wide streets + ground plane (city.json)
│   ├── lextran_gtfs.py       # Lextran static GTFS → routes + stops (transit.json)
│   ├── serve.py              # static server + live GTFS-Realtime proxy (/api/transit/*)
│   ├── twin_server.py        # authoritative shared-world server + agent API (/api/world/*)
│   ├── pack_buildings.py     # merge 3,109 building meshes → one buffer (fast load)
│   ├── transit_common.py     # shared lon/lat → scene projection (georef)
│   ├── extract_roads.py      # aerial-texture road detector (alt. road source)
│   ├── fetch_lfucg_signals.py # download LFUCG real traffic-signal locations
│   ├── smooth_roads.py       # smooth roads + signalise junctions → signals.json
│   ├── ground_buildings.py   # drop floating buildings onto the terrain (keep bridges)
│   ├── build_all.py          # Full pipeline orchestrator
│   ├── verify_viewer.py      # Headless viewer test (requires playwright)
│   └── verify_agents.py      # Headless agent-API sensor test (requires playwright)
├── web/                      # Three.js viewer (static)
│   ├── index.html
│   ├── app.js
│   ├── roads.js              # road ribbons + signals/crosswalks + live signal controller
│   ├── city.js               # city-wide OSM ground plane + streets (city.json)
│   ├── transit.js            # live Lextran buses + routes/stops/arrivals/alerts
│   ├── agents.js             # autonomous agents (car/truck/robot/drone) + sensors (local)
│   ├── netagents.js          # renders the twin_server shared world (agents from any client)
│   ├── style.css
│   ├── lib/                  # Vendored Three.js 0.160 + OrbitControls
│   └── data/                 # Generated extraction output
│       ├── manifest.json     # Unified scene manifest
│       ├── meshes/*.bin      # 16 terrain tiles (verts, UVs, indices)
│       ├── textures/*.jpg    # 18 aerial imagery textures
│       ├── lidar/chunk_*.bin # 64 decimated point-cloud chunks
│       ├── buildings/*.bin   # per-building meshes (legacy / fallback)
│       ├── buildings.pack.bin + .json  # all buildings in ONE buffer (fast load)
│       ├── roads.json        # smoothed road centrelines + real intersections
│       ├── signals.json      # machine-readable signal model (autonomous agents)
│       ├── city.json         # city-wide OSM streets + ground plane
│       └── transit.json      # Lextran routes + stops (live buses via tools/serve.py)
├── client/                  # twin.py — dependency-free Python client for the world API
├── examples/                # drone_demo.py — spawn/fly/collide via the twin server
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
- 449 road centrelines (~70 km), smoothed + draped on terrain; 266 real
  intersections, **51 signalised** (gated against the **real LFUCG traffic-signal
  layer**), 112 stop-controlled, 103 uncontrolled — each with traffic + pedestrian
  signals, crosswalks, and stop bars (see `web/README.md`). Regenerate with
  `python -m tools.smooth_roads`.
- All 3,109 buildings dropped onto the terrain (no floaters; 4 road-spanning bridges
  left elevated) — `python -m tools.ground_buildings`.
- Viewport: ~1.8 km x 3.4 km area centered on UKy Lexington campus
- 3,109 buildings packed into ONE buffer (`buildings.pack.bin`, ~3.9 MB): the
  viewer makes 1 request + 1 draw call instead of ~3,100 — all buildings ready in
  ~2 s. Regenerate with `python -m tools.pack_buildings`.
- City context: full Lextran service area (~18 x 16 km) of OpenStreetMap streets
  (8,481 ways) on a flat ground plane, so the whole bus network has streets + ground
  beyond the campus tiles. Regenerate with `python -m tools.osm_city`.
- Transit: 27 Lextran routes + 878 stops (`transit.json`), plus live buses /
  arrivals / alerts via the `tools/serve.py` proxy. Regenerate with
  `python -m tools.lextran_gtfs`.

## Digital twin — controllable signals + autonomous agents

The road network is built for autonomous-agent simulation: `tools/smooth_roads.py`
emits `web/data/signals.json`, a machine-readable model of every intersection
(approach legs, stop-line coordinates, crosswalk polygons, signal phase groups, and
a fixed-time phase plan). The viewer ticks a deterministic signal state machine and
exposes it at `window.__twin.signals`, so an agent can ask "what is my light right
now and where do I stop" (`getLegState` / `queryByPosition`) and even drive the
lights (`setOverride`).

On top of that, `web/agents.js` adds **controllable agents** — spawn a car, truck,
robot, or drone at `window.__twin.agents`, drive it from your own code, and read
back a POV **camera**, live **position** (scene m / UE cm / UTM-16N), object
**collision detection** (you program the avoidance), and a **ground/surface** probe
that keeps ground vehicles on the road or terrain and tells you which one they're
on. Full API + schemas for both in [web/README.md](web/README.md); smoke-test the
agent sensors with `python tools/verify_agents.py`.

The twin also carries the **real Lexington bus network**. `tools/serve.py` proxies
the live Lextran GTFS-Realtime feed and the viewer animates the actual buses on the
map — with route lines, stops, predicted arrivals, and service alerts — all reachable
at `window.__twin.transit` (`getVehicles` / `getNearestVehicle` / `getArrivals` / …).
Buses are wired into the agent sensor bus too (`sensors.transit`), so an autonomous
agent can yield to or wait for a real campus bus. Because the network spans far past
the campus tiles, `tools/osm_city.py` lays down the rest of Lexington (OSM streets +
a ground plane) so every route has ground beneath it. Smoke-test the live layer with
`python tools/verify_transit.py`.

## Multiplayer twin server — shared world over an API

`window.__twin.agents` is private to one browser tab. For a **shared** world — where
the twin runs on its own server and many scripts/users drive agents that everyone
sees — run the authoritative server:

```sh
python -m tools.twin_server        # serves the viewer + the world API on :8000
```

Run it **as a module from the repo root** (`python -m tools.twin_server`), not
`python tools/twin_server.py` — it imports the `tools` package, and running it as a
loose file breaks that import. It serves the viewer *and* the API on the same port.

> **`404 File not found` on `/api/world/...`?** Something else is answering on `:8000`
> — usually a plain `python -m http.server` or `tools/serve.py` (the transit/bus
> server) left running. Those serve the viewer's files but have no world API. Stop it
> and run `tools/twin_server` instead. They're different servers: `twin_server` has the
> **world** API (`/api/world/*`), `serve.py` has the **transit** proxy (`/api/transit/*`);
> they can't share a port. To run both, put the twin on another port:
> `python -m tools.twin_server --port 8001` and point clients/browser at `:8001`.

It holds every agent in one place, ticks the physics (the same ackermann / differential
/ holonomic-drone kinematics as `agents.js`, with ground from the terrain heightmap and
collisions from the baked building AABBs), and exposes a small REST API. Agents spawned
by ANY client are visible to ALL of them — other scripts and any browser open on the
server (`web/netagents.js` renders the shared world in the **Shared world (server)**
panel section).

Drive it from Python with the dependency-free client (`client/twin.py`):

```python
from twin import Twin
twin  = Twin("http://twin-host:8000", owner="alice")
drone = twin.spawn("drone", position=[0, None, 0])
drone.set_controls(move=[5, 1, 0])          # fly +X and climb
print(drone.state()["position"], drone.collisions())
for other in twin.agents():                 # every agent in the shared world
    print(other["owner"], other["type"], other["position"])
drone.stop(); drone.despawn()
```

The REST surface (all JSON, CORS-open): `GET /api/world/state` (everyone's agents),
`GET /api/world/agents/<id>`, `POST /api/world/spawn`, `POST /api/world/agents/<id>/{controls,driveTo,stop}`,
`DELETE /api/world/agents/<id>`, plus `GET /api/world/{meta,nearest_building}`.

A runnable example — spawn a drone, fly a circuit, fly into a building until the
collision sensor fires, then stop — is in `examples/drone_demo.py`:

```sh
python -m tools.twin_server &                 # the twin
python examples/drone_demo.py --url http://localhost:8000   # a client script
# open http://localhost:8000/ in a browser to watch it in 3-D
```