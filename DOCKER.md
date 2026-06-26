# Running the server in Docker

One command brings up the digital-twin server and, on first start, downloads and builds
the entire city (buildings, roads, traffic lights, intersections, crosswalks, cameras,
buses + the ~8 GB KYAPED LiDAR) before serving:

```bash
docker compose up -d        # build image + start; first run does the full city build
docker compose logs -f      # watch the build stream (it can take a while)
```

Then open `http://localhost:8000/` (or `http://<lan-ip>:8000/` from another device).
API docs for spawning/controlling agents are at `/docs`.

## Prerequisites

- **Docker + Docker Compose.**
- **The UE assets must be present in the repo** as `MESHES/` and `LIDAR/`. They provide the
  scene↔UTM georef anchor that every other layer is built against and there is no public
  download for them, so they are bind-mounted from the repo rather than baked into the image.
  Without them the first-run build can't start (you'll get a flat/empty world).
- **`.env` is optional.** Add `GOOGLE_MAPS_API_KEY` there only if you want the photorealistic
  3D-tiles basemap; the twin runs fine without it.

## How it works

- `restart: unless-stopped` keeps the server running across crashes and reboots.
- `--bootstrap` runs `build_all.py --citywide` automatically when the world is missing, then
  serves. It's **idempotent**: once `web/data/{manifest.json,ground.f32,buildings.pack.json}`
  exist, restarts skip straight to serving.
- On every start the server also tops up the light "live-data" bakes if they're missing —
  `cameras.json` (traffic-camera positions, scraped from the city map) and `transit.json`
  (Lextran routes/stops) — so the camera/transit marker layers fill in even on a box that
  already had the heavy world built. The live `/api/cameras/*` and `/api/transit/*` proxies
  are separate and always on. Reload the viewer once a top-up finishes to see the markers.
- The repo is bind-mounted at `/app`, so the generated `web/data/` and the `extracted/` LiDAR
  cache live on the host and persist between runs. If the first build is interrupted, just
  `docker compose up -d` again — downloads are cached and it resumes.

## Notes

- **First-person cameras (`--render`)** drive a headless Chromium. The compose file sets
  `shm_size: 1gb` and, via `TWIN_CHROMIUM_ARGS`, runs Chromium's WebGL on software SwiftShader
  (`--no-sandbox --use-angle=swiftshader --enable-unsafe-swiftshader`) since the container has no
  GPU. Software rendering of the city POV is slow, so `TWIN_RENDER_TIMEOUT=30` widens the per-frame
  budget and the render service warms up the GL pipeline at startup so the first frame isn't stuck
  compiling shaders. Expect a few FPS, not real-time. Drop `--render` from `command:` if unused.
- **In-process YOLO detection** (`POST /api/cameras/detect`) needs `ultralytics`+`opencv`; the
  image installs them (with the **CPU** PyTorch wheel, since the container has no GPU), so the
  startup banner shows `detect: ready`. CPU inference is slow — fine for occasional per-camera
  runs, not real-time on every feed. To slim the image back down, drop that `RUN pip install
  torch … ultralytics …` layer from the Dockerfile.
- Files written to `web/data/` and `extracted/` are owned by root (the container user). Add a
  `user: "1000:1000"` line to the service if you'd rather they match your host account.
