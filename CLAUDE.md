# CLAUDE.md

Guidance for Claude Code (and other agents) working in this repository.

## Git workflow

- **Never push directly to `main`.** `main` is protected — all changes go through a
  pull request. Work on a feature branch and open a PR with `gh pr create`.
- Only commit or push when explicitly asked.

## Project

Interactive 3D digital twin of Lexington, KY — a Three.js web viewer (`web/`) over
open-format data extracted by Python tools (`tools/`). No Unreal Engine required at
runtime.

- Generated/large data lives under the **gitignored** `web/data/` (and raw LiDAR under
  `extracted/`); it is regenerated from the `tools/` pipeline, not committed.
- The campus core is extracted from UE 4.24.3 assets (`build_all.py`); the rest of the
  city is built from open data only — KyFromAbove/KYAPED LiDAR + OpenStreetMap. See the
  README "Filling in Lexington" and "Full-city buildings + roads" sections for the
  reproduce-from-clone pipeline (`ky_lidar` → `osm_city` → `build_city` →
  `pack_buildings` → `osm_roads` → `smooth_roads`).
- Serve everything with `python -m tools.twin_server` (viewer + shared-world API + live
  Lextran buses on `:8000`).

## Conventions

- Run tools as modules from the repo root (`python -m tools.<name>`), not as loose files
  — several rely on the `tools` package import and on dropping `tools/` from `sys.path`
  so `tools/inspect.py` doesn't shadow the stdlib `inspect`.
- Keep the viewer to one packed buffer / draw call per layer; it's how the full city
  (~114k buildings) stays at 60 fps.
