# UKy Campus / Lexington Digital Twin — Constitution

Core principles governing the interactive 3D digital twin of Lexington, KY (a Three.js
viewer over open-format data extracted by Python tools). These are binding on every
feature; the operational companion is `CLAUDE.md`.

**Version**: 1.0.0 · **Ratified**: 2026-06-18 · **Last amended**: 2026-06-18

## Principle 1 — Reproducible from clone; never commit generated data

Generated and large artifacts (everything under `web/data/`, raw LiDAR under
`extracted/`) are gitignored and regenerated from the `tools/` pipeline, not committed.
A feature is "done" only when a fresh clone can reproduce its data with a documented
command (`python -m tools.<name>`). Hand-authored configuration that is NOT pipeline
output (e.g. camera calibration) lives OUTSIDE `web/data/` so it can be version
controlled.

*Rationale*: the twin is large (~114k buildings, ~25M LiDAR points); the repo stays
small and the pipeline stays the source of truth.

## Principle 2 — One packed buffer / one draw call per layer

Each visual layer renders from a single merged buffer (or a small fixed number of
instanced meshes), never per-object meshes at city scale. This is how the full city
holds 60 fps. New layers MUST NOT introduce per-frame raycasts over large meshes or
per-object draw calls that scale with data size.

*Rationale*: performance is a feature; the city has tens of thousands of elements.

## Principle 3 — Mirror the established live-layer pattern

A new live data layer follows the transit/traffic-camera template: a STATIC half baked
offline by a `tools/` module into a JSON contract, a LIVE half proxied at runtime by
`tools/twin_server.py` (`/api/<layer>/*`), a `web/<layer>.js` viewer module exposing a
`window.__twin.<layer>` query API, a sidebar fieldset, and reuse of the shared
projector + heightmap/flat-ground draping. Do not invent a parallel architecture when
the pattern fits.

*Rationale*: consistency makes the codebase learnable and each layer testable in
isolation.

## Principle 4 — Graceful degradation; never throw into the render loop

Every layer must render *something* with missing data and a missing proxy (markers
without a feed, last-good payload on upstream failure), and must never raise into the
animation loop or 500 the viewer. Proxies serve stale-flagged data on failure and probe
slowly when absent (no 404 spam).

*Rationale*: the viewer runs against partial data constantly (streaming tiles, an
offline proxy); it must stay usable.

## Principle 5 — One georeference, one ground model

All lon/lat → scene conversion goes through `tools/transit_common.Projector` (UTM-16N,
the documented offset); never reinvent the projection. Scene elevation obeys the active
ground model (real terrain heightmap, or flat-world `FLAT_Y` when enabled) consistently
across every layer, so nothing floats or clips.

*Rationale*: buses, stops, roads, cameras, and detected cars must land on the same
streets; divergent math is the root cause of drift and clipping.

## Principle 6 — Verify before merge; PRs only to `main`

`main` is protected. Every change lands on a feature branch via a PR. Substantive
changes are verified before merge — unit-tested math, integration-tested APIs, and
headless-browser checks of viewer behavior (not just "it compiles"). Findings and the
test evidence go in the PR.

*Rationale*: the twin is a system of interacting layers; regressions hide in geometry
and timing that only run-time observation catches.

## Principle 7 — Least privilege and same-origin by default

User-controllable inputs are sanitized at their source (e.g. the `?data=` override is
constrained to a same-origin relative path). The viewer fetches same-origin; proxies
are read-only and rate-limit themselves against upstreams. Third-party data is used
within its terms.

*Rationale*: the viewer is a public web surface fetching from city/agency endpoints.

## Governance

This constitution supersedes ad-hoc preferences. Amendments are made by PR that updates
this file and bumps the version (semantic: MAJOR for a removed/redefined principle,
MINOR for a new principle or section, PATCH for clarifications). `CLAUDE.md` carries the
day-to-day operational rules (git workflow, how to run tools) and must stay consistent
with these principles. Specs live under `specs/NNN-name/`; each non-trivial feature gets
a `spec.md` (and `plan.md` / `tasks.md` as needed) before implementation.
