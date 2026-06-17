#!/usr/bin/env python
"""Shared georeference + WGS84->scene projection for the Lextran transit tools.

Both the offline baker (`tools/lextran_gtfs.py`, which bakes route/stop geometry
into `web/data/transit.json`) and the live proxy (`tools/twin_server.py`, which projects
GTFS-Realtime vehicle positions on the fly) need the SAME map from longitude/
latitude to the viewer's scene metres. That map is the one the road pipeline
already uses and verified (see `tools/osm_roads.py`): the scene is georeferenced to
UTM zone 16N (EPSG:32616) with a rotation-free, unit-scale offset

    A = (lidar.original_coordinates[0] + origin_cm[0]) / 100
    B = -(lidar.original_coordinates[1] + origin_cm[1]) / 100
    easting  = A + sceneX        ->  sceneX = easting  - A
    northing = B - sceneZ        ->  sceneZ = B        - northing

Keeping this in one place guarantees buses, stops, and route lines land on exactly
the same streets the OSM ribbons do. (Verified: the campus bbox centre projects to
lon/lat -84.505, 38.030 = UK campus.)

Run tools as modules from the repo root (`python -m tools.x`) so `tools/inspect.py`
doesn't shadow the stdlib `inspect` module.
"""
import json
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.abspath(os.path.join(_HERE, "..", "web", "data"))


def load_manifest(data_dir=DATA):
    with open(os.path.join(data_dir, "manifest.json")) as f:
        return json.load(f)


def georef(manifest):
    """Return (A, B): the scene<->UTM-16N offsets documented above."""
    oc = manifest["lidar"]["original_coordinates"]
    o = manifest["origin_cm"]
    a = (oc[0] + o[0]) / 100.0
    b = -(oc[1] + o[1]) / 100.0
    return a, b


class Projector:
    """Callable lon/lat (WGS84 degrees) -> (sceneX, sceneZ) in scene metres.

    Identical to `to_scene` in tools/osm_roads.py. `y` (elevation) is intentionally
    NOT handled here: the baker drapes it from the terrain heightmap offline, and
    the live viewer drapes it per-frame with a downward raycast, so the projector
    stays light enough to construct per-request in the proxy.
    """

    def __init__(self, manifest=None, data_dir=DATA):
        from pyproj import Transformer  # heavy import; only when a projector is built

        manifest = manifest if manifest is not None else load_manifest(data_dir)
        self.A, self.B = georef(manifest)
        self._to_utm = Transformer.from_crs(4326, 32616, always_xy=True)

    def __call__(self, lon, lat):
        easting, northing = self._to_utm.transform(lon, lat)
        return (easting - self.A, self.B - northing)

    # explicit alias for readability at call sites
    def to_scene(self, lon, lat):
        return self(lon, lat)
