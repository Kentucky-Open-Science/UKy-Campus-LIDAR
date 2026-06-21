#!/usr/bin/env python3
"""Download Google Photorealistic 3D Tiles for the Lexington bbox and save them locally so
the viewer renders from disk (no Google calls on page load / camera moves).

IMPORTANT — terms of use: Google Maps Platform's STANDARD terms do not permit a permanent
offline copy. Run this ONLY if your agreement with Google allows storing the tiles (e.g. a
private contract scoped to a specific area). It is bbox-limited to Lexington by default and
intended for that licensed, private use.

What it does: walks the 3D-Tiles tree from root.json, prunes to the (padded) Lexington
bbox, descends to the requested fidelity, downloads each tile (glTF/glb + nested tilesets),
and writes a self-contained local tileset to web/data/photoreal_lexington/ with every URI
rewritten to a flat local filename. The viewer auto-loads it (see tiles3d.js / twin_server
/api/photoreal `localTileset`).

    python -m tools.download_photoreal                  # default fidelity (min-error 12)
    python -m tools.download_photoreal --min-error 0    # MAX fidelity (large, slow)
    python -m tools.download_photoreal --min-error 40 --max-tiles 60   # quick sample

Key from .env / GOOGLE_MAPS_API_KEY. Re-running resumes (already-saved tiles are skipped).
"""
import argparse
import hashlib
import json
import os
import sys
import time
import urllib.request
from urllib.parse import urljoin, urlparse, parse_qs

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]

from pyproj import Transformer

REPO = os.path.normpath(os.path.join(_HERE, ".."))
OUT_DEFAULT = os.path.join(REPO, "web", "data", "photoreal_lexington")
TILES_HOST = "tile.googleapis.com"
_TO_LL = Transformer.from_crs(4978, 4326, always_xy=True)   # ECEF -> lon/lat


def load_key():
    k = os.environ.get("GOOGLE_MAPS_API_KEY") or os.environ.get("PHOTOREAL_KEY")
    if not k:
        try:
            for line in open(os.path.join(REPO, ".env"), encoding="utf-8"):
                if line.startswith("GOOGLE_MAPS_API_KEY="):
                    k = line.split("=", 1)[1].strip()
        except FileNotFoundError:
            pass
    return k


_S = (-1.0, -0.5, 0.0, 0.5, 1.0)   # 5x5x5 volume samples


def box_lonlat_bbox(box):
    """OBB [cx,cy,cz, ux,uy,uz, vx,vy,vz, wx,wy,wz] (ECEF) -> (minlon,minlat,maxlon,maxlat).
    Samples the box VOLUME (not just the 8 corners): an earth-scale OBB reaches its
    pole-ward latitude extreme at a FACE centre, so corners-only badly under-covers."""
    c, u, v, w = box[0:3], box[3:6], box[6:9], box[9:12]
    lons, lats = [], []
    for su in _S:
        for sv in _S:
            for sw in _S:
                x = c[0] + su * u[0] + sv * v[0] + sw * w[0]
                y = c[1] + su * u[1] + sv * v[1] + sw * w[1]
                z = c[2] + su * u[2] + sv * v[2] + sw * w[2]
                lon, lat, _ = _TO_LL.transform(x, y, z)
                lons.append(lon); lats.append(lat)
    return min(lons), min(lats), max(lons), max(lats)


def region_lonlat_bbox(region):
    """3D-Tiles `region` [west,south,east,north,minH,maxH] in radians -> lon/lat degrees."""
    import math
    return (math.degrees(region[0]), math.degrees(region[1]),
            math.degrees(region[2]), math.degrees(region[3]))


def tile_lonlat_bbox(bv):
    if "box" in bv:
        return box_lonlat_bbox(bv["box"])
    if "region" in bv:
        return region_lonlat_bbox(bv["region"])
    return None


def overlaps(a, b):
    return not (a[2] < b[0] or a[0] > b[2] or a[3] < b[1] or a[1] > b[3])


class Downloader:
    def __init__(self, key, out, bbox, min_error, max_tiles):
        self.key, self.out, self.bbox = key, out, bbox
        self.min_error, self.max_tiles = min_error, max_tiles
        self.session = None
        self.seen = {}            # remote url -> local filename
        self.n_tiles = self.n_bytes = 0
        os.makedirs(out, exist_ok=True)

    def _auth(self, url):
        p = urlparse(url)
        q = parse_qs(p.query)
        if "key" not in q:
            url += ("&" if p.query else "?") + "key=" + self.key
        if self.session and "session" not in q:
            url += "&session=" + self.session
        return url

    def _fetch(self, url):
        req = urllib.request.Request(self._auth(url), headers={"User-Agent": "uky-twin/1.0"})
        with urllib.request.urlopen(req, timeout=60) as r:
            return r.read()

    def _write(self, fname, data):
        """Atomic write (temp file + os.replace). The resume check trusts that a file on
        disk is COMPLETE, so a download must never leave a half-written file behind if the
        process is killed mid-write — temp+rename guarantees the final name only ever points
        at fully-written bytes."""
        path = os.path.join(self.out, fname)
        tmp = path + ".tmp"
        with open(tmp, "wb") as f:
            f.write(data)
        os.replace(tmp, path)

    def fetch_and_save(self, url):
        """Download a tile content URL; save glb or (recursively) a nested tileset. Returns
        the flat local filename to reference, or None if over the tile budget.

        Resume: filenames are a deterministic hash of the tile URL, so a previously saved
        tile is reused without re-fetching. A present file always means a COMPLETE prior
        download — leaves are written atomically, and a nested tileset's .json is written
        only AFTER its whole subtree finished (and with its child URIs already rewritten to
        local names), so an existing .json needs no re-walk."""
        if url in self.seen:
            return self.seen[url]
        h = hashlib.sha1(urlparse(url).path.encode()).hexdigest()[:16]
        glb, js = h + ".glb", h + ".json"
        if os.path.exists(os.path.join(self.out, glb)):     # resume: leaf already downloaded
            self.seen[url] = glb
            return glb
        if os.path.exists(os.path.join(self.out, js)):      # resume: subtree already complete
            self.seen[url] = js
            return js
        if self.max_tiles and self.n_tiles >= self.max_tiles:
            return None
        data = self._fetch(url)
        if data[:4] == b"glTF":                     # binary glTF leaf content
            self._write(glb, data)
            self.seen[url] = glb
            self.n_tiles += 1; self.n_bytes += len(data)
            if self.n_tiles % 25 == 0:
                print("  %d tiles, %.1f MB" % (self.n_tiles, self.n_bytes / 1e6))
            return glb
        sub = json.loads(data)                       # nested external tileset
        self.seen[url] = js
        self.walk(sub["root"], url, 0)               # downloads subtree + rewrites URIs local
        self._write(js, json.dumps(sub).encode("utf-8"))   # written last = subtree complete
        return js

    def walk(self, tile, base_url, depth):
        """Prune+rewrite a tile subtree in place. Returns True to keep, False to drop."""
        bb = tile_lonlat_bbox(tile.get("boundingVolume", {}))
        if bb and depth > 0 and not overlaps(bb, self.bbox):
            return False                              # outside Lexington -> drop subtree
        c = tile.get("content")
        if c and c.get("uri"):
            local = self.fetch_and_save(self._resolve(base_url, c["uri"]))
            if local:
                tile["content"] = {"uri": local}
            else:
                tile.pop("content", None)
        kids = []
        # descend while this tile is still coarser than the requested fidelity
        if tile.get("geometricError", 1e30) > self.min_error and \
                not (self.max_tiles and self.n_tiles >= self.max_tiles):
            for ch in tile.get("children", []):
                if self.walk(ch, base_url, depth + 1):
                    kids.append(ch)
        tile["children"] = kids
        return bool(tile.get("content") or kids)

    @staticmethod
    def _resolve(base_url, uri):
        return uri if uri.startswith("http") else urljoin(base_url, uri)

    def run(self):
        root_url = "https://%s/v1/3dtiles/root.json" % TILES_HOST
        root = json.loads(self._fetch(root_url))
        # session token is embedded in the root's child URIs
        blob = json.dumps(root)
        i = blob.find("session=")
        self.session = blob[i + 8:].split('"')[0].split("&")[0] if i >= 0 else None
        print("session:", (self.session or "?")[:14], "| bbox:", self.bbox,
              "| min_error:", self.min_error)
        self.walk(root["root"], root_url, 0)
        with open(os.path.join(self.out, "tileset.json"), "w", encoding="utf-8") as f:
            json.dump(root, f)
        return self.n_tiles, self.n_bytes


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=OUT_DEFAULT)
    ap.add_argument("--min-error", type=float, default=12.0,
                    help="stop descending when a tile's geometricError <= this (lower = "
                         "higher fidelity, more data). 0 = maximum.")
    ap.add_argument("--max-tiles", type=int, default=0, help="cap tile count (0 = no cap)")
    ap.add_argument("--pad", type=float, default=0.05, help="bbox padding fraction")
    ap.add_argument("--bbox", type=float, nargs=4, metavar=("MINLON", "MINLAT", "MAXLON", "MAXLAT"),
                    help="override Lexington bbox (defaults to city.json)")
    args = ap.parse_args()

    key = load_key()
    if not key:
        print("FAIL: set GOOGLE_MAPS_API_KEY (in .env)"); return 2

    if args.bbox:
        bb = tuple(args.bbox)
    else:
        with open(os.path.join(REPO, "web", "data", "city.json")) as f:
            bb = tuple(json.load(f)["bbox_lonlat"])
    dlon = (bb[2] - bb[0]) * args.pad
    dlat = (bb[3] - bb[1]) * args.pad
    bbox = (bb[0] - dlon, bb[1] - dlat, bb[2] + dlon, bb[3] + dlat)

    t0 = time.time()
    n, nb = Downloader(key, args.out, bbox, args.min_error, args.max_tiles).run()
    # small manifest the viewer/tools can read
    with open(os.path.join(args.out, "_manifest.json"), "w") as f:
        json.dump({"tiles": n, "bytes": nb, "bbox": bbox, "min_error": args.min_error}, f)
    print("DONE: %d tiles, %.1f MB in %.0fs -> %s" % (n, nb / 1e6, time.time() - t0, args.out))


if __name__ == "__main__":
    main()
