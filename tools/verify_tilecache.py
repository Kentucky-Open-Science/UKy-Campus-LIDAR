#!/usr/bin/env python3
"""Verify the photorealistic-tile disk cache proxy (twin_server /api/gtile).

Proves that a tile, once fetched, is served from disk on the next request (so repeated
local dev sessions don't re-download the same tiles). Uses the real Google API for ONE
root.json + ONE tile, then hits the proxy twice and checks miss -> hit with identical
bytes. Requires GOOGLE_MAPS_API_KEY (read from .env or the environment).

    python -m tools.verify_tilecache
"""
import os
import re
import shutil
import socket
import subprocess
import sys
import time
import urllib.request
from urllib.parse import quote, urlparse, parse_qs

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]

REPO = os.path.normpath(os.path.join(_HERE, ".."))
TILECACHE = os.path.join(REPO, "web", "data", "tilecache")
PORT = 8151


def load_dotenv():
    try:
        with open(os.path.join(REPO, ".env"), encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))
    except FileNotFoundError:
        pass


def wait_port(p, t=40):
    for _ in range(t * 5):
        try:
            socket.create_connection(("127.0.0.1", p), 0.3).close()
            return True
        except OSError:
            time.sleep(0.2)
    return False


def main():
    load_dotenv()
    key = os.environ.get("GOOGLE_MAPS_API_KEY") or os.environ.get("PHOTOREAL_KEY")
    if not key:
        print("FAIL: set GOOGLE_MAPS_API_KEY (e.g. in .env)")
        return 2

    # 1) get a real leaf tile URL from Google's root tileset (server-side, one call).
    root_url = f"https://tile.googleapis.com/v1/3dtiles/root.json?key={key}"
    root = urllib.request.urlopen(root_url, timeout=20).read().decode("utf-8", "replace")
    # find a child content uri that is NOT the root tileset
    uris = re.findall(r'"uri"\s*:\s*"([^"]+)"', root)
    child = next((u for u in uris if "root.json" not in u), None)
    if not child:
        print("FAIL: no child uri found in root.json")
        return 1
    if child.startswith("http"):
        full = child
    else:
        full = "https://tile.googleapis.com" + (child if child.startswith("/") else "/" + child)
    full += ("&" if "?" in full else "?") + "key=" + key
    print("tile path:", urlparse(full).path[:80], "...")

    # 2) start twin_server (minimal) so /api/gtile is live
    shutil.rmtree(TILECACHE, ignore_errors=True)
    env = dict(os.environ)
    srv = subprocess.Popen(
        [sys.executable, "-m", "tools.twin_server", "--port", str(PORT),
         "--no-transit", "--no-cameras"],
        cwd=REPO, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        if not wait_port(PORT):
            print("FAIL: twin_server did not start")
            return 1
        time.sleep(1.0)
        proxy = f"http://127.0.0.1:{PORT}/api/gtile?u=" + quote(full, safe="")

        def hit():
            r = urllib.request.urlopen(proxy, timeout=30)
            return r.headers.get("X-Tile-Cache"), r.read()

        c1, b1 = hit()
        c2, b2 = hit()
        ncache = len([f for f in os.listdir(TILECACHE) if not f.endswith(".ct")]) if os.path.isdir(TILECACHE) else 0
        print(f"request 1: X-Tile-Cache={c1}  bytes={len(b1)}")
        print(f"request 2: X-Tile-Cache={c2}  bytes={len(b2)}")
        print(f"cache files on disk: {ncache}")

        ok = (c1 == "miss" and c2 == "hit" and b1 == b2 and len(b1) > 0 and ncache >= 1)

        # SSRF guard: a non-Google host AND a Google host with a non-tile path must both be
        # refused (the proxy rebuilds the URL from a hardcoded host + /v1/3dtiles/ path).
        def rejected(target):
            url = f"http://127.0.0.1:{PORT}/api/gtile?u=" + quote(target, safe="")
            try:
                urllib.request.urlopen(url, timeout=10)
                return False
            except urllib.error.HTTPError as e:
                return e.code == 403
            except Exception:
                return False
        ssrf_host = rejected("https://example.com/x")
        ssrf_userinfo = rejected("https://tile.googleapis.com@example.com/v1/3dtiles/x")
        ssrf_path = rejected("https://tile.googleapis.com/../evil")
        ssrf_query = rejected("https://tile.googleapis.com/v1/3dtiles/x?u=http://evil.com/")
        print("SSRF guard 403s -> off-host:", ssrf_host, "| userinfo@evil:", ssrf_userinfo,
              "| bad-path:", ssrf_path, "| query-inject:", ssrf_query)
        ok = ok and ssrf_host and ssrf_userinfo and ssrf_path and ssrf_query

        # Browser pass: the real viewer must load tiles THROUGH the proxy and render them,
        # growing the on-disk cache (proves the tiles3d.js proxy plugin is wired).
        browser_ok = True
        try:
            from playwright.sync_api import sync_playwright
            before = len([f for f in os.listdir(TILECACHE) if not f.endswith(".ct")])
            with sync_playwright() as pw:
                b = pw.chromium.launch(args=["--use-gl=angle", "--enable-unsafe-swiftshader"])
                pg = b.new_page(viewport={"width": 1100, "height": 700})
                pg.goto(f"http://127.0.0.1:{PORT}/?flat=0", wait_until="load")
                time.sleep(2.0)
                pg.evaluate("() => window.__viewer.photoreal.setVisible(true)")
                models = 0
                for _ in range(80):
                    models = pg.evaluate("""() => { const ph = window.__viewer.state.photoreal;
                      if (!ph.tiles) return 0; let n = 0; ph.tiles.forEachLoadedModel(() => n++); return n; }""")
                    if models > 0:
                        break
                    time.sleep(0.5)
                b.close()
            after = len([f for f in os.listdir(TILECACHE) if not f.endswith(".ct")])
            print(f"browser: rendered {models} tile models; cache files {before} -> {after}")
            browser_ok = models > 0 and after > before
            if not browser_ok:
                print("  (browser did not render via the proxy / cache did not grow)")
        except ImportError:
            print("browser pass skipped (playwright not installed)")
        ok = ok and browser_ok
    finally:
        srv.terminate()

    if ok:
        print("\nRESULT: PASS — tiles cached to disk; 2nd request served from cache; SSRF blocked")
        return 0
    print("\nRESULT: FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
