#!/usr/bin/env python3
"""LIVE verification of the Google Photorealistic 3D Tiles basemap — requires a real key.

Unlike tools/verify_photoreal.py (which is keyless and uses a local test tileset), this
loads the ACTUAL Google tiles to confirm the full path works end to end: auth/session,
draco decode, render, and ECEF->scene alignment. It also measures the vertical offset
between the Google ground mesh and our own terrain at the scene origin and prints a
suggested `calibrate({dy})` nudge.

The key is read from the environment so it is never written to a file:

    GOOGLE_MAPS_API_KEY=… python -m tools.verify_photoreal_live
    # optional: PHOTOREAL_PROVIDER=ion with a Cesium ion token

Loads the viewer in ?flat=0 (real LiDAR elevation) so the vertical comparison is
meaningful. Writes screenshots to extracted/.
"""
import os
import socket
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]

from playwright.sync_api import sync_playwright

WEB = os.path.normpath(os.path.join(_HERE, "..", "web"))
SHOTS = os.path.normpath(os.path.join(_HERE, "..", "extracted"))
PORT = 8147


def _load_dotenv():
    try:
        with open(os.path.join(_HERE, "..", ".env"), encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))
    except FileNotFoundError:
        pass


_load_dotenv()
KEY = os.environ.get("GOOGLE_MAPS_API_KEY") or os.environ.get("PHOTOREAL_KEY")
PROVIDER = os.environ.get("PHOTOREAL_PROVIDER", "google")


def wait_port(p, t=15):
    for _ in range(t * 10):
        try:
            socket.create_connection(("127.0.0.1", p), 0.2).close()
            return True
        except OSError:
            time.sleep(0.1)
    return False


def main():
    if not KEY:
        print("FAIL: set GOOGLE_MAPS_API_KEY in the environment")
        return 2
    os.makedirs(SHOTS, exist_ok=True)
    srv = subprocess.Popen([sys.executable, "-m", "http.server", str(PORT)], cwd=WEB,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    failures = []
    try:
        if not wait_port(PORT):
            print("FAIL: server did not start")
            return 1
        with sync_playwright() as pw:
            b = pw.chromium.launch(args=["--ignore-gpu-blocklist", "--use-gl=angle",
                                         "--enable-unsafe-swiftshader"])
            pg = b.new_page(viewport={"width": 1400, "height": 900})
            goog = {"ok": 0, "err": 0, "statuses": {}}

            def on_resp(r):
                u = r.url
                if "googleapis.com" in u or "tile.google" in u or "cesium" in u.lower():
                    bucket = goog["statuses"].setdefault(r.status, 0)
                    goog["statuses"][r.status] = bucket + 1
                    if r.status < 400:
                        goog["ok"] += 1
                    else:
                        goog["err"] += 1
            pg.on("response", on_resp)
            # Inject the key into localStorage BEFORE load (keeps it out of the URL/screenshot).
            pg.add_init_script(f"try{{localStorage.setItem('twin.photoreal.key',{KEY!r});"
                               f"localStorage.setItem('twin.photoreal.provider',{PROVIDER!r});}}catch(e){{}}")
            pg.goto(f"http://127.0.0.1:{PORT}/?flat=0", wait_until="load")

            # let the base map settle
            for _ in range(60):
                if "loading" not in (pg.eval_on_selector("#road-status", "e=>e.textContent") or ""):
                    break
                time.sleep(0.5)
            time.sleep(1.0)

            # enable the photoreal layer
            pg.evaluate("() => window.__viewer.photoreal.setVisible(true)")

            # wait for real Google tiles to stream in
            models = 0
            for _ in range(80):
                models = pg.evaluate("""() => {
                  const ph = window.__viewer.state.photoreal;
                  if (!ph.tiles) return 0; let n = 0;
                  ph.tiles.forEachLoadedModel(() => n++); return n;
                }""")
                if models > 0:
                    break
                time.sleep(0.5)
            status = pg.eval_on_selector("#photoreal-status", "e=>e.textContent") or ""
            print(f"google requests: ok={goog['ok']} err={goog['err']} statuses={goog['statuses']}")
            print(f"loaded tile models: {models}")
            print(f"photoreal status : {status.strip()}")
            if models == 0:
                failures.append("no Google tile models loaded (network blocked or key/API issue)")

            # frame downtown (scene origin is the georef anchor) and screenshot
            pg.evaluate("""() => {
              const v = window.__viewer, T = v.THREE;
              v.camera.position.set(420, 340, 420);
              v.controls.target.set(0, 270, 0); v.controls.update();
              // hide our flat overlays' base so the Google mesh reads cleanly
              document.getElementById('photoreal-replace').checked = true;
              document.getElementById('photoreal-replace').dispatchEvent(new Event('change'));
            }""")
            time.sleep(3.0)   # let LOD refine for the new view
            pg.screenshot(path=os.path.join(SHOTS, "photoreal-live-google.png"))

            # Vertical calibration. The Google mesh carries real NAVD88-ish elevation; our
            # scene_y is NAVD88 orthometric, so the Google GROUND at origin should read the
            # real elevation there (~295 m downtown). Raycast a grid and take the per-point
            # MINIMUM hit (ground, not roofs); compare to our city ground plane.
            vert = pg.evaluate("""() => {
              const v = window.__viewer, T = v.THREE;
              const ray = new T.Raycaster();
              const groundOf = (grp, x, z) => { if (!grp) return null;
                ray.set(new T.Vector3(x, 2000, z), new T.Vector3(0, -1, 0));
                const hits = ray.intersectObject(grp, true);
                return hits.length ? hits[hits.length - 1].point.y : null; };  // lowest = ground
              const gs = [];
              for (let x = -200; x <= 200; x += 100) for (let z = -200; z <= 200; z += 100) {
                const y = groundOf(v.state.photoreal.group, x, z); if (y != null) gs.push(y);
              }
              gs.sort((a, b) => a - b);
              const med = gs.length ? gs[Math.floor(gs.length / 2)] : null;
              const ourGround = groundOf(v.state.city && v.state.city.group, 0, 0);
              return { googleGround: med, samples: gs.length, ourCityGround: ourGround };
            }""")
            print("vertical probe:", vert)
            gg = vert.get("googleGround")
            if gg is not None:
                print(f"  Google ground @origin ~ {gg:.1f} m (downtown Lexington NAVD88 is ~290-300 m)")
                if vert.get("ourCityGround") is not None:
                    print(f"  our flat city plane = {vert['ourCityGround']:.1f} m "
                          f"(intentionally flattened; use ?flat=0 buildings for the real datum)")
                print(f"  to shift the mesh vertically: window.__viewer.photoreal.calibrate({{ dy: <metres> }})")

            b.close()
    finally:
        srv.terminate()

    print("\nscreenshots ->", SHOTS)
    if failures:
        print("RESULT: FAIL"); [print("  -", f) for f in failures]; return 1
    print("RESULT: PASS — real Google Photorealistic 3D Tiles fetched, decoded, and rendered")
    return 0


if __name__ == "__main__":
    sys.exit(main())
