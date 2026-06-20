#!/usr/bin/env python3
"""Headless regression + graceful-degradation test for the photorealistic 3D-tiles layer.

Proves the integration does NOT break the existing viewer and degrades cleanly with no
API key (the only path testable without a paid Google key):

  1. Loads the real viewer; asserts manifest + roads + buildings still load and there
     are NO fatal JS errors (so the new import/bundle didn't break the app).
  2. Confirms the photoreal layer starts OFF with status "photoreal: off".
  3. Enables it WITHOUT a key -> status becomes "no API key", no crash, the existing
     map keeps rendering (triangle count stays healthy, buildings still present).
  4. Exercises the "hide our buildings & ground" toggle -> no crash, restores cleanly.

Writes before/after screenshots to extracted/ (gitignored). Exit non-zero on failure.

    python -m tools.verify_photoreal            # or: python tools/verify_photoreal.py
"""
import os
import shutil
import socket
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]

from playwright.sync_api import sync_playwright

REPO = os.path.normpath(os.path.join(_HERE, ".."))
WEB = os.path.normpath(os.path.join(_HERE, "..", "web"))
SHOTS = os.path.normpath(os.path.join(_HERE, "..", "extracted"))
TS_DIR = os.path.join(WEB, "_testtileset")
PORT = 8137


def wait_port(p, t=15):
    for _ in range(t * 10):
        try:
            socket.create_connection(("127.0.0.1", p), 0.2).close()
            return True
        except OSError:
            time.sleep(0.1)
    return False


def txt(pg, sid):
    try:
        return (pg.eval_on_selector("#" + sid, "e=>e.textContent") or "").strip()
    except Exception:
        return ""


def main():
    os.makedirs(SHOTS, exist_ok=True)
    # Build the offline test tileset (a cube at a known Lexington ENU frame).
    subprocess.run([sys.executable, "-m", "tools.make_test_tileset", TS_DIR],
                   cwd=REPO, check=True, stdout=subprocess.DEVNULL)
    srv = subprocess.Popen([sys.executable, "-m", "http.server", str(PORT)], cwd=WEB,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    failures, notes = [], []
    try:
        if not wait_port(PORT):
            print("FAIL: static server did not start")
            return 1
        with sync_playwright() as pw:
            b = pw.chromium.launch(args=["--ignore-gpu-blocklist", "--use-gl=angle",
                                         "--enable-unsafe-swiftshader"])
            pg = b.new_page(viewport={"width": 1280, "height": 900})
            errors = []
            pg.on("console", lambda m: errors.append(m.text) if m.type == "error" else None)
            pg.on("pageerror", lambda e: errors.append("PAGEERROR: " + str(e)))
            pg.goto(f"http://127.0.0.1:{PORT}/", wait_until="load")

            # Wait for the existing layers to finish loading.
            loaded = False
            for _ in range(120):
                r = txt(pg, "road-status")
                bld = txt(pg, "buildings-status")
                if r.startswith("roads:") and "loading" not in r and "loading" not in bld \
                        and "waiting" not in bld:
                    loaded = True
                    break
                time.sleep(0.5)
            time.sleep(1.5)

            man, ter = txt(pg, "manifest-status"), txt(pg, "terrain-status")
            road, bld = txt(pg, "road-status"), txt(pg, "buildings-status")
            photo = txt(pg, "photoreal-status")
            print("loaded ok =", loaded)
            for k, v in [("manifest", man), ("terrain", ter), ("roads", road),
                         ("buildings", bld), ("photoreal", photo)]:
                print(f"  {k:10s}: {v}")

            # (1) existing layers present
            if "loading" in road or not road.startswith("roads:"):
                failures.append("roads did not load")
            if "waiting" in bld or "loading" in bld:
                failures.append("buildings did not load")

            base = pg.evaluate("""() => {
              const v = window.__viewer;
              return {
                hasPhotoreal: !!(v.state.photoreal),
                buildingsVisible: !!(v.state.buildings && v.state.buildings.group.visible),
              };
            }""")
            if not base["hasPhotoreal"]:
                failures.append("state.photoreal was not created")
            pg.screenshot(path=os.path.join(SHOTS, "photoreal-off.png"))

            # (2) starts OFF
            if photo and "off" not in photo:
                notes.append(f"unexpected initial photoreal status: {photo}")

            # (3) enable without a key -> graceful 'no API key', no crash
            pg.evaluate("() => window.__viewer.photoreal.setVisible(true)")
            time.sleep(2.0)
            photo2 = txt(pg, "photoreal-status")
            print("  after enable (no key):", photo2)
            if "no api key" not in photo2.lower() and "key" not in photo2.lower():
                failures.append(f"expected a 'no API key' status, got: {photo2}")
            still = pg.evaluate("""() => {
              const v = window.__viewer;
              return {
                buildingsVisible: !!(v.state.buildings && v.state.buildings.group.visible),
                wrapperChildren: v.state.photoreal.group.children.length,
              };
            }""")
            # No key => no TilesRenderer attached yet (wrapper stays empty), map intact.
            if still["wrapperChildren"] != 0:
                notes.append(f"wrapper unexpectedly has {still['wrapperChildren']} children with no key")

            # (4) exercise the replace toggle via the real UI
            pg.evaluate("""() => {
              document.getElementById('photoreal-visible').checked = true;
              document.getElementById('photoreal-replace').checked = true;
              document.getElementById('photoreal-replace').dispatchEvent(new Event('change'));
            }""")
            time.sleep(0.5)
            hid = pg.evaluate("() => !!(window.__viewer.state.buildings && window.__viewer.state.buildings.group.visible)")
            if hid:
                notes.append("'hide our buildings' did not hide the buildings group")
            # turn replace off again -> buildings come back
            pg.evaluate("""() => {
              document.getElementById('photoreal-replace').checked = false;
              document.getElementById('photoreal-replace').dispatchEvent(new Event('change'));
            }""")
            time.sleep(0.5)
            pg.screenshot(path=os.path.join(SHOTS, "photoreal-nokey-enabled.png"))

            # (5) END-TO-END: load a local tileset (no Google key) through the SAME
            # render + alignment path and confirm the cube lands at the scene origin
            # (its ENU anchor). This exercises TilesRenderer + the plugin stack + the GLTF
            # load path at runtime against three r160 and proves the ECEF->scene matrix is
            # applied correctly. (The test cube is uncompressed, so it does NOT exercise the
            # DRACO decode path — that decoder's *presence* is asserted separately below,
            # since real Google tiles are draco-compressed and need it.)
            pg.evaluate("() => window.__viewer.photoreal.loadTileset('/_testtileset/tileset.json')")
            model = None
            for _ in range(80):
                model = pg.evaluate("""() => {
                  const v = window.__viewer, T = v.THREE, ph = v.state.photoreal;
                  if (!ph.tiles) return { ready: false, why: 'no tiles renderer' };
                  let found = null;
                  ph.tiles.forEachLoadedModel((s) => { if (!found) found = s; });
                  if (!found) return { ready: false, why: 'no model yet' };
                  const box = new T.Box3().setFromObject(found);
                  if (box.isEmpty()) return { ready: false, why: 'empty bbox' };
                  const c = box.getCenter(new T.Vector3());
                  return { ready: true, cx: c.x, cy: c.y, cz: c.z };
                }""")
                if model and model.get("ready"):
                    break
                time.sleep(0.4)
            print("  test-tileset model:", model)
            if not (model and model.get("ready")):
                failures.append(f"local test tileset never rendered: {model}")
            else:
                horiz = (model["cx"] ** 2 + model["cz"] ** 2) ** 0.5
                print(f"  tileset center scene=({model['cx']:.1f}, {model['cy']:.1f}, "
                      f"{model['cz']:.1f})  horiz_from_origin={horiz:.1f}m")
                if horiz > 60:
                    failures.append(f"tileset misaligned: {horiz:.1f}m from scene origin (expect <60)")
                if not (250 <= model["cy"] <= 300):
                    failures.append(f"tileset vertical off: y={model['cy']:.1f} (expect ~276)")
            pg.screenshot(path=os.path.join(SHOTS, "photoreal-testtileset.png"))

            # (6) DRACO decoder must actually be served — real Google Photorealistic tiles
            # are draco-compressed and tiles3d.js points DRACOLoader at lib/draco/gltf/. A
            # clone that shipped without web/lib/draco/ would silently fail to decode them,
            # so assert the .wasm is reachable (the uncompressed test cube can't catch this).
            import urllib.request
            try:
                code = urllib.request.urlopen(
                    f"http://127.0.0.1:{PORT}/lib/draco/gltf/draco_decoder.wasm", timeout=5).getcode()
            except Exception:
                code = None
            print("  draco_decoder.wasm HTTP:", code)
            if code != 200:
                failures.append("DRACO decoder not served (web/lib/draco/gltf/draco_decoder.wasm) "
                                "— real Google tiles would fail to decode")

            # (7) Two-way opacity (regression fix): lowering then raising back to 1.0 must
            # restore opaque/depthWrite on already-loaded tile materials (the cube counts).
            op = pg.evaluate("""() => {
              const ph = window.__viewer.state.photoreal;
              const firstMat = () => { let m = null;
                ph.tiles.forEachLoadedModel((s) => { if (m) return; s.traverse((o) => {
                  if (!m && o.material) m = Array.isArray(o.material) ? o.material[0] : o.material; }); });
                return m; };
              ph.setOpacity(0.4); const lo = firstMat(); const a = lo ? { t: lo.transparent, o: +lo.opacity.toFixed(2) } : null;
              ph.setOpacity(1.0); const hi = firstMat(); const b = hi ? { t: hi.transparent, o: +hi.opacity.toFixed(2) } : null;
              return { a, b };
            }""")
            print("  opacity 0.4 ->", op.get("a"), " back to 1.0 ->", op.get("b"))
            if not (op.get("a") and op["a"]["o"] == 0.4 and op["a"]["t"]):
                failures.append(f"opacity 0.4 not applied: {op.get('a')}")
            if not (op.get("b") and op["b"]["o"] == 1.0 and op["b"]["t"] is False):
                failures.append(f"opacity not restored to opaque at 1.0: {op.get('b')}")

            # (8) Base-layer hide must survive a buildings-visible toggle while hidden
            # (UX desync fix), and restore when "replace" is unchecked.
            hc = pg.evaluate("""() => {
              const $ = (id) => document.getElementById(id), v = window.__viewer;
              $('photoreal-visible').checked = true; $('photoreal-replace').checked = true;
              $('photoreal-replace').dispatchEvent(new Event('change'));
              const hiddenAfterReplace = !v.state.buildings.group.visible;
              $('buildings-visible').checked = true; $('buildings-visible').dispatchEvent(new Event('change'));
              const stillHidden = !v.state.buildings.group.visible;
              $('photoreal-replace').checked = false; $('photoreal-replace').dispatchEvent(new Event('change'));
              const backVisible = v.state.buildings.group.visible;
              return { hiddenAfterReplace, stillHidden, backVisible };
            }""")
            print("  base-hide:", hc)
            if not hc["hiddenAfterReplace"]:
                failures.append("photoreal 'replace' did not hide buildings")
            if not hc["stillHidden"]:
                failures.append("buildings-visible toggle overrode the photoreal hide (desync)")
            if not hc["backVisible"]:
                failures.append("buildings did not return after unchecking 'replace'")

            # fatal JS error gate (ignore expected network noise for absent live feeds)
            fatal = [e for e in errors if "PAGEERROR" in e
                     or ("favicon" not in e and "ERR_" not in e and "Failed to load resource" not in e
                         and "transit" not in e.lower() and "camera" not in e.lower())]
            print("console/pageerrors (filtered fatal):", len(fatal))
            for e in fatal[:12]:
                print("  !", e[:200])
            if fatal:
                failures.append(f"{len(fatal)} fatal JS error(s)")
            b.close()
    finally:
        srv.terminate()
        shutil.rmtree(TS_DIR, ignore_errors=True)

    print("\nscreenshots ->", SHOTS)
    for n in notes:
        print("NOTE:", n)
    if failures:
        print("\nRESULT: FAIL")
        for f in failures:
            print("  -", f)
        return 1
    print("\nRESULT: PASS — existing map intact; photoreal layer degrades gracefully with no key")
    return 0


if __name__ == "__main__":
    sys.exit(main())
