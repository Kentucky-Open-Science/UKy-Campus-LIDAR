#!/usr/bin/env python3
"""Headless runtime check of the autonomous-agent API (web/agents.js).

Starts a static server for web/, opens the viewer in headless Chromium (WebGL via
ANGLE), waits for the road network + agent system to come up, then exercises the
whole sensor suite from inside the page:

  * spawn a car, drive it forward, assert it moves +X and stays on the ground
  * read the POV camera to a pixel buffer and assert it has image content
  * spawn two overlapping cars and assert collision DETECTION fires
  * spawn a drone and assert it reports an above-ground-level altitude
  * confirm georef (UTM) is reported and the main render did not break

Usage:  python tools/verify_agents.py            # auto-starts a server on :8137
        python tools/verify_agents.py 8000        # use an already-running server
Exit code 0 = all assertions passed.
"""
import functools
import http.server
import os
import sys
import threading

_HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.abspath(os.path.join(_HERE, "..", "web"))
SHOTS = os.path.abspath(os.path.join(_HERE, "..", "extracted"))

# tools/ contains an inspect.py that shadows the stdlib; drop our own dir from the
# import path before importing playwright (same guard as verify_viewer.py).
sys.path = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]

from playwright.sync_api import sync_playwright

# In-page test: returns a results dict. Runs after the agent system is live.
PAGE_TEST = r"""
async () => {
  const A = window.__twin.agents, V = window.__viewer;
  const out = { ok: true, notes: [] };
  const fail = (m) => { out.ok = false; out.notes.push(m); };

  if (!A) { return { ok: false, notes: ['window.__twin.agents missing'] }; }

  const t = V.controls.target;
  const at = (dx, dz) => [t.x + (dx || 0), null, t.z + (dz || 0)];

  // --- spawn + drive a car -------------------------------------------------
  const car = A.spawn({ type: 'car', position: at(0, 0), heading: 0, color: 0x33aaff, showHeading: true });
  out.spawnSurface = car.surface;
  const s0 = car.getState();
  out.startPos = s0.position;
  car.setControls({ throttle: 1, steer: 0 });
  for (let i = 0; i < 80; i++) A.tick(0.05);            // ~4 sim-seconds, deterministic
  const s1 = car.getState();
  out.endPos = s1.position;
  out.dx = s1.position[0] - s0.position[0];
  out.dz = s1.position[2] - s0.position[2];
  out.moved = Math.hypot(out.dx, out.dz);
  out.speed = s1.speed;
  out.surfaceAfter = car.surface;
  out.heading = s1.heading;
  out.utm = s1.utm;
  out.onGround = s1.onGround;
  if (!(out.moved > 1)) fail('car did not move under throttle');
  if (!(out.dx > Math.abs(out.dz))) fail('car heading 0 did not move mostly +X (dx=' + out.dx.toFixed(2) + ', dz=' + out.dz.toFixed(2) + ')');
  if (!['road', 'terrain', 'building'].includes(car.surface)) fail('car not on a known surface: ' + car.surface);
  if (!s1.utm || s1.utm.zone !== '16N') fail('no UTM georef on state');

  // --- POV camera ----------------------------------------------------------
  const img = car.camera.read({ format: 'pixels', size: [96, 72] });
  if (!img || img.width !== 96 || img.height !== 72) fail('camera.read returned wrong/empty result');
  else {
    let mn = 255, mx = 0;
    for (let i = 0; i < img.data.length; i += 4) { const v = img.data[i]; if (v < mn) mn = v; if (v > mx) mx = v; }
    out.camMin = mn; out.camMax = mx;
    if (mx - mn < 4) fail('camera image is flat (no content): min=' + mn + ' max=' + mx);
  }
  // dataURL path should also work
  const url = car.camera.read({ format: 'dataURL', size: [32, 24] });
  out.dataUrlOk = typeof url === 'string' && url.startsWith('data:image/png');
  if (!out.dataUrlOk) fail('camera dataURL read failed');

  // --- collision DETECTION (two overlapping cars) --------------------------
  const ca = A.spawn({ type: 'car', position: at(250, 0), name: 'col_a' });
  const cb = A.spawn({ type: 'car', position: at(250, 0), name: 'col_b' });
  let fired = false; ca.onCollision(() => { fired = true; });
  A.tick(0.05); A.tick(0.05);
  out.colContacts = ca.getContacts().length;
  out.colFired = fired;
  if (out.colContacts < 1) fail('overlapping cars reported no contact');
  if (!fired) fail('onCollision did not fire on enter');
  const cdet = ca.getContacts()[0];
  if (cdet) { out.colNormal = cdet.normal; out.colPen = cdet.penetration;
    if (!(cdet.penetration > 0)) fail('contact penetration not positive'); }

  // --- camera.pose() must not throw (regression: Vector3 in the quaternion slot) ---
  try { const ps = car.camera.pose();
    out.poseOk = !!(ps && ps.position && ps.forward && isFinite(ps.forward[0]) && isFinite(ps.position[1]));
  } catch (e) { out.poseOk = false; fail('camera.pose threw: ' + e.message); }
  if (!out.poseOk) fail('camera.pose did not return a valid pose');

  // --- spawn({controller}) must wire the control loop immediately ----------
  let ctrlRan = false;
  A.spawn({ type: 'robot', position: at(-260, 0), name: 'ctrl',
            controller: () => { ctrlRan = true; return { throttle: 1 }; } });
  A.tick(0.05); A.tick(0.05);
  out.controllerWired = ctrlRan;
  if (!ctrlRan) fail('spawn({controller}) did not register the controller');

  // --- setVelocity([0,0,0]) must actively STOP a ground vehicle (not coast) -
  const sv = A.spawn({ type: 'car', position: at(-260, 120), name: 'sv' });
  sv.setControls({ throttle: 1 }); for (let i = 0; i < 20; i++) A.tick(0.05);
  out.svRolling = sv.getState().speed;
  sv.setVelocity([0, 0, 0]); for (let i = 0; i < 40; i++) A.tick(0.05);  // 2 s to brake
  out.svStopped = sv.getState().speed;
  if (!(out.svRolling > 3)) fail('sv car never got rolling');
  if (Math.abs(out.svStopped) > 0.5) fail('setVelocity([0,0,0]) did not stop the car (speed=' + out.svStopped.toFixed(2) + ')');

  // --- drone hover + AGL ---------------------------------------------------
  const d = A.spawn({ type: 'drone', position: at(0, 0) });
  for (let i = 0; i < 20; i++) A.tick(0.05);
  out.droneAGL = d.getState().altitudeAGL;
  out.droneSurface = d.surface;
  if (out.droneAGL == null || out.droneAGL < 1) fail('drone AGL not reported above ground: ' + out.droneAGL);

  // --- lifecycle -----------------------------------------------------------
  out.count = A.count();
  A.despawn(car);
  out.afterDespawn = A.count();
  if (out.afterDespawn !== out.count - 1) fail('despawn did not reduce count');

  return out;
}
"""


class _QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass  # silence the request log

    def copyfile(self, source, outputfile):
        # the browser closes connections mid-stream when we tear down (lidar chunks
        # still loading); swallow the resulting reset rather than dumping a traceback.
        try:
            super().copyfile(source, outputfile)
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
            pass


def serve(directory, port):
    handler = functools.partial(_QuietHandler, directory=directory)
    httpd = http.server.ThreadingHTTPServer(("127.0.0.1", port), handler)
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


def main():
    own_server = len(sys.argv) <= 1
    port = int(sys.argv[1]) if not own_server else 8137
    httpd = serve(WEB, port) if own_server else None
    os.makedirs(SHOTS, exist_ok=True)
    msgs = []
    result = None
    with sync_playwright() as p:
        browser = p.chromium.launch(args=["--use-gl=angle", "--ignore-gpu-blocklist"])
        page = browser.new_page(viewport={"width": 1280, "height": 800})
        page.on("console", lambda m: msgs.append(f"[{m.type}] {m.text}")
                if m.type in ("error", "warning") else None)
        page.on("pageerror", lambda e: msgs.append(f"[pageerror] {e}"))
        page.goto(f"http://localhost:{port}/")
        # wait for the agent system + a ground surface (road ribbons or terrain)
        try:
            page.wait_for_function(
                "() => window.__twin && window.__twin.agents && window.__viewer "
                "&& window.__viewer.state.roadnet "
                "&& (window.__viewer.state.roadnet.layers.roads.children.length "
                "    || window.__viewer.state.terrain.tiles.some(t => t.status==='loaded'))",
                timeout=45000)
        except Exception as e:
            print("FAILED waiting for agent system / ground:", e)
            for m in msgs:
                print("  " + m)
            page.screenshot(path=os.path.join(SHOTS, "agents-verify-fail.png"))
            browser.close()
            return 2
        page.wait_for_timeout(2500)  # let a few terrain tiles + buildings stream in
        result = page.evaluate(PAGE_TEST)

        # --- drive mode: UI spawn -> third-person chase cam + WASD on the agent ---
        drive = {"ok": True, "notes": []}
        try:
            page.click("#agent-spawn")           # spawns a car and enters drive mode
            page.wait_for_timeout(300)
            before = page.evaluate(
                "() => { const v = window.__viewer, A = v.state.agents, a = A.list().slice(-1)[0];"
                " return { enabled: v.controls.enabled,"
                "  hint: !document.getElementById('drive-hint').classList.contains('hidden'),"
                "  id: a.id, type: a.type, pos: a.getState().position, gy: a.groundY,"
                "  cam: [v.camera.position.x, v.camera.position.y, v.camera.position.z] }; }")
            drive["beforeCam"] = before["cam"]
            drive["beforeType"] = before["type"]
            drive["orbitDisabledWhileDriving"] = (before["enabled"] is False)
            drive["hintShown"] = before["hint"]
            page.keyboard.down("w")               # throttle forward
            page.wait_for_timeout(1500)
            page.screenshot(path=os.path.join(SHOTS, "agents-drive-chase.png"))  # third-person view
            page.keyboard.up("w")
            after = page.evaluate(
                "(id) => { const v = window.__viewer, a = v.state.agents.get(id),"
                " s = a.getState(), c = v.camera.position;"
                " return { pos: s.position, speed: s.speed, surface: s.surface, offMap: s.offMap,"
                "  cam: [c.x, c.y, c.z] }; }",
                before["id"])
            ap, ac = after["pos"], after["cam"]
            drive["afterCam"], drive["afterPos"], drive["surface"] = ac, ap, after["surface"]
            drive["speedUnderW"] = after["speed"]
            moved = ((ap[0] - before["pos"][0]) ** 2 + (ap[2] - before["pos"][2]) ** 2) ** 0.5
            camDist = ((ac[0] - ap[0]) ** 2 + (ac[2] - ap[2]) ** 2) ** 0.5  # horizontal cam->agent
            drive["agentMovedUnderW"] = moved
            drive["camDist"] = camDist
            # behind = camera sits opposite the heading (toward -X for heading 0)
            drive["camBehind"] = ac[0] < ap[0]
            drive["camAboveAgent"] = ac[1] > ap[1]
            page.keyboard.press("Escape")          # release
            page.wait_for_timeout(200)
            rel = page.evaluate(
                "() => ({ enabled: window.__viewer.controls.enabled,"
                " hint: !document.getElementById('drive-hint').classList.contains('hidden') })")
            drive["orbitRestoredOnEsc"] = (rel["enabled"] is True)
            drive["hintHiddenOnEsc"] = (rel["hint"] is False)

            def dfail(m):
                drive["ok"] = False
                drive["notes"].append(m)
            if not drive["orbitDisabledWhileDriving"]: dfail("orbit not disabled while driving")
            if not drive["hintShown"]: dfail("drive hint not shown on spawn")
            if not (after["speed"] > 1): dfail(f"agent did not accelerate under W (speed={after['speed']:.2f})")
            if not (moved > 1): dfail(f"agent did not move under W (moved={moved:.2f})")
            if not (4 < camDist < 60): dfail(f"chase distance out of range ({camDist:.1f})")
            if not drive["camBehind"]: dfail("chase cam not behind the agent")
            if not drive["orbitRestoredOnEsc"]: dfail("orbit not restored on Esc")
            if not drive["hintHiddenOnEsc"]: dfail("drive hint not hidden on Esc")
        except Exception as e:  # noqa: BLE001
            drive["ok"] = False
            drive["notes"].append(f"drive test threw: {e}")
        result["drive"] = drive

        page.screenshot(path=os.path.join(SHOTS, "agents-verify.png"))
        browser.close()

    print("=== agent verification result ===")
    for k, v in (result or {}).items():
        print(f"  {k}: {v}")
    print("=== console errors/warnings ===")
    for m in msgs:
        print("  " + m)
    # The viewer optionally polls the transit proxy (/api/transit/*); on this bare
    # static server that 404s once before backing off — an expected optional-resource
    # miss, not a code error. Real JS errors/pageerrors are still caught.
    hard_errors = [m for m in msgs if (m.startswith("[pageerror]") or m.startswith("[error]"))
                   and "Failed to load resource" not in m]
    drive_ok = bool(result and result.get("drive") and result["drive"].get("ok"))
    passed = bool(result and result.get("ok")) and drive_ok and not hard_errors
    print("\nRESULT:", "PASS" if passed else "FAIL")
    if httpd:
        httpd.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
