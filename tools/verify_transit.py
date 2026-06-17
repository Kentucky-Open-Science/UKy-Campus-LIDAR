#!/usr/bin/env python3
"""Headless runtime check of the live transit layer (web/transit.js + the tools/twin_server.py transit proxy).

Starts the transit proxy in MOCK mode (deterministic, offline — replays
tools/_transit_samples), opens the viewer in headless Chromium, waits for the
transit system + the first live buses, then exercises the layer from inside the
page:

  * static geometry: routes + stops loaded from data/transit.json and rendered
  * live buses: spawned from /api/transit/vehicles, projected inside the viewport
  * motion: bus scene positions advance between two polls (mock crawl + interp)
  * query API: getNearestVehicle / getNearestStop / getArrivals / getAlerts
  * agentic hook: an agent's controller sees s.transit and can find a nearby bus

Usage:  python tools/verify_transit.py
Exit code 0 = all assertions passed. Requires playwright (see requirements.txt):
    pip install playwright && playwright install chromium
"""
import os
import sys
import threading

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(_HERE, ".."))
SHOTS = os.path.join(ROOT, "extracted")

# tools/inspect.py shadows the stdlib `inspect` when tools/ is on sys.path, breaking
# numpy's import; drop our own dir BEFORE importing twin_server (it imports numpy) and
# playwright. `tools` stays importable as a package via ROOT.
sys.path.insert(0, ROOT)
sys.path = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]
from tools import twin_server as twin_mod  # noqa: E402
from playwright.sync_api import sync_playwright  # noqa: E402


PAGE_TEST = r"""
async () => {
  const T = window.__twin.transit, A = window.__twin.agents, V = window.__viewer;
  const out = { ok: true, notes: [] };
  const fail = (m) => { out.ok = false; out.notes.push(m); };
  if (!T) return { ok: false, notes: ['window.__twin.transit missing'] };

  // --- static geometry -----------------------------------------------------
  out.routes = T.getRoutes().length;
  out.stops = T.getStops().length;
  if (!(out.routes > 0)) fail('no routes loaded from transit.json');
  if (!(out.stops > 0)) fail('no stops loaded from transit.json');
  out.routeLines = V.state.transit.layers.routes.children.length;
  out.stopMeshes = V.state.transit.layers.stops.children.length;
  if (!out.routeLines) fail('no route line geometry in the scene');

  // --- live buses ----------------------------------------------------------
  const v0 = T.getVehicles();
  out.buses = v0.length;
  out.busMeshes = V.state.transit.layers.buses.children.length;
  if (!(out.buses > 0)) fail('no live buses from the proxy');
  if (out.busMeshes !== out.buses) fail(`bus mesh count ${out.busMeshes} != vehicle count ${out.buses}`);
  // every bus should project into a sane scene range (campus viewport)
  const inb = v0.filter((b) => b.position[0] > -1500 && b.position[0] < 1100 &&
                               b.position[2] > -1300 && b.position[2] < 2700);
  out.busesInBounds = inb.length;
  if (!(inb.length > 0)) fail('no bus projected inside the viewport');
  const b0 = v0[0];
  if (b0.lat == null || b0.lon == null) fail('bus missing lat/lon');
  out.sampleBus = { id: b0.id, route: b0.label, pos: b0.position.map((n) => +n.toFixed(1)) };

  // --- query API -----------------------------------------------------------
  const near = T.getNearestVehicle(b0.position);
  if (!near || near.distance == null) fail('getNearestVehicle returned nothing');
  out.nearestVehDist = near && +near.distance.toFixed(2);
  const ns = T.getNearestStop(b0.position);
  if (!ns) fail('getNearestStop returned nothing'); else out.nearestStop = ns.name;
  // arrivals: find any stop that has predictions in the mock TripUpdate feed
  let arr = [];
  for (const s of T.getStops()) { const a = T.getArrivals(s.id); if (a.length) { arr = a; out.arrStop = s.id; break; } }
  out.arrivals = arr.length;
  if (arr.length) out.sampleEtaMin = arr[0].etaMin;
  out.alerts = T.getAlerts().length;

  return out;
}
"""

MOTION_TEST = r"""
async () => {
  const T = window.__twin.transit;
  const snap = () => T.getVehicles().map((b) => [b.id, b.position[0], b.position[2]]);
  return snap();
}
"""

AGENT_TEST = r"""
async () => {
  const A = window.__twin.agents, T = window.__twin.transit, V = window.__viewer;
  const out = { ok: true, notes: [] };
  const fail = (m) => { out.ok = false; out.notes.push(m); };
  // park an agent on a live bus so a nearby bus is guaranteed, then sense it
  const bus = T.getVehicles()[0];
  if (!bus) return { ok: false, notes: ['no bus to sense'] };
  let sawTransit = false, sawBus = false;
  const car = A.spawn({ type: 'car', position: [bus.position[0], null, bus.position[2]], name: 'sensor' });
  car.setController((s) => {
    if (s.transit) { sawTransit = true;
      const n = s.transit.getNearestVehicle(s.position, 5000);
      if (n) sawBus = true;
    }
    return { brake: 1 };
  });
  for (let i = 0; i < 6; i++) A.tick(0.05);
  out.sawTransitInSensors = sawTransit;
  out.sawNearbyBus = sawBus;
  if (!sawTransit) fail('agent controller did not receive s.transit');
  if (!sawBus) fail('agent could not find a nearby bus via s.transit');
  A.despawn(car);
  return out;
}
"""


def main():
    twin_mod.PROXY = twin_mod.build_proxy(mock=True)
    httpd = twin_mod.make_server(0)   # world stays None: only the transit proxy + static
    port = httpd.server_address[1]
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    os.makedirs(SHOTS, exist_ok=True)
    msgs, result = [], None

    with sync_playwright() as p:
        browser = p.chromium.launch(args=["--use-gl=angle", "--ignore-gpu-blocklist"])
        page = browser.new_page(viewport={"width": 1280, "height": 800})
        page.on("console", lambda m: msgs.append(f"[{m.type}] {m.text}")
                if m.type in ("error", "warning") else None)
        page.on("pageerror", lambda e: msgs.append(f"[pageerror] {e}"))
        page.goto(f"http://localhost:{port}/")
        try:
            page.wait_for_function(
                "() => window.__twin && window.__twin.transit && window.__twin.agents "
                "&& window.__twin.transit.getVehicles().length > 0",
                timeout=45000)
        except Exception as e:
            print("FAILED waiting for transit + buses:", e)
            for m in msgs:
                print("  " + m)
            page.screenshot(path=os.path.join(SHOTS, "transit-verify-fail.png"))
            browser.close(); httpd.shutdown()
            return 2
        page.wait_for_timeout(2500)  # let terrain stream in so buses drape on it

        result = page.evaluate(PAGE_TEST)

        # motion: two snapshots a poll apart should differ (mock crawl + interpolation)
        s1 = page.evaluate(MOTION_TEST)
        page.wait_for_timeout(5000)
        s2 = page.evaluate(MOTION_TEST)
        m1 = {r[0]: (r[1], r[2]) for r in s1}
        moved = 0.0
        for rid, x, z in s2:
            if rid in m1:
                moved = max(moved, ((x - m1[rid][0]) ** 2 + (z - m1[rid][1]) ** 2) ** 0.5)
        result["maxBusMove5s"] = round(moved, 2)
        if not (moved > 0.5):
            result["ok"] = False
            result.setdefault("notes", []).append(f"buses did not move ({moved:.2f} m in 5 s)")

        agent = page.evaluate(AGENT_TEST)
        result["agent"] = agent

        page.screenshot(path=os.path.join(SHOTS, "transit-verify.png"))
        browser.close()

    print("=== transit verification result ===")
    for k, v in (result or {}).items():
        print(f"  {k}: {v}")
    print("=== console errors/warnings ===")
    for m in msgs:
        print("  " + m)
    hard = [m for m in msgs if (m.startswith("[pageerror]") or m.startswith("[error]"))
            and "Failed to load resource" not in m]
    agent_ok = bool(result and result.get("agent") and result["agent"].get("ok"))
    passed = bool(result and result.get("ok")) and agent_ok and not hard
    print("\nRESULT:", "PASS" if passed else "FAIL")
    httpd.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
