// Live Lextran transit layer for the UKy Campus digital twin.
//
// Two halves, mirroring the house style of roads.js (data-driven scene build) and
// agents.js (a deterministic, frame-ticked controller exposed on window.__twin):
//
//   STATIC  — route centrelines + bus stops, baked offline by tools/lextran_gtfs.py
//             into data/transit.json (already projected to scene metres + draped),
//             drawn once as colored route lines and instanced stop markers.
//   LIVE    — moving buses, predicted arrivals, and service alerts, polled from the
//             same-origin proxy tools/serve.py (/api/transit/*). The proxy projects
//             each bus's lon/lat into scene metres for us, so this file never needs
//             a map projection; it interpolates buses between polls, drapes them on
//             the road/terrain with a downward ray (same trick as agents.js ground
//             sensing), and orients them to their GTFS bearing.
//
// createTransitSystem(deps) -> { group, layers, stats, transit }
//   layers  = { routes, stops, buses }                   (toggleable sub-groups)
//   transit = live controller reached at window.__twin.transit:
//     getVehicles() getVehicle(id) getNearestVehicle([x,z],r)
//     getRoutes() getStops() getStop(id) getNearestStop([x,z])
//     getArrivals(stopId) getAlerts() meta() status()
//     tick(dt) start() stop() setPaused(b)
//
// Degrades gracefully: with no proxy you still get routes + stops (a hint shows in
// the panel); with no transit.json you still get live buses. Nothing here throws
// into the render loop.

import * as THREE from 'three';

// ---- module-level scratch (no per-frame allocation; roads.js idiom) ----
const _ray = new THREE.Raycaster();
const _down = Object.freeze(new THREE.Vector3(0, -1, 0));
const _o = new THREE.Vector3();
const D2R = Math.PI / 180;
const BUS_LIFT = 0.35;          // wheels sit just above the road ribbon (LIFT 0.30)
const ROUTE_LIFT = 1.2;         // float route lines above the asphalt
const STOP_LIFT = 0.3;

// bearing (deg CW from north) -> three yaw, where forward = (cos yaw, 0, -sin yaw).
// north(0)->+ -z, east(90)->+x. (Inverse-checked: yaw=atan2(cosθ,sinθ).)
function yawFromBearing(deg) {
  const th = (deg || 0) * D2R;
  return Math.atan2(Math.cos(th), Math.sin(th));
}
function shortestAngle(a, b) { let d = (b - a) % (2 * Math.PI);
  if (d > Math.PI) d -= 2 * Math.PI; if (d < -Math.PI) d += 2 * Math.PI; return d; }

// deterministic fallback colour when a bus's route isn't in transit.json
function hashColor(s) {
  let h = 0; const str = String(s || '0');
  for (let i = 0; i < str.length; i++) h = (h * 31 + str.charCodeAt(i)) | 0;
  return new THREE.Color().setHSL(((h >>> 0) % 360) / 360, 0.65, 0.55);
}
const hex = (c) => new THREE.Color('#' + String(c || '3b82c4').replace(/^#/, ''));

export function createTransitSystem(deps = {}) {
  const {
    scene, dataDir = 'data/', proxyBase = '', groundY: cityGroundY = 285,
    groups = {}, pollMs = 4000, slowPollMs = 15000,
  } = deps;

  const group = new THREE.Group(); group.name = 'transit';
  const layers = {
    routes: new THREE.Group(), stops: new THREE.Group(), buses: new THREE.Group(),
  };
  for (const k of Object.keys(layers)) layers[k].name = 'transit-' + k;
  group.add(layers.routes, layers.stops, layers.buses);

  const stats = { routes: 0, stops: 0, buses: 0 };
  const routesById = new Map();      // id -> route record (static)
  const stops = [];                  // static stop records
  const stopsById = new Map();
  const buses = new Map();           // vehicleId -> bus object
  let feeds = { vehicles: null, trips: null, alerts: null };
  let refGroundY = cityGroundY;   // city ground plane: where buses ride off the campus tiles
  const status = { proxy: 'connecting', static: 'loading', error: null,
                   vehicles: 0, lastVehicleTs: 0, mode: null };

  // ---- shared geometry / materials for buses + stops ----
  const busBodyGeo = new THREE.BoxGeometry(10.5, 3.0, 2.6);
  const busWinGeo = new THREE.BoxGeometry(10.6, 1.0, 2.62);
  const winMat = new THREE.MeshStandardMaterial({ color: 0x0d1b2a, roughness: 0.2, metalness: 0.5 });
  const stopPoleGeo = new THREE.CylinderGeometry(0.12, 0.14, 2.4, 6);
  const stopSignGeo = new THREE.BoxGeometry(0.12, 0.7, 1.0);
  const stopPoleMat = new THREE.MeshStandardMaterial({ color: 0x9aa3ad, roughness: 0.6, metalness: 0.5 });
  const stopSignMat = new THREE.MeshStandardMaterial({ color: 0x1f6fb2, roughness: 0.6, emissive: 0x0a2030, emissiveIntensity: 0.4 });

  // candidate meshes for the downward ground ray (road ribbons win over terrain)
  function groundCandidates() {
    const out = [];
    const rr = groups.roadRibbons && groups.roadRibbons();
    if (rr) out.push(rr);
    if (groups.terrain) out.push(groups.terrain);
    return out;
  }
  // topmost surface (road ribbon or terrain) under (x,z); null if off the campus
  // footprint (the agency runs all over Lexington — buses outside our tiles hide).
  function groundY(x, z) {
    const cands = groundCandidates();
    if (!cands.length) return null;
    _ray.set(_o.set(x, 800, z), _down); _ray.far = 4000;
    const hits = _ray.intersectObjects(cands, true);
    return hits.length ? hits[0].point.y : null;
  }

  // ---------------------------------------------------------- static load ---
  async function loadStatic() {
    let data;
    try {
      const r = await fetch(dataDir + 'transit.json', { cache: 'no-cache' });
      if (!r.ok) throw new Error('HTTP ' + r.status);
      data = await r.json();
    } catch (e) {
      status.static = 'none';
      return; // no static layer; live buses still work
    }
    buildRoutes(data.routes || []);
    buildStops(data.stops || []);
    status.static = 'ok';
  }

  // Route centrelines as flat, vivid, unlit ribbons floating just above the
  // asphalt — readable from a campus-wide view where 1px lines vanish. One merged
  // vertex-coloured geometry (one draw call) over all routes.
  function buildRoutes(routes) {
    const ROUTE_HALF = 1.6;               // ribbon half-width (m)
    const positions = [], colors = [], index = [];
    let base = 0;
    for (const rt of routes) {
      routesById.set(rt.id, rt);
      const col = hex(rt.color);
      for (const poly of rt.shapes || []) {
        const n = poly.length;
        if (n < 2) continue;
        // per-vertex XZ perpendicular (average of adjacent segment directions)
        for (let i = 0; i < n; i++) {
          let dx = 0, dz = 0;
          if (i > 0) { dx += poly[i][0] - poly[i - 1][0]; dz += poly[i][2] - poly[i - 1][2]; }
          if (i < n - 1) { dx += poly[i + 1][0] - poly[i][0]; dz += poly[i + 1][2] - poly[i][2]; }
          const len = Math.hypot(dx, dz) || 1;
          const px = -dz / len * ROUTE_HALF, pz = dx / len * ROUTE_HALF;
          const x = poly[i][0], y = poly[i][1] + ROUTE_LIFT, z = poly[i][2];
          positions.push(x + px, y, z + pz, x - px, y, z - pz);
          colors.push(col.r, col.g, col.b, col.r, col.g, col.b);
        }
        for (let i = 0; i + 1 < n; i++) {
          const a = base + i * 2;
          index.push(a, a + 2, a + 1, a + 1, a + 2, a + 3);
        }
        base += n * 2;
      }
    }
    stats.routes = routesById.size;
    if (!positions.length) return;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
    geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(colors), 3));
    geo.setIndex(index);
    geo.computeBoundingSphere();
    const mat = new THREE.MeshBasicMaterial({
      vertexColors: true, side: THREE.DoubleSide, transparent: true, opacity: 0.85,
      polygonOffset: true, polygonOffsetFactor: -4, polygonOffsetUnits: -4,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.name = 'route-ribbons';
    layers.routes.add(mesh);
  }

  function buildStops(list) {
    stats.stops = list.length;
    if (!list.length) return;
    const poles = new THREE.InstancedMesh(stopPoleGeo, stopPoleMat, list.length);
    const signs = new THREE.InstancedMesh(stopSignGeo, stopSignMat, list.length);
    poles.name = 'stop-poles'; signs.name = 'stop-signs';
    const m = new THREE.Matrix4(), q = new THREE.Quaternion(), s = new THREE.Vector3(1, 1, 1), p = new THREE.Vector3();
    list.forEach((st, i) => {
      stops.push(st); stopsById.set(st.id, st);
      const [x, y, z] = st.pos;
      poles.setMatrixAt(i, m.compose(p.set(x, y + STOP_LIFT + 1.2, z), q, s));
      signs.setMatrixAt(i, m.compose(p.set(x, y + STOP_LIFT + 2.0, z), q, s));
    });
    poles.instanceMatrix.needsUpdate = true; signs.instanceMatrix.needsUpdate = true;
    poles.computeBoundingSphere(); signs.computeBoundingSphere();
    layers.stops.add(poles, signs);
  }

  // ------------------------------------------------------------- buses ---
  function busColor(v) {
    if (v.route && v.route.color) return hex(v.route.color);
    const rt = v.routeId && routesById.get(v.routeId);
    if (rt) return hex(rt.color);
    return hashColor(v.routeId || v.id);
  }
  function busLabel(v) {
    if (v.route && v.route.shortName) return v.route.shortName;
    const rt = v.routeId && routesById.get(v.routeId);
    if (rt && rt.shortName) return rt.shortName;
    return v.routeId || '?';
  }

  function makeSprite(text, color) {
    const c = document.createElement('canvas'); c.width = 128; c.height = 64;
    const g = c.getContext('2d');
    g.fillStyle = '#' + color.getHexString(); g.strokeStyle = 'rgba(0,0,0,0.55)'; g.lineWidth = 6;
    roundRect(g, 8, 8, 112, 48, 12); g.fill(); g.stroke();
    g.fillStyle = '#ffffff'; g.font = 'bold 38px system-ui, sans-serif';
    g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText(String(text).slice(0, 4), 64, 33);
    const tex = new THREE.CanvasTexture(c); tex.anisotropy = 4;
    const spr = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: true }));
    spr.scale.set(7, 3.5, 1);
    return spr;
  }
  function roundRect(g, x, y, w, h, r) {
    g.beginPath(); g.moveTo(x + r, y);
    g.arcTo(x + w, y, x + w, y + h, r); g.arcTo(x + w, y + h, x, y + h, r);
    g.arcTo(x, y + h, x, y, r); g.arcTo(x, y, x + w, y, r); g.closePath();
  }

  function spawnBus(v) {
    const root = new THREE.Group();
    root.name = 'bus-' + v.id;
    const color = busColor(v);
    const bodyMat = new THREE.MeshStandardMaterial({ color, roughness: 0.45, metalness: 0.3 });
    const body = new THREE.Mesh(busBodyGeo, bodyMat); body.position.y = 1.5 + 0.3; root.add(body);
    const win = new THREE.Mesh(busWinGeo, winMat); win.position.y = 2.2; root.add(win);
    const spr = makeSprite(busLabel(v), color); spr.position.set(0, 5.2, 0); root.add(spr);
    layers.buses.add(root);
    const bus = {
      id: v.id, object: root, body, bodyMat, sprite: spr, label: busLabel(v),
      cur: new THREE.Vector3(v.x, 0, v.z), tgt: new THREE.Vector3(v.x, 0, v.z),
      yaw: yawFromBearing(v.bearing), tgtYaw: yawFromBearing(v.bearing),
      data: v, lastSeen: performance.now(), lastGroundY: null,
    };
    bus.object.position.copy(bus.cur);
    bus.object.rotation.y = bus.yaw;
    buses.set(v.id, bus);
    return bus;
  }

  function updateBus(bus, v) {
    bus.tgt.set(v.x, bus.tgt.y, v.z);
    if (v.bearing != null) bus.tgtYaw = yawFromBearing(v.bearing);
    bus.data = v; bus.lastSeen = performance.now();
    const lbl = busLabel(v);
    if (lbl !== bus.label) {                 // route changed -> recolour + relabel
      bus.label = lbl; const col = busColor(v);
      bus.bodyMat.color.copy(col);
      bus.object.remove(bus.sprite); bus.sprite.material.map.dispose(); bus.sprite.material.dispose();
      bus.sprite = makeSprite(lbl, col); bus.sprite.position.set(0, 5.2, 0); bus.object.add(bus.sprite);
    }
  }

  function ingestVehicles(payload) {
    const seen = new Set();
    for (const v of payload.vehicles || []) {
      if (v.x == null || v.z == null) continue;
      seen.add(v.id);
      const bus = buses.get(v.id);
      if (bus) updateBus(bus, v); else spawnBus(v);
    }
    // retire buses unseen for > 2 slow polls (left the area / went out of service)
    const now = performance.now();
    for (const [id, bus] of buses) {
      if (!seen.has(id) && now - bus.lastSeen > Math.max(20000, pollMs * 3)) {
        layers.buses.remove(bus.object);
        bus.bodyMat.dispose(); bus.sprite.material.map.dispose(); bus.sprite.material.dispose();
        buses.delete(id);
      }
    }
    stats.buses = buses.size;
    status.vehicles = buses.size;
    status.lastVehicleTs = payload.ts || 0;
    status.mode = payload.mode || status.mode;
  }

  // ------------------------------------------------------------- polling ---
  let pollTimer = null, slowTimer = null, paused = false, stopped = false;

  const RECONNECT_MS = 20000;   // slow probe while the proxy is absent (no 404 spam)
  async function pollVehicles() {
    if (stopped) return;
    let okProxy = false;
    try {
      const r = await fetch(proxyBase + '/api/transit/vehicles', { cache: 'no-store' });
      if (!r.ok) throw new Error('HTTP ' + r.status);
      const payload = await r.json();
      feeds.vehicles = payload;
      ingestVehicles(payload);
      status.proxy = payload.error ? 'error' : 'ok';
      status.error = payload.error || null;
      okProxy = true;
    } catch (e) {
      status.proxy = 'offline';
      status.error = 'live feed offline — run: python -m tools.serve';
    } finally {
      if (!stopped) {
        // Fast cadence only once the proxy answers; otherwise probe slowly so a
        // plain `http.server` (no proxy) isn't spammed with 404s every few seconds.
        pollTimer = setTimeout(pollVehicles, okProxy ? pollMs : RECONNECT_MS);
        if (okProxy && !slowTimer) pollSlow();   // start arrivals/alerts once confirmed
      }
    }
  }
  async function pollSlow() {
    if (stopped) return;
    for (const kind of ['trips', 'alerts']) {
      try {
        const r = await fetch(proxyBase + '/api/transit/' + kind, { cache: 'no-store' });
        if (r.ok) feeds[kind] = await r.json();
      } catch (e) { /* keep last */ }
    }
    if (!stopped) slowTimer = setTimeout(pollSlow, slowPollMs);
  }

  // ------------------------------------------------------------- per frame ---
  function tick(dt) {
    if (paused || !buses.size) return;
    const k = 1 - Math.exp(-3.5 * Math.min(dt, 0.1));   // frame-rate-independent lerp
    for (const bus of buses.values()) {
      bus.cur.x += (bus.tgt.x - bus.cur.x) * k;
      bus.cur.z += (bus.tgt.z - bus.cur.z) * k;
      // Buses stay visible everywhere — the agency runs all over Lexington, so a
      // bus can sit beyond our terrain tiles. Drape on the campus surface when we
      // have one under it; otherwise ride at its last known elevation (or the
      // campus reference), so off-footprint buses float at street level rather
      // than dropping to y=0.
      const gy = groundY(bus.cur.x, bus.cur.z);
      if (gy != null) { bus.lastGroundY = gy; refGroundY = gy; }
      const y = (gy != null ? gy : (bus.lastGroundY != null ? bus.lastGroundY : refGroundY)) + BUS_LIFT;
      bus.object.position.set(bus.cur.x, y, bus.cur.z);
      bus.yaw += shortestAngle(bus.yaw, bus.tgtYaw) * k;
      bus.object.rotation.y = bus.yaw;
    }
  }

  // ------------------------------------------------------- query API ---
  const nowSec = () => Math.floor(Date.now() / 1000);   // GTFS times are unix seconds

  function vState(bus) {
    const v = bus.data, p = bus.object.position;
    return {
      id: v.id, routeId: v.routeId, route: v.route || (routesById.get(v.routeId) || null),
      label: bus.label, position: [p.x, p.y, p.z], lat: v.lat, lon: v.lon,
      bearing: v.bearing, speed: v.speed, heading: ((bus.yaw / D2R) % 360 + 360) % 360,
      stopId: v.stopId, status: v.status, occupancy: v.occupancy, ts: v.vts,
    };
  }

  const transit = {
    group, layers, stats, tick,
    start() { stopped = false; if (!pollTimer) pollVehicles(); return this; },  // pollSlow starts once the proxy answers
    stop() { stopped = true; clearTimeout(pollTimer); clearTimeout(slowTimer); pollTimer = slowTimer = null; return this; },
    setPaused(b) { paused = !!b; },
    status: () => ({ ...status, routes: stats.routes, stops: stats.stops, buses: stats.buses }),
    meta: () => feeds.vehicles ? { mode: feeds.vehicles.mode, ts: feeds.vehicles.ts } : null,

    getVehicles() { return [...buses.values()].map(vState); },
    getVehicle(id) { const b = buses.get(String(id)); return b ? vState(b) : null; },
    getNearestVehicle(pos, maxR = Infinity) {
      let best = null, bd = maxR * maxR;
      const pz = (pos.length === 2 ? pos[1] : pos[2]);   // accept [x,z] or [x,y,z]
      for (const b of buses.values()) {
        const p = b.object.position, dx = p.x - pos[0], dz = p.z - pz;
        const d = dx * dx + dz * dz;
        if (d < bd) { bd = d; best = b; }
      }
      return best ? { ...vState(best), distance: Math.sqrt(bd) } : null;
    },

    getRoutes() { return [...routesById.values()].map((r) => ({ id: r.id, shortName: r.shortName, longName: r.longName, color: r.color })); },
    getStops() { return stops.map((s) => ({ id: s.id, name: s.name, code: s.code, position: s.pos, routes: s.routes })); },
    getStop(id) { const s = stopsById.get(String(id)); return s ? { id: s.id, name: s.name, code: s.code, position: s.pos, routes: s.routes } : null; },
    getNearestStop(pos) {
      let best = null, bd = Infinity;
      const pz = (pos.length === 2 ? pos[1] : pos[2]);
      for (const s of stops) { const dx = s.pos[0] - pos[0], dz = s.pos[2] - pz; const d = dx * dx + dz * dz;
        if (d < bd) { bd = d; best = s; } }
      return best ? { id: best.id, name: best.name, position: best.pos, routes: best.routes, distance: Math.sqrt(bd) } : null;
    },

    getArrivals(stopId, horizonSec = 3600) {
      const t = feeds.trips; if (!t || !t.byStop) return [];
      const now = nowSec();
      return (t.byStop[String(stopId)] || [])
        .filter((r) => r.arrival && r.arrival - now > -120 && r.arrival - now < horizonSec)
        .map((r) => ({ routeId: r.routeId, tripId: r.tripId, arrival: r.arrival,
                       etaSec: r.arrival - now, etaMin: Math.round((r.arrival - now) / 60), delay: r.delay }))
        .sort((a, b) => a.etaSec - b.etaSec);
    },
    getAlerts() { return (feeds.alerts && feeds.alerts.alerts) || []; },
  };

  // kick off: load static geometry, then start polling the live feed
  loadStatic().finally(() => transit.start());

  return { group, layers, stats, transit };
}
