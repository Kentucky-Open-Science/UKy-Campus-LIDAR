// Live traffic-camera layer for the UKy Campus digital twin.
//
// Mirrors transit.js's two-half split:
//   STATIC — each city traffic camera snapped to the twin intersection it watches,
//            baked offline by tools/lex_cameras.py into data/cameras.json (already
//            projected to scene metres). Drawn once as instanced markers (one pole +
//            one housing draw call for all ~113 cameras — the one-buffer-per-layer
//            convention the rest of the viewer keeps).
//   LIVE   — the tokenized HLS stream URLs, which the city re-signs every ~15 min and
//            so can't be baked. Polled from the same-origin proxy in
//            tools/twin_server.py (/api/cameras/streams). The browser plays them
//            directly with hls.js — the camera origin sends Access-Control-Allow-
//            Origin:* (verified), so no segment proxying is needed. With no proxy the
//            layer still shows markers and the token-free `still` thumbnail.
//
// createCameraSystem(deps) -> { group, layers, stats, cameras }
//   layers   = { markers }                                  (toggleable sub-group)
//   cameras  = controller reached at window.__twin.cameras:
//     list() get(id) getNearest([x,z],r) streamUrl(id) still(id) byInstance(i)
//     status() tick(dt) start() stop()
//
// Degrades gracefully: no cameras.json -> empty layer; no proxy -> markers + the
// token-free thumbnail (a slow probe avoids 404 spam). Nothing here throws into the
// render loop.

import * as THREE from 'three';

const MARK_LIFT = 0.3;          // housing sits this far above the post top
const POLE_H = 6.0;             // camera mast height (m)
const COL_MATCHED = new THREE.Color('#27c4c4');   // sits on a real twin intersection
const COL_UNMATCHED = new THREE.Color('#e0a83a'); // placed at its own GPS (highway ramp etc.)

// Resolve a data path against the page origin and refuse anything that would leave it.
// `dataDir` can originate from the ?data= query param (see app.js DATA_DIR), so this
// keeps that user-controlled value from redirecting a fetch off-origin — i.e. it can
// only ever load same-origin data files (CodeQL js/client-side-request-forgery).
function sameOriginPath(path) {
  const u = new URL(path, window.location.href);
  if (u.origin !== window.location.origin) throw new Error('blocked cross-origin data path: ' + path);
  return u.pathname + u.search;
}

export function createCameraSystem(deps = {}) {
  const {
    scene, dataDir = 'data/', proxyBase = '', groundY: cityGroundY = 285,
    groups = {}, pollMs = 30000,
  } = deps;

  const group = new THREE.Group(); group.name = 'cameras';
  const layers = { markers: new THREE.Group() };
  layers.markers.name = 'camera-markers';
  group.add(layers.markers);

  const stats = { cameras: 0, matched: 0 };
  const cams = [];                  // static records (id, name, pos, intersection, ...)
  const byId = new Map();           // id -> record
  const streams = new Map();        // id -> { hls, dash, still } (live, from proxy)
  let housings = null, poles = null;
  let refGroundY = cityGroundY;
  const status = { proxy: 'connecting', static: 'loading', error: null, cameras: 0 };

  // ---- terrain heightmap drape (O(1) grid lookup; same trick as transit.js) ----
  const CELL = 8;
  let _hm = null, _hmTiles = -1;
  function heightmap() {
    const terr = groups.terrain;
    const n = terr ? terr.children.length : 0;
    if (n === _hmTiles) return _hm;
    _hmTiles = n;
    if (!n) { _hm = null; return null; }
    let minX = Infinity, minZ = Infinity, maxX = -Infinity, maxZ = -Infinity;
    for (const m of terr.children) {
      const bb = m.geometry && m.geometry.boundingBox;
      if (!bb) continue;
      minX = Math.min(minX, bb.min.x); minZ = Math.min(minZ, bb.min.z);
      maxX = Math.max(maxX, bb.max.x); maxZ = Math.max(maxZ, bb.max.z);
    }
    if (!isFinite(minX)) { _hm = null; return null; }
    const cols = Math.ceil((maxX - minX) / CELL) + 1, rows = Math.ceil((maxZ - minZ) / CELL) + 1;
    const y = new Float32Array(cols * rows).fill(NaN);
    for (const m of terr.children) {
      const pos = m.geometry && m.geometry.getAttribute('position');
      if (!pos) continue;
      const a = pos.array;
      for (let i = 0; i < a.length; i += 3) {
        const cx = ((a[i] - minX) / CELL) | 0, cz = ((a[i + 2] - minZ) / CELL) | 0;
        if (cx < 0 || cz < 0 || cx >= cols || cz >= rows) continue;
        const idx = cz * cols + cx;
        if (Number.isNaN(y[idx]) || a[i + 1] > y[idx]) y[idx] = a[i + 1];
      }
    }
    _hm = { minX, minZ, cols, rows, y };
    return _hm;
  }
  function groundY(x, z) {
    const hm = heightmap();
    if (!hm) return null;
    const cx = ((x - hm.minX) / CELL) | 0, cz = ((z - hm.minZ) / CELL) | 0;
    if (cx < 0 || cz < 0 || cx >= hm.cols || cz >= hm.rows) return null;
    const v = hm.y[cz * hm.cols + cx];
    return Number.isNaN(v) ? null : v;
  }

  // ---------------------------------------------------------- static load ---
  async function loadStatic() {
    let data;
    try {
      const r = await fetch(sameOriginPath(dataDir + 'cameras.json'), { cache: 'no-cache' });
      if (!r.ok) throw new Error('HTTP ' + r.status);
      data = await r.json();
    } catch (e) {
      status.static = 'none';
      return; // no markers; live URLs still resolve by id if asked
    }
    buildMarkers(data.cameras || []);
    status.static = 'ok';
  }

  // One InstancedMesh for the masts + one for the housings = two draw calls for every
  // camera. The housing is the pick target (raycast -> instanceId -> camera).
  function buildMarkers(list) {
    stats.cameras = list.length;
    if (!list.length) return;
    const poleGeo = new THREE.CylinderGeometry(0.1, 0.12, POLE_H, 6);
    const poleMat = new THREE.MeshStandardMaterial({ color: 0x8a939d, roughness: 0.7, metalness: 0.4 });
    // a little CCTV bullet: a box body with a short snout, merged into one geometry
    const body = new THREE.BoxGeometry(1.0, 0.7, 0.7);
    const housGeo = body;
    const housMat = new THREE.MeshStandardMaterial({
      roughness: 0.4, metalness: 0.3, emissive: 0x101418, emissiveIntensity: 0.5,
    });
    poles = new THREE.InstancedMesh(poleGeo, poleMat, list.length);
    housings = new THREE.InstancedMesh(housGeo, housMat, list.length);
    poles.name = 'camera-poles'; housings.name = 'camera-housings';
    housings.userData.pick = true;   // app.js click handler looks for this
    const m = new THREE.Matrix4(), q = new THREE.Quaternion(), s = new THREE.Vector3(1, 1, 1), p = new THREE.Vector3();
    list.forEach((c, i) => {
      cams.push(c); byId.set(c.id, c);
      housings.setColorAt(i, c.matched ? COL_MATCHED : COL_UNMATCHED);
      placeInstance(i, c.pos[0], c.pos[1], c.pos[2], m, q, s, p);
    });
    stats.matched = list.filter((c) => c.matched).length;
    poles.instanceMatrix.needsUpdate = true; housings.instanceMatrix.needsUpdate = true;
    if (housings.instanceColor) housings.instanceColor.needsUpdate = true;
    poles.computeBoundingSphere(); housings.computeBoundingSphere();
    poles.frustumCulled = false; housings.frustumCulled = false;
    layers.markers.add(poles, housings);
  }

  function placeInstance(i, x, baseY, z, m, q, s, p) {
    // drape on the terrain heightmap when available, else the baked elevation / city plane
    const gy = groundY(x, z);
    const ground = gy != null ? gy : (Number.isFinite(baseY) ? baseY : refGroundY);
    poles.setMatrixAt(i, m.compose(p.set(x, ground + POLE_H / 2, z), q, s));
    housings.setMatrixAt(i, m.compose(p.set(x, ground + POLE_H + MARK_LIFT, z), q, s));
  }

  // Re-drape only when the terrain tile count changes (tiles stream in after markers
  // are built). Cheap and rare; keeps the camera bullets sitting on the ground.
  let _drapedTiles = -2;
  function redrapeIfNeeded() {
    if (!housings) return;
    const terr = groups.terrain;
    const n = terr ? terr.children.length : 0;
    if (n === _drapedTiles) return;
    _drapedTiles = n;
    const m = new THREE.Matrix4(), q = new THREE.Quaternion(), s = new THREE.Vector3(1, 1, 1), p = new THREE.Vector3();
    cams.forEach((c, i) => placeInstance(i, c.pos[0], c.pos[1], c.pos[2], m, q, s, p));
    poles.instanceMatrix.needsUpdate = true; housings.instanceMatrix.needsUpdate = true;
    poles.computeBoundingSphere(); housings.computeBoundingSphere();
  }

  // ------------------------------------------------------------- polling ---
  let pollTimer = null, stopped = false;
  const RECONNECT_MS = 30000;
  async function pollStreams() {
    if (stopped) return;
    let okProxy = false;
    try {
      const r = await fetch(proxyBase + '/api/cameras/streams', { cache: 'no-store' });
      if (!r.ok) throw new Error('HTTP ' + r.status);
      const payload = await r.json();
      const map = payload.cams || {};
      for (const id of Object.keys(map)) streams.set(id, map[id]);
      status.proxy = payload.error ? 'error' : 'ok';
      status.error = payload.error || null;
      status.cameras = payload.count || streams.size;
      okProxy = !payload.error;
    } catch (e) {
      status.proxy = 'offline';
      status.error = 'live streams offline — run: python -m tools.twin_server';
    } finally {
      if (!stopped) pollTimer = setTimeout(pollStreams, okProxy ? pollMs : RECONNECT_MS);
    }
  }

  // ------------------------------------------------------------- per frame ---
  function tick() {
    redrapeIfNeeded();
  }

  // ------------------------------------------------------- query API ---
  function rec(c) {
    return c ? {
      id: c.id, name: c.name, position: c.pos, lat: c.lat, lon: c.lon,
      intersection: c.intersection, snapDist: c.snapDist, matched: c.matched,
    } : null;
  }

  const cameras = {
    group, layers, stats, tick,
    start() { stopped = false; if (!pollTimer) pollStreams(); return this; },
    stop() { stopped = true; clearTimeout(pollTimer); pollTimer = null; return this; },
    status: () => ({ ...status, cameras: stats.cameras, matched: stats.matched }),

    list() { return cams.map(rec); },
    get(id) { return rec(byId.get(String(id))); },
    byInstance(i) { return rec(cams[i]); },
    getNearest(pos, maxR = Infinity) {
      let best = null, bd = maxR * maxR;
      const pz = (pos.length === 2 ? pos[1] : pos[2]);
      for (const c of cams) {
        const dx = c.pos[0] - pos[0], dz = c.pos[2] - pz;
        const d = dx * dx + dz * dz;
        if (d < bd) { bd = d; best = c; }
      }
      return best ? { ...rec(best), distance: Math.sqrt(bd) } : null;
    },

    // freshest live HLS URL for a camera (null if the proxy hasn't answered yet)
    streamUrl(id) { const s = streams.get(String(id)); return (s && s.hls) || null; },
    dashUrl(id) { const s = streams.get(String(id)); return (s && s.dash) || null; },
    // token-free live snapshot — baked into cameras.json, so it works with no proxy.
    // The map publishes it at 352x240; request 960x720 (the live stream's native size,
    // which the thumbnail endpoint honours) so the no-proxy fallback isn't a soft
    // upscale in the enlarged PiP panel.
    still(id) {
      const s = streams.get(String(id));
      const url = (s && s.still) || (byId.get(String(id)) || {}).still || null;
      return url ? url.replace(/([?&]size=)\d+x\d+/i, '$1960x720') : null;
    },
    hasProxy: () => status.proxy === 'ok',
  };

  loadStatic().finally(() => cameras.start());
  return { group, layers, stats, cameras };
}
