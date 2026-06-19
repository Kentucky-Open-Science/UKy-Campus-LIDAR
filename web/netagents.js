// Shared-world agent layer for the campus digital twin.
//
// When the viewer is served by the authoritative twin server (tools/twin_server.py),
// this polls /api/world/state and renders EVERY agent in the shared world — including
// ones spawned by other people's scripts (client/twin.py) — so a browser is a live
// window onto the same world the scripts drive. Positions are interpolated between
// polls and labelled with type/id/owner; a contact flashes the agent red.
//
// This is distinct from agents.js (window.__twin.agents), which is a single browser's
// PRIVATE sim. netagents is the SHARED one. Degrades silently to nothing when the
// page isn't served by the twin server (the endpoint 404s, then it backs off).
//
// createNetAgents(deps) -> { group, tick(dt), setVisible(b), status(), count }

import * as THREE from 'three';
import { FLAT_WORLD, FLAT_Y } from './flat.js';

const D2R = Math.PI / 180;
// The server computes y from the real terrain; in flat mode the whole viewer is pinned
// to FLAT_Y, so ground the shared-world agents (incl. kinematic camera cars) on it,
// otherwise they'd float/sink relative to the flat roads.
const groundOf = (p) => (FLAT_WORLD ? FLAT_Y : p[1]);
// [length(+X), height(+Y), width(+Z)] per type, matching the server's kinematic defs
const DIMS = { car: [4.3, 1.45, 1.9], truck: [8.5, 3.2, 2.5], robot: [0.8, 0.6, 0.6], drone: [0.9, 0.35, 0.9] };

function shortestAngle(a, b) { let d = (b - a) % (2 * Math.PI);
  if (d > Math.PI) d -= 2 * Math.PI; if (d < -Math.PI) d += 2 * Math.PI; return d; }

export function createNetAgents(deps = {}) {
  const { scene, base = '', pollMs = 120 } = deps;
  const group = new THREE.Group(); group.name = 'netagents';
  const agents = new Map();                 // id -> { object, body, mat, sprite, cur, tgt, yaw, tgtYaw, hit, data }
  const status = { server: 'connecting', count: 0, t: 0 };
  let stopped = false, pollTimer = null;
  const RECONNECT_MS = 4000;

  function dims(type) { return DIMS[type] || [2, 1, 2]; }

  function labelSprite(text) {
    const c = document.createElement('canvas');
    const g = c.getContext('2d');
    g.font = '600 30px system-ui, sans-serif';
    const w = Math.ceil(g.measureText(text).width) + 18, h = 42;
    c.width = w; c.height = h;
    g.font = '600 30px system-ui, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
    g.lineWidth = 5; g.strokeStyle = 'rgba(0,0,0,0.8)'; g.strokeText(text, w / 2, h / 2);
    g.fillStyle = '#fff'; g.fillText(text, w / 2, h / 2);
    const tex = new THREE.CanvasTexture(c); tex.anisotropy = 4;
    const spr = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: true }));
    spr.scale.set(6 * (w / h), 6, 1);
    return spr;
  }

  function spawn(a) {
    const [L, H, W] = dims(a.type);
    const root = new THREE.Group(); root.name = 'net-' + a.id;
    const mat = new THREE.MeshStandardMaterial({ color: a.color != null ? a.color : 0x3577c9,
      roughness: 0.5, metalness: 0.35, emissive: 0x000000 });
    const body = new THREE.Mesh(new THREE.BoxGeometry(L, H, W), mat);
    body.position.y = H / 2;
    root.add(body);
    // forward heading marker (+X)
    const arrow = new THREE.Mesh(new THREE.ConeGeometry(0.22, 0.7, 8),
      new THREE.MeshStandardMaterial({ color: 0xffe14d }));
    arrow.rotation.z = -Math.PI / 2; arrow.position.set(L / 2 + 0.35, H / 2, 0);
    root.add(arrow);
    const sprite = labelSprite(`${a.type} #${a.id} ${a.owner || ''}`.trim());
    sprite.position.set(0, H + 2.2, 0);
    root.add(sprite);
    group.add(root);
    const p = a.position;
    const gy = groundOf(p);
    const obj = { object: root, body, mat, sprite, data: a,
      cur: new THREE.Vector3(p[0], gy, p[2]), tgt: new THREE.Vector3(p[0], gy, p[2]),
      yaw: (a.heading || 0) * D2R, tgtYaw: (a.heading || 0) * D2R };
    root.position.copy(obj.cur);
    agents.set(a.id, obj);
    return obj;
  }

  function ingest(snapshot) {
    const seen = new Set();
    for (const a of snapshot.agents || []) {
      seen.add(a.id);
      let o = agents.get(a.id);
      if (!o) o = spawn(a);
      o.data = a;
      o.tgt.set(a.position[0], groundOf(a.position), a.position[2]);
      o.tgtYaw = (a.heading || 0) * D2R;
      const hit = (a.collisions && a.collisions.length) > 0;
      o.mat.emissive.setHex(hit ? 0x661111 : 0x000000);
      o.mat.emissiveIntensity = hit ? 1 : 0;
    }
    for (const [id, o] of agents) {            // remove agents that left the world
      if (!seen.has(id)) {
        group.remove(o.object);
        o.mat.dispose();
        o.sprite.material.map.dispose(); o.sprite.material.dispose();
        agents.delete(id);
      }
    }
    status.count = agents.size;
    status.t = snapshot.t || 0;
  }

  async function poll() {
    if (stopped) return;
    let ok = false;
    try {
      const r = await fetch(base + '/api/world/state', { cache: 'no-store' });
      if (!r.ok) throw new Error('HTTP ' + r.status);
      ingest(await r.json());
      status.server = 'ok'; ok = true;
    } catch (e) {
      status.server = 'offline';   // not served by the twin server (or it's down)
    } finally {
      if (!stopped) pollTimer = setTimeout(poll, ok ? pollMs : RECONNECT_MS);
    }
  }

  function tick(dt) {
    if (!agents.size) return;
    const k = 1 - Math.exp(-12 * Math.min(dt, 0.1));   // snappy follow (server is authoritative)
    for (const o of agents.values()) {
      o.cur.lerp(o.tgt, k);
      o.object.position.copy(o.cur);
      o.yaw += shortestAngle(o.yaw, o.tgtYaw) * k;
      o.object.rotation.y = o.yaw;
    }
  }

  poll();   // start polling immediately; backs off if there's no twin server
  return {
    group, tick,
    setVisible(b) { group.visible = !!b; },
    status: () => ({ ...status }),
    get count() { return agents.size; },
    stop() { stopped = true; clearTimeout(pollTimer); },
    // shared-world agents for the panel list + follow camera
    list() {
      return [...agents.values()].map((o) => ({
        id: o.data.id, type: o.data.type, owner: o.data.owner, color: o.data.color,
        position: [o.object.position.x, o.object.position.y, o.object.position.z],
      })).sort((a, b) => a.id - b.id);
    },
    get(id) {
      const o = agents.get(Number(id)) || agents.get(id);
      if (!o) return null;
      const p = o.object.position;
      return { id: o.data.id, type: o.data.type, owner: o.data.owner,
               position: [p.x, p.y, p.z], heading: ((o.yaw * 180 / Math.PI) % 360 + 360) % 360 };
    },
  };
}
