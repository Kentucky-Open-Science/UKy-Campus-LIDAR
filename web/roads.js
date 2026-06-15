// Real road network layer for the UKy Campus viewer.
//
// Consumes web/data/roads.json (road centrelines extracted from the aerial
// textures by tools/extract_roads.py — see that file). Points are already in
// three.js scene metres as [x, y, z] with y draped onto the terrain elevation,
// so there is no runtime raycasting: we build raised asphalt ribbons that follow
// the real streets and hills, dashed lane markings, intersection pads, and then
// re-anchor trees / parked cars / mast-arm traffic signals onto that network.
//
// createRoadNetwork(data, opts) -> { group, layers, stats }
//   layers = { roads, markings, trees, cars, signals }  (toggleable sub-groups)

import * as THREE from 'three';

const LIFT = 0.3;            // raise asphalt above terrain to avoid z-fighting (m)
const MARK_LIFT = LIFT + 0.05;

const CAR_COLORS = [
  0xb5352e, 0x2e4f8c, 0xdedede, 0x2b2b2f, 0x3c7a4a,
  0xc7a13a, 0x7a7d82, 0x884ea0, 0x2f8f9d, 0xd0822a,
];

// ---- scratch + helpers ----
const _pos = new THREE.Vector3(), _scl = new THREE.Vector3();
const _quat = new THREE.Quaternion(), _euler = new THREE.Euler();
const _white = new THREE.Color(1, 1, 1);

function mat4(px, py, pz, yaw, sx, sy, sz) {
  _pos.set(px, py, pz);
  _quat.setFromEuler(_euler.set(0, yaw, 0));
  _scl.set(sx, sy, sz);
  return new THREE.Matrix4().compose(_pos, _quat, _scl);
}
// object local +X -> world XZ direction d under three's Y-rotation
const yawFor = (dx, dz) => Math.atan2(-dz, dx);

class InstanceSet {
  constructor(geo, material) { this.geo = geo; this.material = material; this.m = []; this.c = []; }
  add(matrix, color) { this.m.push(matrix); this.c.push(color || null); }
  build(name) {
    const n = this.m.length;
    if (!n) return null;
    const mesh = new THREE.InstancedMesh(this.geo, this.material, n);
    mesh.name = name;
    for (let i = 0; i < n; i++) mesh.setMatrixAt(i, this.m[i]);
    mesh.instanceMatrix.needsUpdate = true;
    if (this.c.some((c) => c)) {
      for (let i = 0; i < n; i++) mesh.setColorAt(i, this.c[i] || _white);
      if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    }
    mesh.computeBoundingSphere();
    return mesh;
  }
}

// Continuous arc-length walk of a polyline: emit fn(point, dirXZ, yaw) once per
// `spacing` metres along the WHOLE line (residual carried across segment joins,
// so shared vertices are never sampled twice and the spacing is honoured).
function alongPolyline(pts, spacing, fn) {
  if (!(spacing > 0)) return; // guard against a non-advancing (infinite) walk
  let next = 0; // arc-length of the next sample, relative to current segment start
  for (let i = 0; i < pts.length - 1; i++) {
    const a = new THREE.Vector3(pts[i][0], pts[i][1], pts[i][2]);
    const b = new THREE.Vector3(pts[i + 1][0], pts[i + 1][1], pts[i + 1][2]);
    const seg = b.clone().sub(a);
    const len = Math.hypot(seg.x, seg.z);
    if (len < 1e-6) continue;
    const dir = new THREE.Vector3(seg.x, 0, seg.z).normalize();
    const yaw = yawFor(dir.x, dir.z);
    for (; next < len; next += spacing) {
      fn(a.clone().lerp(b, next / len), dir, yaw);
    }
    next -= len; // carry remaining distance into the next segment
  }
}

export function createRoadNetwork(data, opts = {}) {
  const o = {
    trees: true, cars: true, signals: true,
    treeSpacing: 16, treeChance: 0.55, treeOffset: 4,
    carSpacing: 13, carChance: 0.22,
    signalSpacing: 70, signalCap: 80,
    ...opts,
  };
  const roads = data.roads || [];
  const intersections = data.intersections || [];

  const group = new THREE.Group(); group.name = 'roadnet';
  const layers = {
    roads: new THREE.Group(), markings: new THREE.Group(),
    trees: new THREE.Group(), cars: new THREE.Group(), signals: new THREE.Group(),
  };
  for (const k of Object.keys(layers)) layers[k].name = 'road-' + k;
  group.add(layers.roads, layers.markings, layers.trees, layers.cars, layers.signals);

  // ---- shared materials / geometries ----
  const mat = {
    asphalt: new THREE.MeshStandardMaterial({ color: 0x2c2f34, roughness: 0.95, side: THREE.DoubleSide, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 }),
    lane: new THREE.MeshStandardMaterial({ color: 0xd9c24c, roughness: 0.7 }),
    trunk: new THREE.MeshStandardMaterial({ color: 0x5b4329, roughness: 0.95 }),
    foliage: new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.9 }),
    carBody: new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.45, metalness: 0.45 }),
    carCabin: new THREE.MeshStandardMaterial({ color: 0x1c2228, roughness: 0.25, metalness: 0.6 }),
    tyre: new THREE.MeshStandardMaterial({ color: 0x111316, roughness: 0.9 }),
    metal: new THREE.MeshStandardMaterial({ color: 0x33383d, roughness: 0.6, metalness: 0.7 }),
    light: new THREE.MeshBasicMaterial({ color: 0xffffff }),
  };
  const box = new THREE.BoxGeometry(1, 1, 1);
  const trunkGeo = new THREE.CylinderGeometry(0.16, 0.22, 1, 6);
  const ballGeo = new THREE.IcosahedronGeometry(1, 1);
  const coneGeo = new THREE.ConeGeometry(1, 1, 8);
  const wheelGeo = new THREE.CylinderGeometry(0.34, 0.34, 0.26, 12); wheelGeo.rotateX(Math.PI / 2);
  const poleGeo = new THREE.CylinderGeometry(0.12, 0.14, 1, 8);
  const lampGeo = new THREE.SphereGeometry(0.16, 10, 8);

  const stats = { roads: roads.length, intersections: intersections.length, km: 0, trees: 0, cars: 0, signals: 0 };

  // ---------- asphalt ribbons (one merged geometry) ----------
  const pos = [], idx = [];
  for (const road of roads) {
    const pts = road.pts;
    const n = pts.length;
    if (n < 2) continue;
    const half = (road.width || 6) / 2;
    const P = pts.map((p) => new THREE.Vector3(p[0], p[1], p[2]));
    for (let i = 0; i + 1 < n; i++) stats.km += P[i].distanceTo(P[i + 1]) / 1000;
    // per-vertex XZ perpendicular (mitre of adjacent segment directions) with
    // mitre-length compensation so the ribbon keeps constant width through bends.
    const perp = [], mf = [];
    for (let i = 0; i < n; i++) {
      let adir = null, bdir = null;
      if (i > 0) { const a = P[i].clone().sub(P[i - 1]); a.y = 0; if (a.lengthSq() > 1e-9) adir = a.normalize(); }
      if (i < n - 1) { const b = P[i + 1].clone().sub(P[i]); b.y = 0; if (b.lengthSq() > 1e-9) bdir = b.normalize(); }
      const d = new THREE.Vector3();
      if (adir) d.add(adir);
      if (bdir) d.add(bdir);
      if (d.lengthSq() < 1e-9) d.set(1, 0, 0);
      d.normalize();
      perp.push(new THREE.Vector3(-d.z, 0, d.x));
      const ref = bdir || adir;
      mf.push(ref ? 1 / Math.max(0.28, Math.abs(d.dot(ref))) : 1); // 1/cos(theta/2), miter-limited
    }
    const start = pos.length / 3;
    for (let i = 0; i < n; i++) {
      const w = half * mf[i];
      const L = P[i].clone().addScaledVector(perp[i], w); L.y += LIFT;
      const R = P[i].clone().addScaledVector(perp[i], -w); R.y += LIFT;
      pos.push(L.x, L.y, L.z, R.x, R.y, R.z);
    }
    for (let i = 0; i + 1 < n; i++) {
      const a = start + i * 2;
      idx.push(a, a + 2, a + 1, a + 1, a + 2, a + 3);
    }
  }
  if (pos.length) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(pos), 3));
    geo.setIndex(idx);
    geo.computeVertexNormals();
    geo.computeBoundingSphere();
    const mesh = new THREE.Mesh(geo, mat.asphalt);
    mesh.name = 'road-ribbons';
    layers.roads.add(mesh);
  }

  // ---------- intersection pads ----------
  const pads = new InstanceSet(box, mat.asphalt);
  for (const p of intersections) {
    pads.add(mat4(p[0], p[1] + LIFT - 0.02, p[2], 0, 11, 0.1, 11));
  }
  addTo(layers.roads, pads.build('intersection-pads'));

  // ---------- dashed lane markings ----------
  const dashes = new InstanceSet(box, mat.lane);
  for (const road of roads) {
    if ((road.width || 0) < 7) continue; // only mark the wider streets
    alongPolyline(road.pts, 5.2, (p, dir, yaw) => {
      dashes.add(mat4(p.x, p.y + MARK_LIFT, p.z, yaw, 2.4, 0.02, 0.22));
    });
  }
  addTo(layers.markings, dashes.build('lane-dashes'));

  // ---------- street trees along the curbs ----------
  if (o.trees) {
  const trunks = new InstanceSet(trunkGeo, mat.trunk);
  const balls = new InstanceSet(ballGeo, mat.foliage);
  const cones = new InstanceSet(coneGeo, mat.foliage);
  for (const road of roads) {
    const half = (road.width || 6) / 2;
    alongPolyline(road.pts, o.treeSpacing, (p, dir) => {
      const perp = new THREE.Vector3(-dir.z, 0, dir.x);
      for (const side of [1, -1]) {
        if (Math.random() > o.treeChance) continue;
        const c = p.clone().addScaledVector(perp, side * (half + o.treeOffset));
        addTree(trunks, balls, cones, c, stats);
      }
    });
  }
  addTo(layers.trees, trunks.build('tree-trunks'));
  addTo(layers.trees, balls.build('tree-deciduous'));
  addTo(layers.trees, cones.build('tree-pine'));
  }

  // ---------- parked cars near the curbs ----------
  if (o.cars) {
  const bodies = new InstanceSet(box, mat.carBody);
  const cabins = new InstanceSet(box, mat.carCabin);
  const wheels = new InstanceSet(wheelGeo, mat.tyre);
  for (const road of roads) {
    if ((road.width || 0) < 7) continue;
    const half = (road.width || 6) / 2;
    alongPolyline(road.pts, o.carSpacing, (p, dir, yaw) => {
      const perp = new THREE.Vector3(-dir.z, 0, dir.x);
      for (const side of [1, -1]) {
        if (Math.random() > o.carChance) continue;
        const c = p.clone().addScaledVector(perp, side * (half - 1.4));
        addCar(bodies, cabins, wheels, c, yaw, stats);
      }
    });
  }
  addTo(layers.cars, bodies.build('car-bodies'));
  addTo(layers.cars, cabins.build('car-cabins'));
  addTo(layers.cars, wheels.build('car-wheels'));
  }

  // ---------- mast-arm traffic signals at major intersections ----------
  if (o.signals) {
  const chosen = [];
  for (const p of intersections) {
    if (chosen.length >= o.signalCap) break;
    if (chosen.every((q) => (p[0] - q[0]) ** 2 + (p[2] - q[2]) ** 2 > o.signalSpacing ** 2)) chosen.push(p);
  }
  const poles = new InstanceSet(poleGeo, mat.metal);
  const arms = new InstanceSet(box, mat.metal);
  const heads = new InstanceSet(box, mat.metal);
  const lamps = new InstanceSet(lampGeo, mat.light);
  for (const p of chosen) { addSignal(poles, arms, heads, lamps, p, stats); }
  addTo(layers.signals, poles.build('signal-poles'));
  addTo(layers.signals, arms.build('signal-arms'));
  addTo(layers.signals, heads.build('signal-heads'));
  addTo(layers.signals, lamps.build('signal-lamps'));
  }

  stats.km = Math.round(stats.km * 10) / 10;
  return { group, layers, stats };

  // ---------- per-object builders (closures over geometry sizes) ----------
  function addTree(trunks, balls, cones, c, stats) {
    const ht = 2.6 + Math.random() * 2.4, ts = 0.85 + Math.random() * 0.5;
    trunks.add(mat4(c.x, c.y + ht / 2, c.z, Math.random() * 6.28, ts, ht, ts));
    const rf = 1.5 + Math.random() * 1.4;
    const green = new THREE.Color().setHSL(0.27 + Math.random() * 0.08, 0.45 + Math.random() * 0.2, 0.3 + Math.random() * 0.12);
    if (Math.random() < 0.35) {
      const hc = rf * 2.6;
      cones.add(mat4(c.x, c.y + ht + hc / 2 - 0.4, c.z, Math.random() * 6.28, rf, hc, rf), green);
    } else {
      balls.add(mat4(c.x, c.y + ht + rf * 0.55, c.z, Math.random() * 6.28, rf, rf * 1.05, rf), green);
    }
    stats.trees++;
  }

  function addCar(bodies, cabins, wheels, c, yaw, stats) {
    const color = new THREE.Color(CAR_COLORS[(Math.random() * CAR_COLORS.length) | 0]);
    const car = new THREE.Matrix4().compose(
      _pos.set(c.x, c.y + 0.05, c.z), _quat.setFromEuler(_euler.set(0, yaw, 0)), _scl.set(1, 1, 1));
    const place = (set, lx, ly, lz, sx, sy, sz, col) => {
      const local = new THREE.Matrix4().compose(
        new THREE.Vector3(lx, ly, lz), new THREE.Quaternion(), new THREE.Vector3(sx, sy, sz));
      set.add(new THREE.Matrix4().multiplyMatrices(car, local), col);
    };
    place(bodies, 0, 0.85, 0, 4.0, 0.7, 1.75, color);
    place(cabins, -0.25, 1.45, 0, 2.1, 0.62, 1.55, null);
    for (const sx of [1.28, -1.28]) for (const sz of [0.82, -0.82]) place(wheels, sx, 0.34, sz, 1, 1, 1, null);
    stats.cars++;
  }

  function addSignal(poles, arms, heads, lamps, p, stats) {
    const base = new THREE.Matrix4().compose(
      _pos.set(p[0], p[1], p[2]), _quat.setFromEuler(_euler.set(0, Math.random() * 6.28, 0)), _scl.set(1, 1, 1));
    const local = (lx, ly, lz, sx, sy, sz) => new THREE.Matrix4().multiplyMatrices(
      base, new THREE.Matrix4().compose(new THREE.Vector3(lx, ly, lz), new THREE.Quaternion(), new THREE.Vector3(sx, sy, sz)));
    const poleH = 5.2, armLen = 6.0, armY = poleH - 0.2;
    poles.add(local(0, poleH / 2, 0, 1, poleH, 1));
    arms.add(local(armLen / 2, armY, 0, armLen, 0.16, 0.16));
    const hx = armLen - 0.25, hy = armY - 0.75;
    heads.add(local(hx, hy, 0, 0.42, 1.35, 0.42));
    lamps.add(local(hx + 0.16, hy + 0.42, 0, 1, 1, 1), new THREE.Color(0xff2a22));
    lamps.add(local(hx + 0.16, hy, 0, 1, 1, 1), new THREE.Color(0xffb01f));
    lamps.add(local(hx + 0.16, hy - 0.42, 0, 1, 1, 1), new THREE.Color(0x2ad24a));
    stats.signals++;
  }
}

function addTo(parent, mesh) { if (mesh) parent.add(mesh); }
