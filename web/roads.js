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
const _tmpcol = new THREE.Color();   // scratch for live lamp recolouring

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
    this.mesh = mesh;     // kept so the signal controller can recolour lamps in place
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
    signalModel: null,   // parsed signals.json -> real signalised intersections + agent API
    ...opts,
  };
  const roads = data.roads || [];
  const intersections = data.intersections || [];
  const sig = o.signalModel && o.signalModel.intersections ? o.signalModel : null;

  const group = new THREE.Group(); group.name = 'roadnet';
  const layers = {
    roads: new THREE.Group(), markings: new THREE.Group(),
    crosswalks: new THREE.Group(),
    trees: new THREE.Group(), cars: new THREE.Group(), signals: new THREE.Group(),
  };
  for (const k of Object.keys(layers)) layers[k].name = 'road-' + k;
  group.add(layers.roads, layers.markings, layers.crosswalks,
            layers.trees, layers.cars, layers.signals);

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
    paint: new THREE.MeshStandardMaterial({ color: 0xf2f2f2, roughness: 0.6, polygonOffset: true, polygonOffsetFactor: -3, polygonOffsetUnits: -3 }),
    signRed: new THREE.MeshStandardMaterial({ color: 0xc02626, roughness: 0.5, side: THREE.DoubleSide }),
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
  // The crossing road ribbons already cover the junction, but acute / multi-leg
  // junctions can leave small wedges. Fill each one with an ORIENTED convex polygon
  // (the hull of where the approaches meet the box) rather than an axis-aligned
  // square — a square spills its corners onto the grass and cuts across the
  // crosswalks. Skipped without a signal model (no per-leg geometry to fit).
  if (sig) {
    const ppos = [], pidx = [];
    for (const it of sig.intersections) {
      const corners = [];
      for (const leg of it.legs) {
        const b = leg.bearingDeg * (Math.PI / 180);
        const ox = Math.cos(b), oz = Math.sin(b), px = -Math.sin(b), pz = Math.cos(b);
        const hw = (leg.width || 7) / 2, r = (it.footprintRadius || 9) + 0.5;
        corners.push([it.center[0] + ox * r + px * hw, it.center[2] + oz * r + pz * hw]);
        corners.push([it.center[0] + ox * r - px * hw, it.center[2] + oz * r - pz * hw]);
      }
      const hull = convexHull2D(corners);
      if (hull.length < 3) continue;
      const cy = it.center[1] + LIFT - 0.02;
      let ccx = 0, ccz = 0; for (const p of hull) { ccx += p[0]; ccz += p[1]; }
      ccx /= hull.length; ccz /= hull.length;
      const base = ppos.length / 3;
      ppos.push(ccx, cy, ccz);                       // centroid, then fan to the hull
      for (const p of hull) ppos.push(p[0], cy, p[1]);
      for (let i = 0; i < hull.length; i++) pidx.push(base, base + 1 + i, base + 1 + ((i + 1) % hull.length));
    }
    if (ppos.length) {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(ppos), 3));
      geo.setIndex(pidx); geo.computeVertexNormals(); geo.computeBoundingSphere();
      const mesh = new THREE.Mesh(geo, mat.asphalt); mesh.name = 'intersection-pads';
      layers.roads.add(mesh);
    }
  }

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

  // ---------- traffic signals / stop signs / crosswalks ----------
  // With a signals.json model: real per-approach signalised intersections with
  // oriented R/Y/G heads, pedestrian walk/don't-walk heads, crosswalks, stop bars,
  // stop signs, and a live signal-state controller for autonomous agents. Without
  // it: the legacy random-mast fallback so the viewer still works standalone.
  let signalController = null;
  if (sig) {
    signalController = buildSignalSystem(sig);
  } else if (o.signals) {
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
  return { group, layers, stats, signals: signalController };

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

  // ------------------------------------------------------------------------
  // Real signalised intersections from signals.json. Builds oriented per-approach
  // fixtures (vehicle R/Y/G heads facing oncoming traffic, pedestrian walk/don't
  // heads, crosswalk stripes, stop bars, stop signs) and returns a live controller
  // that ticks a deterministic fixed-time phase plan and exposes a query/override
  // API for autonomous agents (see web/README.md "signals.json").
  function buildSignalSystem(model) {
    const poles = new InstanceSet(poleGeo, mat.metal);
    const arms = new InstanceSet(box, mat.metal);
    const heads = new InstanceSet(box, mat.metal);
    const vlamps = new InstanceSet(lampGeo, mat.light);     // vehicle R/Y/G bulbs
    const pheads = new InstanceSet(box, mat.metal);
    const plamps = new InstanceSet(box, mat.light);         // ped walk/don't bulbs
    const signs = new InstanceSet(box, mat.signRed);
    const signPoles = new InstanceSet(poleGeo, mat.metal);
    const bars = new InstanceSet(box, mat.paint);           // stop bars (road paint)
    const stripes = new InstanceSet(box, mat.paint);        // crosswalk stripes

    const vehReg = [];   // { index, intId, group, aspect:'R'|'Y'|'G', on, off }
    const pedReg = [];   // { index, intId, group, kind:'walk'|'dont', on, off }
    const RED = 0xff2a22, YEL = 0xffb01f, GRN = 0x2ad24a;
    const RED_D = 0x401010, YEL_D = 0x402c08, GRN_D = 0x0e3a1c;
    const WALK = 0xeafff0, WALK_D = 0x16361f, DONT = 0xff8a2a, DONT_D = 0x3a1e08;
    const D2R = Math.PI / 180;
    let nSignals = 0;

    for (const it of model.intersections) {
      const cx = it.center[0], cz = it.center[2];
      const footR = it.footprintRadius || 9;
      if (it.control === 'signal') nSignals++;
      for (const leg of it.legs) {
        const b = leg.bearingDeg * D2R;
        const ox = Math.cos(b), oz = Math.sin(b);   // outward (away from centre)
        const px = -Math.sin(b), pz = Math.cos(b);  // left-perpendicular
        const hw = (leg.width || 7) / 2;
        const sy = leg.stopPoint ? leg.stopPoint[1] : it.center[1];
        // stop bar across the approach (signal + stop legs)
        if (leg.stopLine && (it.control === 'signal' || it.control === 'stop')) {
          const a = leg.stopLine[0], c = leg.stopLine[1];
          const dx = c[0] - a[0], dz = c[1] - a[1], len = Math.hypot(dx, dz) || 1;
          bars.add(mat4((a[0] + c[0]) / 2, sy + MARK_LIFT, (a[1] + c[1]) / 2,
                        yawFor(dx / len, dz / len), len, 0.02, 0.5));
        }
        if (it.control === 'signal' && leg.signalGroup) {
          // --- vehicle mast-arm head, oriented to face oncoming traffic ---
          const poleH = 6.0;
          const poleX = cx + ox * (footR + 3.5) + px * (hw + 1.2);
          const poleZ = cz + oz * (footR + 3.5) + pz * (hw + 1.2);
          poles.add(mat4(poleX, sy + poleH / 2, poleZ, 0, 1, poleH, 1));
          const armLen = hw + 1.4, ax = -px, az = -pz;   // arm reaches over the lane
          arms.add(mat4(poleX + ax * armLen / 2, sy + poleH - 0.3, poleZ + az * armLen / 2,
                        yawFor(ax, az), armLen, 0.16, 0.16));
          const hX = poleX + ax * armLen, hZ = poleZ + az * armLen, hY = sy + poleH - 0.95;
          heads.add(mat4(hX, hY, hZ, yawFor(ox, oz), 0.45, 1.35, 0.45));
          const lX = hX + ox * 0.28, lZ = hZ + oz * 0.28;   // bulbs face the driver (+out)
          for (const [aspect, on, off, dy] of [['R', RED, RED_D, 0.42], ['Y', YEL, YEL_D, 0], ['G', GRN, GRN_D, -0.42]]) {
            vehReg.push({ index: vlamps.m.length, intId: it.id, group: leg.signalGroup, aspect, on, off });
            vlamps.add(mat4(lX, hY + dy, lZ, 0, 1, 1, 1), new THREE.Color(off));
          }
          // --- pedestrian walk/don't head on the same pole, FACING ACROSS the crosswalk ---
          // Only where this leg actually has a crosswalk (big junctions mark only the
          // principal approaches). The head display face + bulbs point along -perp (toward
          // the crossing pedestrian); bulbs are pushed proud of the housing along that
          // normal and share its yaw, or they would be buried/edge-on.
          if (leg.pedSignal === false) continue;
          const pedY = sy + 2.7, pedYaw = yawFor(-px, -pz);
          const fX = poleX - px * 0.13, fZ = poleZ - pz * 0.13;   // proud of the 0.18-thin face
          pheads.add(mat4(poleX, pedY, poleZ, pedYaw, 0.18, 0.55, 0.5));
          pedReg.push({ index: plamps.m.length, intId: it.id, group: leg.signalGroup, kind: 'dont', on: DONT, off: DONT_D });
          plamps.add(mat4(fX, pedY + 0.13, fZ, pedYaw, 0.06, 0.2, 0.2), new THREE.Color(DONT_D));
          pedReg.push({ index: plamps.m.length, intId: it.id, group: leg.signalGroup, kind: 'walk', on: WALK, off: WALK_D });
          plamps.add(mat4(fX, pedY - 0.13, fZ, pedYaw, 0.06, 0.2, 0.2), new THREE.Color(WALK_D));
        } else if (leg.stopSign) {
          // --- stop sign on a short pole, octagon plate facing the driver ---
          const sX = cx + ox * (footR + 3.8) + px * (hw + 0.8);
          const sZ = cz + oz * (footR + 3.8) + pz * (hw + 0.8), poleH = 2.4;
          signPoles.add(mat4(sX, sy + poleH / 2, sZ, 0, 1, poleH, 1));
          signs.add(mat4(sX, sy + poleH + 0.05, sZ, yawFor(ox, oz), 0.07, 0.66, 0.66));
        }
      }
      // crosswalk stripes (continental bars, parallel to travel, across the road)
      for (const xw of it.crosswalks || []) {
        const leg = it.legs[xw.legIdx];
        if (!leg || !xw.polygon) continue;
        let mx = 0, mz = 0;
        for (const q of xw.polygon) { mx += q[0]; mz += q[1]; }
        mx /= xw.polygon.length; mz /= xw.polygon.length;
        const b = leg.bearingDeg * D2R, ox = Math.cos(b), oz = Math.sin(b);
        const px = -Math.sin(b), pz = Math.cos(b), hw = (leg.width || 7) / 2;
        // crosswalk y is draped at the crosswalk's own ground (falls back to stop point)
        const sy = (xw.y != null) ? xw.y : (leg.stopPoint ? leg.stopPoint[1] : it.center[1]);
        const n = Math.max(4, Math.min(12, Math.floor((2 * hw) / 0.9)));
        for (let k = 0; k < n; k++) {
          const off = -hw + 0.5 + k * (2 * hw - 1.0) / (n - 1);
          stripes.add(mat4(mx + px * off, sy + MARK_LIFT + 0.01, mz + pz * off,
                           yawFor(ox, oz), 2.0, 0.02, 0.45));
        }
      }
    }
    addTo(layers.signals, poles.build('signal-poles'));
    addTo(layers.signals, arms.build('signal-arms'));
    addTo(layers.signals, heads.build('signal-heads'));
    addTo(layers.signals, vlamps.build('signal-veh-lamps'));
    addTo(layers.signals, pheads.build('ped-heads'));
    addTo(layers.signals, plamps.build('ped-lamps'));
    addTo(layers.signals, signs.build('stop-signs'));
    addTo(layers.signals, signPoles.build('stop-sign-poles'));
    addTo(layers.markings, bars.build('stop-bars'));
    addTo(layers.crosswalks, stripes.build('crosswalk-stripes'));
    stats.signals = nSignals;

    // ---- live signal-state controller (deterministic fixed-time phase plan) ----
    function statesAt(plan, t) {
      const cyc = plan.cycleSec, local = ((t + plan.offsetSec) % cyc + cyc) % cyc;
      let acc = 0;
      for (let i = 0; i < plan.phases.length; i++) {
        const ph = plan.phases[i];
        if (local < acc + ph.durSec) return { idx: i, phase: ph, left: acc + ph.durSec - local };
        acc += ph.durSec;
      }
      const last = plan.phases.length - 1;
      return { idx: last, phase: plan.phases[last], left: 0 };
    }
    function secToChange(plan, t, group) {
      const s = statesAt(plan, t), cur = s.phase.groupStates[group];
      let rem = s.left;
      for (let n = 1; n <= plan.phases.length; n++) {
        const ph = plan.phases[(s.idx + n) % plan.phases.length];
        if (ph.groupStates[group] !== cur) return Math.round(rem * 10) / 10;
        rem += ph.durSec;
      }
      return Math.round(rem * 10) / 10;
    }

    const byId = new Map(), legList = [];
    for (const it of model.intersections) {
      byId.set(it.id, it);
      for (const leg of it.legs) {
        legList.push({ intId: it.id, legIdx: leg.idx, stopPoint: leg.stopPoint,
                       group: leg.signalGroup, control: it.control });
      }
    }

    const ctl = {
      model, byId, legs: legList, t: 0, overrides: new Map(),
      now() { return this.t; },
      setClock(t) { this.t = t; this._apply(); },
      tick(dt) { this.t += (dt || 0); this._apply(); },
      update(dt) { this.tick(dt); },
      _apply() {
        const t = this.t, dirty = new Set();
        for (const it of model.intersections) {
          if (it.control !== 'signal' || !it.phasePlan) continue;
          const ov = this.overrides.get(it.id);
          if (ov) it._state = { groupStates: ov.groupStates, pedStates: ov.pedStates || { A: 'dont', B: 'dont' } };
          else { const s = statesAt(it.phasePlan, t); it._state = { groupStates: s.phase.groupStates, pedStates: s.phase.pedStates || {} }; }
        }
        if (vlamps.mesh) {
          for (const r of vehReg) {
            const st = byId.get(r.intId)._state; if (!st) continue;
            const g = st.groupStates[r.group];
            const onAsp = g === 'green' ? 'G' : g === 'yellow' ? 'Y' : 'R';
            vlamps.mesh.setColorAt(r.index, _tmpcol.setHex(r.aspect === onAsp ? r.on : r.off));
          }
          dirty.add(vlamps.mesh);
        }
        if (plamps.mesh) {
          for (const r of pedReg) {
            const st = byId.get(r.intId)._state; if (!st) continue;
            const p = st.pedStates[r.group] || 'dont';
            let lit = r.kind === 'walk' ? (p === 'walk') : (p !== 'walk');
            if (r.kind === 'dont' && p === 'flash') lit = (Math.floor(t * 1.4) % 2 === 0);
            plamps.mesh.setColorAt(r.index, _tmpcol.setHex(lit ? r.on : r.off));
          }
          dirty.add(plamps.mesh);
        }
        for (const m of dirty) if (m.instanceColor) m.instanceColor.needsUpdate = true;
      },
      // ---- agent-facing query / control API ----
      getLegState(intId, legIdx) {
        const it = byId.get(intId); if (!it) return null;
        const leg = it.legs[legIdx]; if (!leg) return null;
        let signal = null, ped = null, sec = null;
        if (it.control === 'signal' && it.phasePlan) {
          const st = it._state || {};
          signal = (st.groupStates || {})[leg.signalGroup] || null;
          ped = (st.pedStates || {})[leg.signalGroup] || null;
          sec = secToChange(it.phasePlan, this.t, leg.signalGroup);
        }
        // canProceed: signals gate on green; uncontrolled approaches may proceed (yield);
        // a stop sign requires a halt first, so it is not a free "go" (agent uses `control`
        // + stopPoint to stop-then-proceed when clear — the controller can't see traffic).
        return { intersection: intId, leg: legIdx, control: it.control, signal, ped,
                 stopPoint: leg.stopPoint, stopLine: leg.stopLine, secToChange: sec,
                 canProceed: it.control === 'signal' ? signal === 'green'
                           : it.control === 'uncontrolled' };
      },
      queryByPosition(pos, maxR) {
        const r2 = (maxR || 60) ** 2; let best = null, bd = r2;
        for (const l of this.legs) {
          if (!l.stopPoint) continue;
          const dx = l.stopPoint[0] - pos[0], dz = l.stopPoint[2] - pos[1];
          const d = dx * dx + dz * dz;
          if (d < bd) { bd = d; best = l; }
        }
        return best ? this.getLegState(best.intId, best.legIdx) : null;
      },
      setOverride(intId, groupStates, pedStates) {
        if (groupStates && Object.values(groupStates).filter((s) => s === 'green').length > 1)
          throw new Error('refusing >1 conflicting green at ' + intId);
        this.overrides.set(intId, { groupStates, pedStates }); this._apply();
      },
      clearOverride(intId) { this.overrides.delete(intId); this._apply(); },
      snapshot() {
        const out = { t: this.t, intersections: {} };
        for (const it of model.intersections) {
          out.intersections[it.id] = {
            control: it.control, center: it.center,
            state: it.control === 'signal' ? it._state || null : null,
            legs: it.legs.map((l) => ({ idx: l.idx, group: l.signalGroup,
              signal: it._state ? (it._state.groupStates || {})[l.signalGroup] : null,
              stopPoint: l.stopPoint })),
          };
        }
        return out;
      },
    };
    ctl._apply();
    return ctl;
  }
}

function addTo(parent, mesh) { if (mesh) parent.add(mesh); }

// Andrew's monotone-chain convex hull of [x,z] points (CCW, no collinear points).
function convexHull2D(pts) {
  const P = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (P.length < 3) return P;
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [];
  for (const p of P) { while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  const up = [];
  for (let i = P.length - 1; i >= 0; i--) { const p = P[i]; while (up.length >= 2 && cross(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  lo.pop(); up.pop();
  return lo.concat(up);
}
