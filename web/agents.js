// Autonomous-agent simulation layer for the UKy Campus digital twin.
//
// Spawn controllable agents (car | truck | robot | drone) on the scene and drive
// them from user code, reading back four sensors:
//   1. CAMERA       — a render-to-target POV image (on-demand; zero cost if unused).
//   2. POSITION      — scene metres + UE cm + UTM-16N georef + heading/velocity.
//   3. COLLISION     — detection ONLY (no avoidance): contacts vs buildings + agents.
//   4. GROUND/SURFACE — a downward raycast keeps non-drone agents touching the
//                       ground and reports WHICH surface (road | terrain | building).
//
// Mirrors the house style of roads.js's signal controller (a plain synchronous,
// deterministic, frame-ticked object) and is reached at window.__twin.agents.
// The user programs avoidance from the readings; this engine never steers for them.
//
// createAgentSystem(deps) -> agentsApi   (see web/README.md "Autonomous-agent API")
//
// Coordinate frame (matches roads.js props): object-local +X is FORWARD, +Z is
// RIGHT, +Y is UP. Scene yaw is applied as object.rotation.y, so the forward unit
// vector is (cos yaw, 0, -sin yaw) — the exact inverse of roads.js's
// yawFor = (dx,dz) => atan2(-dz, dx). Never reinvent this mapping.

import * as THREE from 'three';

// ---- module-level scratch (no per-frame allocation; roads.js _pos/_quat idiom) ----
const _v3a = new THREE.Vector3(), _v3b = new THREE.Vector3(), _v3c = new THREE.Vector3();
const _v3d = new THREE.Vector3(), _v3scl = new THREE.Vector3();
const _box3 = new THREE.Box3();
const _mat4 = new THREE.Matrix4();
const _quat = new THREE.Quaternion();
const _prevClear = new THREE.Color();
const _ray = new THREE.Raycaster();
const _DOWN = Object.freeze(new THREE.Vector3(0, -1, 0));
let _pixelBuf = null;          // lazy Uint8Array for readRenderTargetPixels (GL readback)

// geometry cache shared across all agents of a type (only Group + material clone per agent)
const _geoCache = new Map();

// ---- small math helpers ----
const D2R = Math.PI / 180, R2D = 180 / Math.PI;
const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
const sign = (v) => (v > 0 ? 1 : v < 0 ? -1 : 0);
function wrapToPi(a) { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; }
// inverse pair: spawn does yaw = heading*D2R; this recovers heading in [0,360)
const yawFromHeadingDeg = (deg) => deg * D2R;
const degFromYaw = (yaw) => ((yaw * R2D) % 360 + 360) % 360;
function clampLen(v, maxLen) { const l = v.length(); if (l > maxLen && l > 1e-9) v.multiplyScalar(maxLen / l); return v; }

// ---- per-type definitions (scene metres) ----
// L = along local +X (forward), W = along +Z (width), H = up. rideHeight is 0:
// object.position is the GROUND-CONTACT reference point (footprint centre at the
// wheel-contact plane); body/wheels are offset up inside the group.
const TYPES = {
  car:   { L: 4.3, W: 1.9, H: 1.45, wheelbase: 2.6, track: 1.6, wheelR: 0.34,
           kin: 'ackermann', ground: true, alignToGround: false,
           maxSpeed: 25, maxAccel: 6, maxSteerDeg: 35,
           cam: { mount: [1.6, 1.2, 0], fov: 70, pitch: 0 } },
  truck: { L: 8.5, W: 2.5, H: 3.2, wheelbase: 5.0, track: 2.0, wheelR: 0.55,
           kin: 'ackermann', ground: true, alignToGround: false,
           maxSpeed: 18, maxAccel: 4, maxSteerDeg: 28,
           cam: { mount: [3.0, 2.6, 0], fov: 60, pitch: 0 } },
  robot: { L: 0.8, W: 0.6, H: 0.6, trackWidth: 0.5, wheelR: 0.13,
           kin: 'differential', ground: true, alignToGround: true,
           maxSpeed: 3, maxAccel: 4,
           cam: { mount: [0.2, 0.55, 0], fov: 75, pitch: 0 } },
  drone: { L: 0.9, W: 0.9, H: 0.35,
           kin: 'holonomic3d', ground: false, alignToGround: false,
           maxSpeed: 15, maxAccel: 10, maxClimb: 6, maxYawRateDeg: 120, minClearance: 0.5,
           cam: { mount: [0.1, 0.05, 0], fov: 80, pitch: 25 } },
};
const DEFAULT_COLORS = { car: 0x3577c9, truck: 0xc7702a, robot: 0x4aa05a, drone: 0x9b59b6 };

// =====================================================================
export function createAgentSystem(deps) {
  const { THREE: _T, scene, renderer, viewerCamera, controls, groups, coords, signals, transit } = deps;
  // (we use the imported THREE for scratch; deps.THREE is the same module instance.)

  const group = new THREE.Group(); group.name = 'agents';
  const agents = new Map();        // id -> Agent
  const byName = new Map();        // name -> Agent
  let nextId = 1;

  const rtCache = new Map();       // "WxH" -> WebGLRenderTarget (shared, bounded VRAM)
  const grid = { cell: 64, count: -1, builtCount: 0, map: new Map(), items: [] };  // building broad-phase

  const sys = {
    group, tick, paused: false,
    config: {
      maxAgents: 32,
      collisionTargets: ['buildings', 'agents'],
      cameraSize: [320, 240],
      timeScale: 1,
    },
    t: 0, frame: 0,
    _pip: null,

    spawn, despawn, get, list, clear, count,
    setTimeScale(s) { if (!(s > 0)) throw new Error('timeScale must be > 0'); this.config.timeScale = s; },
    setPaused(b) { this.paused = !!b; },
    snapshot,
    collidables() { rebuildGridIfNeeded(); return grid.items.map((it) => it.mesh); },
    toUE(p) { return coords.sceneToUeCm(p[0], p[1], p[2]); },
    toScene(u) { return coords.ueCmToScene(u[0], u[1], u[2]); },
    toUTM(p) { return sceneToUTM(p[0], p[2]); },
    setPiP(a) { this._pip = resolve(a); },

    // internal hooks used by Agent methods
    renderer, scene, coords, signals, transit, groups, controls,
    _getRT: getRT, _nearbyBuildings: nearbyBuildings, _groundCandidates: groundCandidates,
    _rebuildGridIfNeeded: rebuildGridIfNeeded,
  };

  // ---------------------------------------------------------------- API ---
  function resolve(a) {
    if (a == null) return null;
    if (typeof a === 'object' && a.id != null) return agents.get(a.id) || null;
    if (typeof a === 'number') return agents.get(a) || null;
    if (typeof a === 'string') return byName.get(a) || null;
    return null;
  }
  function get(id) { return resolve(id); }
  function list() { return [...agents.values()].sort((x, y) => x.id - y.id); }
  function count() { let n = 0; for (const a of agents.values()) if (a.alive) n++; return n; }
  function clear() { for (const a of list()) despawn(a); }

  function spawn(opts = {}) {
    const type = opts.type || 'car';
    const def = TYPES[type];
    if (!def) throw new Error(`unknown agent type '${type}'; valid: ${Object.keys(TYPES).join(', ')}`);
    if (count() >= sys.config.maxAgents)
      throw new Error(`agent cap reached (${sys.config.maxAgents}); despawn one first or raise agents.config.maxAgents`);

    const id = nextId++;
    const name = opts.name || `${type}_${id}`;
    if (byName.has(name)) { nextId--; throw new Error(`agent name '${name}' already in use`); }

    const a = new Agent(sys, id, name, type, def, opts);
    agents.set(id, a); byName.set(name, a);
    group.add(a.object);
    a._placeInitial(opts);
    a.state = a.getState();
    return a;
  }

  function despawn(idOrAgent) {
    const a = resolve(idOrAgent);
    if (!a || !a.alive) return false;
    a.alive = false;
    group.remove(a.object);
    a._dispose();
    agents.delete(a.id); byName.delete(a.name);
    if (sys._pip === a) sys._pip = null;
    return true;
  }

  function snapshot() {
    const out = { t: sys.t, frame: sys.frame, agents: {} };
    for (const a of agents.values()) out.agents[a.id] = a.getState();
    return out;
  }

  // ---------------------------------------------------- shared resources ---
  function getRT(w, h) {
    const key = w + 'x' + h;
    let rt = rtCache.get(key);
    if (!rt) {
      rt = new THREE.WebGLRenderTarget(w, h, {
        minFilter: THREE.LinearFilter, magFilter: THREE.LinearFilter,
        depthBuffer: true, stencilBuffer: false,
      });
      rtCache.set(key, rt);
    }
    return rt;
  }

  // ------------------------------------------------ building broad-phase ---
  // World AABB of a building = its local geometry.boundingBox translated by the
  // mesh's position (loadBuilding offsets only position.y to sit on the ground;
  // no rotation/scale). Computed directly from positions so it never depends on
  // matrixWorld timing. Rebuilt only when the building child-count changes
  // (they stream in over several seconds).
  function addBuildingToGrid(mesh) {
    const bb = mesh.geometry && mesh.geometry.boundingBox;
    if (!bb) return;
    const py = mesh.position.y;
    const item = {
      mesh,
      min: [bb.min.x, bb.min.y + py, bb.min.z],
      max: [bb.max.x, bb.max.y + py, bb.max.z],
      cx: (bb.min.x + bb.max.x) / 2,
      cz: (bb.min.z + bb.max.z) / 2,
      name: (mesh.userData && mesh.userData.buildingName) || mesh.name || 'building',
    };
    mesh.userData.surfaceClass = 'building';
    grid.items.push(item);
    const gx = Math.floor(item.cx / grid.cell), gz = Math.floor(item.cz / grid.cell);
    const k = gx + ',' + gz;
    let cell = grid.map.get(k); if (!cell) grid.map.set(k, cell = []);
    cell.push(item);
  }
  // Buildings stream in one-per-frame over several seconds (app.js loadBuildings).
  // Bin only the NEWLY-added meshes each frame (append-only fast path) instead of
  // re-walking all ~3109 every tick; full rebuild only if meshes were removed.
  function addBoxToGrid(b) {
    const item = { mesh: null, min: b.min, max: b.max, cx: b.cx, cz: b.cz, name: b.name, id: b.id };
    grid.items.push(item);
    const gx = Math.floor(item.cx / grid.cell), gz = Math.floor(item.cz / grid.cell);
    const k = gx + ',' + gz;
    let cell = grid.map.get(k); if (!cell) grid.map.set(k, cell = []);
    cell.push(item);
  }
  function rebuildGridIfNeeded() {
    // packed buildings: one merged render mesh, but per-building AABBs are provided
    // here, so the broad-phase grid is built from boxes (no per-mesh children).
    const boxes = groups.buildingBoxes && groups.buildingBoxes();
    if (boxes) {
      if (boxes.length === grid.count) return;
      grid.map.clear(); grid.items.length = 0;
      for (let i = 0; i < boxes.length; i++) addBoxToGrid(boxes[i]);
      grid.count = grid.builtCount = boxes.length;
      return;
    }
    const bg = groups.buildings;
    const n = bg ? bg.children.length : 0;
    if (n === grid.count) return;
    if (bg && n >= grid.builtCount) {
      for (let i = grid.builtCount; i < n; i++) addBuildingToGrid(bg.children[i]);
    } else {
      grid.map.clear(); grid.items.length = 0;
      if (bg) for (const mesh of bg.children) addBuildingToGrid(mesh);
    }
    grid.count = n; grid.builtCount = n;
  }
  function nearbyBuildings(x, z) {
    rebuildGridIfNeeded();
    const gx = Math.floor(x / grid.cell), gz = Math.floor(z / grid.cell), out = [];
    for (let dx = -1; dx <= 1; dx++) for (let dz = -1; dz <= 1; dz++) {
      const cell = grid.map.get((gx + dx) + ',' + (gz + dz));
      if (cell) for (const it of cell) out.push(it);
    }
    return out;
  }

  // candidate meshes for the downward ground ray: road ribbons (raised LIFT=0.3
  // above terrain, so they win), the terrain tile(s) under (x,z), nearby buildings.
  function groundCandidates(x, z) {
    const out = [];
    const rr = groups.roadRibbons && groups.roadRibbons();
    if (rr) for (const m of rr.children) { m.userData.surfaceClass = 'road'; out.push(m); }
    const terr = groups.terrain;
    if (terr) for (const m of terr.children) {
      const bb = m.geometry && m.geometry.boundingBox;
      if (!bb) continue;
      if (x >= bb.min.x - 1 && x <= bb.max.x + 1 && z >= bb.min.z - 1 && z <= bb.max.z + 1) {
        m.userData.surfaceClass = 'terrain'; out.push(m);
      }
    }
    // packed-building boxes have no per-building mesh to raycast (the merged mesh
    // would mean ~580k tris per probe); buildings just aren't a ground surface then.
    for (const it of nearbyBuildings(x, z)) {
      if (!it.mesh) continue;
      it.mesh.userData.surfaceClass = 'building'; out.push(it.mesh);
    }
    return out;
  }

  // scene XZ -> UTM zone 16N (EPSG:32616), via the documented georef only.
  function sceneToUTM(sx, sz) {
    const orig = coords.originalCoordinates && coords.originalCoordinates();
    if (!orig || orig.length !== 3) return null;
    const oc = coords.originCm();
    const A = (orig[0] + oc[0]) / 100, B = -(orig[1] + oc[1]) / 100;
    return { easting: A + sx, northing: B - sz, zone: '16N' };
  }

  // ----------------------------------------------------------- per frame ---
  function tick(dtRaw) {
    if (sys.paused) return;
    const dt = clamp(dtRaw || 0, 0, 0.1) * sys.config.timeScale;
    if (dt <= 0 && sys.frame > 0) return;   // first frame dt can be ~0; still init once
    sys.t += dt; sys.frame++;
    rebuildGridIfNeeded();
    const arr = list();   // ascending id => deterministic order

    // PASS 1 — sense (last frame), decide, integrate
    for (const a of arr) {
      if (!a.alive) continue;
      const sensors = a._buildSensors(dt);
      let controls;
      if (a.controller) {
        try { controls = a.controller(sensors, a); }
        catch (e) { a._warnOnce('controller', `controller threw: ${e && e.message || e}`); controls = a.controls; }
      } else {
        controls = a.controls;
      }
      if (controls && controls !== a.controls) a.controls = a._sanitize(controls);
      a._integrate(dt);
    }

    // PASS 2 — resolve world after EVERYONE moved (order-independent collisions)
    for (const a of arr) { if (a.alive) { a._snapGround(dt); a._finalizeMesh(dt); } }
    for (const a of arr) { if (a.alive) { a._detectCollisions(dt); a.state = a.getState(); } }

    if (sys._pip && sys._pip.alive) { try { pipBlit(sys._pip); } catch (e) { /* PiP is cosmetic */ } }
    if (sys.frame % 30 === 0) updateReadout();
  }

  function pipBlit(a) {
    if (typeof document === 'undefined') return;
    const canvas = document.getElementById('agent-pip-canvas');
    if (!canvas || canvas.classList.contains('hidden')) return;
    const img = a.camera.read({ size: [320, 240], format: 'imageData' });
    if (!img) return;
    if (canvas.width !== img.width) canvas.width = img.width;
    if (canvas.height !== img.height) canvas.height = img.height;
    canvas.getContext('2d').putImageData(img, 0, 0);
  }

  function updateReadout() {
    if (typeof document === 'undefined') return;
    const listEl = document.getElementById('agent-list');
    if (listEl) {
      const L = list();
      listEl.textContent = L.length
        ? L.map((a) => `${a.id} ${a.name} ${a.type} ${a.speed.toFixed(1)}m/s ${a.surface || '--'}`).join('\n')
        : 'agents: none';
    }
    const surfEl = document.getElementById('agent-surface');
    if (surfEl) {
      const a = sys._pip && sys._pip.alive ? sys._pip : list()[0];
      surfEl.textContent = a
        ? `surface: ${a.surface || '--'}  AGL ${a.altitudeAGL != null ? a.altitudeAGL.toFixed(1) : '--'}m  slope ${a.slopeDeg != null ? a.slopeDeg.toFixed(0) : '--'}°`
        : 'surface: --';
    }
  }

  return sys;
}

// =====================================================================
class Agent {
  constructor(sys, id, name, type, def, opts) {
    this.sys = sys;
    this.id = id; this.name = name; this.type = type; this.def = def;
    this.alive = true;

    // kinematic params (sane per-type defaults, overridable per spawn)
    this.maxSpeed = opts.maxSpeed ?? def.maxSpeed;
    this.maxAccel = opts.maxAccel ?? def.maxAccel;
    this.brakeDecel = opts.brakeDecel ?? this.maxAccel;
    this.maxSteerRad = (opts.maxSteerDeg ?? def.maxSteerDeg ?? 35) * D2R;
    this.steerRateRad = 90 * D2R;
    this.wheelbase = def.wheelbase || 2.5;
    this.trackWidth = opts.trackWidth ?? def.trackWidth ?? 0.5;
    this.maxClimb = opts.maxClimb ?? def.maxClimb ?? 6;
    this.maxYawRateRad = (opts.maxYawRateDeg ?? def.maxYawRateDeg ?? 120) * D2R;
    this.minClearance = opts.minClearance ?? def.minClearance ?? 0.5;
    this.groundBound = def.ground;
    this.alignToGround = opts.alignToGround ?? def.alignToGround;
    this.collidable = opts.collidable !== false;

    // dynamic state
    this.yaw = yawFromHeadingDeg(opts.heading || 0);
    this.speed = 0;                 // m/s signed (forward+)
    this.steerAngle = 0;            // current front-wheel angle (rad), slewed
    this.vel = new THREE.Vector3(); // drone world velocity
    this.measVel = new THREE.Vector3(); // measured world velocity (all types)
    this._yawRate = 0;              // rad/s
    this.controls = {};
    // a controller passed to spawn() is registered immediately (spec spawn option)
    this.controller = typeof opts.controller === 'function' ? opts.controller : null;
    this._warned = new Set();

    // ground/surface fields
    this.surface = 'none'; this.groundY = null; this.groundNormal = null;
    this.slopeDeg = null; this.altitudeAGL = null; this.onGround = false; this.offMap = true;

    // collision
    this.contacts = [];
    this._activePairs = new Set();
    this._onCollision = []; this._onContactEnd = [];

    // mesh
    const color = opts.color != null ? opts.color : DEFAULT_COLORS[type];
    const built = buildAgentMesh(type, def, color, !!opts.showHeading);
    this.object = built.root;
    this.halfExtents = built.halfExtents;   // [hx, hy, hz] along local x,y,z
    this.rotors = built.rotors;
    this.object.userData.agentId = id;
    this.object.name = name;
    this.object.rotation.y = this.yaw;
    this.prevPos = new THREE.Vector3();

    // camera sensor
    this.camera = this._buildCamera(opts.camera);

    this.state = null;
  }

  get pos() { return this.object.position; }

  _placeInitial(opts) {
    let p = opts.position;
    if (!p && opts.positionUE) p = this.sys.coords.ueCmToScene(opts.positionUE[0], opts.positionUE[1], opts.positionUE[2]);
    if (!p) { const t = this.sys.controls && this.sys.controls.target; p = t ? [t.x, null, t.z] : [0, null, 0]; }
    const yGiven = p[1];
    // start high so the first downward probe always originates above the ground
    this.object.position.set(p[0], yGiven == null ? 400 : yGiven, p[2]);
    this.prevPos.copy(this.object.position);
    this._snapGround(0.016);
    if (this.type === 'drone' && yGiven == null && this.groundY != null) {
      this.object.position.y = this.groundY + 20;       // default hover altitude
      this.altitudeAGL = 20;
    }
    this.prevPos.copy(this.object.position);
    this._finalizeMesh(0.016);
  }

  _dispose() {
    // dispose only the per-agent body material (the __ownMat flag lives on the
    // MATERIAL's userData, not the mesh's); shared cached mats + the shared,
    // system-scoped render targets are intentionally left alone.
    this.object.traverse((o) => {
      const m = o.material;
      if (m && m.dispose && m.userData && m.userData.__ownMat) m.dispose();
    });
  }

  // -------------------------------------------------------- control API ---
  setController(fn) { this.controller = (typeof fn === 'function') ? fn : null; return this; }
  setControls(c) {
    if (this.controller) this._warnOnce('setControls', 'a controller is active; setControls is ignored (controller output wins). Call setController(null) first.');
    else this.controls = this._sanitize(c || {});
    return this;
  }
  stop() { this.controller = null; this.vel.set(0, 0, 0);
    this.controls = this.type === 'drone' ? { move: [0, 0, 0] } : { throttle: 0, brake: 1, steer: 0 }; return this; }
  setHeading(deg) { this.yaw = yawFromHeadingDeg(deg); this.object.rotation.y = this.yaw; return this; }
  teleport(p) { this.object.position.set(p[0], p[1] != null ? p[1] : this.object.position.y, p[2]);
    this.prevPos.copy(this.object.position); this._snapGround(0.016); this._finalizeMesh(0.016); return this; }

  setVelocity(v) {
    const vx = v[0] || 0, vy = v[1] || 0, vz = v[2] || 0;
    if (this.type === 'drone') { this.setController((s, a) => ({ move: [vx, vy, vz] })); }
    else {
      this.setController((s, a) => {
        const desired = Math.atan2(-vz, vx);
        const err = wrapToPi(desired - a.yaw);
        const targetSpeed = Math.hypot(vx, vz);
        const stopping = targetSpeed < 0.05;   // a (near-)zero target means STOP, not coast
        return { throttle: stopping ? 0 : clamp(targetSpeed / a.maxSpeed, 0, 1),
                 steer: clamp(err / a.maxSteerRad, -1, 1), brake: stopping ? 1 : 0 };
      });
    }
    return this;
  }

  driveTo(target, opts = {}) {
    const tx = target[0], ty = target.length > 2 ? target[1] : null, tz = target.length > 2 ? target[2] : target[1];
    const speed = opts.speed != null ? opts.speed : 0.6 * this.maxSpeed;
    const arriveR = opts.arriveRadius != null ? opts.arriveRadius : 2;
    const stopAtEnd = opts.stop !== false;
    this.setController((s, a) => {
      const p = a.pos;
      const dist = Math.hypot(tx - p.x, tz - p.z);
      const desired = Math.atan2(-(tz - p.z), tx - p.x);   // = yawFor(dx,dz)
      const err = wrapToPi(desired - a.yaw);
      if (a.type === 'drone') {
        const dy = ty != null ? clamp(ty - p.y, -a.maxClimb, a.maxClimb) : 0;
        const horiz = dist > arriveR;
        const fwd = new THREE.Vector3(tx - p.x, 0, tz - p.z);
        if (horiz) clampLen(fwd, speed); else fwd.set(0, 0, 0);
        return { move: [fwd.x, dy, fwd.z] };
      }
      const steer = clamp(err / a.maxSteerRad, -1, 1);
      if (dist <= arriveR) { if (stopAtEnd) a.controller = null; return stopAtEnd ? { throttle: 0, brake: 1, steer: 0 } : { throttle: 0, brake: 0, steer }; }
      return { throttle: clamp(speed / a.maxSpeed, 0, 1), steer, brake: 0 };
    });
    return this;
  }

  onCollision(fn) { if (typeof fn === 'function') this._onCollision.push(fn); return this; }
  onContactEnd(fn) { if (typeof fn === 'function') this._onContactEnd.push(fn); return this; }
  getContacts() { return this.contacts.slice(); }

  _warnOnce(key, msg) { if (this._warned.has(key)) return; this._warned.add(key);
    console.warn(`[agent ${this.name}] ${msg}`); }

  _sanitize(c) {
    const out = {};
    const num = (k, lo, hi) => { if (c[k] != null) { const v = +c[k];
      if (v < lo || v > hi) this._warnOnce('clamp:' + k, `controls.${k}=${v} out of [${lo},${hi}]; clamped`);
      out[k] = clamp(v, lo, hi); } };
    num('throttle', 0, 1); num('brake', 0, 1); num('steer', -1, 1);
    num('left', -1, 1); num('right', -1, 1);
    num('thrust', 0, 1); num('climb', -1, 1);
    if (c.yawRate != null) out.yawRate = +c.yawRate;
    if (c.reverse != null) out.reverse = !!c.reverse;
    if (c.handbrake != null) out.handbrake = !!c.handbrake;
    if (Array.isArray(c.move)) out.move = [+c.move[0] || 0, +c.move[1] || 0, +c.move[2] || 0];
    return out;
  }

  // ----------------------------------------------------------- sensing ---
  _buildSensors(dt) {
    const a = this, s = Object.assign({}, this.state || this.getState());
    s.frame = this.sys.frame; s.dt = dt; s.t = this.sys.t;
    s.collisions = this.contacts;          // last frame's contacts (this frame refilled in pass 2)
    s.signals = this.sys.signals ? this.sys.signals() : null;
    // live Lextran transit (window.__twin.transit): query/avoid/await campus buses.
    // Buses aren't collidable bodies, so a controller senses them here and reacts —
    // e.g. s.transit.getNearestVehicle(this.getState().position) to yield to a bus.
    s.transit = this.sys.transit ? this.sys.transit() : null;
    s.camera = {
      read: (o) => a.camera.read(o),
      pose: () => a.camera.pose(),
    };
    return s;
  }

  getState() {
    const p = this.object.position, yaw = this.yaw;
    const ue = this.sys.coords.sceneToUeCm(p.x, p.y, p.z);
    const orig = this.sys.coords.originalCoordinates && this.sys.coords.originalCoordinates();
    let utm = null, geoUEAbs = null;
    if (orig && orig.length === 3) {
      const oc = this.sys.coords.originCm();
      const A = (orig[0] + oc[0]) / 100, B = -(orig[1] + oc[1]) / 100;
      utm = { easting: A + p.x, northing: B - p.z, zone: '16N' };
      geoUEAbs = [orig[0] + ue[0], orig[1] + ue[1], orig[2] + ue[2]];
    }
    return {
      id: this.id, name: this.name, type: this.type, frame: this.sys.frame, t: this.sys.t,
      position: [p.x, p.y, p.z], positionUE: ue, utm, geoUEAbs,
      heading: degFromYaw(yaw), headingRad: yaw, forward: [Math.cos(yaw), 0, -Math.sin(yaw)],
      velocity: [this.measVel.x, this.measVel.y, this.measVel.z], speed: this.speed,
      angularVel: this._yawRate * R2D,
      surface: this.surface, groundY: this.groundY,
      groundNormal: this.groundNormal ? this.groundNormal.slice() : null,
      slopeDeg: this.slopeDeg, altitudeAGL: this.altitudeAGL,
      onGround: this.onGround, offMap: this.offMap,
    };
  }

  // ------------------------------------------------------- integration ---
  _integrate(dt) {
    const c = this.controls || {};
    if (this.def.kin === 'ackermann') this._integrateAckermann(dt, c);
    else if (this.def.kin === 'differential') this._integrateDifferential(dt, c);
    else this._integrateHolonomic(dt, c);
  }

  _integrateAckermann(dt, c) {
    const dir = c.reverse ? -1 : 1;
    const throttle = c.throttle || 0, brake = (c.handbrake ? 1 : 0) || c.brake || 0;
    const drag = 0.05 * this.speed;
    const a = dir * throttle * this.maxAccel - brake * this.brakeDecel * sign(this.speed) - drag;
    this.speed = clamp(this.speed + a * dt, -0.4 * this.maxSpeed, this.maxSpeed);
    if (Math.abs(this.speed) < 0.02 && throttle === 0) this.speed = 0;   // settle
    const steerTarget = (c.steer || 0) * this.maxSteerRad;
    this.steerAngle += clamp(steerTarget - this.steerAngle, -this.steerRateRad * dt, this.steerRateRad * dt);
    this._yawRate = (this.speed / this.wheelbase) * Math.tan(this.steerAngle);
    this.yaw += this._yawRate * dt;
    this.pos.x += Math.cos(this.yaw) * this.speed * dt;
    this.pos.z += -Math.sin(this.yaw) * this.speed * dt;
  }

  _integrateDifferential(dt, c) {
    let vL, vR;
    if (c.left != null || c.right != null) { vL = c.left || 0; vR = c.right || 0; }
    else { const th = c.throttle || 0, st = c.steer || 0; vL = th - st; vR = th + st; }
    if (c.brake) { vL *= (1 - c.brake); vR *= (1 - c.brake); }
    vL = clamp(vL, -1, 1) * this.maxSpeed; vR = clamp(vR, -1, 1) * this.maxSpeed;
    this.speed = (vL + vR) / 2;
    this._yawRate = (vR - vL) / Math.max(this.trackWidth, 1e-3);
    this.yaw += this._yawRate * dt;
    this.pos.x += Math.cos(this.yaw) * this.speed * dt;
    this.pos.z += -Math.sin(this.yaw) * this.speed * dt;
  }

  _integrateHolonomic(dt, c) {
    const accel = this.maxAccel;
    if (Array.isArray(c.move)) {
      _v3a.set(c.move[0] || 0, 0, c.move[2] || 0); clampLen(_v3a, this.maxSpeed);
      _v3a.y = clamp(c.move[1] || 0, -this.maxClimb, this.maxClimb);
      _v3b.subVectors(_v3a, this.vel); clampLen(_v3b, accel * dt);
      this.vel.add(_v3b);
    } else {
      const thrust = c.thrust || 0, climb = c.climb || 0, yawRate = (c.yawRate || 0) * D2R;
      const fx = Math.cos(this.yaw) * thrust * this.maxSpeed, fz = -Math.sin(this.yaw) * thrust * this.maxSpeed;
      this.vel.x += clamp(fx - this.vel.x, -accel * dt, accel * dt);
      this.vel.z += clamp(fz - this.vel.z, -accel * dt, accel * dt);
      this.vel.y += clamp(climb * this.maxClimb - this.vel.y, -accel * dt, accel * dt);
      this._yawRate = clamp(yawRate, -this.maxYawRateRad, this.maxYawRateRad);
      this.yaw += this._yawRate * dt;
      this.vel.multiplyScalar(0.99);   // air drag — thrust mode only; move-mode
    }                                  // tracks its explicit setpoint exactly
    this.pos.addScaledVector(this.vel, dt);
    this.speed = Math.hypot(this.vel.x, this.vel.z);
  }

  // ----------------------------------------------- ground / surface ---
  _snapGround(dt) {
    const x = this.pos.x, z = this.pos.z;
    const probeUp = Math.max(this.halfExtents[1] * 2, 5);
    const originY = this.pos.y + probeUp;
    _ray.set(_v3a.set(x, originY, z), _DOWN);
    _ray.far = probeUp + 250;
    const cands = this.sys._groundCandidates(x, z);
    const hits = cands.length ? _ray.intersectObjects(cands, false) : [];
    if (!hits.length) {
      if (!this.offMap) this._warnOnce('offmap', 'no ground under agent (off map / data not loaded yet); holding altitude');
      this.surface = 'none'; this.groundY = null; this.groundNormal = null;
      this.slopeDeg = null; this.offMap = true;
      if (this.groundBound) { this.altitudeAGL = null; this.onGround = false; }
      else { this.altitudeAGL = null; this.onGround = false; }
      return;
    }
    const hit = hits[0];
    this.offMap = false;
    this.groundY = hit.point.y;
    this.surface = (hit.object.userData && hit.object.userData.surfaceClass) || 'terrain';
    this._computeNormal(hit.object, x, z, originY, this.groundY);
    if (this.groundBound) {
      this.pos.y = this.groundY;
      this.altitudeAGL = 0; this.onGround = true;
    } else {
      const floor = this.groundY + this.minClearance;
      if (this.pos.y < floor) { this.pos.y = floor; if (this.vel.y < 0) this.vel.y = 0; }
      this.altitudeAGL = this.pos.y - this.groundY;
      this.onGround = this.altitudeAGL < 0.5;
    }
  }

  // slope normal via 3-sample cross product against the SAME hit mesh (terrain has
  // no vertex normals so hit.face.normal is unreliable — sample geometry instead).
  _computeNormal(obj, x, z, originY, y0) {
    const eps = 0.6;
    const y1 = sampleY(obj, x + eps, z, originY);
    const y2 = sampleY(obj, x, z + eps, originY);
    if (y1 == null || y2 == null) {
      this.groundNormal = [0, 1, 0]; this.slopeDeg = 0; return;
    }
    // a = (eps, y1-y0, 0), b = (0, y2-y0, eps); the up-pointing normal is -(a x b)
    // = (-(y1-y0)*eps, +eps^2, -(y2-y0)*eps). Only Y keeps the +eps^2 sign; both
    // horizontal terms are negated (so a slope rising toward +X leans the normal -X).
    _v3b.set(-(y1 - y0) * eps, eps * eps, -(y2 - y0) * eps);
    if (_v3b.y < 0) _v3b.negate();
    _v3b.normalize();
    this.groundNormal = [_v3b.x, _v3b.y, _v3b.z];
    this.slopeDeg = Math.acos(clamp(_v3b.y, -1, 1)) * R2D;
  }

  _finalizeMesh(dt) {
    // orientation: upright (yaw only) by default; optional slope alignment
    if (this.alignToGround && this.groundNormal && this.slopeDeg != null && this.slopeDeg <= 40) {
      _v3a.set(this.groundNormal[0], this.groundNormal[1], this.groundNormal[2]);
      _v3b.set(0, 1, 0);
      _mat4.makeRotationY(this.yaw);
      const qYaw = new THREE.Quaternion().setFromRotationMatrix(_mat4);
      const qTilt = new THREE.Quaternion().setFromUnitVectors(_v3b, _v3a);
      this.object.quaternion.copy(qTilt).multiply(qYaw);
    } else {
      this.object.rotation.set(0, this.yaw, 0);
    }
    if (this.rotors && this.rotors.length) for (const r of this.rotors) r.rotation.y += 25 * dt;
    if (dt > 0) this.measVel.set((this.pos.x - this.prevPos.x) / dt, (this.pos.y - this.prevPos.y) / dt, (this.pos.z - this.prevPos.z) / dt);
    this.prevPos.copy(this.pos);
  }

  // --------------------------------------------------------- collision ---
  _detectCollisions(dt) {
    this.contacts = [];
    const seen = new Set();
    const targets = this.sys.config.collisionTargets || [];
    const acx = this.pos.x, acz = this.pos.z;
    const ayc = this.pos.y + this.halfExtents[1], ahy = this.halfExtents[1];
    const cy = Math.cos(this.yaw), sy = Math.sin(this.yaw);
    const axX = [cy, -sy], axZ = [sy, cy];       // local +X, +Z in XZ plane
    const hx = this.halfExtents[0], hz = this.halfExtents[2];

    if (targets.includes('buildings')) {
      for (const it of this.sys._nearbyBuildings(acx, acz)) {
        const byc = (it.min[1] + it.max[1]) / 2, bhy = (it.max[1] - it.min[1]) / 2;
        if (Math.abs(ayc - byc) >= ahy + bhy) continue;        // Y interval gate
        const bcx = it.cx, bcz = it.cz, bhx = (it.max[0] - it.min[0]) / 2, bhz = (it.max[2] - it.min[2]) / 2;
        const r = obbAabbXZ(acx, acz, axX, axZ, hx, hz, bcx, bcz, bhx, bhz);
        if (!r) continue;
        const key = 'b:' + it.name;
        seen.add(key);
        this._pushContact('building', it.id != null ? it.id : it.name, it.name, r, _v3d.set(0, 0, 0), key);
      }
    }
    if (targets.includes('agents')) {
      for (const other of this.sys.list()) {
        if (other === this || !other.alive || !other.collidable) continue;
        const oyc = other.pos.y + other.halfExtents[1], ohy = other.halfExtents[1];
        if (Math.abs(ayc - oyc) >= ahy + ohy) continue;
        const ocy = Math.cos(other.yaw), osy = Math.sin(other.yaw);
        const r = obbObbXZ(acx, acz, axX, axZ, hx, hz,
                           other.pos.x, other.pos.z, [ocy, -osy], [osy, ocy], other.halfExtents[0], other.halfExtents[2]);
        if (!r) continue;
        const key = 'a:' + Math.min(this.id, other.id) + ':' + Math.max(this.id, other.id);
        seen.add(key);
        this._pushContact('agent', other.id, other.name, r, other.measVel, key);
      }
    }

    // edges
    for (const c of this.contacts) {
      if (!this._activePairs.has(c._key)) { c.phase = 'enter';
        for (const fn of this._onCollision) { try { fn(c); } catch (e) { this._warnOnce('onCollision', 'handler threw'); } } }
      else c.phase = 'stay';
    }
    for (const key of this._activePairs) {
      if (!seen.has(key)) {
        const ev = { with: key[0] === 'b' ? 'building' : 'agent', phase: 'exit', key, frame: this.sys.frame, t: this.sys.t };
        for (const fn of this._onContactEnd) { try { fn(ev); } catch (e) { this._warnOnce('onContactEnd', 'handler threw'); } }
      }
    }
    this._activePairs = seen;
  }

  _pushContact(kind, oid, oname, r, otherVel, key) {
    const acx = this.pos.x, acz = this.pos.z;
    const nx = r.nx, nz = r.nz;
    // closing speed: normal points from OTHER toward THIS; closing>0 when approaching
    const rel = -((this.measVel.x - otherVel.x) * nx + (this.measVel.z - otherVel.z) * nz);
    this.contacts.push({
      with: kind, id: oid, name: oname,
      point: [acx - nx * (this.halfExtents[0] + this.halfExtents[2]) * 0.25, this.pos.y + this.halfExtents[1], acz - nz * (this.halfExtents[0] + this.halfExtents[2]) * 0.25],
      normal: [nx, 0, nz], penetration: r.pen, relativeSpeed: rel,
      phase: 'enter', frame: this.sys.frame, t: this.sys.t, _key: key,
    });
  }

  // --------------------------------------------------------- camera ---
  _buildCamera(camOpt) {
    const a = this;
    if (camOpt === false) {
      return { enabled: false, object: null, _rt: null,
        read() { a._warnOnce('camera', 'agent spawned with camera:false; read() returns null'); return null; },
        pose() { return null; }, setFov() {} };
    }
    const def = this.def.cam, cfg = Object.assign({}, def, camOpt || {});
    const size = cfg.size || this.sys.config.cameraSize;
    const mount = cfg.mount || [0, 1, 0];
    const pitchRad = (cfg.pitch || 0) * D2R;
    const cam = new THREE.PerspectiveCamera(cfg.fov || 70, size[0] / size[1], 0.2, 2000);
    const localMat = buildCamLocal(mount, pitchRad);
    const camera = {
      enabled: cfg.enabled !== false, object: cam, size: size.slice(),
      _localMat: localMat, _mount: mount.slice(), _pitch: pitchRad,
      setFov: (deg) => { cam.fov = deg; cam.updateProjectionMatrix(); },
      pose: () => {
        a.object.updateWorldMatrix(true, false);
        _mat4.multiplyMatrices(a.object.matrixWorld, camera._localMat);
        _mat4.decompose(_v3a, _quat, _v3scl);
        const f = _v3c.set(0, 0, -1).applyQuaternion(_quat);
        return { position: [_v3a.x, _v3a.y, _v3a.z], forward: [f.x, f.y, f.z] };
      },
      read: (opts) => a._readCamera(camera, opts),
    };
    return camera;
  }

  _readCamera(camera, opts = {}) {
    if (!camera.enabled) return null;
    const renderer = this.sys.renderer, scene = this.sys.scene;
    let w = clamp(Math.round((opts.size && opts.size[0]) || camera.size[0]), 1, 512);
    let h = clamp(Math.round((opts.size && opts.size[1]) || camera.size[1]), 1, 512);
    const fmt = opts.format || 'pixels';
    const flipY = opts.flipY !== false;
    const rt = this.sys._getRT(w, h);
    const cam = camera.object;
    // place the POV camera from the agent's world matrix * local mount
    this.object.updateWorldMatrix(true, false);
    _mat4.multiplyMatrices(this.object.matrixWorld, camera._localMat);
    _mat4.decompose(cam.position, cam.quaternion, _v3scl);
    cam.aspect = w / h; cam.updateProjectionMatrix(); cam.updateMatrixWorld(true);

    const prevRT = renderer.getRenderTarget();
    renderer.getClearColor(_prevClear);
    const prevAlpha = renderer.getClearAlpha();
    const prevXR = renderer.xr ? renderer.xr.enabled : false;
    this.object.visible = false;                       // don't photograph our own body
    try {
      if (renderer.xr) renderer.xr.enabled = false;
      renderer.setClearColor(0x9ec4e8, 1);             // sky so frames aren't transparent
      renderer.setRenderTarget(rt);
      renderer.clear();
      renderer.render(scene, cam);
      const need = w * h * 4;
      if (!_pixelBuf || _pixelBuf.length < need) _pixelBuf = new Uint8Array(need);
      renderer.readRenderTargetPixels(rt, 0, 0, w, h, _pixelBuf);
    } finally {
      renderer.setRenderTarget(prevRT);
      renderer.setClearColor(_prevClear, prevAlpha);
      if (renderer.xr) renderer.xr.enabled = prevXR;
      this.object.visible = true;
    }

    // readRenderTargetPixels is bottom-up (GL); flip to top-left origin. The output
    // buffer is allocated EXACTLY w*h*4 (ImageData requires an exact length, and
    // reads are on-demand/throttled so a fresh alloc per read is fine).
    const len = w * h * 4;
    const out = (fmt === 'pixels') ? new Uint8Array(len) : new Uint8ClampedArray(len);
    const row = w * 4;
    if (flipY) {
      for (let y = 0; y < h; y++) { const src = (h - 1 - y) * row, dst = y * row;
        for (let i = 0; i < row; i++) out[dst + i] = _pixelBuf[src + i]; }
    } else {
      for (let i = 0; i < len; i++) out[i] = _pixelBuf[i];
    }
    if (fmt === 'pixels') return { width: w, height: h, data: out };
    const imageData = new ImageData(out, w, h);   // out is a Uint8ClampedArray of length 4*w*h
    if (fmt === 'imageData') return imageData;
    // dataURL
    if (!_blitCanvasFor(w, h)) return null;
    _blitCtx.putImageData(imageData, 0, 0);
    return _blitCanvas.toDataURL('image/png');
  }
}

// =====================================================================
// ---- camera local mount matrix: T(mount) * Ry(-90°) * Rx(-pitch) ----
// Ry(-90°) aims the camera's -Z down the object's +X (forward); Rx(-pitch) tilts
// it downward for the drone gimbal. (Derived from the (cos yaw,0,-sin yaw) frame.)
function buildCamLocal(mount, pitchRad) {
  const t = new THREE.Matrix4().makeTranslation(mount[0], mount[1], mount[2]);
  const ry = new THREE.Matrix4().makeRotationY(-Math.PI / 2);
  const rx = new THREE.Matrix4().makeRotationX(-pitchRad);
  return t.multiply(ry).multiply(rx);
}

let _blitCanvas = null, _blitCtx = null;
function _blitCanvasFor(w, h) {
  if (typeof document === 'undefined') return false;
  if (!_blitCanvas) { _blitCanvas = document.createElement('canvas'); _blitCtx = _blitCanvas.getContext('2d'); }
  if (_blitCanvas.width !== w) _blitCanvas.width = w;
  if (_blitCanvas.height !== h) _blitCanvas.height = h;
  return true;
}

function sampleY(obj, x, z, originY) {
  _ray.set(_v3c.set(x, originY, z), _DOWN);
  _ray.far = originY + 250;
  const h = _ray.intersectObject(obj, false);
  return h.length ? h[0].point.y : null;
}

// ---- SAT (XZ plane). normal points FROM the other box TOWARD this agent. ----
function obbAabbXZ(acx, acz, axX, axZ, hx, hz, bcx, bcz, bhx, bhz) {
  const axes = [axX, axZ, [1, 0], [0, 1]];
  let minOv = Infinity, nx = 0, nz = 0;
  const dx = acx - bcx, dz = acz - bcz;   // from B(other) to A(this)
  for (const L of axes) {
    const lx = L[0], lz = L[1];
    const rA = Math.abs(hx * (axX[0] * lx + axX[1] * lz)) + Math.abs(hz * (axZ[0] * lx + axZ[1] * lz));
    const rB = bhx * Math.abs(lx) + bhz * Math.abs(lz);
    const dist = dx * lx + dz * lz;
    const ov = rA + rB - Math.abs(dist);
    if (ov <= 0) return null;
    if (ov < minOv) { minOv = ov; const s = dist >= 0 ? 1 : -1; nx = lx * s; nz = lz * s; }
  }
  return { nx, nz, pen: minOv };
}
function obbObbXZ(acx, acz, aX, aZ, ahx, ahz, bcx, bcz, bX, bZ, bhx, bhz) {
  const axes = [aX, aZ, bX, bZ];
  let minOv = Infinity, nx = 0, nz = 0;
  const dx = acx - bcx, dz = acz - bcz;
  for (const L of axes) {
    const lx = L[0], lz = L[1], len = Math.hypot(lx, lz) || 1, ux = lx / len, uz = lz / len;
    const rA = Math.abs(ahx * (aX[0] * ux + aX[1] * uz)) + Math.abs(ahz * (aZ[0] * ux + aZ[1] * uz));
    const rB = Math.abs(bhx * (bX[0] * ux + bX[1] * uz)) + Math.abs(bhz * (bZ[0] * ux + bZ[1] * uz));
    const dist = dx * ux + dz * uz;
    const ov = rA + rB - Math.abs(dist);
    if (ov <= 0) return null;
    if (ov < minOv) { minOv = ov; const s = dist >= 0 ? 1 : -1; nx = ux * s; nz = uz * s; }
  }
  return { nx, nz, pen: minOv };
}

// =====================================================================
// ---- mesh builders (primitives + MeshStandard so the scene lights shade them) ----
function cachedGeo(key, make) { let g = _geoCache.get(key); if (!g) _geoCache.set(key, g = make()); return g; }

function buildAgentMesh(type, def, color, showHeading) {
  const root = new THREE.Group();
  const bodyMat = new THREE.MeshStandardMaterial({ color, roughness: 0.5, metalness: 0.35, side: THREE.DoubleSide });
  bodyMat.userData.__ownMat = true;
  const dark = cachedMat('agent-dark', () => new THREE.MeshStandardMaterial({ color: 0x1c2228, roughness: 0.3, metalness: 0.55 }));
  const tyre = cachedMat('agent-tyre', () => new THREE.MeshStandardMaterial({ color: 0x111316, roughness: 0.9 }));
  const rotorMat = cachedMat('agent-rotor', () => new THREE.MeshStandardMaterial({ color: 0x2a2f36, roughness: 0.4, metalness: 0.6 }));
  const halfExtents = [def.L / 2, def.H / 2, def.W / 2];
  const rotors = [];

  const wheelGeo = (r) => cachedGeo('wheel' + r, () => { const g = new THREE.CylinderGeometry(r, r, r * 0.7, 14); g.rotateX(Math.PI / 2); return g; });
  const boxGeo = (key, x, y, z) => cachedGeo(key, () => new THREE.BoxGeometry(x, y, z));

  if (type === 'car' || type === 'truck') {
    const r = def.wheelR;
    const bodyH = def.H - r;
    const body = new THREE.Mesh(boxGeo(type + 'body', def.L * 0.96, bodyH, def.W * 0.92), bodyMat);
    body.position.y = r + bodyH / 2;
    root.add(body);
    if (type === 'car') {
      const cab = new THREE.Mesh(boxGeo('carcab', def.L * 0.5, def.H * 0.4, def.W * 0.85), dark);
      cab.position.set(-def.L * 0.08, r + bodyH * 0.7, 0); root.add(cab);
    } else {
      const cab = new THREE.Mesh(boxGeo('truckcab', def.L * 0.22, def.H * 0.55, def.W * 0.96), bodyMat);
      cab.position.set(def.L * 0.36, r + bodyH * 0.75, 0); root.add(cab);
    }
    const wx = def.wheelbase / 2, wz = def.track / 2;
    for (const sx of [wx, -wx]) for (const sz of [wz, -wz]) {
      const wmesh = new THREE.Mesh(wheelGeo(r), tyre); wmesh.position.set(sx, r, sz); root.add(wmesh);
    }
  } else if (type === 'robot') {
    const r = def.wheelR;
    const body = new THREE.Mesh(boxGeo('robotbody', def.L, def.H * 0.6, def.W), bodyMat);
    body.position.y = r + def.H * 0.3; root.add(body);
    const dome = new THREE.Mesh(cachedGeo('robotdome', () => new THREE.SphereGeometry(def.W * 0.32, 12, 8)), dark);
    dome.position.set(0, r + def.H * 0.6, 0); root.add(dome);
    const mast = new THREE.Mesh(boxGeo('robotmast', 0.08, 0.18, 0.14), dark);
    mast.position.set(def.L * 0.45, r + def.H * 0.5, 0); root.add(mast);
    for (const sz of [def.W / 2, -def.W / 2]) {
      const wmesh = new THREE.Mesh(wheelGeo(r), tyre); wmesh.position.set(0, r, sz); root.add(wmesh);
    }
  } else { // drone
    const bodyY = def.H * 0.6;
    const body = new THREE.Mesh(boxGeo('dronebody', def.L * 0.4, def.H * 0.5, def.W * 0.4), bodyMat);
    body.position.y = bodyY; root.add(body);
    const armGeo = boxGeo('dronearm', def.L * 0.95, def.H * 0.08, def.W * 0.06);
    const rotorGeo = cachedGeo('dronerotor', () => new THREE.CylinderGeometry(def.L * 0.22, def.L * 0.22, def.H * 0.05, 12));
    for (const ang of [Math.PI / 4, 3 * Math.PI / 4, 5 * Math.PI / 4, 7 * Math.PI / 4]) {
      const arm = new THREE.Mesh(armGeo, dark); arm.position.y = bodyY; arm.rotation.y = ang; root.add(arm);
      const rx = Math.cos(ang) * def.L * 0.45, rz = -Math.sin(ang) * def.L * 0.45;
      const rotor = new THREE.Mesh(rotorGeo, rotorMat); rotor.position.set(rx, bodyY + def.H * 0.12, rz);
      root.add(rotor); rotors.push(rotor);
    }
  }

  if (showHeading) {
    const arrow = new THREE.Mesh(cachedGeo('headarrow', () => { const g = new THREE.ConeGeometry(0.18, 0.6, 8); g.rotateZ(-Math.PI / 2); return g; }),
      cachedMat('agent-arrow', () => new THREE.MeshStandardMaterial({ color: 0xffe14d, roughness: 0.4 })));
    arrow.position.set(def.L * 0.5 + 0.3, def.H * 0.6, 0); root.add(arrow);
  }
  return { root, halfExtents, rotors };
}

const _matCache = new Map();
function cachedMat(key, make) { let m = _matCache.get(key); if (!m) _matCache.set(key, m = make()); return m; }
