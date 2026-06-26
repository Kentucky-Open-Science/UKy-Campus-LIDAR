// Lexington Digital Twin viewer — works against the shared data contract (see README.md).
// Data may not exist yet; every layer loads gracefully and reports status.
//
// Coordinates: source data is UE world space (centimeters, Z-up, left-handed).
// Conversion (done per-vertex while building buffers):
//   three.(x, y, z) = ((ue.x, ue.z, ue.y) - origin_cm[swizzled]) * 0.01   [meters]
// The axis swap flips handedness, so triangle winding is ALSO flipped
// (and materials use DoubleSide as belt-and-braces).

import * as THREE from 'three';
import { OrbitControls } from './lib/OrbitControls.js';
import { createRoadNetwork } from './roads.js';
import { createAgentSystem } from './agents.js';
import { createTransitSystem } from './transit.js';
import { createCameraSystem } from './cameras.js';
import { createCameraAnalysis } from './camanalysis.js';
import { solveHomography, applyHomography } from './homography.js';
import { createCitySystem } from './city.js';
import { createStreetLabels } from './labels.js';
import { createNetAgents } from './netagents.js';
import { createPhotorealTiles } from './tiles3d.js';
import { createDrapeField } from './drape.js';
import { FLAT_WORLD, FLAT_Y } from './flat.js';

// ---------------------------------------------------------------- config ---

const params = new URLSearchParams(location.search);
// The optional ?data= override points the viewer at an alternate LOCAL data dir, but it
// must stay a same-origin RELATIVE path: every layer concatenates DATA_DIR into fetch()
// URLs (manifest, tiles, lidar, buildings, roads, transit.json, cameras.json, …), so an
// unsanitized value like ?data=https://evil.com/ or ?data=//evil.com/ would redirect all
// of those requests off-origin (client-side request forgery, CodeQL
// js/client-side-request-forgery). Restrict it to path-safe characters — which drops the
// ':' in a scheme and (with the leading-slash strip) any protocol-relative or absolute
// authority — so the result can only ever resolve against our own origin.
function sanitizeDataDir(raw) {
  let cleaned = String(raw || '')
    .replace(/[^A-Za-z0-9_\-/]/g, '')   // path-safe chars only (no ':', '.', '\\', etc.)
    .replace(/^\/+/, '')                 // no absolute or '//authority' leading slash
    .replace(/\/+$/, '');                // trailing slash is re-added below
  // Hard allowlist: a relative path of safe segments ONLY — no scheme, no '//' authority,
  // no '..'. The positive regex test is also the sanitizing barrier that proves to static
  // analysis (CodeQL js/client-side-request-forgery) that every `fetch(DATA_DIR + …)` stays
  // same-origin — DATA_DIR is the sole user-derived input the data layers put in request
  // URLs. Anything not matching falls back to the default directory.
  if (/^[A-Za-z0-9_-]+(?:\/[A-Za-z0-9_-]+)*$/.test(cleaned)) {
    return cleaned + '/';
  }
  return 'data/';
}
const DATA_DIR = sanitizeDataDir(params.get('data'));
const CM_TO_M = 0.01;

const state = {
  manifest: null,
  originCm: [0, 0, 0],          // UE cm, subtracted from everything (doubles)
  terrain: { tiles: [], group: null, loaded: 0, failed: 0, opacity: 1.0 },
  lidar: {
    chunks: [], group: null, material: null,
    offsetCm: [0, 0, 0], originalCoordinates: null,
    budget: 6_000_000, pumping: false,
  },
  buildings: { tiles: [], group: null, loaded: 0, failed: 0, packed: null, boxes: null },
  roadnet: null,       // real road network + props from roads.json (roads.js)
  city: null,          // city-wide OSM ground plane + streets (city.js)
  cityRoads: null,     // raw city.json roads (for street labels)
  rawRoads: null,      // raw roads.json centrelines [x,y,z] (camera flow-analysis seed)
  labels: null,        // street-name labels (labels.js)
  netagents: null,     // shared-world agents from the twin server (netagents.js)
  transit: null,       // live Lextran transit layer (transit.js)
  cameras: null,       // live traffic-camera layer (cameras.js)
  agents: null,        // autonomous-agent simulation layer (agents.js)
  photoreal: null,     // opt-in Google Photorealistic 3D Tiles basemap (tiles3d.js)
  helpers: null,
  hasRealData: false,
};

// ------------------------------------------------------------- DOM refs ---

const $ = (id) => document.getElementById(id);
const el = {
  viewport: $('viewport'),
  manifestStatus: $('manifest-status'),
  terrainStatus: $('terrain-status'),
  lidarStatus: $('lidar-status'),
  fps: $('fps'),
  cursor: $('cursor-readout'),
  overlay: $('loading-overlay'),
  overlayText: $('loading-text'),
};

function setStatus(node, text, cls) {
  node.textContent = text;
  node.className = 'status' + (cls ? ' ' + cls : '') +
    (node.id === 'cursor-readout' ? ' mono' : '');
}
function showOverlay(text) {
  el.overlay.classList.remove('hidden');
  el.overlayText.textContent = text;
}
function hideOverlay() { el.overlay.classList.add('hidden'); }

// ---------------------------------------------------------- scene setup ---

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setClearColor(0x000000, 0); // CSS gradient sky shows through
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
el.viewport.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(
  55, window.innerWidth / window.innerHeight, 0.5, 50000);
camera.position.set(150, 120, 150);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.1;
controls.target.set(0, 0, 0);

state.terrain.group = new THREE.Group();
state.lidar.group = new THREE.Group();
state.buildings.group = new THREE.Group();
scene.add(state.terrain.group, state.lidar.group, state.buildings.group);
state.lidar.group.visible = false;   // LiDAR off by default (heavy on the GPU); lazy-loaded when enabled

// Lights — terrain uses unlit MeshBasic so it is unaffected, but buildings and
// the procedural city use MeshStandard and need light to shade (and show up).
const hemiLight = new THREE.HemisphereLight(0xbcd4ff, 0x40402f, 1.0);
const sunLight = new THREE.DirectionalLight(0xfff2e0, 1.5);
sunLight.position.set(0.6, 1.0, 0.35); // direction only (target stays at origin)
scene.add(hemiLight, sunLight);

// Real road network + props (streets, intersections, trees, cars, traffic
// signals) extracted from the aerial textures and draped on the terrain — see
// tools/extract_roads.py and roads.js. Loaded from data/roads.json (loadRoads()).

// orientation helpers shown until real data arrives
state.helpers = new THREE.Group();
const grid = new THREE.GridHelper(2000, 20, 0x44546a, 0x33404f);
grid.material.transparent = true;
grid.material.opacity = 0.35;
state.helpers.add(grid, new THREE.AxesHelper(60));
scene.add(state.helpers);

function onRealData() {
  if (!state.hasRealData) {
    state.hasRealData = true;
    state.helpers.visible = false;
  }
}

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// Photorealistic basemap (Google 3D Tiles) — opt-in, OFF by default, so the existing
// viewer is untouched until the user enables it AND supplies a key. See tiles3d.js.
state.photoreal = createPhotorealTiles({
  scene, camera, renderer, dataDir: DATA_DIR,
  onStatus: (s) => { const n = $('photoreal-status'); if (n) setStatus(n, 'photoreal: ' + s); },
  onCredits: (c) => { const n = $('photoreal-credits'); if (n) n.textContent = c; },
});

// Streaming ground-conform field: rebases overlays onto the Google photoreal surface
// per-location — replaces the old single global vertical offset that left everything
// floating away from the camera focus. See drape.js. No-op until photoreal is enabled.
state.drape = createDrapeField({
  getOurGroundY: (x, z) => ourGroundY(x, z),
  getPhotoreal: () => state.photoreal,
});

// ------------------------------------------------------------ coordinates ---

// UE cm (absolute, doubles) -> scene meters (after origin subtraction + swizzle)
function ueCmToScene(ux, uy, uz) {
  const o = state.originCm;
  return [
    (ux - o[0]) * CM_TO_M,
    (uz - o[2]) * CM_TO_M,
    (uy - o[1]) * CM_TO_M,
  ];
}
// UE FRotationMatrix from FRotator [pitch, yaw, roll] in degrees (row-major
// 3x3; UE convention: world = local_row_vector * M). Matches UnrealMath.cpp.
function ueRotationMatrix([pitchDeg, yawDeg, rollDeg]) {
  const d = Math.PI / 180;
  const SP = Math.sin(pitchDeg * d), CP = Math.cos(pitchDeg * d);
  const SY = Math.sin(yawDeg * d), CY = Math.cos(yawDeg * d);
  const SR = Math.sin(rollDeg * d), CR = Math.cos(rollDeg * d);
  return [
    CP * CY, CP * SY, SP,
    SR * SP * CY - CR * SY, SR * SP * SY + CR * CY, -SR * CP,
    -(CR * SP * CY + SR * SY), CY * SR - CR * SP * SY, CR * CP,
  ];
}

// scene meters -> UE cm (absolute)
function sceneToUeCm(sx, sy, sz) {
  const o = state.originCm;
  return [
    sx * 100 + o[0],
    sz * 100 + o[1],
    sy * 100 + o[2],
  ];
}

// ------------------------------------------------------------- manifest ---

function normalizeManifest(m) {
  // origin: prefer explicit origin_cm, else lidar offset, else first tile, else 0
  let origin = m.origin_cm || m.originCm ||
    (m.lidar && (m.lidar.offset_cm || m.lidar.offsetCm)) || null;

  const rawTiles = (m.terrain && m.terrain.tiles) || m.tiles || [];
  const tiles = rawTiles.map((t, i) => {
    const name = t.name || `tile_${i}`;
    return {
      name,
      mesh: t.mesh || t.mesh_bin || t.bin || `meshes/${name}.bin`,
      texture: t.texture || t.jpg || t.image || `textures/${name}.jpg`,
      translationCm: t.translation_cm || t.translationCm || [0, 0, 0],
      // optional scene-placement extras (UE FRotator [pitch, yaw, roll] deg)
      rotationDeg: t.rotation_deg || t.rotation_pyr_deg || [0, 0, 0],
      scale: t.scale || [1, 1, 1],
      visible: t.visible !== false,
    };
  });

  let lidar = null;
  if (m.lidar) {
    const chunks = (m.lidar.chunks || []).map((c, i) =>
      typeof c === 'string'
        ? { file: c, declaredCount: null }
        : { file: c.file || c.path || c.url || `lidar/chunk_${String(i).padStart(3, '0')}.bin`,
            declaredCount: c.count ?? c.points ?? null });
    lidar = {
      offsetCm: m.lidar.offset_cm || m.lidar.offsetCm || [0, 0, 0],
      originalCoordinates: m.lidar.original_coordinates ||
                           m.lidar.originalCoordinates || null,
      chunks,
    };
  }

  if (!origin && tiles.length) origin = tiles[0].translationCm;
  if (!origin) origin = [0, 0, 0];

  let buildings = null;
  if (m.buildings) {
    buildings = (m.buildings.tiles || []).map((b) => ({
      name: b.name,
      file: b.file,
      boundsMinCm: b.bounds_min_cm || b.boundsMinCm,
      boundsMaxCm: b.bounds_max_cm || b.boundsMaxCm,
      heightCm: b.height_cm || b.heightCm || 0,
      footprintAreaM2: b.footprint_area_m2 || b.footprintAreaM2 || 0,
      pointCount: b.point_count || b.pointCount || 0,
      vertexCount: b.vertex_count || b.vertexCount || 0,
      indexCount: b.index_count || b.indexCount || 0,
      groundYm: b.ground_y_m != null ? b.ground_y_m : (b.groundYm != null ? b.groundYm : null),
      bridge: !!b.bridge,
    }));
  }

  return { origin, tiles, lidar, buildings };
}

async function loadManifest() {
  let resp;
  try {
    resp = await fetch(DATA_DIR + 'manifest.json', { cache: 'no-cache' });
  } catch (e) {
    setStatus(el.manifestStatus, `manifest: network error (${e.message})`, 'error');
    return;
  }
  if (!resp.ok) {
    setStatus(el.manifestStatus,
      `manifest: ${DATA_DIR}manifest.json not found (HTTP ${resp.status}) — ` +
      `data not generated yet?`, 'error');
    setStatus(el.terrainStatus, 'terrain: no manifest', 'error');
    setStatus(el.lidarStatus, 'lidar: no manifest', 'error');
    return;
  }
  let m;
  try {
    m = await resp.json();
  } catch (e) {
    setStatus(el.manifestStatus, `manifest: invalid JSON (${e.message})`, 'error');
    return;
  }
  state.manifest = m;
  const norm = normalizeManifest(m);
  state.originCm = norm.origin.map(Number);
  setStatus(el.manifestStatus,
    `manifest: ok — ${norm.tiles.length} tiles, ` +
    `${norm.lidar ? norm.lidar.chunks.length : 0} lidar chunks, ` +
    `${norm.buildings ? norm.buildings.length : 0} buildings\n` +
    `origin_cm: [${state.originCm.map((v) => v.toFixed(0)).join(', ')}]`, 'ok');

  loadTerrain(norm.tiles);
  if (norm.lidar) {
    state.lidar.offsetCm = norm.lidar.offsetCm.map(Number);
    state.lidar.originalCoordinates = norm.lidar.originalCoordinates;
    state.lidar.chunks = norm.lidar.chunks.map((c) => ({
      ...c, status: 'pending', count: 0, points: null,
    }));
    // budget slider max = all chunks (when the manifest declares counts)
    const total = state.lidar.chunks.reduce(
      (s, c) => s + (c.declaredCount || 0), 0);
    if (total > 0) {
      const slider = $('point-budget');
      slider.max = Math.max(1, Math.ceil(total / 1e6));
      // Default to a lighter budget for framerate (the full ~12M point cloud is
      // brutal on integrated GPUs); the slider still reaches the full cloud.
      const defM = Math.min(slider.max, 5);
      slider.value = defM;
      $('point-budget-val').textContent = defM.toFixed(1);
      state.lidar.budget = defM * 1e6;
    }
    // Don't stream the point cloud until the user enables it (off by default for
    // framerate); toggling 'visible' lazy-loads it (see the lidar-visible handler).
    if ($('lidar-visible').checked) lidarPump();
    else setStatus(el.lidarStatus, 'lidar: off — tick “visible” to load the point cloud');
  } else {
    setStatus(el.lidarStatus, 'lidar: not in manifest', 'error');
  }
  if (norm.buildings) {
    loadBuildings(norm.buildings);
  } else {
    setStatus($('buildings-status'), 'buildings: not in manifest', 'error');
  }
}

// -------------------------------------------------------------- terrain ---

// mesh .bin contract (little-endian):
//   u32 vert_count, u32 index_count,
//   f32 positions[vert_count*3] (UE cm), f32 uvs[vert_count*2],
//   u32 indices[index_count] (triangle list)
function parseMeshBin(buffer) {
  if (buffer.byteLength < 8) throw new Error('mesh bin too small');
  const dv = new DataView(buffer);
  const vc = dv.getUint32(0, true);
  const ic = dv.getUint32(4, true);
  const need = 8 + vc * 12 + vc * 8 + ic * 4;
  if (buffer.byteLength < need) {
    throw new Error(`mesh bin truncated: need ${need} B, have ${buffer.byteLength} B`);
  }
  let off = 8;
  const pos = new Float32Array(buffer, off, vc * 3); off += vc * 12;
  const uv = new Float32Array(buffer, off, vc * 2); off += vc * 8;
  const idx = new Uint32Array(buffer, off, ic);
  return { vc, ic, pos, uv, idx };
}

function fallbackTexture(label) {
  const c = document.createElement('canvas');
  c.width = c.height = 256;
  const g = c.getContext('2d');
  for (let y = 0; y < 8; y++) {
    for (let x = 0; x < 8; x++) {
      g.fillStyle = (x + y) % 2 ? '#666e76' : '#3c444c';
      g.fillRect(x * 32, y * 32, 32, 32);
    }
  }
  g.fillStyle = '#ffcf5e';
  g.font = '20px monospace';
  g.fillText('no texture', 60, 120);
  g.fillText(label.slice(0, 18), 30, 150);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.flipY = false;
  return t;
}

const texLoader = new THREE.TextureLoader();

async function loadTile(tile) {
  const url = DATA_DIR + tile.mesh;
  const resp = await fetch(url);
  if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
  const { vc, ic, pos, uv, idx } = parseMeshBin(await resp.arrayBuffer());

  // doubles: scale/rotation/translation + origin handled BEFORE the f32 cast
  // UE world = (v * scale) * FRotationMatrix(rotator) + translation
  const o = state.originCm, t = tile.translationCm;
  const dx = t[0] - o[0], dy = t[1] - o[1], dz = t[2] - o[2];
  const M = ueRotationMatrix(tile.rotationDeg || [0, 0, 0]);
  const [sx, sy, sz] = tile.scale || [1, 1, 1];

  const outPos = new Float32Array(vc * 3);
  for (let i = 0; i < vc; i++) {
    const j = i * 3;
    const lx = pos[j] * sx, ly = pos[j + 1] * sy, lz = pos[j + 2] * sz;
    const wx = lx * M[0] + ly * M[3] + lz * M[6] + dx; // UE world x (rel. origin)
    const wy = lx * M[1] + ly * M[4] + lz * M[7] + dy; // UE world y
    const wz = lx * M[2] + ly * M[5] + lz * M[8] + dz; // UE world z
    outPos[j]     = wx * CM_TO_M; // three.x = ue.x
    outPos[j + 1] = FLAT_WORLD ? FLAT_Y : wz * CM_TO_M; // three.y = ue.z (pinned flat in flat mode)
    outPos[j + 2] = wy * CM_TO_M; // three.z = ue.y
  }

  // axis swap mirrors handedness -> flip triangle winding
  const triCount = Math.floor(ic / 3);
  if (triCount * 3 !== ic) console.warn(`${tile.name}: index count ${ic} not /3`);
  const outIdx = new Uint32Array(triCount * 3);
  for (let i = 0; i < triCount * 3; i += 3) {
    outIdx[i] = idx[i];
    outIdx[i + 1] = idx[i + 2];
    outIdx[i + 2] = idx[i + 1];
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(outPos, 3));
  geometry.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(uv), 2));
  geometry.setIndex(new THREE.BufferAttribute(outIdx, 1));
  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();

  const material = new THREE.MeshBasicMaterial({
    map: fallbackTexture(tile.name),
    side: THREE.DoubleSide, // belt-and-braces on top of the winding flip
  });
  applyOpacity(material);

  texLoader.load(
    DATA_DIR + tile.texture,
    (tex) => {
      tex.colorSpace = THREE.SRGBColorSpace;
      tex.flipY = false; // raw UVs + runtime uvFlip flag control orientation
      tex.anisotropy = Math.min(8, renderer.capabilities.getMaxAnisotropy());
      tex.wrapS = tex.wrapT = THREE.ClampToEdgeWrapping;
      material.map = tex;
      material.needsUpdate = true;
      tile.textureStatus = 'ok';
      updateTerrainStatus();
    },
    undefined,
    () => { tile.textureStatus = 'missing'; updateTerrainStatus(); },
  );
  tile.textureStatus = 'loading';

  const mesh = new THREE.Mesh(geometry, material);
  mesh.name = tile.name;
  state.terrain.group.add(mesh);
  tile.object = mesh;
  tile.stats = { vc, ic };
}

function applyOpacity(material) {
  material.opacity = state.terrain.opacity;
  material.transparent = state.terrain.opacity < 1;
  material.depthWrite = state.terrain.opacity >= 0.5;
  material.needsUpdate = true;
}

function updateTerrainStatus() {
  const tiles = state.terrain.tiles;
  if (!tiles.length) {
    setStatus(el.terrainStatus, 'terrain: no tiles in manifest', 'error');
    return;
  }
  const loaded = tiles.filter((t) => t.status === 'loaded').length;
  const failed = tiles.filter((t) => t.status === 'error');
  const hidden = tiles.filter((t) => t.status === 'hidden').length;
  const texMissing = tiles.filter((t) => t.textureStatus === 'missing').length;
  let txt = `terrain: ${loaded}/${tiles.length - hidden} tiles loaded`;
  if (hidden) txt += ` (${hidden} hidden)`;
  if (texMissing) txt += `, ${texMissing} textures missing`;
  if (failed.length) {
    txt += `\nmissing/failed: ${failed.slice(0, 4).map((t) => t.name).join(', ')}` +
      (failed.length > 4 ? ` +${failed.length - 4} more` : '');
  }
  setStatus(el.terrainStatus, txt, failed.length === tiles.length ? 'error' : (loaded ? 'ok' : null));
}

async function loadTerrain(tiles) {
  state.terrain.tiles = tiles;
  updateTerrainStatus();
  let firstLoad = true;
  for (const tile of tiles) {
    if (!tile.visible) { tile.status = 'hidden'; continue; } // hidden in level
    tile.status = 'loading';
    showOverlay(`terrain: loading ${tile.name}…`);
    try {
      await loadTile(tile);
      tile.status = 'loaded';
      onRealData();
      if (firstLoad) { firstLoad = false; resetView(); }
    } catch (e) {
      tile.status = 'error';
      tile.error = String(e.message || e);
      console.warn(`tile ${tile.name}:`, e);
    }
    updateTerrainStatus();
  }
  hideOverlay();
  resetView();
}

// ---------------------------------------------------------------- lidar ---

// lidar chunk .bin contract (little-endian):
//   u32 count, then count * { f32 x, f32 y, f32 z, u8 r, u8 g, u8 b, u8 a }
//   (16-byte stride; xyz in UE cm RELATIVE to lidar offset_cm)
function parseLidarChunk(buffer) {
  if (buffer.byteLength < 4) throw new Error('lidar chunk too small');
  const dv = new DataView(buffer);
  const count = dv.getUint32(0, true);
  if (buffer.byteLength < 4 + count * 16) {
    throw new Error(`lidar chunk truncated: need ${4 + count * 16} B, ` +
      `have ${buffer.byteLength} B`);
  }
  // typed-array views over the record region (byteOffset 4 is 4-aligned)
  const f32 = new Float32Array(buffer, 4, count * 4); // [x y z rgba] per record
  const u8 = new Uint8Array(buffer, 4, count * 16);

  const o = state.originCm, ofs = state.lidar.offsetCm;
  const dx = ofs[0] - o[0], dy = ofs[1] - o[1], dz = ofs[2] - o[2];

  const pos = new Float32Array(count * 3);
  const col = new Uint8Array(count * 3);
  for (let i = 0; i < count; i++) {
    const f = i * 4, p = i * 3, b = i * 16 + 12;
    pos[p]     = (f32[f]     + dx) * CM_TO_M; // three.x = ue.x
    pos[p + 1] = (f32[f + 2] + dz) * CM_TO_M; // three.y = ue.z
    pos[p + 2] = (f32[f + 1] + dy) * CM_TO_M; // three.z = ue.y
    col[p] = u8[b];
    col[p + 1] = u8[b + 1];
    col[p + 2] = u8[b + 2];
  }
  return { count, pos, col };
}

function lidarMaterial() {
  if (!state.lidar.material) {
    state.lidar.material = new THREE.PointsMaterial({
      vertexColors: true,
      sizeAttenuation: true,
      size: parseFloat($('point-size').value),
    });
  }
  return state.lidar.material;
}

async function loadLidarChunk(chunk) {
  chunk.status = 'loading';
  const resp = await fetch(DATA_DIR + chunk.file);
  if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
  const { count, pos, col } = parseLidarChunk(await resp.arrayBuffer());
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geometry.setAttribute('color', new THREE.BufferAttribute(col, 3, true));
  geometry.computeBoundingSphere();
  const points = new THREE.Points(geometry, lidarMaterial());
  points.name = chunk.file;
  state.lidar.group.add(points);
  chunk.points = points;
  chunk.count = count;
  chunk.status = 'loaded';
  onRealData();
}

function loadedLidarPoints() {
  return state.lidar.chunks.reduce(
    (s, c) => s + (c.status === 'loaded' ? c.count : 0), 0);
}

// chunks are shown in manifest order until the cumulative count passes budget
function applyLidarBudget() {
  let cum = 0;
  for (const c of state.lidar.chunks) {
    if (c.status !== 'loaded') continue;
    c.points.visible = cum < state.lidar.budget;
    cum += c.count;
  }
}

function updateLidarStatus() {
  const chunks = state.lidar.chunks;
  if (!chunks.length) {
    setStatus(el.lidarStatus, 'lidar: no chunks in manifest', 'error');
    return;
  }
  const loaded = chunks.filter((c) => c.status === 'loaded').length;
  const errors = chunks.filter((c) => c.status === 'error').length;
  const visPts = chunks.reduce(
    (s, c) => s + (c.status === 'loaded' && c.points.visible ? c.count : 0), 0);
  let txt = `lidar: ${loaded}/${chunks.length} chunks, ` +
    `${(visPts / 1e6).toFixed(2)}M pts shown ` +
    `(${(loadedLidarPoints() / 1e6).toFixed(2)}M loaded)`;
  if (errors) txt += `, ${errors} failed`;
  setStatus(el.lidarStatus, txt,
    errors === chunks.length ? 'error' : (loaded ? 'ok' : null));
}

// progressive loader: pulls chunks one at a time until budget is met
async function lidarPump() {
  if (state.lidar.pumping) return;
  state.lidar.pumping = true;
  try {
    for (const chunk of state.lidar.chunks) {
      if (loadedLidarPoints() >= state.lidar.budget) break;
      if (chunk.status !== 'pending') continue;
      const i = state.lidar.chunks.indexOf(chunk) + 1;
      showOverlay(`lidar: chunk ${i}/${state.lidar.chunks.length}…`);
      try {
        await loadLidarChunk(chunk);
        // first loaded chunk and no terrain yet: frame the view on it
        if (state.lidar.chunks.filter((c) => c.status === 'loaded').length === 1 &&
            !state.terrain.tiles.some((t) => t.status === 'loaded')) {
          resetView();
        }
      } catch (e) {
        chunk.status = 'error';
        chunk.error = String(e.message || e);
        console.warn(`lidar chunk ${chunk.file}:`, e);
      }
      applyLidarBudget();
      updateLidarStatus();
    }
  } finally {
    state.lidar.pumping = false;
    hideOverlay();
    applyLidarBudget();
    updateLidarStatus();
  }
}

// --------------------------------------------------------------- buildings ---

// building .bin contract (no UVs variant):
//   u32 vert_count, u32 index_count,
//   f32 positions[vert_count*3] (UE cm), u32 indices[index_count] (triangle list)
function parseBuildingBin(buffer) {
  if (buffer.byteLength < 8) throw new Error('building bin too small');
  const dv = new DataView(buffer);
  const vc = dv.getUint32(0, true);
  const ic = dv.getUint32(4, true);
  const need = 8 + vc * 12 + ic * 4;
  if (buffer.byteLength < need) {
    throw new Error(`building bin truncated: need ${need} B, have ${buffer.byteLength} B`);
  }
  const pos = new Float32Array(buffer, 8, vc * 3);
  const idx = new Uint32Array(buffer, 8 + vc * 12, ic);
  return { vc, ic, pos, idx };
}

function buildingHeightColor(heightM, minH, maxH) {
  const range = maxH - minH || 1;
  const t = (heightM - minH) / range;
  return new THREE.Color().setHSL(0.55 + t * 0.15, 0.6, 0.35 + t * 0.3);
}

let _buildingGreyMat = null;
function buildingGreyMaterial() {
  if (!_buildingGreyMat) {
    _buildingGreyMat = new THREE.MeshStandardMaterial({
      color: 0x8899aa, roughness: 0.8, metalness: 0.1, side: THREE.DoubleSide,
    });
  }
  return _buildingGreyMat;
}

function makeBuildingMaterial(bld, minH, maxH) {
  const mode = $('buildings-color-mode').value;
  if (mode === 'grey') {
    return buildingGreyMaterial().clone();
  }
  return new THREE.MeshStandardMaterial({
    color: buildingHeightColor(bld.heightCm / 100, minH, maxH),
    roughness: 0.8, metalness: 0.1, side: THREE.DoubleSide,
  });
}

async function loadBuilding(bld, minH, maxH) {
  const url = DATA_DIR + bld.file;
  const resp = await fetch(url);
  if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
  const { vc, ic, pos, idx } = parseBuildingBin(await resp.arrayBuffer());

  const o = state.originCm;
  const outPos = new Float32Array(vc * 3);
  for (let i = 0; i < vc; i++) {
    const j = i * 3;
    outPos[j]     = (pos[j]     - o[0]) * CM_TO_M;
    outPos[j + 1] = (pos[j + 2] - o[2]) * CM_TO_M;
    outPos[j + 2] = (pos[j + 1] - o[1]) * CM_TO_M;
  }

  // flip winding for handedness change (same as terrain)
  const triCount = Math.floor(ic / 3);
  const outIdx = new Uint32Array(triCount * 3);
  for (let i = 0; i < triCount * 3; i += 3) {
    outIdx[i] = idx[i];
    outIdx[i + 1] = idx[i + 2];
    outIdx[i + 2] = idx[i + 1];
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(outPos, 3));
  geometry.setIndex(new THREE.BufferAttribute(outIdx, 1));
  geometry.computeVertexNormals(); // MeshStandard needs normals to shade (else black)
  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();

  const material = makeBuildingMaterial(bld, minH, maxH);
  const mesh = new THREE.Mesh(geometry, material);
  mesh.name = bld.name;
  // Drop the building so its base sits on the terrain (ground_y_m baked by
  // tools/ground_buildings.py), unless it is a bridge crossing over a road.
  if (FLAT_WORLD && geometry.boundingBox) {
    mesh.position.y = FLAT_Y - geometry.boundingBox.min.y;   // base on the flat ground
  } else if (bld.groundYm != null && !bld.bridge && geometry.boundingBox) {
    mesh.position.y = bld.groundYm - geometry.boundingBox.min.y;
  }
  mesh.userData = { buildingName: bld.name, heightCm: bld.heightCm };
  state.buildings.group.add(mesh);
  bld.object = mesh;
  bld.stats = { vc, ic };
}

function updateBuildingsStatus() {
  const tiles = state.buildings.tiles;
  if (!tiles.length) {
    setStatus($('buildings-status'), 'buildings: no tiles in manifest', 'error');
    return;
  }
  const loaded = tiles.filter((t) => t.status === 'loaded').length;
  const failed = tiles.filter((t) => t.status === 'error');
  let txt = `buildings: ${loaded}/${tiles.length} loaded`;
  if (failed.length) {
    txt += `\nfailed: ${failed.slice(0, 3).map((t) => t.name).join(', ')}` +
      (failed.length > 3 ? ` +${failed.length - 3} more` : '');
  }
  setStatus($('buildings-status'), txt,
    failed.length === tiles.length ? 'error' : (loaded ? 'ok' : null));
}

// Dispatcher: prefer the single packed buffer (tools/pack_buildings.py) — one
// fetch + one draw call instead of ~3,100 — and fall back to the per-building
// stream if the pack isn't present.
async function loadBuildings(blds) {
  state.buildings.tiles = blds || [];
  try {
    const r = await fetch(DATA_DIR + 'buildings.pack.json', { cache: 'no-cache' });
    if (r.ok) { await loadBuildingsPacked(await r.json()); return; }
  } catch (e) { /* no pack -> per-building fallback */ }
  await loadBuildingsPerTile(blds);
}

// Fast path: one packed mesh. Positions are already final scene-space (axis-
// swapped, origin-subtracted, ground-dropped by the packer), so we upload them
// directly and compute normals once. Picking + colour-by-height are preserved via
// the per-building ranges in the sidecar; agent collision uses the baked AABBs.
async function loadBuildingsPacked(meta) {
  showOverlay('buildings: loading packed mesh…');
  let buf;
  try {
    const r = await fetch(DATA_DIR + (meta.bin || 'buildings.pack.bin'), { cache: 'no-cache' });
    if (!r.ok) throw new Error('HTTP ' + r.status);
    buf = await r.arrayBuffer();
  } catch (e) {
    hideOverlay();
    console.warn('packed buildings failed, falling back per-tile:', e);
    return loadBuildingsPerTile(state.buildings.tiles);
  }
  const dv = new DataView(buf);
  // header: 'BPK1' (4) + u32 count, totalVerts, totalIndices
  const count = dv.getUint32(4, true), tv = dv.getUint32(8, true), ti = dv.getUint32(12, true);
  let off = 16;
  const pos = new Float32Array(buf, off, tv * 3); off += tv * 12;
  const idx = new Uint32Array(buf, off, ti);

  const blds = meta.buildings || [];
  // Flat world: shift each building's vertices so its base sits on FLAT_Y (otherwise
  // campus buildings, baked at the old terrain height, would float once the ground is
  // flattened). Done before the AABBs below so collision/raycast stay in sync.
  if (FLAT_WORLD) {
    for (const b of blds) {
      const base = (b.min && b.min[1] != null) ? b.min[1] : null;
      if (base == null) continue;
      const shift = FLAT_Y - base;
      if (!shift) continue;
      for (let v = b.vStart; v < b.vStart + b.vCount; v++) pos[v * 3 + 1] += shift;
      b.min[1] += shift; b.max[1] += shift;
    }
  }
  // Single-pass min/max (not Math.min(...heights)): the full-city pack is ~114k
  // buildings, and spreading an array that large into a call throws RangeError
  // ("Maximum call stack size exceeded") once it exceeds the engine's argument
  // ceiling. A loop is O(N), allocates nothing, and is the only safe form here.
  let minH = Infinity, maxH = -Infinity;
  for (const b of blds) {
    const h = b.heightM;
    if (h < minH) minH = h;
    if (h > maxH) maxH = h;
  }
  const mode = $('buildings-color-mode').value;
  const col = new Float32Array(tv * 3);
  const c = new THREE.Color();
  for (const b of blds) {
    if (mode === 'grey') c.setHex(0x8899aa);
    else c.copy(buildingHeightColor(b.heightM, minH, maxH));
    for (let v = b.vStart; v < b.vStart + b.vCount; v++) {
      col[v * 3] = c.r; col[v * 3 + 1] = c.g; col[v * 3 + 2] = c.b;
    }
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
  geo.setIndex(new THREE.BufferAttribute(idx, 1));
  geo.computeVertexNormals();
  geo.computeBoundingBox(); geo.computeBoundingSphere();
  const material = new THREE.MeshStandardMaterial({
    vertexColors: true, roughness: 0.8, metalness: 0.1, side: THREE.DoubleSide,
  });
  const mesh = new THREE.Mesh(geo, material);
  mesh.name = 'buildings-packed';
  mesh.userData = { packed: true, buildings: blds };
  state.buildings.group.add(mesh);
  state.buildings.packed = { mesh, meta, minH, maxH };
  // per-building scene AABBs for the agent collision broad-phase (no un-merging)
  state.buildings.boxes = blds.map((b, i) => ({
    id: i, name: b.name, min: b.min, max: b.max,
    cx: (b.min[0] + b.max[0]) / 2, cz: (b.min[2] + b.max[2]) / 2,
  }));

  hideOverlay();
  onRealData();
  setStatus($('buildings-status'),
    `buildings: ${count} packed (1 fetch, 1 draw call)`, 'ok');
}

async function loadBuildingsPerTile(blds) {
  state.buildings.tiles = blds;
  updateBuildingsStatus();
  if (!blds.length) return;

  // Compute height range for color mapping (single-pass — see packed-path note re: RangeError)
  let minH = Infinity, maxH = -Infinity;
  for (const b of blds) {
    const h = b.heightCm / 100;
    if (h < minH) minH = h;
    if (h > maxH) maxH = h;
  }

  let firstLoad = true;
  for (const bld of blds) {
    bld.status = 'loading';
    showOverlay(`buildings: loading ${bld.name}…`);
    try {
      await loadBuilding(bld, minH, maxH);
      bld.status = 'loaded';
      onRealData();
      if (firstLoad) { firstLoad = false; }
    } catch (e) {
      bld.status = 'error';
      bld.error = String(e.message || e);
      console.warn(`building ${bld.name}:`, e);
    }
    updateBuildingsStatus();
  }
  hideOverlay();
}

// faceIndex (into the packed index buffer) -> owning building, via the sorted
// per-building index ranges (binary search).
function buildingAtFace(blds, faceIndex) {
  const ii = faceIndex * 3;
  let lo = 0, hi = blds.length - 1;
  while (lo <= hi) {
    const m = (lo + hi) >> 1, b = blds[m];
    if (ii < b.iStart) hi = m - 1;
    else if (ii >= b.iStart + b.iCount) lo = m + 1;
    else return b;
  }
  return null;
}

// ------------------------------------------------------------------ city ---

// Load the city-wide OSM context (ground plane + streets) from data/city.json.
// Returns the city ground elevation (used by the transit layer for off-campus
// buses); resolves to a sane default if the file isn't present.
async function loadCity() {
  let data;
  try {
    const r = await fetch(DATA_DIR + 'city.json', { cache: 'no-cache' });
    if (!r.ok) throw new Error('HTTP ' + r.status);
    data = await r.json();
  } catch (e) {
    setStatus($('city-status'),
      `city: ${DATA_DIR}city.json not found — run tools/osm_city.py`, null);
    return 285;
  }
  state.city = createCitySystem(data, { scene });
  state.cityRoads = data.roads || [];   // kept for street-name labels
  scene.add(state.city.group);
  onRealData();
  const s = state.city.stats;
  setStatus($('city-status'),
    `city: ${s.streets} streets (${Math.round(s.segments / 1000)}k segments)\n` +
    `ground plane @ ${s.groundY.toFixed(0)} m`, 'ok');
  return state.city.groundY;
}

// ----------------------------------------------------------------- roads ---

// Load the road network extracted from the aerial textures (already draped on
// terrain elevation by tools/extract_roads.py), build ribbons + props.
async function loadRoads() {
  let data;
  try {
    const resp = await fetch(DATA_DIR + 'roads.json', { cache: 'no-cache' });
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    data = await resp.json();
  } catch (e) {
    setStatus($('road-status'), `roads: ${DATA_DIR}roads.json not found — ` +
      `run tools/extract_roads.py`, 'error');
    return;
  }
  // Optional machine-readable signal model (real signalised intersections + the
  // agent API). Missing -> the viewer degrades to the legacy random-mast fallback.
  let signalModel = null;
  try {
    const sresp = await fetch(DATA_DIR + 'signals.json', { cache: 'no-cache' });
    if (sresp.ok) signalModel = await sresp.json();
  } catch (e) { /* no signals.json: legacy fallback */ }

  // Flat world: pin every road/intersection/crosswalk vertex to FLAT_Y before the
  // network (and the signal model the agents query) is built, so ribbons, lane
  // markings, stop bars, crosswalks, signal masts, and the agent stop-line geometry
  // all sit at one elevation. roads.js then needs no flat-specific code.
  if (FLAT_WORLD) {
    for (const r of data.roads || []) for (const p of r.pts || []) p[1] = FLAT_Y;
    for (const it of data.intersections || []) if (Array.isArray(it) && it.length >= 3) it[1] = FLAT_Y;
    if (signalModel) for (const it of signalModel.intersections || []) {
      if (it.center) it.center[1] = FLAT_Y;
      for (const leg of it.legs || []) if (leg && leg.stopPoint) leg.stopPoint[1] = FLAT_Y;
      for (const xw of it.crosswalks || []) if (xw && xw.y != null) xw.y = FLAT_Y;
    }
  }

  state.roadnet = createRoadNetwork(data, { trees: false, cars: false, signalModel });
  scene.add(state.roadnet.group);
  // Keep the raw campus centrelines ([x,y,z]) for the camera flow-analysis tool, which
  // borrows the nearest road heading near a camera to seed a calibration (see camanalysis.js).
  state.rawRoads = data.roads || [];

  // Conform the road network (ribbons, lane markings, crosswalks, intersection pads,
  // signals + props) onto the photoreal surface: cache its baked geometry in the drape
  // field and size the field grid to the network's extent. The field then lifts each
  // vertex onto Google's mesh as tiles stream in (no-op until photoreal is enabled).
  if (state.drape) {
    state.drape.registerTree(state.roadnet.group);
    const _bb = new THREE.Box3().setFromObject(state.roadnet.group);
    if (Number.isFinite(_bb.min.x)) state.drape.setBounds(_bb.min.x, _bb.min.z, _bb.max.x, _bb.max.z);
  }

  // City-wide OSM context (flat ground plane + the full street network) so the
  // entire Lextran service area has ground + streets beyond the ~2x3 km campus
  // tiles. Optional (data/city.json from tools/osm_city.py); campus works without
  // it. Its ground elevation is where off-campus buses ride.
  const cityGroundY = await loadCity();

  // Live Lextran transit layer: baked route lines + stops (data/transit.json) plus
  // moving buses / arrivals / alerts proxied at runtime by tools/twin_server.py
  // (/api/transit/*). Created unconditionally — it renders routes+stops with no
  // proxy and live buses with no transit.json, and never throws into the loop.
  // Drapes buses on the campus road ribbons + terrain, and on the city plane
  // (cityGroundY) once they roam past the campus tiles.
  state.transit = createTransitSystem({
    scene, dataDir: DATA_DIR, proxyBase: '', groundY: cityGroundY,
    drapeOffsetAt: (x, z) => (state.drape ? state.drape.offsetAt(x, z) : 0),
    groups: {
      terrain: state.terrain.group,
      roadRibbons: () => state.roadnet && state.roadnet.layers.roads,
    },
  });
  scene.add(state.transit.group);
  startTransitStatusPolling();

  // Live traffic-camera layer: camera markers snapped to the twin intersection each
  // one watches (baked data/cameras.json from tools/lex_cameras.py), plus the live
  // tokenized HLS stream URLs proxied at runtime by tools/twin_server.py
  // (/api/cameras/*). Click a marker (or a list row) to open the real-time stream in
  // a picture-in-picture panel — just the video, no detection. Same graceful-
  // degradation contract as transit: markers with no proxy, live video with it.
  state.cameras = createCameraSystem({
    scene, dataDir: DATA_DIR, proxyBase: '', groundY: cityGroundY,
    groups: { terrain: state.terrain.group },
  });
  scene.add(state.cameras.group);

  // Street-name labels (campus streets + major city arterials), one per unique
  // name, distance-culled each frame so only nearby ones draw.
  state.labels = createStreetLabels(
    { roads: data.roads || [], cityRoads: state.cityRoads || [], cityY: cityGroundY },
    { maxLabels: 2500 });
  const lblCb = $('labels-visible');
  state.labels.group.visible = lblCb ? lblCb.checked : true;
  scene.add(state.labels.group);

  // Shared-world agents from the authoritative twin server (tools/twin_server.py):
  // renders every agent in the shared world, including ones spawned by other clients'
  // scripts. Backs off to nothing when the page isn't served by the twin server.
  state.netagents = createNetAgents({ scene, base: '', pollMs: 120,
    drapeOffsetAt: (x, z) => (state.drape ? state.drape.offsetAt(x, z) : 0),
    camCarsVisible: $('road-cars') ? $('road-cars').checked : true,        // Roads & props "cars" toggle
    camLabelsVisible: $('road-car-labels') ? $('road-car-labels').checked : true });
  const naCb = $('netagents-visible');
  state.netagents.group.visible = naCb ? naCb.checked : true;
  scene.add(state.netagents.group);

  // Autonomous-agent simulation layer (cars/trucks/robots/drones with camera,
  // position, collision-detection, and ground/surface sensors). Created
  // unconditionally — it works with or without signals.json. Terrain/buildings
  // may still be streaming; the ground raycast degrades to surface:'none' until
  // they arrive, then re-snaps automatically (see agents.js).
  state.agents = createAgentSystem({
    THREE, scene, renderer,
    viewerCamera: camera, controls,
    groups: {
      terrain: state.terrain.group,
      buildings: state.buildings.group,
      // packed buildings expose per-building AABBs here (one merged render mesh has
      // no per-building children for the broad-phase to walk); null in legacy mode.
      buildingBoxes: () => state.buildings.boxes,
      roadRibbons: () => state.roadnet && state.roadnet.layers.roads,
    },
    coords: {
      ueCmToScene, sceneToUeCm, ueRotationMatrix,
      originCm: () => state.originCm,
      originalCoordinates: () => state.lidar.originalCoordinates,
    },
    signals: () => state.roadnet && state.roadnet.signals,
    transit: () => state.transit && state.transit.transit,
  });
  scene.add(state.agents.group);

  // Expose the live controllers for external/console autonomy. MERGE (never
  // clobber): the old code only set window.__twin inside `if (signals)`, which
  // dropped the whole twin (and now the agents API) when signals.json is absent.
  window.__twin = Object.assign(window.__twin || {}, {
    signals: state.roadnet.signals || null,
    model: signalModel,
    agents: state.agents,
    transit: state.transit.transit,
    cameras: state.cameras.cameras,
  });

  onRealData();
  // if roads arrive before/without any terrain or lidar, frame them
  if (!state.terrain.tiles.some((t) => t.status === 'loaded') &&
      !state.lidar.chunks.some((c) => c.status === 'loaded')) resetView();
  updateRoadStatus();
}

function updateRoadStatus() {
  const node = $('road-status');
  if (!node || !state.roadnet) return;
  const s = state.roadnet.stats;
  const model = window.__twin && window.__twin.model;
  let line2;
  if (model) {
    const c = { signal: 0, stop: 0, uncontrolled: 0 };
    for (const it of model.intersections) c[it.control] = (c[it.control] || 0) + 1;
    line2 = `${c.signal} signalised, ${c.stop} stop-controlled, ${c.uncontrolled} uncontrolled`;
  } else {
    line2 = `${s.trees} trees, ${s.cars} cars, ${s.signals} signals`;
  }
  setStatus(node,
    `roads: ${s.roads} (${s.km} km), ${s.intersections} intersections\n${line2}`, 'ok');
}

// ------------------------------------------------------------------- UI ---

$('terrain-visible').addEventListener('change', () => {
  applyBaseLayerVis();   // AND with the photoreal "replace" hide so they can't desync
});
$('lidar-visible').addEventListener('change', (e) => {
  state.lidar.group.visible = e.target.checked;
  if (e.target.checked) lidarPump();   // lazy-load the cloud the first time it's enabled
});
$('terrain-opacity').addEventListener('input', (e) => {
  state.terrain.opacity = parseFloat(e.target.value);
  $('terrain-opacity-val').textContent = state.terrain.opacity.toFixed(2);
  for (const t of state.terrain.tiles) {
    if (t.object) applyOpacity(t.object.material);
  }
});
$('terrain-wireframe').addEventListener('change', (e) => {
  for (const t of state.terrain.tiles) {
    if (t.object) t.object.material.wireframe = e.target.checked;
  }
});
$('point-size').addEventListener('input', (e) => {
  const v = parseFloat(e.target.value);
  $('point-size-val').textContent = v.toFixed(2);
  if (state.lidar.material) state.lidar.material.size = v;
});
$('point-budget').addEventListener('input', (e) => {
  const v = parseFloat(e.target.value);
  $('point-budget-val').textContent = v.toFixed(1);
  state.lidar.budget = Math.round(v * 1e6);
  applyLidarBudget();
  updateLidarStatus();
  lidarPump(); // load more if budget grew
});
$('camera-reset').addEventListener('click', () => { exitDrive(); exitFollow(); resetView(); });

$('road-visible').addEventListener('change', (e) => {
  if (state.roadnet) state.roadnet.group.visible = e.target.checked;
});
for (const key of ['roads', 'markings', 'crosswalks', 'signals']) {
  const cb = $('road-' + key);
  if (cb) cb.addEventListener('change', (e) => {
    if (state.roadnet) state.roadnet.layers[key].visible = e.target.checked;
  });
}
// "cars" = the live traffic-stream (camera-detected) cars in the shared world, not the
// static prop cars — toggle their visibility via netagents.
$('road-cars')?.addEventListener('change', (e) => {
  if (state.netagents) state.netagents.setCamCarsVisible(e.target.checked);
});
$('road-car-labels')?.addEventListener('change', (e) => {
  if (state.netagents) state.netagents.setCamLabelsVisible(e.target.checked);
});
$('labels-visible')?.addEventListener('change', (e) => {
  if (state.labels) state.labels.setVisible(e.target.checked);
});
$('netagents-visible')?.addEventListener('change', (e) => {
  if (state.netagents) state.netagents.setVisible(e.target.checked);
});

// Collapsible panel sections — click a section's legend to fold/unfold it.
for (const lg of document.querySelectorAll('#panel fieldset legend')) {
  lg.addEventListener('click', () => lg.parentElement.classList.toggle('collapsed'));
}

// ------------------------------------------------------------------ city ---
$('city-visible')?.addEventListener('change', (e) => {
  if (state.city) state.city.group.visible = e.target.checked;
});
$('city-ground')?.addEventListener('change', () => {
  applyBaseLayerVis();   // AND with the photoreal "replace" hide so they can't desync
});
$('city-streets')?.addEventListener('change', (e) => {
  if (state.city) state.city.layers.streets.visible = e.target.checked;
});

// ----------------------------------------------------- photorealistic 3D ---
// When the Google photoreal mesh is on (with "replace" checked) our grey OSM extrusions
// + ground duplicate what the mesh already shows, so hide them. Overlays — roads,
// cameras, agents, transit, labels — stay on top, exactly like the reference map.
// Base-layer visibility = (its own checkbox) AND NOT (photoreal hiding the base), so the
// per-layer checkboxes and the photoreal hide can't fight each other in either direction.
function baseHidden() {
  return !!($('photoreal-visible')?.checked && $('photoreal-replace')?.checked);
}
function applyBaseLayerVis() {
  const hide = baseHidden();
  const checked = (id) => ($(id) ? $(id).checked : true);
  if (state.buildings?.group) state.buildings.group.visible = checked('buildings-visible') && !hide;
  if (state.terrain?.group) state.terrain.group.visible = checked('terrain-visible') && !hide;
  if (state.city?.layers?.ground) state.city.layers.ground.visible = checked('city-ground') && !hide;
}
$('photoreal-visible')?.addEventListener('change', (e) => {
  if (state.photoreal) state.photoreal.setVisible(e.target.checked);
  applyBaseLayerVis();
});
$('photoreal-opacity')?.addEventListener('input', (e) => {
  const v = parseFloat(e.target.value);
  $('photoreal-opacity-val').textContent = v.toFixed(2);
  if (state.photoreal) state.photoreal.setOpacity(v);
});
$('photoreal-replace')?.addEventListener('change', applyBaseLayerVis);
$('photoreal-detail')?.addEventListener('input', (e) => {
  const v = parseFloat(e.target.value);
  $('photoreal-detail-val').textContent = String(v);
  if (state.photoreal) state.photoreal.setDetail(v);   // lower px err = higher fidelity
  const a = $('photoreal-adaptive'); if (a) a.checked = false;   // manual drag = stop auto-FPS
});
// The photorealistic mesh has REAL elevation; in flat mode our overlays are pinned to
// FLAT_Y and get buried under it. This reloads in real-elevation mode (?flat=0) with the
// layer auto-on, so roads/labels/traffic drape on the photoreal ground.
$('photoreal-realelev')?.addEventListener('click', () => {
  const p = new URLSearchParams(location.search);
  p.set('flat', '0');
  p.set('photoreal', '1');
  location.search = p.toString();
});
$('photoreal-key-save')?.addEventListener('click', () => {
  const k = ($('photoreal-key')?.value || '').trim();
  if (!k || !state.photoreal) return;
  state.photoreal.setKey(k);   // stores locally + persists to .env via the server
  const cb = $('photoreal-visible');
  if (cb && !cb.checked) {
    cb.checked = true;
    state.photoreal.setVisible(true);
    applyBaseLayerVis();
  }
  $('photoreal-key').value = '';
});
// Reflect the persisted detail value + a configured key so we don't re-prompt.
if ($('photoreal-detail') && state.photoreal && state.photoreal.detail != null) {
  $('photoreal-detail').value = String(state.photoreal.detail);
  $('photoreal-detail-val').textContent = String(state.photoreal.detail);
}
state.photoreal?.probeKey?.().then((r) => {
  if (r && r.key && $('photoreal-key')) {
    $('photoreal-key').placeholder = 'key loaded from .env — just tick "visible"';
  }
}).catch(() => {});
// Photorealistic Google-tiles basemap is OFF by default — the twin ships as the stylised
// flat 3D city. Opt in with ?photoreal=1 (best with ?flat=0 for real elevation) or tick
// "visible" in the panel. Tiles stream LIVE from the Map Tiles API using a server/.env key;
// the layer (tiles3d.js) and the ground-conform field (drape.js) are retained but dormant.
if (params.get('photoreal') === '1') {
  const cb = $('photoreal-visible');
  if (cb) cb.checked = true;
  if (state.photoreal) state.photoreal.setVisible(true);
  applyBaseLayerVis();
}

// --------------------------------------------------------------- transit ---
// Live Lextran layer (transit.js). Master + per-layer toggles null-guard on
// state.transit (created after roads.json loads). Status is refreshed on a timer
// from transit.status() as the proxy polls come in.
$('transit-visible')?.addEventListener('change', (e) => {
  if (state.transit) state.transit.group.visible = e.target.checked;
});
for (const key of ['routes', 'stops', 'buses']) {
  const cb = $('transit-' + key);
  if (cb) cb.addEventListener('change', (e) => {
    if (state.transit) state.transit.layers[key].visible = e.target.checked;
  });
}

let _transitStatusTimer = null;
function startTransitStatusPolling() {
  if (_transitStatusTimer) return;
  _transitStatusTimer = setInterval(updateTransitStatus, 1000);
  updateTransitStatus();
}
function updateTransitStatus() {
  const node = $('transit-status');
  if (!node || !state.transit) return;
  const s = state.transit.transit.status();
  const proxyTxt = { ok: 'live', mock: 'mock', offline: 'proxy offline', error: 'feed error',
                     connecting: 'connecting…' }[s.mode === 'mock' ? 'mock' : s.proxy] || s.proxy;
  let txt = `transit: ${s.routes} routes, ${s.stops} stops\n${s.buses} buses (${proxyTxt})`;
  setStatus(node, txt, s.proxy === 'ok' ? 'ok' : (s.proxy === 'offline' ? 'error' : null));
  const alertsNode = $('transit-alerts');
  if (alertsNode) {
    const alerts = state.transit.transit.getAlerts();
    if (s.proxy === 'offline') {
      alertsNode.textContent = 'run  python -m tools.twin_server  for live buses';
    } else if (alerts.length) {
      const a = alerts[0];
      alertsNode.textContent = `⚠ ${alerts.length} alert${alerts.length > 1 ? 's' : ''}: ` +
        (a.header || a.effect || '').slice(0, 80);
    } else {
      alertsNode.textContent = 'no active service alerts';
    }
  }
  refreshBusList();
  const naNode = $('netagents-status');
  if (naNode && state.netagents) {
    const ns = state.netagents.status();
    setStatus(naNode, ns.server === 'ok'
      ? `shared world: ${ns.count} agent(s) live`
      : 'shared world: no twin server (run python -m tools.twin_server)',
      ns.server === 'ok' ? 'ok' : null);
  }
  refreshNetAgentList();
  updateCameraStatus();
}

// ---------------------------------------------------------- traffic cameras ---
// Live camera layer (cameras.js). Master + markers toggles null-guard on
// state.cameras. The list is filterable (113 cameras); a click on a row — or on a
// camera marker in the 3D scene — opens the real-time HLS stream in a PiP panel.
$('cameras-visible')?.addEventListener('change', (e) => {
  if (state.cameras) state.cameras.group.visible = e.target.checked;
});
$('cameras-markers')?.addEventListener('change', (e) => {
  if (state.cameras) state.cameras.layers.markers.visible = e.target.checked;
});
$('cameras-filter')?.addEventListener('input', () => refreshCameraList(true));

function updateCameraStatus() {
  const node = $('cameras-status');
  if (!node || !state.cameras) return;
  const s = state.cameras.cameras.status();
  const proxyTxt = { ok: 'live streams', offline: 'proxy offline', error: 'feed error',
                     connecting: 'connecting…' }[s.proxy] || s.proxy;
  setStatus(node, `cameras: ${s.cameras} (${s.matched} on intersections)\n${proxyTxt}`,
    s.proxy === 'ok' ? 'ok' : (s.proxy === 'offline' ? 'error' : null));
  refreshCameraList();
  // if the PiP panel is open on a still and the proxy just came online, go live
  if (pip.id != null && !pip.live) {
    const url = state.cameras.cameras.streamUrl(pip.id);
    if (url) attachStream(pip.id, url);
  }
}

let _camSig = '';
function refreshCameraList(force) {
  const node = $('cameras-list');
  if (!node) return;
  if (!state.cameras) { node.textContent = '—'; return; }
  const q = ($('cameras-filter')?.value || '').trim().toLowerCase();
  let list = state.cameras.cameras.list();
  if (q) list = list.filter((c) => (c.name || '').toLowerCase().includes(q) ||
    (c.id || '').toLowerCase().includes(q));
  list.sort((a, b) => String(a.name).localeCompare(String(b.name), undefined, { numeric: true }));
  const sig = list.map((c) => c.id).join('|') + '#' + (pip.id || '') + '#' + q +
    '#' + (state.cameras.cameras.hasProxy() ? '1' : '0');
  if (!force && sig === _camSig) return;
  _camSig = sig;
  node.textContent = '';
  if (!list.length) {
    const d = document.createElement('div');
    d.className = 'hint'; d.style.margin = '2px 0';
    d.textContent = q ? 'no cameras match' : 'no cameras — run python -m tools.lex_cameras';
    node.appendChild(d);
    return;
  }
  for (const c of list) {
    const row = document.createElement('div');
    row.className = 'bus-row' + (String(c.id) === String(pip.id) ? ' active' : '');
    const dot = document.createElement('span');
    dot.className = 'cam-dot';
    dot.style.background = c.matched ? '#27c4c4' : '#e0a83a';
    dot.title = c.matched ? `on ${c.intersection} (${c.snapDist} m)` : 'no twin intersection nearby';
    const info = document.createElement('span');
    info.textContent = c.name || c.id;
    row.appendChild(dot); row.appendChild(info);
    row.addEventListener('click', () => openCamera(c.id));
    node.appendChild(row);
  }
}

// ------- camera PiP: real-time HLS stream + detection overlay -------
const pip = { id: null, hls: null, live: false, stillTimer: null };
// Tell the detector(s) which camera is being viewed so a --follow-active detector only
// burns GPU on the camera someone is actually watching (per-active-camera perf bounding).
// The signal is per-camera and TTL'd server-side, so we refresh it on a heartbeat while
// the PiP is open (otherwise it would expire mid-view and idle the detector), and clear
// only THIS camera on close (not every viewer's).
let _activeTimer = null;
function _postActive(body) {
  fetch('/api/cameras/active', { method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body) }).catch(() => {});
}
function setActiveCamera(id) {
  if (_activeTimer) { clearInterval(_activeTimer); _activeTimer = null; }
  if (id == null) { _postActive({ camera: null }); return; }
  _postActive({ camera: id });
  _activeTimer = setInterval(() => _postActive({ camera: id }), 5000);  // < server ACTIVE_TTL
}
function clearActiveCamera(id) {
  if (_activeTimer) { clearInterval(_activeTimer); _activeTimer = null; }
  _postActive({ camera: id, active: false });   // remove only this camera's viewer signal
}
// Move the orbit camera to look at a traffic camera's intersection (the scene position
// baked in cameras.json — the snapped junction centre for matched cameras). Clicking a
// camera should take you there, so we drop any follow/drive lock first, then frame the
// junction with a comfortable oblique view.
function flyToCamera(cam) {
  if (!cam || !cam.position) return;
  exitDrive(); exitFollow();
  const x = cam.position[0], z = cam.position[2];
  const gy = FLAT_WORLD ? FLAT_Y : (Number.isFinite(cam.position[1]) ? cam.position[1]
    : ((state.city && state.city.groundY) || 285));
  controls.target.set(x, gy, z);
  camera.position.set(x + 70, gy + 85, z + 70);   // oblique view of the intersection
  controls.update();
}
function openCamera(id) {
  if (!state.cameras) return;
  const cam = state.cameras.cameras.get(id);
  if (!cam) return;
  flyToCamera(cam);   // take the 3D view to this camera's intersection
  pip.id = id;
  $('cam-pip').classList.remove('hidden');
  $('cam-pip').dispatchEvent(new CustomEvent('cam-open'));   // reset calibration tool
  setActiveCamera(id);   // tell detectors which camera is being viewed (perf bounding)
  camDetect.refresh();   // reflect whether a server-side YOLO detector is already running
  $('cam-pip-title').textContent = cam.name + (cam.matched ? '' : ' · (no junction)');
  const url = state.cameras.cameras.streamUrl(id);
  if (url) {
    attachStream(id, url);
  } else {
    showStill(id);   // no fresh URL yet — token-free thumbnail until the proxy answers
  }
  refreshCameraList(true);
}
function teardownStream() {
  const v = $('cam-pip-video');
  if (pip.hls) { try { pip.hls.destroy(); } catch (e) { /* noop */ } pip.hls = null; }
  if (pip.stillTimer) { clearInterval(pip.stillTimer); pip.stillTimer = null; }
  pip.live = false;
  try { v.pause(); } catch (e) { /* noop */ }
  v.removeAttribute('poster');
  v.removeAttribute('src');
  try { v.load(); } catch (e) { /* noop */ }
}
function attachStream(id, url) {
  teardownStream();
  const v = $('cam-pip-video');
  const note = $('cam-pip-note');
  const cam = state.cameras.cameras.get(id);
  const Hls = window.Hls;
  if (v.canPlayType('application/vnd.apple.mpegurl')) {   // Safari / iOS native HLS
    v.src = url; v.play().catch(() => {});
    pip.live = true;
  } else if (Hls && Hls.isSupported()) {
    const hls = new Hls({ liveSyncDurationCount: 3, lowLatencyMode: true, backBufferLength: 30 });
    hls.on(Hls.Events.MANIFEST_PARSED, () => v.play().catch(() => {}));
    hls.on(Hls.Events.ERROR, (evt, data) => {
      if (!data || !data.fatal) return;
      if (data.type === Hls.ErrorTypes.NETWORK_ERROR) hls.startLoad();   // token may have rotated
      else { note.textContent = 'stream error — falling back to snapshot'; showStill(id); }
    });
    hls.loadSource(url); hls.attachMedia(v);
    pip.hls = hls; pip.live = true;
  } else {
    showStill(id); return;
  }
  note.textContent = cam && cam.intersection
    ? `live · ${cam.intersection}${cam.snapDist != null ? ' · ' + cam.snapDist + ' m' : ''}`
    : 'live';
}
function showStill(id) {
  teardownStream();
  const v = $('cam-pip-video');
  const note = $('cam-pip-note');
  const base = state.cameras.cameras.still(id);
  if (!base) { note.textContent = 'no stream available'; return; }
  const bust = () => { v.setAttribute('poster', base + (base.includes('?') ? '&' : '?') + '_t=' + Date.now()); };
  bust();
  pip.stillTimer = setInterval(() => { if (pip.id === id && !pip.live) bust(); }, 3000);
  note.textContent = 'snapshot · start  python -m tools.twin_server  for live video';
}
$('cam-pip-close')?.addEventListener('click', () => {
  teardownStream();
  if (pip.id != null) clearActiveCamera(pip.id);   // clear only this camera's viewer signal
  pip.id = null;
  $('cam-pip').classList.add('hidden');
  refreshCameraList(true);
});
$('cam-pip-native')?.addEventListener('click', async () => {
  const v = $('cam-pip-video');
  try {
    if (document.pictureInPictureElement) await document.exitPictureInPicture();
    else if (v.requestPictureInPicture) await v.requestPictureInPicture();
  } catch (e) { $('cam-pip-note').textContent = 'browser PiP unavailable for this stream'; }
});
// drag the PiP panel by its title bar
(function makeCamPipDraggable() {
  const panel = $('cam-pip'), bar = panel?.querySelector('.cam-pip-bar');
  if (!bar) return;
  let dx = 0, dy = 0, dragging = false;
  bar.addEventListener('pointerdown', (e) => {
    if (e.target.tagName === 'BUTTON') return;
    dragging = true; bar.setPointerCapture(e.pointerId);
    const r = panel.getBoundingClientRect();
    dx = e.clientX - r.left; dy = e.clientY - r.top;
    panel.style.right = 'auto';
  });
  bar.addEventListener('pointermove', (e) => {
    if (!dragging) return;
    panel.style.left = Math.max(0, e.clientX - dx) + 'px';
    panel.style.top = Math.max(0, e.clientY - dy) + 'px';
  });
  bar.addEventListener('pointerup', (e) => { dragging = false; try { bar.releasePointerCapture(e.pointerId); } catch (_) {} });
})();

// ---- camera calibration + click-to-spawn (Phase 1 geometry tool, on the PiP) ----
// Calibrate: click an image point in the camera, then the matching spot in the 3D twin,
// 4+ times per quad, then Solve & Save -> a per-(camera,quad) homography. Spawn mode:
// click the video where a car is and a kinematic twin car appears (visible to everyone).
const camCal = (() => {
  const overlay = $('cam-pip-overlay');
  let mode = false, spawn = false, detect = false;
  let detectTimer = null, lastDets = [], lastDetMeta = null, _detSig = '';
  const clickCars = [];            // [{id,x,z}] kinematic cars from clicks
  let heartbeat = null;
  // Auto-analysis: watch the detector's boxes for ~12 s, infer per-quad traffic flow, and
  // propose a road-aligned calibration seed (see camanalysis.js). Pure overlay + a proposal
  // the user Accepts into the calibration session; never spawns or persists on its own.
  const analysis = createCameraAnalysis({
    homography: { solveHomography, applyHomography },
    // both road sources: campus centrelines ([x,y,z]) + the city street net ([x,z]). The
    // nearestRoad scanner reads x=pt[0], z=pt[last], so the differing element counts are fine.
    getRoads: () => [...(state.rawRoads || []), ...(state.cityRoads || [])],
    getDetections: async (camId) => {
      const r = await fetch(`/api/cameras/detections?camera=${encodeURIComponent(camId)}`, { cache: 'no-store' });
      return r.json();
    },
    cameraOf: (camId) => (api() ? api().get(camId) : null),
    // Bootstrap an UNCALIBRATED camera: start/stop a raw image-space (analysis-mode) YOLO
    // detector on the server so Analysis has boxes to watch even with zero calibration. The
    // analysis:true flag keeps this detector in its own server slot, so it never clobbers (or
    // is clobbered by) a normal "Run YOLO" detector for the same camera — and stopping it
    // leaves any normal detector untouched.
    startDetector: (camId) => fetch('/api/cameras/detect', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ camera: camId, on: true, analysis: true }),
    }).then((r) => r.json()),
    stopDetector: (camId) => fetch('/api/cameras/detect', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ camera: camId, on: false, analysis: true }),
    }).then((r) => r.json()),
  });
  let analysisProposal = null;     // last proposal (per-quad {pairs,H,road}) for Accept + overlay
  // class colours/names match tools/camera_detect.py CLASS_COLOR / DETECT_CLASSES
  const CLS_COLOR = { 0: '#e25fae', 1: '#4aa05a', 2: '#27c4c4', 3: '#c77f2a', 5: '#e0a83a', 7: '#8e6bd0' };
  const CLS_NAME = { 0: 'ped', 1: 'bike', 2: 'car', 3: 'moto', 5: 'bus', 7: 'truck' };
  const api = () => state.cameras && state.cameras.cameras;
  const calStatus = (t) => { const n = $('cam-cal-status'); if (n) n.textContent = t; };
  const fullOf = (quad, qu, qv) => {                 // quad-local -> full-frame normalized
    const col = quad % 2, row = quad >= 2 ? 1 : 0;
    return [col * 0.5 + qu * 0.5, row * 0.5 + qv * 0.5];
  };
  // The PiP <video> is object-fit:contain, so the video CONTENT is letterboxed inside the
  // stage whenever the panel aspect != the stream's (true even at the default size, and
  // always after a resize). Map between full-frame-normalized [0,1] coords and overlay
  // pixels through the REAL displayed content box, so clicks and overlay geometry track
  // the visible pixels — not the black bars. Without this the homography is authored from
  // misregistered clicks and both click- and detector-spawned cars land in the wrong place.
  function contentBox(W, H) {
    const v = $('cam-pip-video');
    const vw = (v && v.videoWidth) || 960, vh = (v && v.videoHeight) || 720;  // streams are 960x720
    const scale = Math.min(W / vw, H / vh);
    const dispW = vw * scale, dispH = vh * scale;
    return { offX: (W - dispW) / 2, offY: (H - dispH) / 2, dispW, dispH };
  }
  // full-frame-normalized -> overlay px (within the content box). Exposed for the click
  // handler's inverse mapping too.
  function toPx(cb, fu, fv) { return [cb.offX + fu * cb.dispW, cb.offY + fv * cb.dispH]; }
  function clickToContent(r, clientX, clientY) {   // overlay px -> full-frame-normalized
    const cb = contentBox(r.width, r.height);
    const u = (clientX - r.left - cb.offX) / cb.dispW;
    const v = (clientY - r.top - cb.offY) / cb.dispH;
    return { u, v, inside: u >= 0 && u <= 1 && v >= 0 && v <= 1 };
  }

  function draw() {
    if (!overlay) return;
    const r = overlay.getBoundingClientRect();
    // Resize the backing store only when the measured size actually changes — assigning
    // canvas.width/height ALWAYS reallocates+clears it, so doing it every 200ms poll was
    // needless reflow + GC churn. clearRect below still clears each frame.
    const w = Math.max(2, Math.round(r.width)), h = Math.max(2, Math.round(r.height));
    if (overlay.width !== w) overlay.width = w;
    if (overlay.height !== h) overlay.height = h;
    const g = overlay.getContext('2d'), W = overlay.width, H = overlay.height;
    g.clearRect(0, 0, W, H);
    if (!mode && !spawn && !detect) return;
    const cb = contentBox(W, H);
    if (detect) {                                                       // live detection boxes
      g.lineWidth = 2; g.font = '11px system-ui, sans-serif';
      for (const d of lastDets) {
        const col = d.quad % 2, row = d.quad >= 2 ? 1 : 0;             // box is normalized within its quad
        const [x, y] = toPx(cb, col * 0.5 + d.box[0] * 0.5, row * 0.5 + d.box[1] * 0.5);
        const [x2, y2] = toPx(cb, col * 0.5 + d.box[2] * 0.5, row * 0.5 + d.box[3] * 0.5);
        const c = CLS_COLOR[d.cls] || '#27c4c4';
        g.strokeStyle = c; g.strokeRect(x, y, x2 - x, y2 - y);
        g.fillStyle = c; g.fillText((CLS_NAME[d.cls] || '?') + (d.conf != null ? ' ' + d.conf : ''), x + 2, Math.max(10, y - 3));
      }
    }
    if (!mode && !spawn) return;
    g.strokeStyle = 'rgba(159,194,232,0.45)'; g.lineWidth = 1;          // 2x2 quad grid (over the content box)
    const mid = toPx(cb, 0.5, 0.5), tl = toPx(cb, 0, 0), br = toPx(cb, 1, 1);
    g.beginPath(); g.moveTo(mid[0], tl[1]); g.lineTo(mid[0], br[1]); g.moveTo(tl[0], mid[1]); g.lineTo(br[0], mid[1]); g.stroke();
    const a = api(); if (!a) return;
    const s = a.calib.session();
    if (mode && s.quad != null) {                                      // highlight active quad
      const col = s.quad % 2, row = s.quad >= 2 ? 1 : 0;
      const [hx, hy] = toPx(cb, col * 0.5, row * 0.5);
      g.fillStyle = 'rgba(73,176,122,0.10)'; g.fillRect(hx, hy, cb.dispW * 0.5, cb.dispH * 0.5);
    }
    g.font = '12px system-ui, sans-serif';
    s.pairs.forEach((p, i) => {
      const [fu, fv] = fullOf(s.quad, p.img[0], p.img[1]); const [x, y] = toPx(cb, fu, fv);
      g.fillStyle = '#27c46a'; g.beginPath(); g.arc(x, y, 5, 0, 7); g.fill();
      g.fillStyle = '#fff'; g.fillText(String(i + 1), x + 7, y - 7);
      const uv = a.calib.sceneToImage(s.camId, s.quad, p.scene[0], p.scene[1]);   // reproj fit
      if (uv) { const [cx, cy] = toPx(cb, uv[0], uv[1]); g.strokeStyle = '#ff8a2a'; g.lineWidth = 2;
        g.beginPath(); g.moveTo(cx - 6, cy); g.lineTo(cx + 6, cy); g.moveTo(cx, cy - 6); g.lineTo(cx, cy + 6); g.stroke(); }
    });
    if (s.pendingImg) { const [fu, fv] = fullOf(s.quad, s.pendingImg[0], s.pendingImg[1]); const [px, py] = toPx(cb, fu, fv);
      g.fillStyle = '#ffd23a'; g.beginPath(); g.arc(px, py, 6, 0, 7); g.fill(); }
    drawAnalysis(g, cb);
  }

  // Overlay the inferred traffic flow (arrows, image space) + the proposed correspondence
  // image points. Drawn whenever a proposal exists, on top of the calibration overlay, so
  // the user sees WHY a seed was suggested and where the points sit before accepting.
  function drawAnalysis(g, cb) {
    const arrows = analysis.arrows();
    if (arrows && arrows.length) {
      for (const ar of arrows) {
        const [fu, fv] = fullOf(ar.quad, ar.mid[0], ar.mid[1]);
        const [x, y] = toPx(cb, fu, fv);
        // arrow length scales with the displayed quad size so it reads at any panel size
        const L = Math.min(cb.dispW, cb.dispH) * 0.12 * (ar.primary ? 1 : 0.8);
        // image dir is in quad-local units; the quad maps to half the content box in each
        // axis, so scale x and y by dispW/2 and dispH/2 to keep the on-screen angle correct
        let ex = ar.dir[0] * (cb.dispW * 0.5), ey = ar.dir[1] * (cb.dispH * 0.5);
        const el2 = Math.hypot(ex, ey) || 1; ex = ex / el2 * L; ey = ey / el2 * L;
        g.strokeStyle = ar.color; g.fillStyle = ar.color; g.lineWidth = ar.primary ? 3 : 2;
        g.beginPath(); g.moveTo(x, y); g.lineTo(x + ex, y + ey); g.stroke();
        // arrowhead
        const ang = Math.atan2(ey, ex), hs = 7;
        g.beginPath(); g.moveTo(x + ex, y + ey);
        g.lineTo(x + ex - hs * Math.cos(ang - 0.4), y + ey - hs * Math.sin(ang - 0.4));
        g.lineTo(x + ex - hs * Math.cos(ang + 0.4), y + ey - hs * Math.sin(ang + 0.4));
        g.closePath(); g.fill();
      }
    }
    // proposed image points (faint squares) so the user previews the seed before Accept
    if (analysisProposal && analysisProposal.quads) {
      g.lineWidth = 1.5; g.strokeStyle = 'rgba(74,143,192,0.9)';
      for (const q of Object.keys(analysisProposal.quads)) {
        const pr = analysisProposal.quads[q];
        for (const p of (pr.pairs || [])) {
          const [fu, fv] = fullOf(Number(q), p.img[0], p.img[1]);
          const [x, y] = toPx(cb, fu, fv);
          g.strokeRect(x - 4, y - 4, 8, 8);
        }
      }
    }
  }

  function showBars() {
    const running = analysis.isRunning();
    const haveProposal = !!analysisProposal;
    // the calibration bar (which holds Accept seed) is also useful when analysis is running
    // or has produced a proposal, even before the user enters Calibrate mode.
    const anyMode = mode || spawn || detect || running || haveProposal;
    $('cam-cal-bar')?.classList.toggle('hidden', !anyMode);
    overlay?.classList.toggle('hidden', !anyMode);
    $('cam-cal-toggle')?.classList.toggle('active', mode);
    $('cam-cal-spawn')?.classList.toggle('active', spawn);
    $('cam-det-toggle')?.classList.toggle('active', detect);
    $('cam-analysis')?.classList.toggle('running', running);
    $('cam-analysis')?.classList.toggle('ready', !running && haveProposal);
    $('cam-cal-accept')?.classList.toggle('hidden', !haveProposal);
    draw();
  }
  function reset() {
    mode = false; spawn = false; stopDetect();
    analysis.stop(); analysisProposal = null;     // drop any in-flight analysis + its proposal
    // Tear down click-spawned cars + their keep-alive heartbeat on PiP close / camera
    // switch — otherwise the 2 s heartbeat keeps re-posing orphaned cars forever (the
    // user can no longer see or clear them once the PiP is closed).
    if (heartbeat) { clearInterval(heartbeat); heartbeat = null; }
    for (const c of clickCars.splice(0)) fetch(`/api/world/agents/${c.id}`, { method: 'DELETE' }).catch(() => {});
    showBars();
  }

  // ---- live detection boxes: poll the relay the detector publishes to ----
  async function pollDetections() {
    if (!detect || !pip.id) return;
    try {
      const r = await fetch(`/api/cameras/detections?camera=${encodeURIComponent(pip.id)}`, { cache: 'no-store' });
      const d = await r.json();
      lastDets = d.dets || [];
      lastDetMeta = d.stale ? null : { age: d.age, n: lastDets.length };
      calStatus(d.stale
        ? 'detections: none — run  python -m tools.camera_detect --camera ' + pip.id
        : `detections: ${lastDets.length} (live, ${d.age}s)`);
      // only repaint when the boxes actually changed — a static scene was repainting
      // (and reflowing the canvas) 5x/second for nothing (the _camSig/_busSig pattern).
      const sig = (d.stale ? 'stale|' : '') + lastDets.map((x) => x.quad + ':' + x.box.join(',') + ':' + x.cls).join('|');
      if (sig !== _detSig) { _detSig = sig; draw(); }
    } catch (e) { calStatus('detections: relay offline'); }
    if (detect) detectTimer = setTimeout(pollDetections, 200);
  }
  function enterDetect() {
    if (!pip.id) return;
    detect = !detect;
    if (detect) { showBars(); pollDetections(); }
    else { stopDetect(); showBars(); }
  }
  function stopDetect() { detect = false; clearTimeout(detectTimer); detectTimer = null; lastDets = []; }

  function enterCalibrate() {
    if (!pip.id || !api()) return;
    mode = !mode; if (mode) spawn = false;
    if (mode) { api().calib.begin(pip.id); calStatus('quad ?: click a point in the camera image, then the SAME spot in the 3D twin (×4+)'); }
    showBars();
  }
  function enterSpawn() {
    if (!pip.id || !api()) return;
    spawn = !spawn; if (spawn) mode = false;
    calStatus(spawn ? 'spawn: click the video where a car is (the quad must be calibrated)' : 'spawn off');
    showBars();
  }
  // ---- Analysis: auto-assist calibration from observed traffic flow ----
  // Self-contained bootstrap for a COLD, uncalibrated camera: starts a raw image-space
  // (analysis-mode) YOLO detector on the server (via the analysis controller's startDetector
  // dep), watches its published boxes for ~12 s with a live countdown, infers the dominant
  // flow direction(s) per quad (image space), overlays them as arrows, and — if the camera
  // maps near a road — proposes a road-aligned image->scene correspondence the user can Accept
  // into calibration. No prior Run YOLO / calibration needed. It stops the detector it started
  // on finish or cancel (leaving any normal detector untouched). Re-clicking while running
  // cancels. If YOLO can't run (no GPU/deps on the server) it degrades to a clear status.
  function enterAnalysis() {
    if (!pip.id) return;
    if (analysis.isRunning()) { analysis.stop(); calStatus('analysis: cancelled'); showBars(); return; }
    if (!detect) enterDetect();         // surface the live boxes too while we watch (visual)
    analysisProposal = null;
    analysis.start(pip.id, {
      durationMs: 12000,
      onUpdate: (vw) => {
        calStatus(vw.status);
        if (!vw.running) {              // finished: stash the proposal, light up Accept
          analysisProposal = vw.proposal || null;
        }
        showBars();                     // refreshes arrows + button states each tick
      },
    });
    showBars();
  }
  // Load the proposed (image, scene) correspondences for every proposed quad into the
  // calibration session so the user can review/nudge in the 3D view, then Solve & Save.
  // We seed ONE quad per Accept (the calib session is single-quad); if several quads were
  // proposed we take the one with the most points and tell the user to re-Accept for the rest.
  function acceptSeed() {
    const a = api(); if (!a || !analysisProposal) return;
    const quads = analysisProposal.quads || {};
    const entries = Object.entries(quads).filter(([, q]) => q.pairs && q.pairs.length >= 1);
    if (!entries.length) { calStatus('analysis: nothing to accept — no on-road seed was produced'); return; }
    entries.sort((x, y) => (y[1].pairs.length - x[1].pairs.length));
    const [quadStr, pr] = entries[0];
    const quad = Number(quadStr);
    // Re-begin a clean session and inject the seed pairs directly. addImagePoint locks the
    // quad from the first point's full-frame coords, so we feed each pair as (image -> scene).
    a.calib.begin(pip.id);
    let added = 0;
    for (const p of pr.pairs) {
      const [fu, fv] = fullOf(quad, p.img[0], p.img[1]);     // quad-local -> full-frame for the API
      const ai = a.calib.addImagePoint(fu, fv);
      if (ai && ai.error) continue;                          // skip a point that fell in another quad
      const as = a.calib.addScenePoint(p.scene[0], p.scene[1]);
      if (!as.error) added++;
    }
    mode = true; spawn = false;                              // enter Calibrate so clicks edit the seed
    const n = a.calib.session().pairs.length;
    const more = entries.length - 1;
    calStatus(`seed loaded: quad ${quad}, ${n} point(s)${more ? ` (+${more} more quad(s) — Accept again after Solve & Save)` : ''}. ` +
      `Drag-free: click a video point then its 3D spot to refine, or just Solve & Save.`);
    // keep the proposal so the on-video preview squares stay until the user saves/cancels
    showBars();
  }
  async function save() {
    const a = api(); if (!a) return;
    const cam = a.get(pip.id);
    const v = $('cam-pip-video');
    const res = await a.calib.save({ intersection: cam && cam.intersection,
      imgW: v && v.videoWidth, imgH: v && v.videoHeight });
    if (res.error && !res.ok) calStatus('save: ' + res.error);
    else calStatus(`saved quad ${res.quad} — reproj error ${res.reprojError.toFixed(1)} m. Calibrate another quad or use Spawn mode.`);
    draw();
  }
  function onScenePoint(x, z) {
    const a = api(); if (!a) return;
    const r = a.calib.addScenePoint(x, z);
    if (r.error) calStatus(r.error);
    else calStatus(`quad ${a.calib.session().quad}: ${r.pairs} point(s). ${r.pairs >= 4 ? 'Solve & Save, or add more.' : 'Add ' + (4 - r.pairs) + ' more.'}`);
    draw();
  }
  async function spawnClickCar(x, z, quad) {
    try {
      const r = await fetch('/api/world/spawn', { method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ type: 'car', kinematic: true, position: [x, null, z], color: 0xe0a83a,
          source: { cam: pip.id, quad, manual: true } }) });
      const a = await r.json();
      if (a && a.id != null) { clickCars.push({ id: a.id, x, z }); ensureHeartbeat();
        calStatus(`car #${a.id} at (${x.toFixed(0)}, ${z.toFixed(0)}) — quad ${quad}`); }
      else calStatus('spawn failed: ' + (a.error || '?'));
    } catch (e) { calStatus('spawn failed (twin server not running?)'); }
  }
  function ensureHeartbeat() {            // keep click-cars alive (re-pose) past the TTL
    if (heartbeat || !clickCars.length) return;
    heartbeat = setInterval(() => {
      for (const c of clickCars) fetch(`/api/world/agents/${c.id}/pose`,
        { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ x: c.x, z: c.z }) }).catch(() => {});
    }, 2000);
  }
  async function clearCars() {
    for (const c of clickCars.splice(0)) await fetch(`/api/world/agents/${c.id}`, { method: 'DELETE' }).catch(() => {});
    if (heartbeat) { clearInterval(heartbeat); heartbeat = null; }
    calStatus('cleared spawned cars');
  }

  overlay?.addEventListener('click', (e) => {
    const a = api(); if (!a || !pip.id) return;
    const r = overlay.getBoundingClientRect();
    // map the click through the letterboxed content box, not the raw overlay rect, so
    // the recorded image point matches the visible video pixel (see contentBox).
    const { u, v, inside } = clickToContent(r, e.clientX, e.clientY);
    if (!inside) { calStatus('click on the video, not the black letterbox bars'); return; }
    if (mode) {
      const res = a.calib.addImagePoint(u, v);
      if (res && res.error) calStatus(res.error);
      else calStatus(`quad ${a.calib.session().quad}: now click the SAME spot in the 3D twin`);
      draw();
    } else if (spawn) {
      const sc = a.calib.imageToScene(pip.id, u, v);
      if (!sc) calStatus('that quad is not calibrated yet — Calibrate it first');
      else spawnClickCar(sc.x, sc.z, sc.quad);
    }
  });
  $('cam-det-toggle')?.addEventListener('click', enterDetect);
  $('cam-cal-toggle')?.addEventListener('click', enterCalibrate);
  $('cam-analysis')?.addEventListener('click', enterAnalysis);
  $('cam-cal-accept')?.addEventListener('click', acceptSeed);
  $('cam-cal-spawn')?.addEventListener('click', enterSpawn);
  $('cam-cal-save')?.addEventListener('click', save);
  $('cam-cal-undo')?.addEventListener('click', () => { api() && api().calib.undo(); draw(); calStatus('removed last point'); });
  $('cam-cal-clearcars')?.addEventListener('click', clearCars);
  $('cam-pip-close')?.addEventListener('click', reset);
  $('cam-pip')?.addEventListener('cam-open', reset);
  if (typeof ResizeObserver !== 'undefined' && overlay) new ResizeObserver(() => draw()).observe(overlay);

  return { active: () => mode, spawning: () => spawn, detecting: () => detect,
           analyzing: () => analysis.isRunning(), analysis,
           onScenePoint, reset, _draw: draw, _dets: () => lastDets,
           // surfaces feedback when a calibration scene-click missed the ground
           noGround: () => calStatus('clicked off the ground — aim at the road surface in the 3D view') };
})();
window.__camCal = camCal;                 // test/console hook
window.__camCalDets = () => camCal._dets();

// ---- Run YOLO: start/stop the server's in-process detector for the open camera ----
// Replaces running `python -m tools.camera_detect --camera <id>` by hand — clicking the
// PiP button asks the twin server (POST /api/cameras/detect) to run YOLO for this camera,
// which spawns live detected vehicles/pedestrians into the shared twin. State is read back
// on open so the button reflects whether a detector is already running.
const camDetect = (() => {
  const btn = $('cam-det-run');
  let running = false;
  const note = (t) => { const n = $('cam-pip-note'); if (n) n.textContent = t; };
  function render() {
    if (!btn) return;
    btn.classList.toggle('active', running);
    btn.textContent = running ? '● Detecting' : 'Run YOLO';
  }
  async function refresh() {
    if (!pip.id) { running = false; render(); return; }
    try {
      const r = await fetch(`/api/cameras/detect?camera=${encodeURIComponent(pip.id)}`, { cache: 'no-store' });
      const d = await r.json();
      running = !!d.running; render();
    } catch (e) { running = false; render(); }
  }
  async function toggle() {
    if (!pip.id) return;
    const want = !running;
    note(want ? 'starting YOLO detector…' : 'stopping detector…');
    try {
      const r = await fetch('/api/cameras/detect', { method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ camera: pip.id, on: want }) });
      const d = await r.json();
      if (d.error) { running = false; render(); note('detector: ' + d.error); return; }
      running = !!d.running; render();
      note(running ? `detector running (${d.model || 'yolo'}) — live cars spawning in the twin`
                   : 'detector stopped');
      if (running && !camCal.detecting()) $('cam-det-toggle')?.click();   // show the boxes too
    } catch (e) { note('detector: twin server not reachable'); }
  }
  btn?.addEventListener('click', toggle);
  return { refresh, running: () => running };
})();
window.__camDetect = camDetect;

$('buildings-visible').addEventListener('change', () => {
  applyBaseLayerVis();   // AND with the photoreal "replace" hide so they can't desync
});
$('buildings-wireframe').addEventListener('change', (e) => {
  if (state.buildings.packed) {
    state.buildings.packed.mesh.material.wireframe = e.target.checked;
    return;
  }
  for (const t of state.buildings.tiles) {
    if (t.object) t.object.material.wireframe = e.target.checked;
  }
});
$('buildings-color-mode').addEventListener('change', () => {
  const mode = $('buildings-color-mode').value;
  if (state.buildings.packed) {
    const { mesh, minH, maxH, meta } = state.buildings.packed;
    const col = mesh.geometry.getAttribute('color');
    const arr = col.array;                 // write the packed Float32Array directly:
    const c = new THREE.Color();            // setXYZ per vertex over ~millions of verts
    for (const b of meta.buildings) {       // hitched the toggle (same fast path as load)
      if (mode === 'grey') c.setHex(0x8899aa);
      else c.copy(buildingHeightColor(b.heightM, minH, maxH));
      const r = c.r, gr = c.g, bl = c.b;
      for (let v = b.vStart, e = b.vStart + b.vCount; v < e; v++) {
        const o = v * 3; arr[o] = r; arr[o + 1] = gr; arr[o + 2] = bl;
      }
    }
    col.needsUpdate = true;
    return;
  }
  let minH = Infinity, maxH = -Infinity;
  for (const b of state.buildings.tiles) {
    const h = b.heightCm / 100;
    if (h < minH) minH = h;
    if (h > maxH) maxH = h;
  }
  for (const t of state.buildings.tiles) {
    if (!t.object) continue;
    const oldMat = t.object.material;
    const newMat = mode === 'grey'
      ? buildingGreyMaterial().clone()
      : new THREE.MeshStandardMaterial({
          color: buildingHeightColor(t.heightCm / 100, minH, maxH),
          roughness: 0.8, metalness: 0.1, side: THREE.DoubleSide,
        });
    newMat.wireframe = oldMat.wireframe;
    t.object.material = newMat;
    oldMat.dispose();
  }
});

// ----------------------------------------------------------------- agents ---
// Spawn/clear buttons + camera PiP. The live per-agent readout (#agent-list /
// #agent-surface) is refreshed from inside agents.tick (throttled), so app.js
// only wires the controls and an initial refresh. All handlers null-guard on
// state.agents (created after roads.json loads).
function refreshAgentUI() {
  if (!state.agents) return;
  const list = state.agents.list();
  $('agent-list').textContent = list.length
    ? list.map((a) => `${a.id} ${a.name} ${a.type} ${a.getState().speed.toFixed(1)}m/s ${a.surface || '--'}`).join('\n')
    : 'agents: none';
  const sel = $('agent-pip-select');
  const prev = sel.value;
  sel.innerHTML = list.map((a) => `<option value="${a.id}">${a.name}</option>`).join('');
  if (list.some((a) => String(a.id) === prev)) sel.value = prev;
}
$('agent-spawn').addEventListener('click', () => {
  if (!state.agents) { $('agent-list').textContent = 'agents: roads not loaded yet'; return; }
  const t = controls.target;
  let agent;
  try {
    agent = state.agents.spawn({ type: $('agent-type').value, position: [t.x, null, t.z], heading: 0, showHeading: true });
  } catch (e) { $('agent-list').textContent = 'spawn failed: ' + e.message; return; }
  refreshAgentUI();
  // Spawning takes you to the wheel: third-person chase cam + WASD on the new agent.
  $('agent-pip-select').value = String(agent.id);  // PiP (if on) follows the driven agent
  applyPiP();
  enterDrive(agent);
});
$('agent-clear').addEventListener('click', () => {
  exitDrive();
  if (state.agents) state.agents.clear();
  state.agents && state.agents.setPiP(null);
  $('agent-pip').checked = false;
  $('agent-pip-canvas').classList.add('hidden');
  refreshAgentUI();
});
function applyPiP() {
  if (!state.agents) return;
  const on = $('agent-pip').checked;
  const a = on ? state.agents.get(Number($('agent-pip-select').value)) : null;
  state.agents.setPiP(a || null);
  $('agent-pip-canvas').classList.toggle('hidden', !(on && a));
}
$('agent-pip').addEventListener('change', applyPiP);
$('agent-pip-select').addEventListener('change', applyPiP);

function resetView() {
  // Don't steal the camera from an active follow/drive — loaders call resetView()
  // as terrain/tiles stream in, which would otherwise yank you off the bus/agent.
  if (follow.id != null || drive.agent) return;
  const box = new THREE.Box3();
  if (state.terrain.group.children.length && state.terrain.group.visible) {
    box.expandByObject(state.terrain.group);
  }
  if (box.isEmpty() && state.lidar.group.children.length) {
    box.expandByObject(state.lidar.group);
  }
  if (box.isEmpty() && state.roadnet && state.roadnet.group.visible) {
    box.expandByObject(state.roadnet.group); // no terrain/lidar: frame the roads
  }
  if (box.isEmpty()) {
    camera.position.set(150, 120, 150);
    controls.target.set(0, 0, 0);
  } else {
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const radius = Math.max(size.x, size.z, 1) * 0.6;
    controls.target.copy(center);
    camera.position.set(
      center.x + radius * 0.7,
      center.y + radius * 0.8,
      center.z + radius * 0.7);
  }
  controls.update();
}

// ------------------------------------------------------- WASD / QE fly ---

const keys = new Set();
window.addEventListener('keydown', (e) => {
  if (e.target && /^(INPUT|TEXTAREA|SELECT)$/.test(e.target.tagName)) return;
  if (e.code === 'Escape' && drive.agent) { exitDrive(); return; } // release the driven agent
  if (e.code === 'Escape' && follow.id != null) { exitFollow(); return; } // release the followed target
  keys.add(e.code);
});
window.addEventListener('keyup', (e) => keys.delete(e.code));
window.addEventListener('blur', () => keys.clear());

const FLY_SPEED = 90; // m/s; shift = 4x
const _fwd = new THREE.Vector3(), _right = new THREE.Vector3(),
  _move = new THREE.Vector3();

function applyFly(dt) {
  if (!keys.size) return;
  _move.set(0, 0, 0);
  camera.getWorldDirection(_fwd);
  _right.crossVectors(_fwd, camera.up);
  if (_right.lengthSq() < 1e-10) {
    // looking straight up/down: derive right from the camera's local X axis
    _right.setFromMatrixColumn(camera.matrixWorld, 0);
  }
  _right.normalize();
  if (keys.has('KeyW')) _move.add(_fwd);
  if (keys.has('KeyS')) _move.sub(_fwd);
  if (keys.has('KeyD')) _move.add(_right);
  if (keys.has('KeyA')) _move.sub(_right);
  if (keys.has('KeyE')) _move.y += 1;
  if (keys.has('KeyQ')) _move.y -= 1;
  if (_move.lengthSq() === 0) return;
  const speed = FLY_SPEED *
    ((keys.has('ShiftLeft') || keys.has('ShiftRight')) ? 4 : 1);
  _move.normalize().multiplyScalar(speed * dt);
  camera.position.add(_move);
  controls.target.add(_move); // fly = move the orbit target with the camera
}

// --------------------------------------------- third-person agent driving ---
// Spawning an agent enters "drive mode": the main camera becomes a chase cam that
// follows the agent, and WASD/QE drive the AGENT instead of flying the camera.
// (OrbitControls is disabled while chasing — we set the camera manually — and the
// mouse wheel adjusts the chase distance. Esc, Clear, or despawn releases it.)
const drive = { agent: null, distance: 12 };
const _chasePos = new THREE.Vector3(), _chaseTgt = new THREE.Vector3();

function enterDrive(agent) {
  if (!agent || !agent.alive) return;
  if (follow.id != null) exitFollow();   // can't follow and drive an agent at once
  if (drive.agent && drive.agent !== agent && drive.agent.alive && drive.agent.stop) {
    drive.agent.stop(); // don't leave the previously-driven agent rolling on its last input
  }
  drive.agent = agent;
  const L = agent.halfExtents ? agent.halfExtents[0] * 2 : 4; // footprint length
  drive.distance = Math.max(6, L * 2.4 + 4);
  controls.enabled = false;          // chase cam drives the camera; no orbit/pan while driving
  keys.clear();
  if (agent.stop) agent.stop();      // start from rest
  updateChaseCamera(0, true);        // snap behind the agent immediately
  updateDriveHint();
}

function exitDrive() {
  const a = drive.agent;
  drive.agent = null;
  if (a && a.alive && a.stop) a.stop();   // don't keep rolling once released
  controls.enabled = true;
  if (a && a.alive) controls.target.copy(a.object.position); // resume orbit around it
  controls.update();
  updateDriveHint();
}

function updateDriveHint() {
  const node = $('drive-hint');
  if (!node) return;
  const a = drive.agent;
  if (a && a.alive) {
    node.innerHTML = `Driving <b>${a.name}</b> &middot; ` +
      (a.type === 'drone'
        ? 'W throttle &middot; A/D turn &middot; E/Q up/down'
        : 'W throttle &middot; S brake/reverse &middot; A/D steer') +
      ' &middot; <b>Esc</b> release';
    node.classList.remove('hidden');
  } else {
    node.classList.add('hidden');
  }
}

// held keys -> agent controls (called each frame before agents.tick)
function applyAgentDrive() {
  const a = drive.agent;
  if (!a || !a.alive) return;
  const w = keys.has('KeyW'), s = keys.has('KeyS'),
        left = keys.has('KeyA'), right = keys.has('KeyD'),
        up = keys.has('KeyE'), down = keys.has('KeyQ');
  if (a.type === 'drone') {
    const yawDegMax = a.maxYawRateRad ? a.maxYawRateRad * 180 / Math.PI : 120;
    a.setControls({
      thrust: w ? 1 : 0,
      climb: (up ? 1 : 0) - (down ? 1 : 0),
      yawRate: ((left ? 1 : 0) - (right ? 1 : 0)) * yawDegMax, // A=left (+yaw)
    });
  } else {
    const steer = (left ? 1 : 0) - (right ? 1 : 0);    // A=left (+yaw), D=right
    let throttle = 0, brake = 0, reverse = false;
    if (w) throttle = 1;
    else if (s) {
      if (a.speed > 0.5) brake = 1;          // rolling forward -> brake first
      else { reverse = true; throttle = 1; } // stopped/slow -> back up
    }
    a.setControls({ throttle, brake, steer, reverse });
  }
}

// chase camera: sit behind + above the agent along its heading and look at it
function updateChaseCamera(dt, snap) {
  const a = drive.agent;
  if (!a || !a.alive) return;
  const p = a.object.position, yaw = a.yaw;
  const fx = Math.cos(yaw), fz = -Math.sin(yaw);     // forward = (cos,0,-sin)
  const h = drive.distance * 0.5;
  _chasePos.set(p.x - fx * drive.distance, p.y + h, p.z - fz * drive.distance);
  const groundY = a.groundY != null ? a.groundY : p.y;
  if (_chasePos.y < groundY + 2) _chasePos.y = groundY + 2; // keep the cam above ground
  const eye = (a.halfExtents ? a.halfExtents[1] : 1) + 0.5;
  _chaseTgt.set(p.x, p.y + eye, p.z);
  const k = snap ? 1 : 1 - Math.exp(-8 * dt);        // frame-rate-independent smoothing
  camera.position.lerp(_chasePos, k);
  controls.target.lerp(_chaseTgt, k);
  camera.lookAt(controls.target);
}

// mouse wheel adjusts chase distance while driving an agent or following a bus
// (OrbitControls handles zoom otherwise)
renderer.domElement.addEventListener('wheel', (e) => {
  if (drive.agent) {
    e.preventDefault();
    drive.distance = Math.max(3, Math.min(150, drive.distance * (e.deltaY > 0 ? 1.1 : 0.9)));
  } else if (follow.id != null) {
    e.preventDefault();
    follow.distance = Math.max(5, Math.min(220, follow.distance * (e.deltaY > 0 ? 1.1 : 0.9)));
  }
}, { passive: false });

// ----------------------------------------------- 3rd-person follow camera ---
// Click a bus OR a shared-world agent (in a panel list or in the scene) to enter a
// chase view that tracks it. Mirrors the agent drive chase cam, but the target
// drives itself — we only follow, reading its live position/heading each frame.
// `kind` is 'bus' (window.__twin.transit) or 'net' (the twin server, netagents.js).
const follow = { kind: null, id: null, distance: 34 };
const _bfPos = new THREE.Vector3(), _bfTgt = new THREE.Vector3();

function followTarget() {
  if (!follow.kind || follow.id == null) return null;
  if (follow.kind === 'bus') {
    const v = state.transit && state.transit.transit.getVehicle(follow.id);
    return v ? { position: v.position, heading: v.heading, label: 'bus ' + v.label } : null;
  }
  if (follow.kind === 'net') {
    const a = state.netagents && state.netagents.get(follow.id);
    return a ? { position: a.position, heading: a.heading, label: `${a.type} #${a.id}` } : null;
  }
  return null;
}
function enterFollow(kind, id, distance) {
  follow.kind = kind; follow.id = String(id);
  if (!followTarget()) { follow.kind = null; follow.id = null; return; }  // no such target
  if (drive.agent) exitDrive();          // release any agent drive first
  follow.distance = distance || 34;
  controls.enabled = false;              // we own the camera while following
  keys.clear();
  updateFollowCamera(0, true);           // snap behind the target now
  updateFollowHint();
  refreshBusList(); refreshNetAgentList();
}
function exitFollow() {
  const t = followTarget();
  follow.kind = null; follow.id = null;
  controls.enabled = true;
  if (t) controls.target.set(t.position[0], t.position[1], t.position[2]);
  controls.update();
  updateFollowHint();
  refreshBusList(); refreshNetAgentList();
}
function updateFollowCamera(dt, snap) {
  const t = followTarget();
  if (!t) { if (follow.id != null) exitFollow(); return; }   // target left the world
  const p = t.position, yaw = (t.heading || 0) * Math.PI / 180;
  const fx = Math.cos(yaw), fz = -Math.sin(yaw);             // forward = (cos,0,-sin)
  const d = follow.distance;
  _bfPos.set(p[0] - fx * d, p[1] + d * 0.45, p[2] - fz * d);
  if (_bfPos.y < p[1] + 4) _bfPos.y = p[1] + 4;              // keep the cam above it
  _bfTgt.set(p[0], p[1] + 3, p[2]);
  const k = snap ? 1 : 1 - Math.exp(-6 * dt);
  camera.position.lerp(_bfPos, k);
  controls.target.lerp(_bfTgt, k);
  camera.lookAt(controls.target);
}
function updateFollowHint() {
  const node = $('drive-hint');
  if (!node) return;
  const t = followTarget();
  if (follow.id != null && t) {
    node.innerHTML = `Following <b>${t.label}</b> &middot; wheel = zoom &middot; <b>Esc</b> release`;
    node.classList.remove('hidden');
  } else if (!drive.agent) {
    node.classList.add('hidden');
  }
}

// Populate the live-bus list in the panel (called ~1 Hz from updateTransitStatus).
let _busSig = '';
function refreshBusList() {
  const node = $('transit-bus-list');
  if (!node) return;
  if (!state.transit) { node.textContent = '—'; return; }
  const buses = state.transit.transit.getVehicles().slice().sort((a, b) =>
    String(a.label).localeCompare(String(b.label), undefined, { numeric: true }) ||
    String(a.id).localeCompare(String(b.id), undefined, { numeric: true }));
  // only rebuild when the membership/labels/selection change (not every position tick)
  const sig = buses.map((v) => `${v.id}:${v.label}:${v.occupancy || ''}`).join('|') +
    '#' + (follow.kind === 'bus' ? follow.id : '');
  if (sig === _busSig) return;
  _busSig = sig;
  node.textContent = '';
  if (!buses.length) {
    const d = document.createElement('div');
    d.className = 'hint'; d.style.margin = '2px 0';
    d.textContent = 'no live buses — run python -m tools.twin_server';
    node.appendChild(d);
    return;
  }
  for (const v of buses) {
    const row = document.createElement('div');
    row.className = 'bus-row' + (follow.kind === 'bus' && String(v.id) === follow.id ? ' active' : '');
    const badge = document.createElement('span');
    badge.className = 'bus-badge';
    badge.style.background = v.route && v.route.color ? '#' + v.route.color : '#3b82c4';
    badge.textContent = String(v.label).slice(0, 4);
    const info = document.createElement('span');
    info.textContent = '#' + v.id + (v.occupancy ? ' · ' + v.occupancy.replace(/_/g, ' ') : '');
    row.appendChild(badge); row.appendChild(info);
    row.addEventListener('click', () => enterFollow('bus', v.id));
    node.appendChild(row);
  }
}

// Populate the shared-world agent list (Shared world panel) — click a row to enter
// a 3rd-person chase behind that agent. Refreshed ~1 Hz from updateTransitStatus.
let _netSig = '';
function refreshNetAgentList() {
  const node = $('netagents-list');
  if (!node) return;
  if (!state.netagents) { node.textContent = '—'; return; }
  const list = state.netagents.list();
  // only rebuild when the membership/selection changes, so rows aren't detached
  // from under a click every second (positions don't affect the list).
  const sig = list.map((a) => `${a.id}:${a.type}:${a.owner}`).join('|') +
    '#' + (follow.kind === 'net' ? follow.id : '');
  if (sig === _netSig) return;
  _netSig = sig;
  node.textContent = '';
  if (!list.length) {
    const d = document.createElement('div');
    d.className = 'hint'; d.style.margin = '2px 0';
    d.textContent = 'no agents — spawn from a script (client/twin.py)';
    node.appendChild(d);
    return;
  }
  for (const a of list) {
    const row = document.createElement('div');
    row.className = 'bus-row' + (follow.kind === 'net' && String(a.id) === follow.id ? ' active' : '');
    const badge = document.createElement('span');
    badge.className = 'bus-badge';
    badge.style.background = a.color != null
      ? '#' + (a.color & 0xffffff).toString(16).padStart(6, '0') : '#4aa05a';
    badge.textContent = a.type.slice(0, 4);
    const info = document.createElement('span');
    info.textContent = `#${a.id}` + (a.owner ? ' ' + a.owner : '');
    row.appendChild(badge); row.appendChild(info);
    row.addEventListener('click', () => enterFollow('net', a.id, 18));   // closer (agents are small)
    node.appendChild(row);
  }
}

// Click a bus or a shared-world agent in the 3D scene to follow it (distinguish a
// click from an orbit drag).
// Ground point under the current raycaster ray: the FLAT_Y plane in flat mode, else a
// terrain raycast (fallback: the city ground plane). Used to pick the twin point that
// matches a clicked camera-image point during calibration.
const _calPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
const _calHit = new THREE.Vector3();
function groundPointFromRay() {
  if (FLAT_WORLD) {
    _calPlane.constant = -FLAT_Y;
    return raycaster.ray.intersectPlane(_calPlane, _calHit) ? { x: _calHit.x, z: _calHit.z } : null;
  }
  if (state.terrain.group.children.length) {
    const h = raycaster.intersectObjects(state.terrain.group.children, false);
    if (h.length) return { x: h[0].point.x, z: h[0].point.z };
  }
  _calPlane.constant = -((state.city && state.city.groundY) || 285);
  return raycaster.ray.intersectPlane(_calPlane, _calHit) ? { x: _calHit.x, z: _calHit.z } : null;
}

let _clickX = 0, _clickY = 0, _clickT = 0;
renderer.domElement.addEventListener('pointerdown', (e) => {
  _clickX = e.clientX; _clickY = e.clientY; _clickT = performance.now();
});
renderer.domElement.addEventListener('pointerup', (e) => {
  if (Math.hypot(e.clientX - _clickX, e.clientY - _clickY) > 6) return;  // was a drag
  if (performance.now() - _clickT > 500) return;                         // long press
  mouseNdc.x = (e.clientX / window.innerWidth) * 2 - 1;
  mouseNdc.y = -(e.clientY / window.innerHeight) * 2 + 1;
  raycaster.setFromCamera(mouseNdc, camera);
  // 0) CALIBRATION: a scene click provides the ground point matching a pending image point
  if (state.cameras && camCal.active() && state.cameras.cameras.calib.session().pendingImg) {
    const gp = groundPointFromRay();
    if (gp) { camCal.onScenePoint(gp.x, gp.z); return; }
    camCal.noGround(); return;   // consume the click + tell the user it missed the ground
  }
  // 1) a bus / shared-world agent -> follow it
  const movers = [];
  if (state.transit) movers.push(...state.transit.layers.buses.children);
  if (state.netagents) movers.push(...state.netagents.group.children);
  if (movers.length) {
    const hits = raycaster.intersectObjects(movers, true);
    if (hits.length) {
      let o = hits[0].object;
      while (o && !(o.name && (o.name.startsWith('bus-') || o.name.startsWith('net-')))) o = o.parent;
      if (o && o.name) {
        if (o.name.startsWith('bus-')) enterFollow('bus', o.name.slice(4));
        else enterFollow('net', o.name.slice(4), 18);
        return;
      }
    }
  }
  // 2) a traffic-camera marker -> open its live stream in the PiP panel
  if (state.cameras && state.cameras.group.visible && state.cameras.layers.markers.visible) {
    const picks = state.cameras.layers.markers.children.filter((o) => o.userData && o.userData.pick);
    if (picks.length) {
      const hits = raycaster.intersectObjects(picks, false);
      if (hits.length && hits[0].instanceId != null) {
        const cam = state.cameras.cameras.byInstance(hits[0].instanceId);
        if (cam) { openCamera(cam.id); return; }
      }
    }
  }
  // 3) otherwise identify the building under the click (one-off raycast, NOT per-frame)
  if (state.buildings.group.visible && state.buildings.group.children.length) {
    const hits = raycaster.intersectObjects(state.buildings.group.children, false);
    if (hits.length) {
      const obj = hits[0].object;
      let name = null, hM = null;
      if (obj.userData && obj.userData.packed) {
        const b = buildingAtFace(obj.userData.buildings, hits[0].faceIndex);
        if (b) { name = b.name; hM = b.heightM; }
      } else if (obj.userData && obj.userData.buildingName) {
        name = obj.userData.buildingName; hM = obj.userData.heightCm / 100;
      }
      if (name) setStatus(el.cursor, `building: ${name} (${hM.toFixed(1)} m)`);
    }
  }
});

// -------------------------------------------------- cursor world readout ---

const raycaster = new THREE.Raycaster();
const mouseNdc = new THREE.Vector2();
let mouseMoved = false;
// Cursor readout uses an analytic ray-vs-ground-plane intersection (O(1)) instead
// of raycasting the merged 100k+ building mesh every mouse-move — the latter is a
// brute-force ~2M-triangle test per frame that collapsed fps to single digits while
// orbiting/panning. Building identity moved to click (pickBuildingAt).
const _groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
const _hitPt = new THREE.Vector3();
renderer.domElement.addEventListener('pointermove', (e) => {
  mouseNdc.x = (e.clientX / window.innerWidth) * 2 - 1;
  mouseNdc.y = -(e.clientY / window.innerHeight) * 2 + 1;
  mouseMoved = true;
});

function fmt(v, d = 1) { return v.toFixed(d); }

function updateCursorReadout() {
  if (!mouseMoved) return;
  mouseMoved = false;
  raycaster.setFromCamera(mouseNdc, camera);
  // intersect the flat ground plane analytically — no geometry traversal
  const gy = (state.city && typeof state.city.groundY === 'number')
    ? state.city.groundY : 0;
  _groundPlane.constant = -gy;
  if (!raycaster.ray.intersectPlane(_groundPlane, _hitPt)) {
    setStatus(el.cursor, 'cursor: --');
    return;
  }
  const p = _hitPt;
  const ue = sceneToUeCm(p.x, p.y, p.z);
  let txt = `scene m: ${fmt(p.x)}, ${fmt(p.y)}, ${fmt(p.z)}\n` +
    `UE cm: ${fmt(ue[0], 0)}, ${fmt(ue[1], 0)}, ${fmt(ue[2], 0)}`;
  const oc = state.lidar.originalCoordinates;
  if (oc && oc.length === 3) {
    txt += `\ngeo: ${fmt(oc[0] + ue[0], 0)}, ${fmt(oc[1] + ue[1], 0)}, ` +
      `${fmt(oc[2] + ue[2], 0)}`;
  }
  setStatus(el.cursor, txt);
}

// ------------------------------------------------------------ main loop ---

let frames = 0;
let fpsTimer = performance.now();
const clock = new THREE.Clock();

// --------------------------------------------- drape overlays onto photoreal ---
// Our overlays (road ribbons + lane markings + signals, street labels, buses, shared-
// world/YOLO cars, camera markers, LiDAR) are baked at OUR DTM/LiDAR elevation
// (NAVD88-ish). Google's photorealistic mesh sits on the WGS84 ellipsoid — tens of metres
// higher here, and the gap VARIES with location (geoid + terrain-source mismatch + the
// alignment fit's own residual), so a single global lift can only make the two surfaces
// coincide at ONE point and leaves everything else floating or sunk.
//
// The fix lives in drape.js: a streaming ground-conform FIELD that samples the gap
// (Google ground minus our ground, via downward rays) on a coarse grid, settles each
// cell against LOD streaming, and rebases each overlay by the LOCALLY interpolated gap.
// The road network conforms per-vertex (so hills/overpasses keep their relative profile),
// buses + shared-world cars sample the field per-object, and the agent sim ground-snaps
// onto the now-conformed road ribbons on its own (see agents.js). updateDrape() just
// drives the field each frame and rides the sparse point layers (LiDAR/labels/cameras) on
// a single focus-sampled offset. ourGroundY() is the per-(x,z) probe of OUR baking surface.
const _drapeRay = new THREE.Raycaster();
const _drapeFrom = new THREE.Vector3();
const _drapeDown = new THREE.Vector3(0, -1, 0);
function ourGroundY(x, z) {
  _drapeFrom.set(x, 6000, z);
  _drapeRay.set(_drapeFrom, _drapeDown);
  // raycast our DTM terrain first (matches the road baking), then the city ground plane;
  // both are hit even when hidden by "replace" (the raycaster ignores .visible).
  for (const s of [state.terrain, state.city]) {
    if (s && s.group) { const h = _drapeRay.intersectObject(s.group, true); if (h.length) return h[0].point.y; }
  }
  return null;
}
// Drive the streaming ground-conform field each frame (drape.js). The field rebases the
// road network PER-VERTEX and buses / shared-world cars PER-OBJECT onto Google's mesh as
// tiles stream in; the driving-agent sim ground-snaps onto the now-conformed road ribbons
// on its own (it raycasts the real, lifted geometry). LiDAR / labels / camera markers are
// sparse point layers, so they ride a single focus-sampled offset (sub-metre local
// variation across a marker is invisible) instead of per-vertex conforming.
function updateDrape(now) {
  const pr = state.photoreal;
  const active = !!(pr && pr.enabled && pr.group.visible && !FLAT_WORLD);
  const t = controls.target;
  state.drape.update(now, t.x, t.z, active);
  const o = state.drape.offsetAt(t.x, t.z);
  for (const s of [state.lidar, state.labels, state.cameras]) {
    if (s && s.group && s.group.position.y !== o) s.group.position.y = o;
  }
}

// Adaptive photoreal fidelity (auto-FPS): nudge the tile errorTarget to hold ~60fps. Lower
// errorTarget = finer tiles (sharper, heavier). With FPS headroom we creep toward the 1px
// high-fidelity floor; below the band we back off fast. Disabled by the panel's "adaptive"
// checkbox or by dragging the detail slider (manual override).
let _emaFps = 60, _adaptAt = 0;
function adaptPhotorealQuality(fpsVal, now) {
  const pr = state.photoreal;
  const on = pr && pr.enabled && pr.group.visible && !FLAT_WORLD;
  const adaptive = $('photoreal-adaptive') ? $('photoreal-adaptive').checked : true;
  if (!on || !adaptive) { _emaFps = fpsVal; return; }
  _emaFps = _emaFps * 0.6 + fpsVal * 0.4;          // smooth so one slow frame doesn't lurch
  if (now - _adaptAt < 1000) return;               // retune at ~1 Hz
  _adaptAt = now;
  const FLOOR = 1, CEIL = 16;
  let d = pr.detail || 4;
  if (_emaFps < 50) d = Math.min(CEIL, d + Math.max(0.5, d * 0.2));   // slow -> coarser, fast
  else if (_emaFps > 57 && d > FLOOR) d = Math.max(FLOOR, d - 0.5);   // headroom -> finer
  else return;
  d = Math.round(d * 2) / 2;
  if (d !== pr.detail) {
    pr.setDetail(d);
    const sl = $('photoreal-detail'); if (sl) sl.value = String(d);
    const lab = $('photoreal-detail-val'); if (lab) lab.textContent = String(d);
  }
}

function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(clock.getDelta(), 0.1);
  if (drive.agent && !drive.agent.alive) exitDrive(); // driven agent was despawned
  if (drive.agent) {
    applyAgentDrive();          // WASD -> agent controls
  } else if (follow.id != null) {
    /* camera follows the target (updateFollowCamera below); no free-fly / orbit */
  } else {
    applyFly(dt);               // WASD -> free camera fly
    controls.update();          // (skip while chasing: updateChaseCamera owns the camera)
  }
  updateCursorReadout();
  // transit first so its interpolated bus positions are fresh when agents sense them
  if (state.transit) state.transit.transit.tick(dt);
  if (state.cameras) state.cameras.cameras.tick(dt); // re-drape camera markers as terrain streams in
  if (state.netagents) state.netagents.tick(dt); // interpolate shared-world agents
  if (state.roadnet && state.roadnet.signals) state.roadnet.signals.tick(dt);
  if (state.agents) state.agents.tick(dt); // integrate agents after signals, before render
  if (drive.agent) updateChaseCamera(dt);  // follow AFTER the agent moved this frame
  if (follow.id != null) updateFollowCamera(dt); // follow AFTER the target moved this frame
  if (state.labels) state.labels.tick(camera, controls.target); // cull labels around the look-at point
  if (state.photoreal) state.photoreal.update();  // stream/LOD the photorealistic basemap (no-op when off)
  updateDrape(performance.now());   // lift overlays onto the photoreal surface (no-op when off)
  renderer.render(scene, camera);

  frames++;
  const now = performance.now();
  if (now - fpsTimer >= 500) {
    const fpsVal = Math.round((frames * 1000) / (now - fpsTimer));
    el.fps.textContent =
      `${fpsVal} fps — ` +
      `${renderer.info.render.triangles.toLocaleString()} tris, ` +
      `${renderer.info.render.points.toLocaleString()} pts`;
    adaptPhotorealQuality(fpsVal, now);   // hold ~60fps by tuning tile errorTarget
    frames = 0;
    fpsTimer = now;
  }
}

setStatus(el.manifestStatus, `manifest: loading ${DATA_DIR}manifest.json…`);
loadManifest();
loadRoads();
animate();

// debug hook (handy for screenshots / console poking; harmless in production)
window.__viewer = { THREE, scene, camera, controls, state, resetView,
                    get agents() { return state.agents; },
                    get photoreal() { return state.photoreal; },
                    get follow() { return follow; } };

// ---------------------------------------------- first-person POV rendering ---
// Render an agent's first-person view of the shared scene to a JPEG data URL. The
// twin server (tools/twin_server.py --render) drives a headless browser and calls
// this for /api/world/agents/<id>/camera, so scripts get a real FPV video feed (for
// a vision model). The server passes the camera's exact eye position + look
// direction (computed from the authoritative pose); we hide that agent's own mesh
// so it doesn't photograph itself, render the whole scene, and read back pixels.
const _povCam = new THREE.PerspectiveCamera(72, 4 / 3, 0.3, 8000);
let _povRT = null, _povCanvas = null, _povCtx = null, _povBuf = null;
const _prevClear = new THREE.Color();
window.__renderPOV = function (id, ex, ey, ez, fx, fy, fz, w, h) {
  w = Math.max(16, Math.min(1024, w | 0 || 320));
  h = Math.max(16, Math.min(1024, h | 0 || 240));
  if (!_povRT || _povRT.width !== w || _povRT.height !== h) {
    if (_povRT) _povRT.dispose();
    _povRT = new THREE.WebGLRenderTarget(w, h, {
      minFilter: THREE.LinearFilter, magFilter: THREE.LinearFilter, depthBuffer: true });
  }
  _povCam.aspect = w / h; _povCam.updateProjectionMatrix();
  _povCam.position.set(ex, ey, ez);
  _povCam.lookAt(ex + fx, ey + fy, ez + fz);

  // hide the rendering agent's own body, the street-name labels, and every floating
  // agent label sprite — all of which would billboard into the frame as clutter
  // (and confuse a vision model). Bodies of OTHER agents stay visible.
  const own = state.netagents && state.netagents.group.getObjectByName('net-' + id);
  const ownVis = own ? own.visible : false; if (own) own.visible = false;
  const labelsVis = state.labels ? state.labels.group.visible : false;
  if (state.labels) state.labels.group.visible = false;
  const hiddenSprites = [];
  if (state.netagents) state.netagents.group.traverse((o) => {
    if (o.isSprite && o.visible) { o.visible = false; hiddenSprites.push(o); }
  });

  const prevRT = renderer.getRenderTarget();
  renderer.getClearColor(_prevClear); const prevAlpha = renderer.getClearAlpha();
  try {
    renderer.setClearColor(0x9ec4e8, 1);           // sky, so frames aren't transparent
    renderer.setRenderTarget(_povRT);
    renderer.clear();
    renderer.render(scene, _povCam);
    if (!_povBuf || _povBuf.length < w * h * 4) _povBuf = new Uint8Array(w * h * 4);
    renderer.readRenderTargetPixels(_povRT, 0, 0, w, h, _povBuf);
  } finally {
    renderer.setRenderTarget(prevRT);
    renderer.setClearColor(_prevClear, prevAlpha);
    if (own) own.visible = ownVis;
    if (state.labels) state.labels.group.visible = labelsVis;
    for (const s of hiddenSprites) s.visible = true;
  }

  if (!_povCanvas) { _povCanvas = document.createElement('canvas'); _povCtx = _povCanvas.getContext('2d'); }
  if (_povCanvas.width !== w) _povCanvas.width = w;
  if (_povCanvas.height !== h) _povCanvas.height = h;
  const img = _povCtx.createImageData(w, h), row = w * 4;
  for (let y = 0; y < h; y++) {            // GL readback is bottom-up; flip to top-left
    const src = (h - 1 - y) * row, dst = y * row;
    for (let i = 0; i < row; i++) img.data[dst + i] = _povBuf[src + i];
  }
  _povCtx.putImageData(img, 0, 0);
  return _povCanvas.toDataURL('image/jpeg', 0.72);
};
