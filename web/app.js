// UKy Campus viewer — works against the shared data contract (see README.md).
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

// ---------------------------------------------------------------- config ---

const params = new URLSearchParams(location.search);
const DATA_DIR = (params.get('data') || 'data').replace(/\/+$/, '') + '/';
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
  buildings: { tiles: [], group: null, loaded: 0, failed: 0 },
  roadnet: null,       // real road network + props from roads.json (roads.js)
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
      slider.value = slider.max;  // show all points by default
      $('point-budget-val').textContent = parseFloat(slider.max).toFixed(1);
      state.lidar.budget = total;
    }
    lidarPump();
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
    outPos[j + 1] = wz * CM_TO_M; // three.y = ue.z
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
  if (bld.groundYm != null && !bld.bridge && geometry.boundingBox) {
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

async function loadBuildings(blds) {
  state.buildings.tiles = blds;
  updateBuildingsStatus();
  if (!blds.length) return;

  // Compute height range for color mapping
  const heights = blds.map((b) => b.heightCm / 100);
  const minH = Math.min(...heights);
  const maxH = Math.max(...heights);

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

  state.roadnet = createRoadNetwork(data, { trees: false, cars: false, signalModel });
  scene.add(state.roadnet.group);
  if (state.roadnet.signals) {
    // expose the live signal controller so an autonomous agent can query "what is my
    // light right now / where do I stop" and even drive the lights deterministically.
    window.__twin = { signals: state.roadnet.signals, model: signalModel };
  }
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

$('terrain-visible').addEventListener('change', (e) => {
  state.terrain.group.visible = e.target.checked;
});
$('lidar-visible').addEventListener('change', (e) => {
  state.lidar.group.visible = e.target.checked;
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
$('camera-reset').addEventListener('click', () => resetView());

$('road-visible').addEventListener('change', (e) => {
  if (state.roadnet) state.roadnet.group.visible = e.target.checked;
});
for (const key of ['roads', 'markings', 'crosswalks', 'trees', 'cars', 'signals']) {
  const cb = $('road-' + key);
  if (cb) cb.addEventListener('change', (e) => {
    if (state.roadnet) state.roadnet.layers[key].visible = e.target.checked;
  });
}

$('buildings-visible').addEventListener('change', (e) => {
  state.buildings.group.visible = e.target.checked;
});
$('buildings-wireframe').addEventListener('change', (e) => {
  for (const t of state.buildings.tiles) {
    if (t.object) t.object.material.wireframe = e.target.checked;
  }
});
$('buildings-color-mode').addEventListener('change', () => {
  const mode = $('buildings-color-mode').value;
  const heights = state.buildings.tiles.map((b) => b.heightCm / 100);
  const minH = Math.min(...heights);
  const maxH = Math.max(...heights);
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

function resetView() {
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

// -------------------------------------------------- cursor world readout ---

const raycaster = new THREE.Raycaster();
const mouseNdc = new THREE.Vector2();
let mouseMoved = false;
renderer.domElement.addEventListener('pointermove', (e) => {
  mouseNdc.x = (e.clientX / window.innerWidth) * 2 - 1;
  mouseNdc.y = -(e.clientY / window.innerHeight) * 2 + 1;
  mouseMoved = true;
});

function fmt(v, d = 1) { return v.toFixed(d); }

function updateCursorReadout() {
  if (!mouseMoved) return;
  mouseMoved = false;
  const hasTerrain = state.terrain.group.children.length && state.terrain.group.visible;
  const hasBuildings = state.buildings.group.children.length && state.buildings.group.visible;
  if (!hasTerrain && !hasBuildings) {
    setStatus(el.cursor, 'cursor: (no terrain or buildings to raycast)');
    return;
  }
  raycaster.setFromCamera(mouseNdc, camera);
  const targets = [];
  if (hasBuildings) targets.push(...state.buildings.group.children);
  if (hasTerrain) targets.push(...state.terrain.group.children);
  const hits = raycaster.intersectObjects(targets, false);
  if (!hits.length) {
    setStatus(el.cursor, 'cursor: --');
    return;
  }
  const p = hits[0].point;
  const obj = hits[0].object;
  const ue = sceneToUeCm(p.x, p.y, p.z);
  let txt = `scene m: ${fmt(p.x)}, ${fmt(p.y)}, ${fmt(p.z)}\n` +
    `UE cm: ${fmt(ue[0], 0)}, ${fmt(ue[1], 0)}, ${fmt(ue[2], 0)}`;
  if (obj.userData && obj.userData.buildingName) {
    txt += `\nbuilding: ${obj.userData.buildingName}` +
      ` (${(obj.userData.heightCm / 100).toFixed(1)}m)`;
  }
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

function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(clock.getDelta(), 0.1);
  applyFly(dt);
  controls.update();
  updateCursorReadout();
  if (state.roadnet && state.roadnet.signals) state.roadnet.signals.tick(dt);
  renderer.render(scene, camera);

  frames++;
  const now = performance.now();
  if (now - fpsTimer >= 500) {
    el.fps.textContent =
      `${Math.round((frames * 1000) / (now - fpsTimer))} fps — ` +
      `${renderer.info.render.triangles.toLocaleString()} tris, ` +
      `${renderer.info.render.points.toLocaleString()} pts`;
    frames = 0;
    fpsTimer = now;
  }
}

setStatus(el.manifestStatus, `manifest: loading ${DATA_DIR}manifest.json…`);
loadManifest();
loadRoads();
animate();

// debug hook (handy for screenshots / console poking; harmless in production)
window.__viewer = { THREE, scene, camera, controls, state, resetView };
