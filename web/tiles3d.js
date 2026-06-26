// Photorealistic 3D basemap — Google Photorealistic 3D Tiles (the same data Google
// Earth / Cesium use) streamed into our Three.js world via the NASA-AMMOS 3D Tiles
// renderer (vendored in lib/3d-tiles-renderer.module.js, pinned to a build that
// supports our three r160).
//
// This is a SELF-CONTAINED, OPT-IN layer:
//   * It does nothing until enabled AND a key is present, so the existing viewer is
//     untouched when off (no tiles fetched, no per-frame cost, no draw calls).
//   * The tiles arrive in geocentric ECEF; we map them into the viewer's
//     UTM-16N-derived scene frame with the static similarity transform baked by
//     tools/fit_tiles_align.py (lib/tiles_align.json) — buildings line up with the
//     OSM extrusions to within a couple of metres across the whole city.
//   * A key never lives in the repo: it comes from ?gkey=, localStorage, or a
//     gitignored web/data/photoreal.json / the twin server. See loadKey().
//
// Provider: Google Maps Platform "Map Tiles API" key (default) OR a Cesium ion token
// (set provider:'ion'); ion serves the same Google dataset as asset 2275207.
import * as THREE from 'three';
import {
	TilesRenderer,
	GoogleCloudAuthPlugin,
	CesiumIonAuthPlugin,
	GLTFExtensionsPlugin,
	TileCompressionPlugin,
	TilesFadePlugin,
	DRACOLoader,
} from './lib/3d-tiles-renderer.module.js';

const KEY_LS = 'twin.photoreal.key';
const PROVIDER_LS = 'twin.photoreal.provider';
const CALIB_LS = 'twin.photoreal.calib';
const DETAIL_LS = 'twin.photoreal.detail';   // errorTarget px (lower = higher fidelity)
// Default streaming fidelity (target screen-space error in px). Tiles are pulled LIVE
// from the Map Tiles API at this detail; lower => the renderer subdivides to finer tiles.
// 4 px is high fidelity (close to the native resolution of Google's mesh) while staying
// at 60 fps over the whole city; the panel's detail slider can push to 1–2 for maximum
// detail or raise it for performance. Kept in sync with web/index.html #photoreal-detail.
const DEFAULT_DETAIL = 4;
const ION_GOOGLE_ASSET = 2275207; // Cesium ion's Google Photorealistic 3D Tiles asset

// Baked ECEF->scene transform (fallback if lib/tiles_align.json can't be fetched).
// Regenerate both with: python -m tools.fit_tiles_align
const ALIGN_FALLBACK = [
	0.9965012826316548, 0.07540847331370641, 0.032264993455353504, 0.0,
	0.07925076557576816, -0.7838558061114713, -0.6156554761534254, 0.0,
	-0.02113726107689028, 0.6161378995927309, -0.7871909417773663, 0.0,
	-843.6174901865147, -6369164.875048548, -21120.116280738614, 1.0,
];

const DEFAULT_CALIB = { dx: 0, dy: 0, dz: 0, yaw: 0, scaleMul: 1 };

// Scratch objects for sampleGroundY() — the vertical ground probe used to drape our
// overlays (roads/labels/traffic) onto the photoreal surface. Module-level so the probe
// allocates nothing per call (it can run several times a second from the render loop).
const _groundRay = new THREE.Raycaster();
const _groundFrom = new THREE.Vector3();
const _groundDown = new THREE.Vector3(0, -1, 0);

// Resolve an API key/token without ever committing one. Priority: explicit ?gkey=,
// then localStorage (set via the panel), then a gitignored data/photoreal.json (which
// the twin server can populate from the GOOGLE_MAPS_API_KEY env var).
// Resolve { key, cache }. `cache` is true when twin_server is present (it serves the
// /api/gtile disk-cache proxy) so the viewer can route tile fetches through it.
async function loadKey(dataDir) {
	const params = new URLSearchParams(location.search);
	const fromUrl = params.get('gkey') || params.get('ionkey');
	if (fromUrl) {
		try { localStorage.setItem(KEY_LS, fromUrl); } catch (_) {}
		if (params.get('ionkey')) { try { localStorage.setItem(PROVIDER_LS, 'ion'); } catch (_) {} }
	}
	let key = fromUrl || null;
	if (!key) { try { key = localStorage.getItem(KEY_LS) || null; } catch (_) {} }
	// Ask the server: it provides the key (twin_server reads GOOGLE_MAPS_API_KEY / .env)
	// and advertises whether the tile-cache proxy is available. 200 {key:null} when unset.
	let cache = false;
	for (const url of ['/api/photoreal', dataDir + 'photoreal.json']) {
		try {
			const r = await fetch(url, { cache: 'no-store' });
			if (r.ok) {
				const j = await r.json();
				if (j && j.cache) cache = true;
				if (j && j.key && !key) {
					if (j.provider) { try { localStorage.setItem(PROVIDER_LS, j.provider); } catch (_) {} }
					key = j.key;
				}
				if (cache) break;          // twin_server answered; no need for the static file
			}
		} catch (_) {}
	}
	return { key, cache };
}

function loadCalib() {
	try {
		const j = JSON.parse(localStorage.getItem(CALIB_LS) || 'null');
		if (j && typeof j === 'object') return Object.assign({}, DEFAULT_CALIB, j);
	} catch (_) {}
	return Object.assign({}, DEFAULT_CALIB);
}

// Reroute Google tile fetches through the twin server's on-disk cache proxy (/api/gtile)
// so repeated local sessions reuse already-downloaded tiles. This patches fetch at the
// NETWORK layer only — NOT via preprocessURL — which is critical: the renderer must keep
// resolving (and storing) real tile.googleapis.com URLs so nested-tileset transforms stay
// correct; we only redirect the actual network request. Scoped to googleapis URLs;
// everything else passes through untouched. Installed once, restored on dispose.
let _origFetch = null;
function installFetchCache(base) {
	if (_origFetch || typeof globalThis.fetch !== 'function') return;
	_origFetch = globalThis.fetch.bind(globalThis);
	const proxied = (u) => base + '/api/gtile?u=' + encodeURIComponent(u);
	globalThis.fetch = (input, init) => {
		try {
			if (typeof input === 'string') {
				if (input.indexOf('tile.googleapis.com') !== -1) return _origFetch(proxied(input), init);
			} else if (input && typeof input.url === 'string' && input.url.indexOf('tile.googleapis.com') !== -1) {
				return _origFetch(new Request(proxied(input.url), input), init);
			}
		} catch (_) {}
		return _origFetch(input, init);
	};
}
function uninstallFetchCache() {
	if (_origFetch) { try { globalThis.fetch = _origFetch; } catch (_) {} _origFetch = null; }
}

/**
 * Create the photorealistic-tiles layer. Returns a handle the app drives:
 *   setVisible(on), setOpacity(v), update(dt), setKey(k), calibrate(partial),
 *   getCredits(), dispose(). All no-ops degrade gracefully with no key.
 */
export function createPhotorealTiles({ scene, camera, renderer, dataDir = 'data/', onStatus, onCredits }) {
	// `wrapper` carries the user calibration (scene-space translate/rotate/scale) and
	// is what we toggle/scene-add; tiles.group (baked ECEF->scene matrix) nests inside.
	const wrapper = new THREE.Group();
	wrapper.name = 'photoreal-3dtiles';
	wrapper.visible = false;
	scene.add(wrapper);

	let tiles = null;
	let enabled = false;
	let initStarted = false;
	let disposed = false;
	let opacity = 1;
	let provider = 'google';
	try { provider = localStorage.getItem(PROVIDER_LS) || 'google'; } catch (_) {}
	let calib = loadCalib();
	const align = new THREE.Matrix4().fromArray(ALIGN_FALLBACK);
	let credits = '';
	let maxAniso = 0;   // lazily set from renderer caps; sharpens tile textures at grazing angles
	let mode = provider === 'ion' ? 'ion' : 'google';   // 'google' | 'ion' | 'custom'
	let gen = 0;   // build generation; guards against concurrent (re)build races
	let serverCache = false;   // twin_server present -> route tiles through /api/gtile cache
	let detail = (() => { try { const v = parseFloat(localStorage.getItem(DETAIL_LS)); return Number.isFinite(v) ? v : DEFAULT_DETAIL; } catch (_) { return DEFAULT_DETAIL; } })();
	let buildAt = 0, gotTileset = false, gotModel = false, watchdogFired = false;
	let creditsAt = 0;

	const setStatus = (s) => { if (onStatus) onStatus(s); };
	const decoderURL = new URL('lib/draco/gltf/', document.baseURI).href;

	// Lazily fetch the precise baked matrix on the FIRST build, so the layer issues no
	// network request at all until it is enabled; falls back to the embedded constant.
	let alignReady = null;
	function ensureAlign() {
		if (!alignReady) {
			alignReady = fetch('lib/tiles_align.json', { cache: 'force-cache' })
				.then((r) => (r.ok ? r.json() : null))
				.then((j) => { if (j && Array.isArray(j.elements) && j.elements.length === 16) align.fromArray(j.elements); })
				.catch(() => {});
		}
		return alignReady;
	}

	function applyCalib() {
		wrapper.position.set(calib.dx || 0, calib.dy || 0, calib.dz || 0);
		wrapper.rotation.set(0, calib.yaw || 0, 0);
		const s = calib.scaleMul || 1;
		wrapper.scale.set(s, s, s);
		wrapper.updateMatrixWorld(true);
	}

	function styleModel(model) {
		const translucent = opacity < 1;
		if (!maxAniso) { try { maxAniso = renderer.capabilities.getMaxAnisotropy() || 1; } catch (_) { maxAniso = 1; } }
		model.traverse((o) => {
			if (!o.material) return;
			const mats = Array.isArray(o.material) ? o.material : [o.material];
			for (const m of mats) {
				m.side = THREE.DoubleSide;            // belt-and-braces vs winding/normals
				// Max anisotropic filtering: keeps the photoreal imagery crisp at the grazing
				// angles you see most of the city at, instead of blurring into mush.
				if (m.map && m.map.anisotropy !== maxAniso) { m.map.anisotropy = maxAniso; m.map.needsUpdate = true; }
				// Two-way: raising opacity back to 1 must restore opaque/depthWrite, not
				// leave tiles stuck translucent. r160 needs needsUpdate when `transparent` flips.
				m.transparent = translucent;
				m.opacity = translucent ? opacity : 1;
				m.depthWrite = !translucent;
				m.needsUpdate = true;
			}
		});
	}

	// Build + configure the TilesRenderer. `tilesetUrl` null => Google/ion auth path
	// (uses `key`); otherwise loads that tileset directly with no auth (custom / Cesium
	// ion glTF exports / the offline test tileset). Shared by init() and loadTileset().
	// A generation token guards against a rapid enable->setKey race building two
	// renderers concurrently (both await alignReady): a stale build disposes and bails.
	async function buildTiles(tilesetUrl, key) {
		const myGen = ++gen;
		await ensureAlign();
		if (disposed || myGen !== gen) return;

		const t = tilesetUrl ? new TilesRenderer(tilesetUrl) : new TilesRenderer();
		if (!tilesetUrl) {
			// A provider discovered by loadKey() (?ionkey= or server {provider:'ion'}) is
			// written to localStorage; re-read it here so ion takes effect on the SAME
			// session instead of only after a reload.
			try { provider = localStorage.getItem(PROVIDER_LS) || provider; } catch (_) {}
			if (provider === 'ion') {
				mode = 'ion';
				t.registerPlugin(new CesiumIonAuthPlugin({ apiToken: key, assetId: ION_GOOGLE_ASSET, autoRefreshToken: true }));
			} else {
				mode = 'google';
				t.registerPlugin(new GoogleCloudAuthPlugin({ apiToken: key, autoRefreshToken: true }));
			}
			// Route tile fetches through the server's disk cache (network-layer redirect;
			// see installFetchCache — does NOT touch URL resolution, so alignment is safe).
			if (serverCache) installFetchCache(location.origin);
		} else {
			mode = 'custom';
		}
		const draco = new DRACOLoader().setDecoderPath(decoderURL);
		t.registerPlugin(new GLTFExtensionsPlugin({ dracoLoader: draco }));
		t.registerPlugin(new TileCompressionPlugin());
		t.registerPlugin(new TilesFadePlugin());
		// Fidelity: errorTarget is the target screen-space error in px; lower => the
		// renderer keeps subdividing to finer tiles (more detail, more bandwidth/memory).
		if (detail != null) t.errorTarget = detail;

		// City-scale fidelity/streaming tuning. Keep far more tiles resident so flying around
		// doesn't constantly re-fetch (steadier detail, fewer pop-ins), stream them in with
		// more concurrency, and never cap subdivision depth — errorTarget alone governs how
		// fine we go. Guarded so a vendored-renderer API change can't throw the whole layer.
		try {
			if (t.lruCache) {
				t.lruCache.minSize = Math.max(t.lruCache.minSize || 0, 900);
				t.lruCache.maxSize = Math.max(t.lruCache.maxSize || 0, 1500);
				if ('maxBytesSize' in t.lruCache) t.lruCache.maxBytesSize = Math.max(t.lruCache.maxBytesSize || 0, 6.0e8);
			}
			if (t.downloadQueue) t.downloadQueue.maxJobs = Math.max(t.downloadQueue.maxJobs || 0, 12);
			if (t.parseQueue) t.parseQueue.maxJobs = Math.max(t.parseQueue.maxJobs || 0, 6);
			t.maxDepth = Infinity;
		} catch (_) {}

		// Superseded while we were awaiting (e.g. setKey)? throw this renderer away.
		if (disposed || myGen !== gen) { try { t.dispose(); } catch (_) {} return; }
		tiles = t;
		buildAt = (typeof performance !== 'undefined' ? performance.now() : 0);
		gotTileset = gotModel = watchdogFired = false;

		// Bake the ECEF->scene matrix onto tiles.group; calibration rides on wrapper.
		tiles.group.matrixAutoUpdate = false;
		tiles.group.matrix.copy(align);
		tiles.group.matrixWorldNeedsUpdate = true;
		wrapper.add(tiles.group);
		applyCalib();

		tiles.setCamera(camera);
		tiles.setResolutionFromRenderer(camera, renderer);

		tiles.addEventListener('load-model', (e) => { gotModel = true; styleModel(e.scene); });
		tiles.addEventListener('load-tile-set', () => { gotTileset = true; setStatus(mode === 'custom' ? '3D tiles: streaming' : 'photorealistic tiles: streaming'); });
		// 0.3.43 dispatches no 'load-error' event (added in 0.4.x); kept for forward-compat.
		// The real auth/network-failure signal is the watchdog in update().
		tiles.addEventListener('load-error', (e) => {
			const msg = (e && e.error && (e.error.status || e.error.message)) || 'load error';
			setStatus('tile error: ' + msg + (mode === 'custom' ? '' : ' (check API key / Map Tiles API enabled)'));
		});
	}

	async function init() {
		if (initStarted || disposed) return;
		initStarted = true;
		const { key, cache } = await loadKey(dataDir);
		serverCache = !!cache;
		// Stream LIVE from the Map Tiles API (routed through the twin server's bounded
		// disk cache when present — see installFetchCache). No offline copy is kept.
		if (!key) {
			setStatus('no API key — add a Google Maps key below');
			initStarted = false;       // allow retry once a key is provided
			return;
		}
		setStatus('loading photorealistic tiles…');
		await buildTiles(null, key);
	}

	function update() {
		if (!enabled || !tiles || disposed) return;
		camera.updateMatrixWorld();
		tiles.setResolutionFromRenderer(camera, renderer);
		tiles.update();
		// Watchdog for a bad key / disabled Map Tiles API / no network: 0.3.43 fires no
		// error event and silently retries the root fetch, so without this the status
		// would sit on "loading…" forever. If nothing has loaded after a grace period,
		// surface the actionable hint.
		if (!watchdogFired && !gotTileset && !gotModel && mode !== 'custom'
			&& buildAt && (performance.now() - buildAt) > 12000) {
			watchdogFired = true;
			setStatus('no tiles loaded — check the API key, that the Map Tiles API is enabled, and your network');
		}
		if (onCredits) {
			// Attribution text changes rarely (only when new tiles stream in), so
			// throttle the getAttributions + string build to ~2 Hz instead of every
			// frame — the unthrottled path allocated ~5 arrays/strings per frame for
			// a UI string that is almost always identical to the previous frame.
			const now = (typeof performance !== 'undefined' ? performance.now() : 0);
			if (now - creditsAt >= 500) {
				creditsAt = now;
				const list = tiles.getAttributions ? tiles.getAttributions([]) : [];
				const txt = list.map((a) => a && a.value).filter(Boolean).join(' · ');
				const lead = mode === 'google' ? 'Imagery © Google' : mode === 'ion' ? 'Cesium ion' : '';
				const next = [lead, txt].filter(Boolean).join(' · ');
				if (next !== credits) { credits = next; onCredits(credits); }
			}
		}
	}

	return {
		group: wrapper,
		get enabled() { return enabled; },
		get tiles() { return tiles; },
		get calib() { return calib; },
		update,
		// Probe the photoreal SURFACE elevation (scene Y) straight below (x,z), used to
		// drape our overlays onto the Google mesh. Casts a ray straight down from high
		// above and returns the hit whose Y is closest to `refY` — i.e. the GROUND under
		// the point, ignoring building roofs / tree canopy (far above) and the antipodal
		// backface of the earth-scale mesh (far below). Returns null when the layer is off
		// or no tile is loaded there yet (caller keeps its last offset).
		sampleGroundY(x, z, refY) {
			if (!tiles || !enabled || !wrapper.visible) return null;
			const ref = Number.isFinite(refY) ? refY : 0;
			_groundFrom.set(x, ref + 4000, z);
			_groundRay.set(_groundFrom, _groundDown);
			const hits = _groundRay.intersectObject(wrapper, true);
			if (!hits.length) return null;
			let best = null, bd = Infinity;
			for (const h of hits) {
				if (h.point.y < ref - 2000) continue;     // skip the antipodal backface
				const d = Math.abs(h.point.y - ref);
				if (d < bd) { bd = d; best = h.point.y; }
			}
			return best;
		},
		setVisible(on) {
			enabled = !!on;
			wrapper.visible = enabled;
			if (enabled) init();
		},
		// Load an arbitrary 3D Tiles tileset (custom build, Cesium ion glTF export, or
		// the offline test tileset) with NO Google key — same alignment + render path.
		// Pass a same-origin or CORS-enabled tileset.json URL.
		loadTileset(url) {
			if (tiles || disposed || !url || initStarted) return;   // not while a build is pending
			initStarted = true;          // block the Google init() path
			enabled = true;
			wrapper.visible = true;
			setStatus('loading 3D tiles…');
			return buildTiles(url, null);
		},
		setOpacity(v) {
			opacity = Math.max(0, Math.min(1, v));
			if (tiles && tiles.forEachLoadedModel) tiles.forEachLoadedModel((m) => styleModel(m));
		},
		setProvider(p) {
			provider = p === 'ion' ? 'ion' : 'google';
			try { localStorage.setItem(PROVIDER_LS, provider); } catch (_) {}
		},
		setKey(k) {
			try { localStorage.setItem(KEY_LS, k); } catch (_) {}
			// Persist to the gitignored .env via the server so it survives restarts and is
			// reused without re-prompting (best-effort; no-op under plain http.server).
			try {
				fetch('/api/photoreal', {
					method: 'POST', headers: { 'Content-Type': 'application/json' },
					body: JSON.stringify({ key: k, provider }),
				}).catch(() => {});
			} catch (_) {}
			// Rebuild so the new key takes effect. Bump `gen` SYNCHRONOUSLY first so a
			// build still in flight (awaiting loadKey/alignReady, tiles still null) is
			// cancelled deterministically instead of racing to add a second renderer.
			gen++;
			if (tiles) { try { tiles.dispose(); } catch (_) {} tiles = null; }
			wrapper.clear();
			initStarted = false;
			if (enabled) init();
		},
		// Higher fidelity = lower errorTarget px. Applies live and persists.
		setDetail(errorTargetPx) {
			detail = Math.max(0.5, Number(errorTargetPx) || 8);
			try { localStorage.setItem(DETAIL_LS, String(detail)); } catch (_) {}
			if (tiles) tiles.errorTarget = detail;
		},
		get detail() { return detail; },
		// Resolve whether a key is already configured (server/.env, localStorage, or URL)
		// so the UI can avoid re-prompting. Returns { key:bool, cache:bool }.
		async probeKey() { const r = await loadKey(dataDir); return { key: !!r.key, cache: !!r.cache }; },
		calibrate(partial) {
			calib = Object.assign({}, calib, partial || {});
			try { localStorage.setItem(CALIB_LS, JSON.stringify(calib)); } catch (_) {}
			applyCalib();
			return calib;
		},
		getCredits() { return credits; },
		dispose() {
			disposed = true;
			uninstallFetchCache();
			if (tiles) { try { tiles.dispose(); } catch (_) {} tiles = null; }
			scene.remove(wrapper);
		},
	};
}
