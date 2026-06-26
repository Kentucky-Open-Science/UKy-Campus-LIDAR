// Streaming ground-conform field — make every overlay sit ON the Google
// Photorealistic 3D mesh, not float above it.
//
// THE PROBLEM. Our overlays (road ribbons, lane markings, crosswalks, signals,
// trees, parked cars, buses, agents, camera markers, LiDAR) are baked at OUR
// DTM/LiDAR elevation (NAVD88 orthometric metres). Google's photoreal mesh is a
// different surface (WGS84 photogrammetry). tools/fit_tiles_align.py lines the two
// up horizontally and converts height with a single constant geoid undulation, so
// a residual remains that VARIES across the city (local relief differences, geoid
// variation, fit residuals, overpasses). The previous drape corrected it with ONE
// global vertical offset sampled under the camera — which can only make the two
// surfaces coincide at a single point, leaving everything else floating or sunk
// (and sliding vertically as you pan). That is the floating you see.
//
// THE FIX. Model the residual as a SPATIAL FIELD:
//
//     offset(x, z) = photorealGroundY(x, z) - ourGroundY(x, z)
//
// This difference is smooth and slowly varying (both surfaces track the terrain),
// so we sample it lazily on a coarse grid by raycasting both surfaces, settle each
// cell against LOD streaming (commit only when recent samples agree — the same
// stability idea as before, but PER CELL instead of one global value), and then
// rebase each overlay vertex/object onto the Google surface by adding the locally
// interpolated offset to its baked Y. At-grade roads land on the mesh; an overpass
// keeps its height above the mesh because we ADD the offset to the baked profile
// rather than clamping to the ground. The field absorbs the geoid + fit residual
// automatically, so no datum constant needs tuning.
//
// Applied on the CPU to the real geometry (one pass, amortised, then idle once the
// field settles) so it stays one draw call per layer, raycasts/collisions see the
// conformed surface (agents drive on the lifted roads for free), and the whole
// thing is unit-testable without a GPU. The pure math (bilinear sample + settle
// median) is exported for tests/drape_math.test.mjs.
import * as THREE from 'three';

// ---- pure, GPU-free core (exported for Node tests) -----------------------

// Bilinear sample of a node grid `vals` (row-major, cols×rows, node[i,j] at world
// origin + (i,j)*cell) at world (x,z). `known` is a 0/1 mask the same shape. Returns
// { v, ready }: ready=false when none of the 4 surrounding nodes is known (caller
// keeps the baked elevation there). Missing corners fall back to the mean of the
// known ones, so a half-filled neighbourhood still yields a sane, continuous value.
export function bilinearSample(vals, known, cols, rows, ox, oz, cell, x, z) {
	const fx = (x - ox) / cell, fz = (z - oz) / cell;
	let i0 = Math.floor(fx), j0 = Math.floor(fz);
	if (i0 < 0) i0 = 0; else if (i0 > cols - 2) i0 = cols - 2;
	if (j0 < 0) j0 = 0; else if (j0 > rows - 2) j0 = rows - 2;
	const tx = Math.min(1, Math.max(0, fx - i0));
	const tz = Math.min(1, Math.max(0, fz - j0));
	const i1 = i0 + 1, j1 = j0 + 1;
	const k00 = known[j0 * cols + i0], k10 = known[j0 * cols + i1];
	const k01 = known[j1 * cols + i0], k11 = known[j1 * cols + i1];
	if (!(k00 || k10 || k01 || k11)) return { v: 0, ready: false };
	// Fill unknown corners with the mean of the known ones so the patch is continuous.
	let sum = 0, cnt = 0;
	if (k00) { sum += vals[j0 * cols + i0]; cnt++; }
	if (k10) { sum += vals[j0 * cols + i1]; cnt++; }
	if (k01) { sum += vals[j1 * cols + i0]; cnt++; }
	if (k11) { sum += vals[j1 * cols + i1]; cnt++; }
	const fb = sum / cnt;
	const v00 = k00 ? vals[j0 * cols + i0] : fb;
	const v10 = k10 ? vals[j0 * cols + i1] : fb;
	const v01 = k01 ? vals[j1 * cols + i0] : fb;
	const v11 = k11 ? vals[j1 * cols + i1] : fb;
	const a = v00 * (1 - tx) + v10 * tx;
	const b = v01 * (1 - tx) + v11 * tx;
	return { v: a * (1 - tz) + b * tz, ready: true };
}

// Settle test for one cell's recent samples. Returns the committed value (median)
// when the spread of the recent window is tight enough that LOD has converged,
// else null (keep waiting). `samples` is an array of numbers (most-recent last).
export function settle(samples, { window = 5, maxRange = 1.0 } = {}) {
	if (samples.length < Math.min(4, window)) return null;
	const w = samples.slice(-window).slice().sort((a, b) => a - b);
	const range = w[w.length - 1] - w[0];
	if (range > maxRange) return null;
	return w[(w.length / 2) | 0];
}

// ---- the live field (THREE-aware) ----------------------------------------

/**
 * createDrapeField({ getOurGroundY, getPhotoreal, cell, ... })
 *   getOurGroundY(x,z) -> number|null   // our DTM/city ground (the baking surface)
 *   getPhotoreal()     -> photoreal handle with .enabled and sampleGroundY(x,z,refY)
 *
 * Drives the grid (update), answers offsetAt(x,z) for dynamic objects, and conforms
 * registered STATIC geometry (road meshes + instanced props) in budgeted chunks.
 */
export function createDrapeField({
	getOurGroundY,
	getPhotoreal,
	cell = 48,                 // grid spacing (m). The field is smooth; ~50 m is ample.
	sampleBudget = 24,         // cells probed per frame (each = 2 raycasts)
	resampleMs = 4000,         // re-probe a settled cell at most this often
	vertexBudget = 60000,      // conformed vertices/instances per frame while dirty
	maxRingM = 1600,           // only maintain cells within this radius of the focus
} = {}) {
	let cols = 0, rows = 0, ox = 0, oz = 0;          // grid geometry (origin = min corner)
	let vals = null, known = null, lastAt = null;    // grid state
	let hist = null;                                  // per-cell recent-sample ring
	let version = 0;                                  // bumps whenever a cell commits
	let strength = 0;                                 // eased 0..1 (off..on)
	let haveBounds = false;

	function setBounds(minX, minZ, maxX, maxZ) {
		// Pad a little so edge roads have a node beyond them to interpolate against.
		minX -= cell; minZ -= cell; maxX += cell; maxZ += cell;
		ox = minX; oz = minZ;
		cols = Math.max(2, Math.ceil((maxX - minX) / cell) + 1);
		rows = Math.max(2, Math.ceil((maxZ - minZ) / cell) + 1);
		vals = new Float32Array(cols * rows);
		known = new Uint8Array(cols * rows);
		lastAt = new Float32Array(cols * rows);       // 0 = never sampled
		hist = new Array(cols * rows);
		haveBounds = true;
	}

	const nodeX = (i) => ox + i * cell;
	const nodeZ = (j) => oz + j * cell;

	// Probe a budget of cells nearest the focus that are due for (re)sampling.
	function probe(now, fx, fz) {
		const pr = getPhotoreal && getPhotoreal();
		if (!pr || !pr.enabled) return;
		const ci = Math.round((fx - ox) / cell), cj = Math.round((fz - oz) / cell);
		const maxR = Math.min(Math.max(cols, rows), Math.ceil(maxRingM / cell));
		let did = 0;
		for (let r = 0; r <= maxR && did < sampleBudget; r++) {
			// walk the ring at radius r (Chebyshev) around the focus cell
			for (let dj = -r; dj <= r && did < sampleBudget; dj++) {
				for (let di = -r; di <= r && did < sampleBudget; di++) {
					if (Math.max(Math.abs(di), Math.abs(dj)) !== r) continue;  // ring only
					const i = ci + di, j = cj + dj;
					if (i < 0 || j < 0 || i >= cols || j >= rows) continue;
					const idx = j * cols + i;
					if (lastAt[idx] && now - lastAt[idx] < resampleMs) continue;
					const x = nodeX(i), z = nodeZ(j);
					const ourY = getOurGroundY(x, z);
					if (ourY == null) continue;
					const gY = pr.sampleGroundY(x, z, ourY);
					lastAt[idx] = now;
					did++;
					if (gY == null) continue;
					let h = hist[idx]; if (!h) h = hist[idx] = [];
					h.push(gY - ourY);
					if (h.length > 8) h.shift();
					const c = settle(h);
					if (c != null && (!known[idx] || Math.abs(c - vals[idx]) > 0.25)) {
						vals[idx] = c; known[idx] = 1; version++;
					}
				}
			}
		}
	}

	// raw (un-strengthed) interpolated offset at world (x,z)
	function rawOffset(x, z) {
		if (!haveBounds) return { v: 0, ready: false };
		return bilinearSample(vals, known, cols, rows, ox, oz, cell, x, z);
	}

	// ---- registry of static geometry to conform ----
	// Each entry caches the BAKED y + (x,z) of every vertex/instance once, so we can
	// recompute conformed y = bakedY + offset(x,z) idempotently as the field evolves.
	const items = [];
	let dirty = false, cursor = 0;          // re-conform queue cursor across frames

	function registerMesh(mesh) {
		const g = mesh.geometry;
		if (!g || !g.attributes || !g.attributes.position) return;
		const p = g.attributes.position;
		const n = p.count;
		const baseY = new Float32Array(n), xz = new Float32Array(n * 2);
		for (let k = 0; k < n; k++) {
			baseY[k] = p.getY(k);
			xz[2 * k] = p.getX(k); xz[2 * k + 1] = p.getZ(k);
		}
		items.push({ kind: 'mesh', attr: p, baseY, xz, n });
		dirty = true;
	}

	function registerInstanced(inst) {
		const n = inst.count;
		const baseY = new Float32Array(n), xz = new Float32Array(n * 2);
		const m = new THREE.Matrix4();
		for (let k = 0; k < n; k++) {
			inst.getMatrixAt(k, m);
			const e = m.elements;
			baseY[k] = e[13]; xz[2 * k] = e[12]; xz[2 * k + 1] = e[14];   // translation
		}
		items.push({ kind: 'inst', inst, baseY, xz, n, _m: m });
		dirty = true;
	}

	// Walk an Object3D subtree and register every draped mesh/instanced mesh.
	function registerTree(root) {
		if (!root) return;
		root.traverse((o) => {
			if (o.isInstancedMesh) registerInstanced(o);
			else if (o.isMesh || o.isPoints) registerMesh(o);
		});
	}

	// Apply conformed Y to up to vertexBudget vertices/instances this frame.
	function flush() {
		if (!dirty) return;
		let budget = vertexBudget;
		while (budget > 0 && items.length) {
			if (cursor >= items.length) { cursor = 0; dirty = false; break; }
			const it = items[cursor];
			const take = Math.min(budget, it.n - (it._k || 0));
			const start = it._k || 0;
			if (it.kind === 'mesh') {
				const p = it.attr;
				for (let k = start; k < start + take; k++) {
					const o = rawOffset(it.xz[2 * k], it.xz[2 * k + 1]);
					p.setY(k, it.baseY[k] + (o.ready ? o.v * strength : 0));
				}
				if (start + take >= it.n) p.needsUpdate = true;
			} else {
				const m = it._m;
				for (let k = start; k < start + take; k++) {
					const o = rawOffset(it.xz[2 * k], it.xz[2 * k + 1]);
					it.inst.getMatrixAt(k, m);
					m.elements[13] = it.baseY[k] + (o.ready ? o.v * strength : 0);
					it.inst.setMatrixAt(k, m);
				}
				if (start + take >= it.n) it.inst.instanceMatrix.needsUpdate = true;
			}
			it._k = start + take;
			budget -= take;
			if (it._k >= it.n) { it._k = 0; cursor++; }
		}
	}

	let lastVersion = -1, lastStrength = -1;

	return {
		setBounds,
		registerTree,
		registerMesh,
		registerInstanced,
		get version() { return version; },
		get strength() { return strength; },
		get ready() { return haveBounds && version > 0; },
		// Offset to add to a DYNAMIC object's baked Y so it sits on the Google surface.
		offsetAt(x, z) {
			if (strength <= 0.001) return 0;
			const o = rawOffset(x, z);
			return o.ready ? o.v * strength : 0;
		},
		// Called once per frame from the render loop.
		update(now, focusX, focusZ, active) {
			// ease strength toward on/off so enabling/disabling is smooth, not a snap
			const target = active ? 1 : 0;
			strength += (target - strength) * 0.1;
			if (Math.abs(target - strength) < 0.002) strength = target;
			if (active && haveBounds) probe(now, focusX, focusZ);
			// Re-conform static geometry when the field or the on/off ease changed.
			if (version !== lastVersion || Math.abs(strength - lastStrength) > 0.003) {
				dirty = true; lastVersion = version; lastStrength = strength;
			}
			flush();
		},
	};
}
