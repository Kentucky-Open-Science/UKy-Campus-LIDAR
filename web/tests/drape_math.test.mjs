// Unit tests for the GPU-free core of the streaming ground-conform field (drape.js).
// Run from web/:  node tests/drape_math.test.mjs
// No WebGL/browser needed — this validates the bilinear sampling + LOD settle logic
// that decides where every overlay vertex lands on the Google photoreal surface.
import { bilinearSample, settle } from '../drape.js';

let fails = 0;
const approx = (a, b, eps = 1e-6) => Math.abs(a - b) <= eps;
function check(name, cond, got) {
	if (cond) { console.log(`  ok  ${name}`); }
	else { console.log(`  FAIL ${name}  (got ${JSON.stringify(got)})`); fails++; }
}

// ---- bilinearSample over a 2x2 node grid, cell=10, origin (0,0) ----
// node values: (0,0)=0  (10,0)=10  (0,10)=20  (10,10)=30
const vals = Float32Array.from([0, 10, 20, 30]);
const all = Uint8Array.from([1, 1, 1, 1]);
const S = (x, z, v = vals, k = all) => bilinearSample(v, k, 2, 2, 0, 0, 10, x, z);

check('corner (0,0) -> 0', approx(S(0, 0).v, 0), S(0, 0));
check('corner (10,10) -> 30', approx(S(10, 10).v, 30), S(10, 10));
check('edge midpoint (5,0) -> 5', approx(S(5, 0).v, 5), S(5, 0));
check('centre (5,5) -> 15', approx(S(5, 5).v, 15), S(5, 5));
check('quarter (2.5,2.5) -> 7.5', approx(S(2.5, 2.5).v, 7.5), S(2.5, 2.5));
check('clamps outside (-100,-100) -> 0', approx(S(-100, -100).v, 0), S(-100, -100));

// partial coverage: only one corner known -> fills the patch with that value, ready=true
const oneKnown = Uint8Array.from([0, 0, 0, 1]);   // only (10,10)=30 known
const p = S(5, 5, vals, oneKnown);
check('partial: one known corner -> its value', approx(p.v, 30) && p.ready === true, p);

// no coverage -> ready=false so the caller keeps the baked elevation
const none = Uint8Array.from([0, 0, 0, 0]);
const z = S(5, 5, vals, none);
check('no coverage -> ready=false, v=0', z.ready === false && z.v === 0, z);

// ---- settle: commit a tight window, reject a noisy one ----
check('settle tight window -> median ~1.0',
	approx(settle([1.0, 1.1, 0.9, 1.05, 1.0]), 1.0, 0.11), settle([1.0, 1.1, 0.9, 1.05, 1.0]));
check('settle noisy window -> null (keep waiting)',
	settle([1, 5, 1, 5, 1]) === null, settle([1, 5, 1, 5, 1]));
check('settle too-few samples -> null',
	settle([1, 1]) === null, settle([1, 1]));
check('settle rejects a single big outlier',
	settle([2.0, 2.0, 2.0, 8.0]) === null, settle([2.0, 2.0, 2.0, 8.0]));

console.log(fails ? `\nDRAPE MATH: ${fails} FAILED` : '\nDRAPE MATH: ALL PASS');
process.exit(fails ? 1 : 0);
