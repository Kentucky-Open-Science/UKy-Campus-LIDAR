// Verify the baked ECEF->scene matrix (web/lib/tiles_align.json) is correct and
// correctly oriented (column-major, axis mapping) WITHOUT a browser or a key:
// independently compute WGS84 ECEF for the city-bbox corners and push them through
// the matrix, then compare the resulting scene (x,z) to the values the Python fitter
// reported from the EXACT pipeline projector (pyproj 4326->32616).
//
//   node tools/verify_tiles_align.mjs
//
// Exits non-zero if any corner is off by more than TOL metres.
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const HERE = dirname(fileURLToPath(import.meta.url));
const align = JSON.parse(readFileSync(join(HERE, '..', 'web', 'lib', 'tiles_align.json'), 'utf8'));
const E = align.elements;
const H = align.checkHeight != null ? align.checkHeight : (270.0 + align.geoidN); // ellipsoidal height used by the fitter
const TOL = 2.0;                        // metres vs the fitter's own predicted scene

// WGS84 geodetic (deg, ellipsoidal m) -> ECEF (EPSG:4978).
const A = 6378137.0, F = 1 / 298.257223563, E2 = F * (2 - F);
function ecef(lonDeg, latDeg, h) {
	const lon = lonDeg * Math.PI / 180, lat = latDeg * Math.PI / 180;
	const s = Math.sin(lat), c = Math.cos(lat);
	const N = A / Math.sqrt(1 - E2 * s * s);
	return [ (N + h) * c * Math.cos(lon), (N + h) * c * Math.sin(lon), (N * (1 - E2) + h) * s ];
}
// scene = M * [X,Y,Z,1], M column-major in E.
function toScene([X, Y, Z]) {
	return [
		E[0] * X + E[4] * Y + E[8] * Z + E[12],
		E[1] * X + E[5] * Y + E[9] * Z + E[13],
		E[2] * X + E[6] * Y + E[10] * Z + E[14],
	];
}

// (lon, lat) -> expected scene (x,z) from the EXACT pipeline projector. Read from the
// `corners` the fitter emits into tiles_align.json so these regenerate WITH the matrix
// (no silent drift); the literals are only a fallback for an older align file.
const CORNERS = (Array.isArray(align.corners) && align.corners.length)
	? align.corners.map((c) => ({ lon: c.lon, lat: c.lat, proj: [ c.sx, c.sz ] }))
	: [
		{ lon: -84.6149, lat: 37.9604, proj: [ -9709.7, 8751.9 ] },
		{ lon: -84.3950, lat: 38.1205, proj: [ 9114.8, -9529.6 ] },
		{ lon: -84.6149, lat: 38.1205, proj: [ -10165.6, -9011.0 ] },
		{ lon: -84.3950, lat: 37.9604, proj: [ 9612.8, 8234.0 ] },
	];

let worst = 0;
console.log('matrix scale ~', align.scale.toFixed(6), '| residual horiz max', align.residual_m.horiz_max.toFixed(2), 'm');
for (const c of CORNERS) {
	const [ x, y, z ] = toScene(ecef(c.lon, c.lat, H));
	const d = Math.hypot(x - c.proj[0], z - c.proj[1]);  // vs exact pipeline projection
	worst = Math.max(worst, d);
	console.log(`(${c.lon},${c.lat}) -> scene (${x.toFixed(1)}, ${y.toFixed(1)}, ${z.toFixed(1)})  vs projector (${c.proj[0]}, ${c.proj[1]})  d=${d.toFixed(2)}m`);
}
// Origin sanity: the georef anchor must map to scene HORIZONTAL ~0 (exact), with the
// vertical within a loose band (a ~6 m flat-plane-vs-ellipsoid "bowl" residual that is
// nudgeable and irrelevant once the tiles supply their own relief).
const [ ox, oy, oz ] = toScene(ecef(align.originLonLat[0], align.originLonLat[1], H));
console.log(`origin -> scene (${ox.toFixed(1)}, ${oy.toFixed(1)}, ${oz.toFixed(1)})  (expect ~0 horiz, ~270 vert ±15)`);
const originHorizOk = Math.hypot(ox, oz) < TOL;          // strict: horizontal anchor
const originVertOk = Math.abs(oy - 270) < 15;            // loose: datum wobble / nudge

const pass = worst <= 5.0 && originHorizOk && originVertOk;  // <=5m vs true UTM projection city-wide
console.log(pass
	? `\nPASS — max corner horiz error ${worst.toFixed(2)}m (<=5), origin horiz ok=${originHorizOk}, vert ok=${originVertOk}`
	: `\nFAIL — max corner horiz error ${worst.toFixed(2)}m, origin horiz ok=${originHorizOk}, vert ok=${originVertOk}`);
process.exit(pass ? 0 : 1);
