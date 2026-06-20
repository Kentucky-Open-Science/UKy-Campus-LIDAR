// Reproducible vendoring of the 3D Tiles renderer into a single ESM file.
//
//   cd web && node lib/_vendor.mjs
//
// Pins 3d-tiles-renderer@0.3.43 (last line supporting three r160, the viewer's
// vendored revision) and bundles it + the three.js addons it needs (GLTFLoader,
// DRACOLoader, BufferGeometryUtils, Pass) into web/lib/3d-tiles-renderer.module.js.
//
// Only the EXACT bare specifier "three" is kept external so it resolves through the
// page importmap to the one vendored three.module.js (three is single-instance
// sensitive). The addons (imported as "three/examples/jsm/*") are bundled IN and
// reference that same external core, so no extra importmap entries are needed and
// there is no runtime build step. Run after `npm install` in web/.
//
// Lives in lib/ (not tools/) so Node resolves `esbuild` from web/node_modules.
import { build } from 'esbuild';
import { copyFileSync, mkdirSync } from 'node:fs';

await build( {
	entryPoints: [ 'lib/_tiles3d.entry.js' ],
	outfile: 'lib/3d-tiles-renderer.module.js',
	bundle: true,
	format: 'esm',
	legalComments: 'none',
	logLevel: 'info',
	plugins: [ {
		name: 'external-three-core-only',
		setup( b ) {
			// Externalize ONLY `three`; let `three/examples/jsm/*` bundle in.
			b.onResolve( { filter: /^three$/ }, () => ( { path: 'three', external: true } ) );
		},
	} ],
} );

console.log( 'vendored -> web/lib/3d-tiles-renderer.module.js' );

// Vendor the DRACO decoder too — real Google Photorealistic tiles ship draco-compressed
// glTF, and tiles3d.js points DRACOLoader at lib/draco/gltf/. Copy it from the SAME
// pinned three (version-locked to the bundle) so a fresh clone reproduces it without a
// hand-copy. Without this the headline real-tiles path silently fails to decode.
const DRACO_SRC = 'node_modules/three/examples/jsm/libs/draco/gltf/';
const DRACO_DST = 'lib/draco/gltf/';
mkdirSync( DRACO_DST, { recursive: true } );
for ( const f of [ 'draco_decoder.js', 'draco_decoder.wasm', 'draco_wasm_wrapper.js' ] ) {
	copyFileSync( DRACO_SRC + f, DRACO_DST + f );
}
console.log( 'vendored -> web/lib/draco/gltf/ (DRACO decoder, three-pinned)' );
