// Bundle entry for the vendored 3d-tiles-renderer ESM. Built by web/lib/_vendor.mjs
// (`npm run vendor:tiles`), which also vendors the DRACO decoder into lib/draco/gltf/.
//
// We pin 3d-tiles-renderer@0.3.43 because it is the last line that supports the
// viewer's vendored three.js r160 (0.4.x requires three >=0.166). esbuild bundles
// this into web/lib/3d-tiles-renderer.module.js with `three` kept EXTERNAL, so the
// addons (GLTFLoader/DRACOLoader/KTX2Loader) and the renderer all share the single
// `three` instance resolved through the page importmap. No runtime build step.
export {
	TilesRenderer,
	WGS84_ELLIPSOID,
	Ellipsoid,
} from '3d-tiles-renderer';

export {
	GoogleCloudAuthPlugin,
	CesiumIonAuthPlugin,
	GLTFExtensionsPlugin,
	TileCompressionPlugin,
	TilesFadePlugin,
} from '3d-tiles-renderer/plugins';

// DRACO decoder for Google's draco-compressed glTF tiles. Bundled in; it imports
// `three` (external) only. The decoder .wasm/.js is vendored under lib/draco/.
export { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader.js';
