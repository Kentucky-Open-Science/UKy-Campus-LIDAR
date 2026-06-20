// Flat-world mode — pin the whole scene to ONE ground elevation.
//
// The viewer carried two elevation systems: the campus LiDAR terrain (real relief) and
// a separate, lower flat city plane beyond it. Moving vehicles drape onto whichever
// surface is under them, and at the campus/city seam buses sampled the low city plane
// while the road they were on sat ~2 m higher — so they sank through the asphalt.
//
// Rather than reconcile the two systems, flat mode collapses them: terrain, roads,
// ground plane, buildings, buses, agents, and camera markers all share FLAT_Y, so a
// vehicle can never be at a different height than the road it's on. The campus relief
// is intentionally discarded; restore it with ?flat=0.
const _p = new URLSearchParams(location.search);
// Default is now REAL elevation: the photorealistic basemap is on by default and has true
// 3D terrain, so roads/labels/traffic must use real elevation to drape onto it (in flat
// mode they'd be buried under the terrain). Pass ?flat=1 to restore the old single-plane
// mode (useful when the photoreal layer is off and you want buses pinned to one height).
export const FLAT_WORLD = _p.get('flat') === '1';
export const FLAT_Y = 285;                            // the single ground elevation (scene metres)
