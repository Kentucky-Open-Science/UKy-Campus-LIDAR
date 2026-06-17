// City-scale OSM context for the UKy Campus digital twin.
//
// The LiDAR/terrain only covers the ~2x3 km campus core, but the Lextran bus
// network spans ~18x16 km of Lexington. city.js lays down the rest of the city as
// context so every route, stop, and bus has ground and streets beneath it:
//
//   GROUND  — one big flat plane at a reference elevation just below the campus
//             terrain (so the detailed terrain reads as a raised island on top).
//   STREETS — the full OSM highway network for the service area, projected to scene
//             metres and drawn as one merged, class-coloured LineSegments (a single
//             draw call over ~200k segments).
//
// Data is baked by tools/osm_city.py into data/city.json (2-D [x,z] at y=groundY).
// createCitySystem(data, deps) -> { group, layers:{ ground, streets }, stats, groundY }
//
// This is deliberately lightweight (unlit lines + one plane): it's the backdrop the
// campus, roads, and live transit sit in front of, not another detailed model.

import * as THREE from 'three';

// class -> line colour (major roads brighter so the network reads at a glance)
const STREET_COLOR = {
  motorway: 0xf2c14e, motorway_link: 0xf2c14e,
  trunk: 0xf2c14e, trunk_link: 0xe0b048,
  primary: 0xe0a83c, primary_link: 0xc9943a,
  secondary: 0xbfc4cb, secondary_link: 0xaab0b8,
  tertiary: 0x99a1ab, tertiary_link: 0x99a1ab,
  residential: 0x6c757f, unclassified: 0x6c757f, living_street: 0x636c76,
};
const DEFAULT_STREET = 0x6c757f;
const STREET_LIFT = 0.4;     // above the ground plane
const GROUND_COLOR = 0x171c23;

export function createCitySystem(data, deps = {}) {
  const groundY = (data && typeof data.groundY === 'number') ? data.groundY : 285;
  const roads = (data && data.roads) || [];
  const bb = (data && data.bbox_scene) || [-1000, -1000, 1000, 1000];

  const group = new THREE.Group(); group.name = 'city';
  const layers = { ground: new THREE.Group(), streets: new THREE.Group() };
  layers.ground.name = 'city-ground'; layers.streets.name = 'city-streets';
  group.add(layers.ground, layers.streets);

  // ---- ground plane (covers the street bbox + a margin) ----
  const margin = 600;
  const w = (bb[2] - bb[0]) + margin * 2, d = (bb[3] - bb[1]) + margin * 2;
  const cx = (bb[0] + bb[2]) / 2, cz = (bb[1] + bb[3]) / 2;
  const planeGeo = new THREE.PlaneGeometry(w, d);
  const plane = new THREE.Mesh(planeGeo, new THREE.MeshBasicMaterial({ color: GROUND_COLOR }));
  plane.rotation.x = -Math.PI / 2;             // lie flat in XZ
  plane.position.set(cx, groundY, cz);
  plane.renderOrder = -2;                        // behind everything else
  plane.name = 'city-plane';
  layers.ground.add(plane);

  // ---- streets (one merged, class-coloured LineSegments) ----
  let segCount = 0;
  const positions = [], colors = [];
  const y = groundY + STREET_LIFT;
  const _c = new THREE.Color();
  for (const r of roads) {
    const pts = r.pts; if (!pts || pts.length < 2) continue;
    _c.setHex(STREET_COLOR[r.class] != null ? STREET_COLOR[r.class] : DEFAULT_STREET);
    for (let i = 0; i + 1 < pts.length; i++) {
      const a = pts[i], b = pts[i + 1];
      positions.push(a[0], y, a[1], b[0], y, b[1]);
      colors.push(_c.r, _c.g, _c.b, _c.r, _c.g, _c.b);
      segCount++;
    }
  }
  if (positions.length) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
    geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(colors), 3));
    geo.computeBoundingSphere();
    const mat = new THREE.LineBasicMaterial({ vertexColors: true });
    const lines = new THREE.LineSegments(geo, mat);
    lines.name = 'city-street-lines'; lines.renderOrder = -1;
    layers.streets.add(lines);
  }

  const stats = { streets: roads.length, segments: segCount, groundY };
  return { group, layers, stats, groundY };
}
