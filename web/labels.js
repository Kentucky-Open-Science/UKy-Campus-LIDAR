// Street-name labels for the UKy Campus / Lexington twin.
//
// Names already live in the data: data/roads.json (campus streets, OSM names) and
// data/city.json (the wider OSM network). This builds one upright billboard label
// per unique street name — placed at the street's midpoint — and distance-culls
// them each frame so only nearby labels draw (keeps it fast and uncluttered). City
// labels are limited to major arterials so we don't paper the map with every
// residential cul-de-sac.
//
// createStreetLabels(sources, opts) -> { group, tick(camera), setVisible(b), count }

import * as THREE from 'three';

const CITY_LABEL_CLASSES = new Set([
  'motorway', 'trunk', 'primary', 'secondary', 'tertiary',
  'motorway_link', 'trunk_link', 'primary_link',
]);

export function createStreetLabels(sources = {}, opts = {}) {
  const FAR = opts.far || 480;            // cull labels beyond this camera distance (m)
  const MAX = opts.maxLabels || 220;
  const group = new THREE.Group();
  group.name = 'street-labels';

  // Collect candidates keeping the LONGEST road per unique name. Campus and city
  // are kept separate so we can ALWAYS include every campus street (the in-view
  // detailed core) and only then fill the budget with the longest city arterials —
  // otherwise the long city roads crowd out the short campus streets you're looking
  // at. A name present on campus wins (its placement is in-view).
  const campus = new Map(), city = new Map();
  const considerInto = (map, name, pts3) => {
    if (!name || pts3.length < 2) return;
    let len = 0;
    for (let i = 0; i + 1 < pts3.length; i++) {
      len += Math.hypot(pts3[i + 1][0] - pts3[i][0], pts3[i + 1][2] - pts3[i][2]);
    }
    const prev = map.get(name);
    if (prev && prev.len >= len) return;
    map.set(name, { name, len, mid: midpoint(pts3) });
  };

  for (const r of sources.roads || []) considerInto(campus, r.name, r.pts);  // [x,y,z]
  const cy = sources.cityY != null ? sources.cityY : 285;
  for (const r of sources.cityRoads || []) {                                  // [x,z] flat
    if (!CITY_LABEL_CLASSES.has(r.class) || !r.name || campus.has(r.name)) continue;
    considerInto(city, r.name, r.pts.map((p) => [p[0], cy, p[1]]));
  }

  const campusArr = [...campus.values()].sort((a, b) => b.len - a.len);
  const cityArr = [...city.values()].sort((a, b) => b.len - a.len);
  const chosen = campusArr.concat(cityArr).slice(0, MAX);   // all campus, then top city
  const sprites = [];
  for (const L of chosen) {
    const spr = makeLabelSprite(L.name);
    spr.position.set(L.mid[0], L.mid[1] + 3.5, L.mid[2]);
    spr.visible = false;
    spr.renderOrder = 3;                  // over the route ribbons
    group.add(spr);
    sprites.push(spr);
  }

  // per-frame: show only labels within FAR of the camera (XZ distance)
  function tick(camera) {
    if (!group.visible) return;
    const cx = camera.position.x, cz = camera.position.z;
    for (const s of sprites) {
      const dx = cx - s.position.x, dz = cz - s.position.z;
      s.visible = (dx * dx + dz * dz) < FAR * FAR;
    }
  }

  return {
    group, tick, count: sprites.length,
    setVisible(b) { group.visible = !!b; },
  };
}

// midpoint of a polyline by XZ arc length (returns [x, y, z])
function midpoint(pts) {
  let total = 0;
  for (let i = 0; i + 1 < pts.length; i++) {
    total += Math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][2] - pts[i][2]);
  }
  let acc = 0; const half = total / 2;
  for (let i = 0; i + 1 < pts.length; i++) {
    const a = pts[i], b = pts[i + 1];
    const seg = Math.hypot(b[0] - a[0], b[2] - a[2]);
    if (acc + seg >= half) {
      const t = seg ? (half - acc) / seg : 0;
      return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
    }
    acc += seg;
  }
  const m = pts[pts.length >> 1];
  return [m[0], m[1], m[2]];
}

function makeLabelSprite(text) {
  const fontPx = 40, pad = 12;
  const c = document.createElement('canvas');
  const g = c.getContext('2d');
  g.font = `600 ${fontPx}px system-ui, sans-serif`;
  const w = Math.ceil(g.measureText(text).width) + pad * 2;
  const h = fontPx + pad * 2;
  c.width = w; c.height = h;
  g.font = `600 ${fontPx}px system-ui, sans-serif`;
  g.textAlign = 'center'; g.textBaseline = 'middle';
  g.lineWidth = 7; g.strokeStyle = 'rgba(0,0,0,0.85)';
  g.strokeText(text, w / 2, h / 2);
  g.fillStyle = '#ffffff';
  g.fillText(text, w / 2, h / 2);
  const tex = new THREE.CanvasTexture(c);
  tex.anisotropy = 4; tex.minFilter = THREE.LinearFilter;
  const spr = new THREE.Sprite(new THREE.SpriteMaterial({
    map: tex, transparent: true, depthTest: true, depthWrite: false,
  }));
  const worldH = 8;                       // label height in world metres
  spr.scale.set(worldH * (w / h), worldH, 1);
  return spr;
}
