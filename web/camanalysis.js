// Camera traffic-flow analysis -> calibration seed (the PiP "Analysis" button).
//
// Goal: auto-ASSIST camera->scene calibration. We can't fully solve a homography from
// pixels alone, but moving vehicles give us two strong, free signals:
//   1. the DOMINANT traffic-flow direction(s) in IMAGE space (per 2x2 quad), and
//   2. tire-points (bbox bottom-centre) sampled along the lanes,
// which we align to the nearest real road heading at the camera's mapped location to
// PROPOSE an image->scene correspondence the user can accept into calibration, nudge in
// the 3D view, then Solve & Save.
//
// Dependency-free + offline-safe (constitution: no runtime deps). It consumes the EXISTING
// detection relay (tools/twin_server.py /api/cameras/detections) by polling it for a window
// — no live camera or GPU here; if no detector is publishing, or too few moving tracks are
// seen, it degrades to a clear status and emits nothing.
//
//   createCameraAnalysis(deps) -> controller (see the returned object at the bottom)
//
// All image coordinates are QUAD-LOCAL normalized [0,1] (the same space the homography and
// the detector boxes use); scene coordinates are metres (x,z), matching homography.js.

// ----------------------------------------------------------------- geometry ---

// 2x2 quad layout helpers (mirror cameras.js quadOf / the overlay's fullOf).
export function quadColRow(quad) { return { col: quad % 2, row: quad >= 2 ? 1 : 0 }; }
// quad-local (qu,qv) in [0,1] -> full-frame normalized [0,1] (for overlay drawing).
export function quadToFull(quad, qu, qv) {
  const { col, row } = quadColRow(quad);
  return [col * 0.5 + qu * 0.5, row * 0.5 + qv * 0.5];
}

// Tire point of a detector box: bottom-centre, in the box's own quad-local [0,1] space.
// box = [x1,y1,x2,y2] normalized WITHIN the quad (exactly what the relay publishes).
export function tirePoint(box) {
  return [(box[0] + box[2]) / 2, box[3]];
}

// Vehicle COCO classes we trust for FLOW (cars/trucks/buses/motorcycles). Pedestrians and
// bicycles move erratically / off-road, so they're excluded from the direction estimate
// (the detector's class ids: 2=car 3=moto 5=bus 7=truck — see camera_detect.DETECT_CLASSES).
export const FLOW_CLASSES = new Set([2, 3, 5, 7]);

function len2(x, y) { return Math.hypot(x, y); }
function norm2(x, y) { const l = len2(x, y) || 1; return [x / l, y / l]; }

// Principal direction of a set of 2D vectors via the orientation tensor (a.k.a. the
// "doubled-angle" trick): summing v*v^T would let opposite vectors cancel, so we build the
// structure tensor and take its dominant eigenvector — that gives the AXIS the motion lies
// along (undirected), which is exactly what a two-way street needs. Returns a unit axis
// [ax,ay] (sign arbitrary) or null if there's no coherent direction.
export function principalAxis(vecs) {
  let sxx = 0, sxy = 0, syy = 0, n = 0;
  for (const v of vecs) {
    const l = len2(v[0], v[1]);
    if (l < 1e-9) continue;
    const ux = v[0] / l, uy = v[1] / l;       // weight every track equally (unit), so a
    sxx += ux * ux; sxy += ux * uy; syy += uy * uy;   // few fast cars don't dominate
    n++;
  }
  if (n < 2) return null;
  sxx /= n; sxy /= n; syy /= n;
  // dominant eigenvector of [[sxx,sxy],[sxy,syy]] (symmetric 2x2, closed form)
  const tr = sxx + syy, det = sxx * syy - sxy * sxy;
  const disc = Math.max(0, tr * tr / 4 - det);
  const l1 = tr / 2 + Math.sqrt(disc);        // larger eigenvalue
  let ax, ay;
  if (Math.abs(sxy) > 1e-12) { ax = l1 - syy; ay = sxy; }
  else { ax = sxx >= syy ? 1 : 0; ay = sxx >= syy ? 0 : 1; }
  const [nx, ny] = norm2(ax, ay);
  // coherence = how anisotropic the spread is (1 = perfectly linear, 0 = isotropic blob)
  const coherence = l1 > 1e-12 ? (l1 - (tr - l1)) / (l1 + (tr - l1) || 1) : 0;
  return { axis: [nx, ny], coherence, n };
}

// Split tracks into up to two opposing flow directions along a principal axis. Each track's
// net displacement is projected onto the axis; positive projections average into one mean
// direction, negatives into the other. Returns [{dir:[x,y], count, speed}] sorted by count
// (1 element for one-way, 2 for two-way). dir points the way traffic actually travels.
export function splitTwoWay(disps, axis) {
  const pos = { sx: 0, sy: 0, n: 0, sp: 0 }, neg = { sx: 0, sy: 0, n: 0, sp: 0 };
  for (const d of disps) {
    const proj = d[0] * axis[0] + d[1] * axis[1];
    const bin = proj >= 0 ? pos : neg;
    const [ux, uy] = norm2(d[0], d[1]);
    bin.sx += ux; bin.sy += uy; bin.n++; bin.sp += len2(d[0], d[1]);
  }
  const out = [];
  for (const b of [pos, neg]) {
    if (b.n === 0) continue;
    const [dx, dy] = norm2(b.sx, b.sy);
    out.push({ dir: [dx, dy], count: b.n, speed: b.sp / b.n });
  }
  out.sort((a, b) => b.count - a.count);
  return out;
}

// ------------------------------------------------------- image-space tracker ---
// The relay only publishes the LATEST frame's boxes (no IDs), so we re-derive short tracks
// by greedy nearest-neighbour association of tire-points between consecutive polled frames,
// per quad. Cheap and robust enough for a 10-15 s window: we only need each vehicle's net
// travel direction, not a perfect trajectory.
class QuadTracker {
  constructor(assocDist = 0.12, maxMiss = 4) {
    this.assoc = assocDist;     // max tire-point jump between frames (quad-normalized units)
    this.maxMiss = maxMiss;     // drop a track after this many frames unmatched
    this.tracks = [];           // {pts:[[u,v]], cls, miss, t0, t1}
  }
  step(points, t) {            // points: [{uv:[u,v], cls}]
    const free = this.tracks.filter((tr) => tr.miss <= this.maxMiss);
    const used = new Set();
    const cand = [];
    points.forEach((p, pi) => {
      free.forEach((tr, ti) => {
        const last = tr.pts[tr.pts.length - 1];
        const d = len2(p.uv[0] - last[0], p.uv[1] - last[1]);
        if (d <= this.assoc) cand.push([d, pi, ti]);
      });
    });
    cand.sort((a, b) => a[0] - b[0]);
    const matchedPt = new Set(), matchedTr = new Set();
    for (const [, pi, ti] of cand) {
      if (matchedPt.has(pi) || matchedTr.has(ti)) continue;
      const tr = free[ti], p = points[pi];
      tr.pts.push(p.uv); tr.miss = 0; tr.t1 = t; if (p.cls != null) tr.cls = p.cls;
      matchedPt.add(pi); matchedTr.add(ti); used.add(tr);
    }
    for (const tr of this.tracks) if (!used.has(tr)) tr.miss++;
    points.forEach((p, pi) => {
      if (matchedPt.has(pi)) return;
      this.tracks.push({ pts: [p.uv], cls: p.cls, miss: 0, t0: t, t1: t });
    });
  }
  // finished tracks with real displacement: net vector (end-start) + mean tire point.
  // minDisp filters parked cars / detector jitter; minPts wants a real tracklet.
  flows(minDisp = 0.06, minPts = 3) {
    const out = [];
    for (const tr of this.tracks) {
      if (tr.pts.length < minPts) continue;
      const a = tr.pts[0], b = tr.pts[tr.pts.length - 1];
      const disp = [b[0] - a[0], b[1] - a[1]];
      if (len2(disp[0], disp[1]) < minDisp) continue;
      let cx = 0, cy = 0;
      for (const q of tr.pts) { cx += q[0]; cy += q[1]; }
      out.push({ disp, mid: [cx / tr.pts.length, cy / tr.pts.length],
                 cls: tr.cls, end: b, start: a, dt: Math.max(1e-3, tr.t1 - tr.t0) });
    }
    return out;
  }
}

// --------------------------------------------------------- road-heading match ---

// Nearest road point + local heading to a scene (x,z), scanning polylines. `roads` is an
// array of { pts } where each pt is [x,z] OR [x,y,z]; we read x = pt[0] and z = the LAST elem.
// Returns { x, z, dir:[dx,dz] (unit, segment direction), dist } for the closest segment, or
// null. dir is undirected here (caller resolves the sign from the observed flow).
export function nearestRoad(roads, sx, sz, maxDist = 120) {
  let best = null, bd = maxDist * maxDist;
  for (const r of roads || []) {
    const pts = r.pts || r;
    if (!pts || pts.length < 2) continue;
    for (let i = 0; i + 1 < pts.length; i++) {
      const ax = pts[i][0], az = pts[i][pts[i].length - 1];
      const bx = pts[i + 1][0], bz = pts[i + 1][pts[i + 1].length - 1];
      const dx = bx - ax, dz = bz - az;
      const seg2 = dx * dx + dz * dz;
      if (seg2 < 1e-6) continue;
      let t = ((sx - ax) * dx + (sz - az) * dz) / seg2;
      t = Math.max(0, Math.min(1, t));
      const px = ax + t * dx, pz = az + t * dz;
      const dd = (sx - px) * (sx - px) + (sz - pz) * (sz - pz);
      if (dd < bd) {
        bd = dd;
        const l = Math.hypot(dx, dz) || 1;
        best = { x: px, z: pz, dir: [dx / l, dz / l], dist: Math.sqrt(dd),
                 name: r.name || null };
      }
    }
  }
  return best;
}

// =====================================================================
export function createCameraAnalysis(deps = {}) {
  // deps: { homography:{solveHomography,applyHomography}, getRoads:()=>[...],
  //         getDetections:async(camId)=>{dets,stale,age}, cameraOf:(id)=>rec,
  //         startDetector:async(camId)=>any, stopDetector:async(camId)=>any }
  // startDetector/stopDetector (optional) let Analysis bootstrap a COLD, uncalibrated camera:
  // start() asks the server to spin up a raw image-space YOLO detector (analysis mode), so
  // there are boxes to track even with zero calibration; finish/stop tears it down again.
  const { homography, getRoads, getDetections, cameraOf, startDetector, stopDetector } = deps;

  const state = {
    running: false, camId: null, status: 'idle',
    trackers: new Map(),           // quad -> QuadTracker
    frames: 0, lastFrame: null,
    flowsByQuad: new Map(),        // quad -> [{dir,count,speed,axis,mid,...}] (image space)
    proposal: null,                // { camId, quads:{ quad: { pairs:[{img,scene}], H?, road } } }
    arrows: [],                    // overlay arrows: [{quad, mid:[qu,qv], dir:[u,v], color}]
    onUpdate: null,                // callback(state) after each poll / on finish
    _timer: null, _deadline: 0,
    _detStarted: false,            // true once WE started the raw analysis detector (so we stop it)
  };

  function setStatus(s) { state.status = s; if (state.onUpdate) state.onUpdate(view()); }

  // Stop the raw analysis detector we started (idempotent; fire-and-forget). We pass
  // analysis:true on the server side so this can ONLY ever stop the analysis-mode detector,
  // never a normal "Run YOLO" detector that may also be running for this camera.
  function stopDet() {
    if (state._detStarted && stopDetector && state.camId) {
      const id = state.camId;
      state._detStarted = false;
      Promise.resolve().then(() => stopDetector(id)).catch(() => {});
    } else {
      state._detStarted = false;
    }
  }

  function reset() {
    if (state._timer) { clearTimeout(state._timer); state._timer = null; }
    stopDet();
    state.running = false;
    state.trackers = new Map();
    state.frames = 0; state.lastFrame = null;
    state.flowsByQuad = new Map();
    state.proposal = null; state.arrows = [];
  }

  // ---- collection loop: poll the detection relay, feed the per-quad trackers ----
  async function poll() {
    if (!state.running) return;
    const remain = Math.max(0, Math.round((state._deadline - Date.now()) / 1000));
    let d = null;
    try { d = await getDetections(state.camId); } catch (e) { d = null; }
    if (!d) {
      // The relay endpoint itself failed (server down). Nothing to wait for.
      setStatus('analysis: detection relay offline — is the twin server running?');
      return finish(true);
    }
    if (d.stale || !d.dets) {
      // No detector publishing YET. With analysis-mode auto-start this is just the warm-up
      // window (the YOLO process is loading the model / opening the stream), so keep waiting
      // with a live countdown until the deadline; only report "nothing" if we time out.
      setStatus(`analysis: starting YOLO + waiting for traffic… ${remain}s`);
      if (Date.now() >= state._deadline) { return finish(false); }
    } else if (d.frame !== state.lastFrame) {       // only ingest genuinely new frames
      state.lastFrame = d.frame;
      const now = (typeof performance !== 'undefined' ? performance.now() : Date.now()) / 1000;
      const byQuad = new Map();
      for (const det of d.dets) {
        if (!FLOW_CLASSES.has(det.cls)) continue;
        const q = det.quad;
        if (!byQuad.has(q)) byQuad.set(q, []);
        byQuad.get(q).push({ uv: tirePoint(det.box), cls: det.cls });
      }
      for (const [q, pts] of byQuad) {
        if (!state.trackers.has(q)) state.trackers.set(q, new QuadTracker());
        state.trackers.get(q).step(pts, now);
      }
      state.frames++;
      setStatus(`analysis: collecting traffic… ${state.frames} frames, ${remain}s`);
    } else {
      // a frame arrived but it's the same one we already ingested — still tick the countdown
      setStatus(`analysis: collecting traffic… ${state.frames} frames, ${remain}s`);
    }
    if (Date.now() >= state._deadline) return finish(false);
    state._timer = setTimeout(poll, 150);
  }

  // ---- analysis: per-quad principal flow + map-aligned correspondence proposal ----
  function finish(aborted) {
    state.running = false;
    if (state._timer) { clearTimeout(state._timer); state._timer = null; }
    stopDet();                 // tear down the raw analysis detector we spun up (if any)
    if (aborted) return;

    const cam = cameraOf ? cameraOf(state.camId) : null;
    const camPos = cam && cam.position ? [cam.position[0], cam.position[cam.position.length - 1]] : null;
    const roads = (getRoads ? getRoads() : []) || [];
    const road = camPos ? nearestRoad(roads, camPos[0], camPos[1]) : null;

    state.flowsByQuad = new Map();
    state.arrows = [];
    const proposal = { camId: state.camId, road, camPos, quads: {} };
    let totalTracks = 0, quadsWithFlow = 0;

    for (const [quad, tr] of state.trackers) {
      const flows = tr.flows();
      totalTracks += flows.length;
      if (flows.length < 2) continue;                 // need a couple of moving vehicles
      const disps = flows.map((f) => f.disp);
      const pa = principalAxis(disps);
      if (!pa) continue;
      const dirs = splitTwoWay(disps, pa.axis);       // 1 (one-way) or 2 (two-way) directions
      quadsWithFlow++;
      state.flowsByQuad.set(quad, { axis: pa.axis, coherence: pa.coherence, dirs, flows });

      // overlay arrows anchored at each direction's mean tire-point cluster
      for (const [k, fd] of dirs.entries()) {
        // anchor = mean of the tire mids whose displacement matches this direction's sign
        let cx = 0, cy = 0, n = 0;
        for (const f of flows) {
          const s = f.disp[0] * fd.dir[0] + f.disp[1] * fd.dir[1];
          if (s >= 0) { cx += f.mid[0]; cy += f.mid[1]; n++; }
        }
        const mid = n ? [cx / n, cy / n] : flows[0].mid;
        state.arrows.push({ quad, mid, dir: fd.dir, primary: k === 0,
                            color: k === 0 ? '#49d08a' : '#ffb24a' });
      }

      // ---- correspondence proposal: align image flow to the road heading ----
      // We can only seed scene points when we know where this camera sits on the map AND
      // there is a road nearby to borrow a heading + centreline from.
      if (camPos && road) {
        const pairs = proposeCorrespondence(quad, flows, dirs, road, camPos);
        proposal.quads[quad] = { pairs, road, dirs };
        // If we have >=4 spread image points, try to solve a SEED homography too, so the
        // user can preview/accept a full fit (not just nudge raw points).
        if (homography && pairs.length >= 4) {
          const H = homography.solveHomography(pairs.map((p) => p.img), pairs.map((p) => p.scene));
          if (H) proposal.quads[quad].H = H;
        }
      }
    }

    state.proposal = (Object.keys(proposal.quads).length || (road && camPos)) ? proposal : null;

    // ---- status summary ----
    if (state.frames === 0) {
      // We auto-started the analysis detector, so an empty window means YOLO never produced
      // boxes — usually missing GPU/ultralytics deps on the server, or no vehicles in frame.
      setStatus('analysis: no detections — YOLO did not publish (check the server has '
                + 'ultralytics+opencv), or no vehicles were visible. Retry when traffic is up.');
    } else if (quadsWithFlow === 0) {
      setStatus(`analysis: saw ${state.frames} frames but too few moving vehicles to infer flow — ` +
                'let traffic build up and retry');
    } else {
      const dirWord = [...state.flowsByQuad.values()]
        .reduce((s, q) => s + q.dirs.length, 0);
      const seeded = Object.values(proposal.quads).filter((q) => q.pairs && q.pairs.length).length;
      let msg = `analysis: ${quadsWithFlow} quad(s), ${dirWord} flow direction(s) from ${totalTracks} tracks`;
      if (!road) msg += ' · no road within range to map to (flow only)';
      else if (seeded) msg += ` · proposed ${seeded} quad seed(s) — Accept to load into calibration`;
      setStatus(msg);
    }
    if (state.onUpdate) state.onUpdate(view());
  }

  // Build image->scene correspondence candidates for one quad by laying the observed tire
  // points onto the road centreline near the camera, ordered along the dominant flow and
  // offset to the lane side implied by their direction cluster. This is a SEED the user
  // refines — not a final calibration — so we favour a spread of points over precision.
  function proposeCorrespondence(quad, flows, dirs, road, camPos) {
    // Resolve the road's directed heading to the PRIMARY image flow: pick whichever of
    // (+roadDir, -roadDir) we declare "image-forward". We can't know the true mapping from
    // image to world rotation without calibration, so we anchor on the road centreline and
    // spread points proportionally to each track's position ALONG the image flow axis. The
    // user nudges the result; the value here is a correctly-ORIENTED, on-road starting set.
    const primary = dirs[0].dir;                         // dominant image-space travel dir
    const rdir = road.dir;                               // unit road direction at camera
    // lateral (perpendicular) road axis, for a small two-lane offset
    const perp = [-rdir[1], rdir[0]];
    const LANE = 3.0;                                    // ~half a lane offset (m)
    const SPAN = 18.0;                                   // spread samples over ~this many m

    // order tracks by their projection onto the primary image flow direction
    const scored = flows.map((f) => ({
      f, s: f.mid[0] * primary[0] + f.mid[1] * primary[1],
      side: ((f.disp[0] * primary[0] + f.disp[1] * primary[1]) >= 0) ? 1 : -1,
    })).sort((a, b) => a.s - b.s);

    const sMin = scored[0].s, sMax = scored[scored.length - 1].s, range = (sMax - sMin) || 1;
    // de-dup near-coincident image samples so the homography solve isn't degenerate
    const picked = [];
    for (const it of scored) {
      const uv = it.f.mid;
      if (picked.some((p) => len2(p.img[0] - uv[0], p.img[1] - uv[1]) < 0.04)) continue;
      const along = ((it.s - sMin) / range - 0.5) * SPAN;     // metres along the road
      const lat = it.side * (LANE / 2);                        // lane side from flow sign
      const scene = [
        road.x + rdir[0] * along + perp[0] * lat,
        road.z + rdir[1] * along + perp[1] * lat,
      ];
      picked.push({ img: uv, scene, side: it.side });
      if (picked.length >= 8) break;                           // a healthy spread is enough
    }
    return picked;
  }

  // ------------------------------------------------------- public controller ---
  function view() {
    return {
      running: state.running, camId: state.camId, status: state.status,
      frames: state.frames, arrows: state.arrows,
      flowsByQuad: state.flowsByQuad, proposal: state.proposal,
    };
  }

  return {
    isRunning: () => state.running,
    status: () => state.status,
    arrows: () => state.arrows,                 // [{quad, mid:[qu,qv], dir:[u,v], color, primary}]
    flows: () => state.flowsByQuad,
    proposal: () => state.proposal,             // {camId, road, camPos, quads:{q:{pairs,H?,...}}}
    view,
    // Start a collection+analysis window for `camId`. onUpdate(view) fires on each poll and
    // at the end. durationMs defaults to 12 s (in the requested 10-15 s band).
    start(camId, { durationMs = 12000, onUpdate = null } = {}) {
      reset();
      state.camId = String(camId);
      state.running = true;
      state.onUpdate = onUpdate;
      state._deadline = Date.now() + Math.max(3000, durationMs);
      const secs = Math.round(durationMs / 1000);
      // Bootstrap a COLD camera: ask the server to start a raw image-space (analysis-mode)
      // YOLO detector so there are boxes to track even with zero calibration. Fire-and-forget
      // — the poll loop just watches the relay; if start fails (no GPU/deps) the relay stays
      // stale and we time out with a clear message. Mark _detStarted optimistically so we
      // always issue the matching stop on finish/cancel (a stop on a non-running detector is a
      // harmless no-op server-side).
      if (startDetector) {
        state._detStarted = true;
        Promise.resolve().then(() => startDetector(state.camId)).catch(() => {});
        setStatus(`analysis: starting YOLO + collecting traffic… ${secs}s`);
      } else {
        setStatus(`analysis: collecting vehicle tracks (~${secs}s)…`);
      }
      poll();
      return this;
    },
    stop() { reset(); setStatus('analysis: stopped'); },
    // expose the pure helpers for unit tests / console
    _internals: { principalAxis, splitTwoWay, nearestRoad, tirePoint, QuadTracker, quadToFull },
  };
}
