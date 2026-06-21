export const meta = {
  name: 'twin-code-review',
  description: 'Exhaustive multi-agent review of the Lexington digital twin: camera detection/render accuracy, homography placement, server kinematics, perf, robustness; each finding adversarially verified',
  phases: [
    { title: 'Review', detail: 'parallel deep readers, one per dimension' },
    { title: 'Verify', detail: 'adversarially verify each finding as it lands' },
  ],
}

const ROOT = 'C:/Users/Sam/Desktop/UKy-Campus-LIDAR'

const FINDINGS_SCHEMA = {
  type: 'object',
  additionalProperties: false,
  properties: {
    dimension: { type: 'string' },
    summary: { type: 'string', description: 'one-paragraph assessment of this dimension' },
    findings: {
      type: 'array',
      items: {
        type: 'object',
        additionalProperties: false,
        properties: {
          title: { type: 'string' },
          severity: { type: 'string', enum: ['critical', 'high', 'medium', 'low', 'optimization'] },
          category: { type: 'string', enum: ['bug', 'accuracy', 'performance', 'robustness', 'security', 'correctness', 'maintainability'] },
          file: { type: 'string' },
          lines: { type: 'string', description: 'e.g. 140-177 or single line 252' },
          description: { type: 'string', description: 'what is wrong and why it matters, concretely' },
          evidence: { type: 'string', description: 'the specific code/quote that proves it' },
          recommendation: { type: 'string', description: 'precise fix, code-level' },
        },
        required: ['title', 'severity', 'category', 'file', 'lines', 'description', 'recommendation'],
      },
    },
  },
  required: ['dimension', 'findings'],
}

const VERDICT_SCHEMA = {
  type: 'object',
  additionalProperties: false,
  properties: {
    verdict: { type: 'string', enum: ['confirmed', 'rejected', 'partial', 'uncertain'] },
    severity_adjusted: { type: 'string', enum: ['critical', 'high', 'medium', 'low', 'optimization'] },
    is_real: { type: 'boolean' },
    reasoning: { type: 'string' },
    fix_correct: { type: 'boolean', description: 'is the recommended fix correct and safe?' },
    fix_notes: { type: 'string', description: 'corrections or refinements to the proposed fix' },
  },
  required: ['verdict', 'is_real', 'reasoning'],
}

const COMMON = [
  'You are reviewing a real, working codebase: an interactive 3D digital twin of Lexington, KY.',
  'Repo root: ' + ROOT + '. Use Read/Grep/Bash freely. DO NOT edit any files - review only.',
  '',
  'Architecture you must respect when judging (from the project constitution):',
  '- One packed buffer / one draw call per visual layer; no per-frame raycasts over large meshes; no per-object draw calls that scale with data size.',
  '- Graceful degradation: layers must render something with missing data/proxy and must NEVER throw into the render loop.',
  '- One georeference + one ground model: scene elevation obeys the active ground model (real terrain heightmap, or flat-world FLAT_Y=285 when ?flat is on, which is the DEFAULT).',
  '- Calibration is hand-authored config in calibration/cameras.json (NOT under gitignored web/data/).',
  '',
  'Coordinate conventions that MUST stay consistent end-to-end (verify they actually do):',
  '- Each camera HLS stream is a 2x2 quad of 4 independent wide-angle cameras. quad index 0=TL,1=TR,2=BL,3=BR.',
  '- Detector splits the frame, runs YOLO per CALIBRATED quad, takes the vehicle tire point = bbox bottom-centre, normalizes it to [0,1] WITHIN the quad (u=cx/sw, v=y2/sh), maps it through the per-(camera,quad) homography H to scene (x,z).',
  '- The homography was authored from clicks normalized [0,1] within the quad. applyHomography(H,[u,v]) -> [x,z] in scene metres.',
  '- Detector also relays raw boxes normalized within the quad ([x1/sw, y1/sh, x2/sw, y2/sh]); the PiP overlay draws them as (col*0.5 + box*0.5).',
  '',
  'Only report findings you can substantiate by reading the actual code. For each finding give file + line range + the exact code evidence + a concrete code-level fix. Prefer a few high-confidence findings over many speculative ones, but be thorough within your dimension. Severity optimization = a perf/cleanup win that is not a bug.',
  'Return ONLY the structured object.',
].join('\n')

const DIMENSIONS = [
  {
    key: 'detect-accuracy',
    prompt: COMMON + '\n\n' + [
      'DIMENSION: Camera vehicle-detection ACCURACY and rendering correctness.',
      'Read deeply and trace the whole path:',
      '- tools/camera_detect.py  (apply_h, heading_from_motion, SceneTracker._dedup/update, tire_to_scene, CameraDetector.run, quad split, follow_active idle path, publish_detections)',
      '- web/cameras.js          (quadOf, imageToScene, sceneToImage, calib.solve/save)',
      '- web/app.js              (camCal.draw detection-box rendering ~1335-1374, pollDetections ~1388-1408, CLS_COLOR/CLS_NAME)',
      '- tools/twin_server.py    (_get_detections / _post_detections relay, DET_TTL)',
      '',
      'Questions to answer concretely with evidence:',
      '1. Are detected vehicles placed at the correct scene (x,z)? Is the tire-point normalization in the detector (u,v within quad) IDENTICAL to what calibration authored and what imageToScene/quadOf assume? Any off-by-one / row,col / TL-vs-BL mismatch between detector quad split, web quadOf, and the PiP box draw?',
      '2. Heading: heading_from_motion + the min_move gate. Do newly spawned cars get a sane heading, or do they all face +X until they move? Is atan2(-dz,dx) consistent with the server forward = (cos yaw, 0, -sin yaw) and netagents arrow (+X)?',
      '3. Dedup/association: greedy clustering + greedy NN association - can it swap track identities, drop cars, or merge two distinct cars in adjacent lanes (dedup_m=4, assoc_m=8)? Is order-dependence a real accuracy problem at a busy intersection?',
      '4. follow_active idle path: while idle it does NOT publish empty detections, so the PiP overlay shows stale boxes up to DET_TTL. Does idling correctly age out/despawn cars? Any ghost cars?',
      '5. PiP overlay vs spawned car: overlay boxes are server-published and lag the buffered video by ~1-2s - is the drawn box mapped to the right quad region of the displayed video?',
      '6. Multiple cameras publishing detections: any cross-camera mixups in the relay or active-camera handling?',
    ].join('\n'),
  },
  {
    key: 'homography',
    prompt: COMMON + '\n\n' + [
      'DIMENSION: Homography solver + numerical correctness, and JS<->Python parity.',
      'Read: web/homography.js (matMul3, invertHomography, applyHomography, normalize/Hartley, solveLinear Gaussian elim, solveHomography inhomogeneous DLT via normal equations, reprojError); tools/camera_detect.py apply_h (must match applyHomography exactly).',
      '',
      'Answer with evidence:',
      '1. Is the DLT correct? The normal-equations path squares the condition number - given Hartley normalization, is accuracy adequate or is there a real precision risk for near-degenerate sets?',
      '2. solveLinear pivot threshold 1e-15 and degeneracy guards - correct rejection of collinear/coincident points? Any case where it returns a bogus H instead of null?',
      '3. invertHomography det threshold and applyHomography w-near-zero handling (points at/behind the horizon) - does it return NaN sanely and do all callers guard for it?',
      '4. reprojError returns MAX residual in scene metres - right quality metric, surfaced correctly to the user?',
      '5. Parity: does python apply_h produce the SAME result as JS applyHomography for the same H and point? Any rounding / None-vs-NaN divergence that places detector cars differently from a click-spawned car?',
      '6. Is least-squares for >4 points actually true LSQ? Any bug assembling A^T A / A^T b?',
    ].join('\n'),
  },
  {
    key: 'placement-ux',
    prompt: COMMON + '\n\n' + [
      'DIMENSION: User-guided camera->twin placement (calibration authoring) correctness and UX robustness.',
      'Read: web/app.js camCal IIFE ~1315-1492 (enterCalibrate, addImagePoint, onScenePoint, save, draw, sceneToImage cross markers, spawn/clearCars, heartbeat) and the pointerup handler ~1922-1979 + groundPointFromRay ~1907-1920; web/cameras.js calib object ~214-312; tools/twin_server.py _calib_post ~1112-1128, load_calib/save_calib ~89-101.',
      '',
      'Answer with evidence:',
      '1. End-to-end: image click -> 3D twin ground click -> pair -> 4+ -> Solve&Save -> reload -> spawn. Does the SAVED homography round-trip and place a car in the right lane? Any state lost between save and reload (intersection field, imgW/imgH, points)?',
      '2. Quad locking: addImagePoint locks the quad on the first point; what if the user clicks across quad boundaries or wants to recalibrate a quad? Is the session reset correctly on PiP open/close (cam-open, reset)?',
      '3. groundPointFromRay: flat mode intersects FLAT_Y plane; terrain mode raycasts terrain children. Is the plane constant sign correct (constant = -FLAT_Y)? Does the picked (x,z) match the ground the car renders on?',
      '4. save(): the overloaded sol.error (numeric reprojError vs error string) guard - any path where it saves a bad/missing H or crashes on res.reprojError.toFixed when reprojError is a string/undefined?',
      '5. clearCars/heartbeat: click-cars kept alive by a 2s re-pose vs server kinematic_ttl=5s - correct? Does closing the PiP or clearing leak cars or timers? Does reset() stop the heartbeat?',
      '6. Reprojection overlay (sceneToImage): orange cross markers drawn at the correct full-frame pixel including the quad offset (col*0.5+qu*0.5)?',
      '7. Security: sameOriginPath guard - actually enforced on every fetch (calib GET/POST, streams, detections, still)? Any bypass? Note: pollStreams and pollDetections fetch proxyBase paths directly - are those guarded?',
    ].join('\n'),
  },
  {
    key: 'server-world',
    prompt: COMMON + '\n\n' + [
      'DIMENSION: Server world / kinematic lifecycle / concurrency / resource safety.',
      'Read tools/twin_server.py: Agent (~257-330, set_pose ~374-383, integrate kinematic early-return ~385-396, snap_ground ~459-476, finalize, detect ~484-507, state), World (~550-617: spawn, despawn, tick TTL sweep, snapshot, run), detection relay globals DETECTIONS/ACTIVE/DET_LOCK + DET_TTL/ACTIVE_TTL (~80-101), _calib_post/save_calib, Handler do_GET/do_POST/do_DELETE, CameraProxy.',
      '',
      'Answer with evidence:',
      '1. Kinematic correctness: does snap_ground overwrite the y set by set_pose for ground-bound cars (self.y=gy)? Intended given the client overrides height? Does kinematic car heading/x/z survive a tick untouched?',
      '2. TTL sweep in tick(): despawn() mutates self.agents while iterating arr (a copied list) under self.lock - safe? kinematic_ttl=5s vs client heartbeat 2s and detector max-fps - premature despawn / flicker?',
      '3. detect(): kinematic cars run full O(agents^2) AABB collision every tick and can flash red on contact - wasteful and arguably wrong for camera cars. Flag?',
      '4. Thread-safety: DETECTIONS/ACTIVE under DET_LOCK - consistent? World.lock usage - any read outside the lock, unbounded growth (DETECTIONS never evicts old cameras; save_calib read-modify-write race under concurrent POSTs)?',
      '5. set_pose validation: NaN/inf x,z,heading from a flaky detector - any guard? Could a bad pose blow up collision or the client?',
      '6. Robustness: do_POST/do_GET error handling - can a malformed body 500 the server or leak a traceback? CameraProxy scrape parsing fragility (also in lex_cameras.py)?',
      '7. agent cap (max_agents=64) vs a busy intersection - could camera cars starve the cap? How does the detector handle spawn returning None (cap reached)?',
    ].join('\n'),
  },
  {
    key: 'viewer-perf',
    prompt: COMMON + '\n\n' + [
      'DIMENSION: Viewer performance and concrete optimizations (headline: 60fps over ~114k buildings).',
      'Read: web/cameras.js (heightmap() rebuild trigger + cost ~73-113, placeInstance/redrapeIfNeeded, buildMarkers instancing, frustumCulled=false on markers, pollStreams cadence, still() regex); web/netagents.js (poll 120ms, ingest spawn/dispose, tick lerp, label sprite churn); web/app.js (camCal.draw canvas resize-on-every-draw + 200ms detection poll redraw, pointermove cursor readout, the main animation loop - grep requestAnimationFrame/animate, per-frame allocations, tick calls).',
      'Grep for the render loop and per-frame work. Look for: per-frame allocations (new THREE.* in hot paths), redundant getBoundingClientRect + canvas reallocation, raycasts over large meshes per frame/move, instanced vs per-object meshes, frustumCulled disabled, too-aggressive polling, CanvasTexture churn, missing geometry/material disposal (leaks).',
      '',
      'Give concrete, safe optimizations (severity optimization unless an actual perf bug). Quantify impact where you can (per-frame vs per-event, O(n) over how many elements).',
    ].join('\n'),
  },
  {
    key: 'robustness-integration',
    prompt: COMMON + '\n\n' + [
      'DIMENSION: Cross-cutting robustness + END-TO-END integration correctness (bugs single-file review misses).',
      'Trace BOTH end-to-end paths and look for integration mismatches:',
      'A) DETECTION: YOLO box -> tire normalize within quad -> apply_h (python) -> scene (x,z) -> twin spawn/pose -> /api/world/state -> netagents render at groundOf(p) -> viewer vs the PiP box overlay.',
      'B) PLACEMENT: user image click (full-frame u,v) -> quadOf -> quad-local -> pair with 3D raycast ground (x,z) -> solveHomography -> save calibration/cameras.json -> reload via /api/cameras/calib -> imageToScene -> spawn.',
      'Read across: tools/camera_detect.py, web/homography.js, web/cameras.js, web/app.js, tools/twin_server.py, web/netagents.js, web/flat.js, tools/lex_cameras.py.',
      '',
      'Answer with evidence:',
      '1. Same quad-local normalization at calibration time (clicks) and detection time (YOLO boxes)? If the calibration video resolution differs from the detector frame resolution, does normalized-within-quad stay correct? Confirm or refute.',
      '2. Height consistency: detector/spawn passes y=null; server computes terrain y; client renders at FLAT_Y when flat (default). Do detector cars and click cars land at the SAME height as the roads? Any mode where they float/sink?',
      '3. Does the camera MARKER position (baked, snapped to intersection center within 75m) relate to where calibrated cars land? Could the user be confused the marker is at the junction but cars land per-homography?',
      '4. Do a click-spawned car and a detector car for the same camera/quad collide in identity, color, or source tagging? Could PiP Clear cars despawn detector cars or vice versa?',
      '5. Graceful degradation across the path: proxy offline, no calibration, no terrain yet, world not running - does any step throw into the render loop or leave a silent/confusing failure?',
      '6. README/spec claims vs actual code: drift that would mislead an operator running this in production?',
    ].join('\n'),
  },
]

phase('Review')
log('Reviewing ' + DIMENSIONS.length + ' dimensions, verifying each finding adversarially as it lands')

const results = await pipeline(
  DIMENSIONS,
  (d) => agent(d.prompt, { label: 'review:' + d.key, phase: 'Review', schema: FINDINGS_SCHEMA }),
  (review, d) => {
    if (!review || !review.findings || !review.findings.length) {
      return { dimension: d.key, summary: review && review.summary, verified: [] }
    }
    return parallel(review.findings.map((f) => () =>
      agent(
        COMMON + '\n\n' + [
          'ADVERSARIAL VERIFICATION. A prior reviewer reported this finding. Your job is to REFUTE it.',
          'Open the cited file at the cited lines, read the surrounding code, and decide whether the finding is REAL or a false positive.',
          'Default to skepticism: if the code actually handles the case, or the bug cannot occur given how callers use it, mark is_real=false and verdict=rejected.',
          'If it is real, confirm it, adjust severity if over/under-stated, and judge whether the proposed fix is correct and safe (note any refinement). Cite line evidence.',
          '',
          'FINDING UNDER REVIEW (dimension ' + d.key + '):',
          'title: ' + f.title,
          'severity: ' + f.severity + '   category: ' + f.category,
          'file: ' + f.file + '   lines: ' + f.lines,
          'description: ' + f.description,
          'evidence: ' + (f.evidence || '(none given)'),
          'recommendation: ' + f.recommendation,
          '',
          'Return ONLY the verdict object.',
        ].join('\n'),
        { label: 'verify:' + d.key + ':' + (f.title || '').slice(0, 26), phase: 'Verify', schema: VERDICT_SCHEMA, effort: 'high' },
      ).then((v) => ({ finding: f, verdict: v })).catch(() => ({ finding: f, verdict: null }))
    )).then((verified) => ({ dimension: d.key, summary: review.summary, verified }))
  },
)

const all = []
for (const r of results.filter(Boolean)) {
  for (const item of (r.verified || [])) {
    if (!item) continue
    const v = item.verdict
    const real = v ? (v.is_real !== false && v.verdict !== 'rejected') : true
    all.push({
      dimension: r.dimension,
      title: item.finding.title,
      severity: (v && v.severity_adjusted) || item.finding.severity,
      category: item.finding.category,
      file: item.finding.file,
      lines: item.finding.lines,
      description: item.finding.description,
      evidence: item.finding.evidence,
      recommendation: item.finding.recommendation,
      verdict: v ? v.verdict : 'unverified',
      is_real: v ? v.is_real : null,
      verify_reasoning: v ? v.reasoning : null,
      fix_correct: v ? v.fix_correct : null,
      fix_notes: v ? v.fix_notes : null,
      kept: real,
    })
  }
}

const order = { critical: 0, high: 1, medium: 2, low: 3, optimization: 4 }
all.sort((a, b) => (order[a.severity] === undefined ? 9 : order[a.severity]) - (order[b.severity] === undefined ? 9 : order[b.severity]))
const kept = all.filter((f) => f.kept)
const rejected = all.filter((f) => !f.kept)

log('Done: ' + all.length + ' findings total - ' + kept.length + ' confirmed/uncertain kept, ' + rejected.length + ' rejected')
return {
  summaries: results.filter(Boolean).map((r) => ({ dimension: r.dimension, summary: r.summary })),
  confirmed: kept,
  rejected: rejected.map((f) => ({ dimension: f.dimension, title: f.title, severity: f.severity, why_rejected: f.verify_reasoning })),
  counts: {
    total: all.length, kept: kept.length, rejected: rejected.length,
    bySeverity: kept.reduce((m, f) => { m[f.severity] = (m[f.severity] || 0) + 1; return m }, {}),
  },
}
