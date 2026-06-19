// Planar homography (image <-> scene ground) for the camera-detected-cars feature.
//
// A fixed traffic camera over a ~planar road maps image pixels to ground coordinates by
// a 3x3 projective homography. We solve it from >=4 point correspondences clicked during
// calibration, then apply it to a detected car's tire point to get a scene (x,z).
//
// Dependency-free (constitution: no external runtime deps): inhomogeneous DLT with
// Hartley normalization (essential here — scene coords are hundreds-to-thousands of
// metres, which makes the un-normalized normal equations ill-conditioned), solved via
// Gaussian elimination with partial pivoting. Exact for 4 points, least-squares for >4.
//
//   solveHomography(srcPts, dstPts) -> H (row-major 9-array) mapping src -> dst
//   applyHomography(H, [u,v]) -> [x,y]
//   invertHomography(H) -> H^-1   (or null if singular)

// ---- 3x3 helpers (row-major flat arrays of 9) ----
export function matMul3(A, B) {
  const C = new Array(9).fill(0);
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) {
    let s = 0; for (let k = 0; k < 3; k++) s += A[r * 3 + k] * B[k * 3 + c];
    C[r * 3 + c] = s;
  }
  return C;
}

export function invertHomography(H) {
  const [a, b, c, d, e, f, g, h, i] = H;
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const det = a * A + b * B + c * C;
  if (!isFinite(det) || Math.abs(det) < 1e-18) return null;
  const inv = 1 / det;
  return [
    A * inv, (c * h - b * i) * inv, (b * f - c * e) * inv,
    B * inv, (a * i - c * g) * inv, (c * d - a * f) * inv,
    C * inv, (b * g - a * h) * inv, (a * e - b * d) * inv,
  ];
}

export function applyHomography(H, p) {
  const u = p[0], v = p[1];
  const x = H[0] * u + H[1] * v + H[2];
  const y = H[3] * u + H[4] * v + H[5];
  const w = H[6] * u + H[7] * v + H[8];
  if (!isFinite(w) || Math.abs(w) < 1e-18) return [NaN, NaN];
  return [x / w, y / w];
}

// Hartley normalization: translate the set to centroid 0 and scale so the mean distance
// to the origin is sqrt(2). Returns { T (3x3), pts (normalized) }; T maps original->norm.
function normalize(pts) {
  let cx = 0, cy = 0;
  for (const p of pts) { cx += p[0]; cy += p[1]; }
  cx /= pts.length; cy /= pts.length;
  let dsum = 0;
  for (const p of pts) dsum += Math.hypot(p[0] - cx, p[1] - cy);
  const meanD = dsum / pts.length || 1;
  const s = Math.SQRT2 / meanD;
  const T = [s, 0, -s * cx, 0, s, -s * cy, 0, 0, 1];
  const out = pts.map((p) => [s * (p[0] - cx), s * (p[1] - cy)]);
  return { T, pts: out };
}

// Solve the n x n linear system M x = b (Gaussian elimination, partial pivot). Mutates copies.
function solveLinear(M, b) {
  const n = b.length;
  const A = M.map((row, r) => row.concat([b[r]]));   // augmented
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(A[r][col]) > Math.abs(A[piv][col])) piv = r;
    if (Math.abs(A[piv][col]) < 1e-15) return null;   // singular / degenerate
    [A[col], A[piv]] = [A[piv], A[col]];
    const d = A[col][col];
    for (let c = col; c <= n; c++) A[col][c] /= d;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const factor = A[r][col];
      if (factor === 0) continue;
      for (let c = col; c <= n; c++) A[r][c] -= factor * A[col][c];
    }
  }
  return A.map((row) => row[n]);
}

// Solve homography H (src -> dst) from >=4 correspondences. Returns row-major 9-array,
// or null if degenerate. H is normalized so H[8] === 1.
export function solveHomography(srcPts, dstPts) {
  if (!srcPts || !dstPts || srcPts.length !== dstPts.length || srcPts.length < 4) return null;
  const ns = normalize(srcPts), nd = normalize(dstPts);
  const S = ns.pts, D = nd.pts;
  // inhomogeneous DLT: 2 rows per point, 8 unknowns (h11..h32; h33 = 1)
  const rows = [], rhs = [];
  for (let k = 0; k < S.length; k++) {
    const [x, y] = S[k], [X, Y] = D[k];
    rows.push([x, y, 1, 0, 0, 0, -x * X, -y * X]); rhs.push(X);
    rows.push([0, 0, 0, x, y, 1, -x * Y, -y * Y]); rhs.push(Y);
  }
  // least-squares via normal equations: (A^T A) h = A^T b  (8x8)
  const m = 8;
  const M = Array.from({ length: m }, () => new Array(m).fill(0));
  const bb = new Array(m).fill(0);
  for (let r = 0; r < rows.length; r++) {
    const row = rows[r], yv = rhs[r];
    for (let i = 0; i < m; i++) {
      bb[i] += row[i] * yv;
      for (let j = 0; j < m; j++) M[i][j] += row[i] * row[j];
    }
  }
  const h = solveLinear(M, bb);
  if (!h) return null;
  const Hn = [h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1];
  // denormalize: H = inv(T_dst) * Hn * T_src
  const Tdi = invertHomography(nd.T);
  if (!Tdi) return null;
  let H = matMul3(Tdi, matMul3(Hn, ns.T));
  if (!isFinite(H[8]) || Math.abs(H[8]) < 1e-18) return null;
  H = H.map((v) => v / H[8]);   // normalize so H[8] = 1
  return H;
}

// Largest residual (in dst units) over the correspondences — a calibration quality read.
export function reprojError(H, srcPts, dstPts) {
  let max = 0;
  for (let k = 0; k < srcPts.length; k++) {
    const [x, y] = applyHomography(H, srcPts[k]);
    max = Math.max(max, Math.hypot(x - dstPts[k][0], y - dstPts[k][1]));
  }
  return max;
}
