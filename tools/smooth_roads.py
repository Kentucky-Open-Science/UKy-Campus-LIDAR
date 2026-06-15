#!/usr/bin/env python
"""Smooth the campus road network and bake a machine-readable signal model.

This is an OFFLINE post-processor: it reads the EXISTING web/data/roads.json
(produced by tools/osm_roads.py) plus web/data/manifest.json and rewrites them —
no Overpass / network needed. It does two things the digital twin needs:

  1. CLEAN GEOMETRY (no jagged edges).  Each road centreline is corner-cut with
     3 iterations of Chaikin (which never overshoots — stays inside the polyline's
     convex hull, so a road never bows off the asphalt), resampled to a uniform
     4 m arc-length, then re-draped on a finer, Gaussian-smoothed terrain
     heightmap and its elevation profile is low-passed (1-D Gaussian along
     arc-length) and clamped to the ground so the ~0.3 m vertical "stair-steps"
     from the old coarse heightmap vanish without the ribbon sinking or floating.
     Shared junction vertices are pinned + welded (identical x,y,z on every leg)
     so smoothing never tears a join apart.

  2. REAL SIGNALISED INTERSECTIONS.  The raw 452 "intersections" are every shared
     OSM node — wildly over-counted. We cluster them, derive each junction's DEGREE
     by bearing-clustering the legs (a mid-street vertex has 2 collinear legs and is
     dropped; a real crossing has >=3 distinct legs), and classify each as a traffic
     SIGNAL / STOP / uncontrolled junction from its degree + the road classes that
     cross it. For every signal/stop leg we compute the stop-bar, stop-point, and
     crosswalk geometry, and for signals a traffic-engineering-correct fixed-time
     phase plan (opposing legs share a phase; conflicting movements are separated by
     yellow + all-red; pedestrians walk only parallel to a green movement).

Outputs (web/data/):
  roads.json     - smoothed pts; `intersections` now = the real merged centres.
                   The previous file is backed up to roads.raw.json once.
  signals.json   - the machine-readable contract the viewer + autonomous agents read
                   (see web/roads.js createSignalController and web/README.md).

Usage:  python tools/smooth_roads.py [--r-merge 18] [--iters 3] [--qc]
"""
import argparse, json, math, os, struct, array, hashlib
import numpy as np
from scipy import ndimage as ndi

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, '..', 'web', 'data'))

# ---- tunables (metres unless noted) ----
R_MERGE_M    = 18.0   # cluster raw OSM junction nodes closer than this into one junction
WELD_R_M     = 1.0    # road endpoints within this share a physical node -> snap + weld Y
CHAIKIN_ITERS = 3     # corner-cut passes (rounds hard OSM kinks; never overshoots)
RESAMPLE_M   = 4.0    # uniform arc-length spacing after smoothing
MIN_SEG_M    = 1.0    # collapse consecutive vertices closer than this (kills tiny tails)
MIN_SMOOTH_M = 12.0   # roads shorter than this are left raw (Chaikin would distort them)
LEG_SCAN_M   = 14.0   # gather road vertices within this of a centre to find its legs
LEG_SAMPLE_M = 14.0   # sample a leg's bearing this far out (robust to first-vertex noise)
REACH_CAP_M  = 45.0   # measure how far a leg's road runs, capped here
LEG_MIN_REACH_M = 16.0  # drop legs whose road dead-ends sooner (ramp / parking mouth)
BEARING_TOL  = math.radians(25)   # two legs within this angle are the same leg
MERGE_TOL    = math.radians(34)   # fuse legs within this as one approach (divided/curved)
NAME_MERGE_TOL = math.radians(60)  # fuse SAME-NAMED legs within this (divided carriageways)
OPP_TOL      = math.radians(30)   # two legs within this of 180deg form one through-street
SIG_SPACING_M = 45.0  # min spacing between SIGNAL intersections (closer -> demote to stop)
SIG_MATCH_M  = 45.0   # a junction is signalised if a real LFUCG signal is within this
LFUCG_SIGNALS = os.path.join(HERE, 'lfucg_traffic_signals.geojson')  # cached ground truth
HM_MPP_CM    = 200.0  # terrain heightmap cell (cm); finer than osm_roads' 400 -> less stair-step
HM_BLUR      = 1.0    # Gaussian pre-blur of the heightmap grid (cells)
Y_SIGMA      = 2.2    # 1-D Gaussian sigma (samples) along each road's elevation profile
Y_CLAMP_LO   = 0.20   # smoothed road y may sit at most this far BELOW re-draped ground (m)
Y_CLAMP_HI   = 0.60   # ... and at most this far above (m)
MAX_GRADE    = 0.22   # cap per-segment road grade (22%) — removes heightmap-fill spikes

# road class -> rank (higher = more major). links inherit their parent class.
RANK = {'primary': 4, 'primary_link': 4, 'secondary': 3, 'secondary_link': 3,
        'tertiary': 2, 'tertiary_link': 2, 'unclassified': 1, 'residential': 1,
        'living_street': 1, 'service': 0}
RANK_NAME = {4: 'primary', 3: 'secondary', 2: 'tertiary', 1: 'residential', 0: 'service'}


# ----------------------------------------------------------------- terrain ---
def load_mesh_positions(mp):
    with open(mp, 'rb') as f:
        vc = struct.unpack('<I', f.read(4))[0]; struct.unpack('<I', f.read(4))
        pos = array.array('f'); pos.frombytes(f.read(vc * 12))
    return np.array(pos).reshape(-1, 3)


def build_heightmap(manifest):
    """Rasterise terrain elevation (local_y cm) over the terrain extent and return
    a bilinear sampler elev(lx_cm, lz_cm) -> elevation cm. Replicated from
    tools/extract_roads.build_heightmap but numpy+scipy only (no skimage import),
    at a finer cell + a Gaussian pre-blur to kill the coarse-grid stair-steps."""
    tiles = manifest['terrain']['tiles']
    gx0 = gz0 = 1e18; gx1 = gz1 = -1e18
    mats = []
    for t in tiles:
        mp = os.path.join(DATA, t['mesh'])
        if not os.path.exists(mp):
            continue
        P = load_mesh_positions(mp)
        if len(P) < 3:
            continue
        mats.append(P)
        gx0, gx1 = min(gx0, P[:, 0].min()), max(gx1, P[:, 0].max())
        gz0, gz1 = min(gz0, P[:, 2].min()), max(gz1, P[:, 2].max())
    hw = int(math.ceil((gx1 - gx0) / HM_MPP_CM)) + 1
    hh = int(math.ceil((gz1 - gz0) / HM_MPP_CM)) + 1
    acc = np.zeros((hh, hw), np.float64); cnt = np.zeros((hh, hw), np.int64)
    for P in mats:
        cx = ((P[:, 0] - gx0) / HM_MPP_CM).astype(int)
        cy = ((P[:, 2] - gz0) / HM_MPP_CM).astype(int)
        ok = (cx >= 0) & (cx < hw) & (cy >= 0) & (cy < hh)
        np.add.at(acc, (cy[ok], cx[ok]), P[ok, 1])
        np.add.at(cnt, (cy[ok], cx[ok]), 1)
    have = cnt > 0
    grid = np.zeros((hh, hw), np.float64)
    grid[have] = acc[have] / cnt[have]
    if not have.all():                       # fill holes with nearest before blurring
        idx = ndi.distance_transform_edt(~have, return_distances=False, return_indices=True)
        grid = grid[tuple(idx)]
    if HM_BLUR > 0:
        grid = ndi.gaussian_filter(grid, HM_BLUR)

    def elev(lx, lz):
        fx = np.clip((np.asarray(lx) - gx0) / HM_MPP_CM, 0, hw - 1.001)
        fy = np.clip((np.asarray(lz) - gz0) / HM_MPP_CM, 0, hh - 1.001)
        x0 = np.floor(fx).astype(int); y0 = np.floor(fy).astype(int)
        dx = fx - x0; dy = fy - y0
        return ((grid[y0, x0] * (1 - dx) + grid[y0, x0 + 1] * dx) * (1 - dy) +
                (grid[y0 + 1, x0] * (1 - dx) + grid[y0 + 1, x0 + 1] * dx) * dy)
    return elev


def drape_scene(elev, sx, sz):
    """Elevation (scene metres) at a scene XZ point. sceneX=local_x/100, sceneZ=-local_z/100."""
    return float(elev(np.array([sx * 100.0]), np.array([-sz * 100.0]))[0]) / 100.0


def load_real_signals(manifest):
    """Project the cached LFUCG 'Traffic Signal' points (lon/lat, EPSG:4326) into scene
    metres using the same UTM-16N georeference the OSM roads use, so a junction can be
    matched to a real signalised intersection. Returns [(sceneX, sceneZ), ...] or None
    if the cache or pyproj is unavailable (then the caller falls back to a heuristic).
    Refresh the cache from LFUCG open data with tools/fetch_lfucg_signals.py."""
    if not os.path.exists(LFUCG_SIGNALS):
        return None
    try:
        from pyproj import Transformer
        gj = json.load(open(LFUCG_SIGNALS))
        oc, O = manifest['lidar']['original_coordinates'], manifest['origin_cm']
        A = (oc[0] + O[0]) / 100.0
        B = -(oc[1] + O[1]) / 100.0
        to_utm = Transformer.from_crs(4326, 32616, always_xy=True)
        out = []
        for f in gj.get('features', []):
            g = f.get('geometry')
            if not g or g.get('type') != 'Point':
                continue
            lon, lat = g['coordinates'][:2]
            e, n = to_utm.transform(lon, lat)
            out.append((e - A, B - n))
        return out or None
    except Exception as exc:
        print(f'      (real-signal gating unavailable: {exc})')
        return None


# ----------------------------------------------------------------- helpers ---
def angdiff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def cluster_points(P, R):
    """Single-link union-find clustering of Nx2 points within radius R (grid-bucketed)."""
    from collections import defaultdict
    n = len(P); par = list(range(n))

    def find(x):
        while par[x] != x:
            par[x] = par[par[x]]; x = par[x]
        return x
    g = defaultdict(list)
    for i, (x, z) in enumerate(P):
        g[(int(x // R), int(z // R))].append(i)
    R2 = R * R
    for (cx, cz), idxs in g.items():
        for dx in (-1, 0, 1):
            for dz in (-1, 0, 1):
                for j in g.get((cx + dx, cz + dz), []):
                    for i in idxs:
                        if i < j and (P[i][0] - P[j][0]) ** 2 + (P[i][1] - P[j][1]) ** 2 <= R2:
                            par[find(i)] = find(j)
    out = defaultdict(list)
    for i in range(n):
        out[find(i)].append(i)
    return list(out.values())


def chaikin(pts, iters):
    """Chaikin corner-cutting on a list of (x,z); the two endpoints are preserved."""
    for _ in range(iters):
        if len(pts) < 3:
            break
        out = [pts[0]]
        for i in range(len(pts) - 1):
            p, q = pts[i], pts[i + 1]
            out.append((0.75 * p[0] + 0.25 * q[0], 0.75 * p[1] + 0.25 * q[1]))
            out.append((0.25 * p[0] + 0.75 * q[0], 0.25 * p[1] + 0.75 * q[1]))
        out.append(pts[-1])
        pts = out
    return pts


def resample_xz(pts, step):
    """Uniform arc-length resample of an (x,z) polyline; endpoints preserved."""
    if len(pts) < 2:
        return list(pts)
    out = [pts[0]]; carry = step
    for i in range(len(pts) - 1):
        ax, az = pts[i]; bx, bz = pts[i + 1]
        seg = math.hypot(bx - ax, bz - az)
        if seg < 1e-9:
            continue
        d = carry
        while d < seg:
            t = d / seg
            out.append((ax + (bx - ax) * t, az + (bz - az) * t)); d += step
        carry = d - seg
    out.append(pts[-1])
    return out


def polylen(pts):
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))


def dedupe_xz(pts, min_seg):
    """Drop consecutive (x,z) points closer than min_seg, preserving both endpoints.
    Resampling leaves a tiny dangling final segment; collapsing it stops the later Y
    weld (which nudges the endpoint a few cm) from becoming a huge grade over a ~1 cm
    segment."""
    if len(pts) <= 2:
        return list(pts)
    out = [pts[0]]
    for p in pts[1:-1]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= min_seg:
            out.append(p)
    last = pts[-1]
    if len(out) >= 2 and math.hypot(last[0] - out[-1][0], last[1] - out[-1][1]) < min_seg:
        out[-1] = last           # snap the endpoint onto the too-close penultimate slot
    else:
        out.append(last)
    return out


def slope_limit(ys, seglen, max_grade, iters=4):
    """Cap the per-segment grade of an elevation profile so a spike bled in by the
    sparse-heightmap nearest-fill can't produce a physically impossible road grade.
    A single forward pass alone biases the profile toward its start, so we alternate
    forward/backward passes a few times until it converges. Real grades below
    max_grade are untouched."""
    n = len(ys)
    for _ in range(iters):
        for i in range(1, n):
            m = max_grade * seglen[i - 1]
            ys[i] = min(max(ys[i], ys[i - 1] - m), ys[i - 1] + m)
        for i in range(n - 2, -1, -1):
            m = max_grade * seglen[i]
            ys[i] = min(max(ys[i], ys[i + 1] - m), ys[i + 1] + m)
    return ys


# ----------------------------------------------------------------- legs ---
def legs_for(center, RV, roads):
    """Return the distinct legs of a junction at `center` (scene XZ).

    Each road contributing a vertex near the centre is sampled in each direction
    it extends; the resulting outward bearings are clustered (BEARING_TOL) so a
    through-road yields two opposing legs and a mid-street vertex yields exactly
    two collinear legs (degree 2 -> not a junction). Returns a list of legs:
        {bearing, rank, class, width, roadIndex, name}
    """
    raw = []   # dicts: bearing, rank, class, width, roadIndex, name, reach
    for ri, V in enumerate(RV):
        d2 = (V[:, 0] - center[0]) ** 2 + (V[:, 1] - center[1]) ** 2
        j = int(d2.argmin())
        if d2[j] > LEG_SCAN_M * LEG_SCAN_M:
            continue
        r = roads[ri]
        for direction in (-1, 1):
            k = j; dist = 0.0; prev = V[j]; bsamp = None
            while 0 <= k + direction < len(V):
                k += direction
                dist += math.hypot(V[k][0] - prev[0], V[k][1] - prev[1]); prev = V[k]
                if bsamp is None and dist >= LEG_SAMPLE_M:     # bearing sampled ~here
                    bsamp = math.atan2(V[k][1] - center[1], V[k][0] - center[0])
                if dist >= REACH_CAP_M:                        # `reach` = how far road runs
                    break
            if bsamp is None and dist >= 6.0:                  # short road: use its far end
                bsamp = math.atan2(prev[1] - center[1], prev[0] - center[0])
            if bsamp is not None and dist >= 6.0:
                raw.append({'bearing': bsamp, 'rank': RANK.get(r.get('class'), 1),
                            'class': r.get('class'), 'width': float(r.get('width', 7)),
                            'roadIndex': ri, 'name': r.get('name'), 'reach': dist})
    # 1) cluster near-identical bearings (major legs first -> major road represents)
    used = [False] * len(raw); legs = []
    for i in sorted(range(len(raw)), key=lambda i: -raw[i]['rank']):
        if used[i]:
            continue
        grp = [i]; used[i] = True
        for k in range(len(raw)):
            if not used[k] and angdiff(raw[i]['bearing'], raw[k]['bearing']) < BEARING_TOL:
                used[k] = True; grp.append(k)
        bs = [raw[x]['bearing'] for x in grp]
        best = max(grp, key=lambda x: (raw[x]['rank'], raw[x]['width']))
        legs.append({'bearing': math.atan2(sum(math.sin(b) for b in bs), sum(math.cos(b) for b in bs)),
                     'rank': raw[best]['rank'], 'class': raw[best]['class'],
                     'width': raw[best]['width'], 'roadIndex': raw[best]['roadIndex'],
                     'name': raw[best]['name'], 'reach': max(raw[x]['reach'] for x in grp)})
    # 2) merge legs that are really ONE approach: a divided carriageway or a split/curved
    #    way shows up as 2+ legs at similar bearings (and a divided road's two carriageways
    #    fan out, so same-named legs merge at a WIDER angle). Greedily fuse the closest
    #    mergeable pair, keeping the higher-rank/wider road and widening for the median.
    while len(legs) > 1:
        bestp = None; bestd = 1e9
        for i in range(len(legs)):
            for k in range(i + 1, len(legs)):
                d = angdiff(legs[i]['bearing'], legs[k]['bearing'])
                same = legs[i]['name'] and legs[i]['name'] == legs[k]['name']
                tol = NAME_MERGE_TOL if same else MERGE_TOL
                if d < tol and d < bestd:
                    bestd = d; bestp = (i, k)
        if bestp is None:
            break
        i, k = bestp; a, b = legs[i], legs[k]
        keep = dict(a if (a['rank'], a['width']) >= (b['rank'], b['width']) else b)
        keep['bearing'] = math.atan2(math.sin(a['bearing']) + math.sin(b['bearing']),
                                     math.cos(a['bearing']) + math.cos(b['bearing']))
        keep['width'] = max(a['width'], b['width'])
        keep['reach'] = max(a['reach'], b['reach'])
        legs = [legs[x] for x in range(len(legs)) if x not in (i, k)] + [keep]
    # 3) drop stub legs (ramps / parking-lot mouths that dead-end quickly) when the
    #    junction still has >=3 real approaches without them.
    real = [l for l in legs if l['reach'] >= LEG_MIN_REACH_M]
    if len(real) >= 3:
        legs = real
    return legs


def count_streets(legs):
    """Pair opposing legs (~180deg) into through-streets; return the per-street max rank.
    Two crossing arterials -> two streets; an arterial with a driveway tee -> the
    arterial (one street) + the driveway (one street)."""
    used = [False] * len(legs); ranks = []
    for i in range(len(legs)):
        if used[i]:
            continue
        used[i] = True; rank = legs[i]['rank']
        for k in range(i + 1, len(legs)):
            if not used[k] and abs(angdiff(legs[i]['bearing'], legs[k]['bearing']) - math.pi) < OPP_TOL:
                used[k] = True; rank = max(rank, legs[k]['rank']); break
        ranks.append(rank)
    return ranks


# ----------------------------------------------------------------- main ---
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--r-merge', type=float, default=R_MERGE_M)
    ap.add_argument('--iters', type=int, default=CHAIKIN_ITERS)
    ap.add_argument('--qc', action='store_true', help='print extra QC diagnostics')
    args = ap.parse_args()

    roads_path = os.path.join(DATA, 'roads.json')
    raw_backup = os.path.join(DATA, 'roads.raw.json')
    data = json.load(open(roads_path))
    manifest = json.load(open(os.path.join(DATA, 'manifest.json')))
    roads = data['roads']
    raw_inter = data.get('intersections', [])
    print(f'[1/6] loaded {len(roads)} roads, {len(raw_inter)} raw intersection nodes')

    print('[2/6] building terrain heightmap (finer + blurred) ...')
    elev = build_heightmap(manifest)

    # ---- cluster raw junction nodes into real junction centres (XZ) ----
    print('[3/6] clustering junction nodes + deriving legs ...')
    IP = np.array([[p[0], p[2]] for p in raw_inter]) if raw_inter else np.zeros((0, 2))
    clusters = cluster_points(IP, args.r_merge) if len(IP) else []
    centers = [tuple(IP[idx].mean(0)) for idx in clusters]   # scene (x, z)

    # ---- smooth every road's XZ, pinning vertices that sit on a junction centre ----
    print(f'[4/6] smoothing roads (Chaikin x{args.iters} + {RESAMPLE_M:.0f} m resample + Y low-pass) ...')
    # spatial index of centres for snap test
    # ---- weld groups: road ENDPOINTS that share a physical node (tight radius) ----
    # OSM-shared endpoints already coincide exactly; clustering them at WELD_R_M lets us
    # snap to one position + share one Y so smoothing never tears a join. We weld at the
    # TRUE node (average of coincident endpoints), NOT the 18 m junction centroid — the
    # latter is only for fixture placement and yanking endpoints to it would kink roads.
    # (A through-road merely passes NEAR a node without an endpoint there; the
    # intersection pad covers that small gap, so only endpoints need welding.)
    ends = []                               # (road_index, end_kind: 0=first|1=last, x, z)
    for ri, r in enumerate(roads):
        if len(r['pts']) >= 2:
            ends.append((ri, 0, r['pts'][0][0], r['pts'][0][2]))
            ends.append((ri, 1, r['pts'][-1][0], r['pts'][-1][2]))
    EP = np.array([[e[2], e[3]] for e in ends]) if ends else np.zeros((0, 2))
    weld_pos = {}                           # (road, end_kind) -> shared (x, z)
    weld_groups = []                        # list of [(road, end_kind), ...] with >=2 members
    for grp in (cluster_points(EP, WELD_R_M) if len(EP) else []):
        if len(grp) < 2:
            continue
        mx = float(np.mean([EP[i][0] for i in grp])); mz = float(np.mean([EP[i][1] for i in grp]))
        members = [(ends[i][0], ends[i][1]) for i in grp]
        weld_groups.append(members)
        for m in members:
            weld_pos[m] = (mx, mz)

    new_pts_xz = []                         # per road: list of (x,z)
    for ri, r in enumerate(roads):
        pts = [(p[0], p[2]) for p in r['pts']]
        if len(pts) < 2:
            new_pts_xz.append(pts); continue
        if (ri, 0) in weld_pos:
            pts[0] = weld_pos[(ri, 0)]      # snap welded endpoints to the shared node (tiny move)
        if (ri, 1) in weld_pos:
            pts[-1] = weld_pos[(ri, 1)]
        # leave very short roads raw (Chaikin would distort them)
        if polylen(pts) < MIN_SMOOTH_M or len(pts) < 3:
            new = list(pts)
        else:
            new = resample_xz(chaikin(pts, args.iters), RESAMPLE_M)   # endpoints preserved
        new_pts_xz.append(dedupe_xz(new, MIN_SEG_M))

    # ---- shared Y per junction centre (welds the legs at identical elevation) ----
    center_y = []
    for ci, (cx, cz) in enumerate(centers):
        center_y.append(drape_scene(elev, cx, cz))

    # ---- re-drape + low-pass Y for every road, then weld anchor Ys ----
    def measure_roughness(rs):
        rough = []
        for r in rs:
            ys = [p[1] for p in r['pts']]
            for i in range(1, len(ys) - 1):
                rough.append(abs(ys[i - 1] - 2 * ys[i] + ys[i + 1]))
        rough.sort()
        if not rough:
            return 0.0, 0.0
        return rough[len(rough) // 2], rough[int(len(rough) * 0.95)]

    before_med, before_p95 = measure_roughness(roads)
    new_roads = []
    for ri, r in enumerate(roads):
        xz = new_pts_xz[ri]
        if len(xz) < 2:
            new_roads.append(r); continue
        sx = np.array([p[0] for p in xz]); sz = np.array([p[1] for p in xz])
        ground = elev(sx * 100.0, -sz * 100.0) / 100.0          # re-draped ground (m)
        if len(ground) >= 5:
            ys = ndi.gaussian_filter1d(ground, Y_SIGMA, mode='nearest')
        else:
            ys = ground.copy()
        ys = np.clip(ys, ground - Y_CLAMP_LO, ground + Y_CLAMP_HI)
        if len(ys) >= 2:                       # kill heightmap-fill spikes (impossible grades)
            seglen = np.hypot(np.diff(sx), np.diff(sz))
            ys = slope_limit(ys.astype(float), seglen, MAX_GRADE)
        new_roads.append({**r, 'pts': [[round(float(x), 2), round(float(y), 2), round(float(z), 2)]
                                       for x, y, z in zip(sx, ys, sz)]})
    # weld each shared node to ONE elevation so legs meet with no vertical lip. Use the
    # MEAN of the incident roads' already-smoothed endpoint Ys (a tiny adjustment), NOT a
    # fresh raw drape — re-draping here would re-inject the heightmap-fill spikes that
    # slope_limit just removed.
    for members in weld_groups:
        idxs = []
        for (ri, ek) in members:
            pts_ri = new_roads[ri]['pts']
            idxs.append((pts_ri, 0 if ek == 0 else len(pts_ri) - 1))
        vals = [p[i][1] for (p, i) in idxs if 0 <= i < len(p)]
        if not vals:
            continue
        wy = round(sum(vals) / len(vals), 2)
        for (p, i) in idxs:
            if 0 <= i < len(p):
                p[i][1] = wy
    after_med, after_p95 = measure_roughness(new_roads)

    # ---- classify junctions + build the signal model ----
    print('[5/6] classifying junctions + baking signal model ...')
    RVs = [np.array([[p[0], p[2]] for p in r['pts']]) for r in new_roads]
    junctions = []                          # (center_xz, center_y, degree, legs, ctrl_rank)
    for ci, ctr in enumerate(centers):
        legs = legs_for(ctr, RVs, new_roads)
        if len(legs) < 3:
            continue                        # mid-street vertex / corner / dead-end -> not a junction
        junctions.append({'cidx': ci, 'center': ctr, 'cy': center_y[ci],
                          'degree': len(legs), 'legs': legs,
                          'ctrl': max(l['rank'] for l in legs)})

    # decide control type. If the real LFUCG traffic-signal layer is available, it is
    # AUTHORITATIVE for which junctions are signalised (ground truth) — this removes the
    # false signals my heuristic put on ramp clusters / interchanges. Without it we fall
    # back to the degree+class heuristic.
    real_sig = load_real_signals(manifest)
    if real_sig is not None:
        print(f'      using {len(real_sig)} real LFUCG traffic signals as ground truth')
    for j in junctions:
        st = count_streets(j['legs'])
        n_major = sum(1 for rnk in st if rnk >= 2)     # distinct tertiary+ streets
        if real_sig is not None:
            cx0, cz0 = j['center']
            near = any((cx0 - rx) ** 2 + (cz0 - rz) ** 2 < SIG_MATCH_M ** 2 for rx, rz in real_sig)
            is_signal = j['degree'] >= 3 and near
        else:
            is_signal = j['ctrl'] >= 3 and n_major >= 2
        if is_signal:
            j['control'] = 'signal'
        elif j['ctrl'] >= 2 or j['degree'] >= 4:
            j['control'] = 'stop'
        else:
            j['control'] = 'uncontrolled'

    # enforce minimum spacing between signals (demote the lesser to stop)
    sigs = sorted([j for j in junctions if j['control'] == 'signal'],
                  key=lambda j: (-j['degree'], -j['ctrl']))
    kept = []
    for j in sigs:
        if all((j['center'][0] - k['center'][0]) ** 2 + (j['center'][1] - k['center'][1]) ** 2
               > SIG_SPACING_M ** 2 for k in kept):
            kept.append(j)
        else:
            j['control'] = 'stop'

    intersections_out = build_signal_model(junctions, elev)

    # ---- write smoothed roads.json (real centres) + signals.json ----
    print('[6/6] writing roads.json + signals.json ...')
    if not os.path.exists(raw_backup):
        json.dump(data, open(raw_backup, 'w'))
        print(f'      backed up original -> {raw_backup}')
    data['roads'] = new_roads
    data['intersections'] = [[round(j['center'][0], 2), round(j['cy'], 2),
                              round(j['center'][1], 2)] for j in junctions]
    data['note'] = (data.get('note', '') +
                    ' | smoothed + signalised by tools/smooth_roads.py')
    json.dump(data, open(roads_path, 'w'))

    signals = {
        'version': 1,
        'note': 'machine-readable intersection/signal model for autonomous agents; '
                'scene metres, three.js Y-up, [x, y, z]. See web/roads.js createSignalController.',
        'tickHz': 10,
        'intersections': intersections_out,
    }
    json.dump(signals, open(os.path.join(DATA, 'signals.json'), 'w'))

    n_sig = sum(1 for j in junctions if j['control'] == 'signal')
    n_stop = sum(1 for j in junctions if j['control'] == 'stop')
    n_unc = sum(1 for j in junctions if j['control'] == 'uncontrolled')
    print(f'      roads: {len(new_roads)}   junctions: {len(junctions)} '
          f'(from {len(raw_inter)} raw nodes)')
    print(f'      SIGNALS={n_sig}  STOP={n_stop}  UNCONTROLLED={n_unc}')
    print(f'      vertical roughness median|d2y|: {before_med:.3f} -> {after_med:.3f} m '
          f'| p95: {before_p95:.3f} -> {after_p95:.3f} m')
    print('done.')


def build_signal_model(junctions, elev):
    """Turn classified junctions into the signals.json `intersections` list, with
    per-leg stop-bar / stop-point / crosswalk geometry and (for signals) a fixed-time
    phase plan. All geometry is precomputed in scene metres so the viewer is a thin
    renderer and an agent can consume the coordinates directly. Every fixture Y is
    draped at its OWN ground (not the junction centre) so nothing floats/sinks on a
    sloped approach."""
    out = []
    for k, j in enumerate(junctions):
        legs = j['legs']
        cx, cz = j['center']; cy = j['cy']
        iid = 'int_%04d' % k
        half_max = max(l['width'] for l in legs) / 2.0
        foot_r = half_max + 2.0                       # intersection box radius
        # Movement groups: greedily pair opposing legs (~180deg) so the legs WITHIN a
        # group never conflict; each group gets its own green phase. Major legs are
        # paired first. This is what guarantees no conflicting greens (a plain
        # "axis vs cross" split breaks on skewed / 5+-leg junctions).
        labels = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H']
        used_l = [False] * len(legs); gi = 0
        order = sorted(range(len(legs)), key=lambda i: (-legs[i]['rank'], -legs[i]['width']))
        for i in order:
            if used_l[i]:
                continue
            used_l[i] = True
            lab = labels[gi] if gi < len(labels) else labels[-1]
            gi += 1
            legs[i]['_group'] = lab
            best = -1; bestd = OPP_TOL
            for kk in order:                           # nearest still-free opposing leg
                if used_l[kk]:
                    continue
                dd = abs(angdiff(legs[i]['bearing'], legs[kk]['bearing']) - math.pi)
                if dd < bestd:
                    bestd = dd; best = kk
            if best >= 0:
                used_l[best] = True; legs[best]['_group'] = lab
        group_labels = labels[:min(gi, len(labels))]
        group_rank = {}; group_width = {}
        for l in legs:
            group_rank[l['_group']] = max(group_rank.get(l['_group'], 0), l['rank'])
            group_width[l['_group']] = max(group_width.get(l['_group'], 0), l['width'])

        # At big (5+ leg) junctions a crosswalk on every leg reads as an unrealistic
        # ring; real complex junctions mark only the principal approaches. Keep crosswalks
        # (and pedestrian heads) on the 4 highest-rank/widest legs there.
        if len(legs) >= 5:
            xw_legs = set(sorted(range(len(legs)),
                                 key=lambda i: (-legs[i]['rank'], -legs[i]['width']))[:4])
        else:
            xw_legs = set(range(len(legs)))

        legs_out = []
        crosswalks = []
        for li, l in enumerate(legs):
            b = l['bearing']                          # outward (away from centre)
            ox, oz = math.cos(b), math.sin(b)         # outward unit (scene XZ)
            rx, rz = -oz, ox                           # right-hand unit
            hw = l['width'] / 2.0
            # Place the crosswalk just beyond the CROSSING road(s) — the box edge along
            # this approach is set by the perpendicular streets' width, not this road's —
            # so it lands on the approach instead of bunching toward the centre.
            cross = [o['width'] / 2.0 for oi, o in enumerate(legs) if oi != li
                     and angdiff(o['bearing'], b) > math.radians(50)
                     and abs(angdiff(o['bearing'], b) - math.pi) > math.radians(50)]
            edge = (max(cross) if cross else hw)
            xwalk_d = edge + 1.8
            stop_d = edge + 4.0                       # stop bar sits BEHIND the crosswalk
            # stop bar endpoints (across the road) + stop point (inbound right lane centre).
            # Y is draped at the bar's OWN ground (~14 m out from centre), not the centre,
            # so fixtures hug a sloped approach instead of floating/sinking.
            sb_cx, sb_cz = cx + ox * stop_d, cz + oz * stop_d
            sb_y = round(drape_scene(elev, sb_cx, sb_cz), 2)
            stop_line = [[round(sb_cx + rx * hw, 2), round(sb_cz + rz * hw, 2)],
                         [round(sb_cx - rx * hw, 2), round(sb_cz - rz * hw, 2)]]
            stop_pt = [round(sb_cx + rx * hw * 0.5, 2), sb_y, round(sb_cz + rz * hw * 0.5, 2)]
            # crosswalk polygon (4 corners, depth 2.2 m along travel, full road width)
            depth = 1.1
            xw_cx, xw_cz = cx + ox * xwalk_d, cz + oz * xwalk_d
            xw_y = round(drape_scene(elev, xw_cx, xw_cz), 2)
            poly = [[round(xw_cx + ox * depth + rx * hw, 2), round(xw_cz + oz * depth + rz * hw, 2)],
                    [round(xw_cx + ox * depth - rx * hw, 2), round(xw_cz + oz * depth - rz * hw, 2)],
                    [round(xw_cx - ox * depth - rx * hw, 2), round(xw_cz - oz * depth - rz * hw, 2)],
                    [round(xw_cx - ox * depth + rx * hw, 2), round(xw_cz - oz * depth + rz * hw, 2)]]
            controlled = j['control'] in ('signal', 'stop')
            has_xw = controlled and (li in xw_legs)
            grp = l.get('_group') if j['control'] == 'signal' else None
            leg = {
                'idx': li,
                'bearingDeg': round(math.degrees(b), 1),
                'roadIndex': l['roadIndex'],
                'roadName': l['name'],
                'class': l['class'],
                'width': round(l['width'], 1),
                'stopLine': stop_line,
                'stopPoint': stop_pt,
                'signalGroup': grp,
                'stopSign': (j['control'] == 'stop'),
                'pedSignal': bool(j['control'] == 'signal' and has_xw),  # ped head only where a crosswalk exists
                'vehSignalKey': '%s:%d' % (iid, li),
                'pedSignalKey': '%s:p%d' % (iid, li),
            }
            legs_out.append(leg)
            if has_xw:
                # pedGroup = the group whose RED enables this crossing = the leg's own
                # group (a crosswalk across leg L's road is safe while L is stopped).
                crosswalks.append({'legIdx': li, 'polygon': poly, 'y': xw_y, 'pedGroup': grp})

        inter = {
            'id': iid,
            'center': [round(cx, 2), round(cy, 2), round(cz, 2)],
            'degree': j['degree'],
            'control': j['control'],
            'controllingClass': RANK_NAME.get(j['ctrl'], 'residential'),
            'footprintRadius': round(foot_r, 1),
            'legs': legs_out,
            'crosswalks': crosswalks,
        }
        if j['control'] == 'signal':
            inter['phasePlan'] = make_phase_plan(iid, group_labels, group_rank, group_width)
        out.append(inter)
    return out


def make_phase_plan(iid, groups, group_rank, group_width):
    """Fixed-time ring with ONE green phase per movement group, so exactly one group is
    ever green and conflicting greens are impossible by construction. Each green splits
    into a WALK interval then an in-green flashing-don't-walk (FDW) clearance sized to
    the crossed leg's width, followed by that group's yellow + an all-red clearance — so
    a pedestrian who steps off at the end of WALK has enough time to finish crossing
    before the conflicting road turns green. A crosswalk shows WALK/FLASH only while its
    own group is red (during the previous group's green), never while it has green/yellow."""
    GREEN = {4: 28, 3: 24, 2: 18, 1: 16, 0: 14}
    yellow, all_red, PED_SPEED = 4, 2, 1.2     # m/s walking speed
    k = len(groups)
    phases = []
    for gi, g in enumerate(groups):
        nxt = groups[(gi + 1) % k] if k > 1 else None   # this green = walk window for `nxt`
        grn = GREEN.get(group_rank.get(g, 1), 18)
        # FDW clearance must cover the crossing minus the yellow+all-red that follow it.
        cross_w = group_width.get(nxt, 11.0) if nxt else 0.0
        need = int(math.ceil(cross_w / PED_SPEED)) - (yellow + all_red)
        fdw = max(3, min(grn - 7, need)) if nxt else 0
        walk = grn - fdw
        allred = {gg: 'red' for gg in groups}
        veh_g = {gg: ('green' if gg == g else 'red') for gg in groups}
        phases.append({'groupStates': dict(veh_g),
                       'pedStates': {gg: ('walk' if gg == nxt else 'dont') for gg in groups}, 'durSec': walk})
        if fdw > 0:
            phases.append({'groupStates': dict(veh_g),   # still green for vehicles, FDW for peds
                           'pedStates': {gg: ('flash' if gg == nxt else 'dont') for gg in groups}, 'durSec': fdw})
        phases.append({'groupStates': {gg: ('yellow' if gg == g else 'red') for gg in groups},
                       'pedStates': {gg: 'dont' for gg in groups}, 'durSec': yellow})
        phases.append({'groupStates': dict(allred),
                       'pedStates': {gg: 'dont' for gg in groups}, 'durSec': all_red})
    cycle = sum(p['durSec'] for p in phases)
    off = int(hashlib.md5(iid.encode()).hexdigest(), 16) % max(cycle, 1)
    return {'cycleSec': cycle, 'offsetSec': off, 'groups': groups, 'phases': phases}


if __name__ == '__main__':
    main()
