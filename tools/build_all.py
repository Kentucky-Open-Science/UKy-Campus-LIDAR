"""Orchestrate full pipeline: extract everything -> generate web/data/manifest.json.

Run:  python tools/build_all.py [--skip-textures] [--skip-meshes] [--skip-lidar] [--skip-buildings]

After extraction is complete, merges the per-domain manifests and writes the
unified manifest.json that the web viewer (web/app.js) expects.

The data contract expected by the viewer is documented in web/README.md.
"""
import argparse
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from extract_texture import main as extract_textures_main
from extract_mesh import main as extract_meshes_main

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Args:
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)


def run_extraction(skip_textures, skip_meshes, skip_lidar, skip_buildings,
                   skip_pack=False, with_city=False, with_transit=False,
                   skip_scene=False, citywide=False):
    """Run extraction steps (these may be no-ops if data already exists)."""
    print('=' * 60)
    print(' Lexington Digital Twin — full data extraction pipeline')
    print('=' * 60)

    if not skip_textures:
        print('\n--- Step 1: Textures (uasset -> JPEG) ---')
        extract_textures_main(
            args=Args(all=True, swap=True, probe=False, channel_test=False,
                      manifest=False, path=None, outdir=os.path.join(ROOT, 'web', 'data', 'textures')))
        extract_textures_main(
            args=Args(all=False, path=None, swap=False, probe=False,
                      channel_test=False, manifest=True, outdir=None))

    if not skip_meshes:
        print('\n--- Step 2: Meshes (uasset -> .bin) ---')
        extract_meshes_main(
            args=Args(paths=[], all=True,
                      manifest=os.path.join(ROOT, 'extracted', 'manifest-meshes.json')))

    if not skip_scene:
        # Scene transforms (origin_node -> origin_cm, per-tile placement) come from the
        # GRID blueprint. merge_manifests() needs extracted/manifest-scene.json, and the
        # buildings extractor needs the origin_cm it carries — so this must run before both.
        # Run as a module so tools.* imports resolve and tools/inspect.py doesn't shadow
        # the stdlib inspect.
        print('\n--- Step 3: Scene (blueprint -> manifest-scene.json) ---')
        subprocess.run([sys.executable, '-m', 'tools.extract_scene'], cwd=ROOT, check=True)

    if not skip_lidar:
        print('\n--- Step 4: LiDAR (uasset -> chunked .bin) ---')
        subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'extract_lidar.py')],
                       cwd=ROOT, check=True)

    # Merge BEFORE buildings: web/data/manifest.json carries the scene<->UTM georef that
    # tools/extract_buildings_hybrid reads. (This merge previously ran AFTER buildings, so a
    # from-scratch build had no manifest for the buildings extractor to read — it only ever
    # worked when a manifest from an earlier run happened to be lying around.)
    print('\n--- Step 5: Merge manifests -> web/data/manifest.json (georef + terrain) ---')
    merge_manifests()

    if citywide:
        # ===================== FULL CITY (all of Lexington) =====================
        # Open-data fill over the whole Lextran service area: OpenStreetMap + the statewide
        # KyFromAbove LiDAR. This is a BIG job — an ~8 GB LiDAR download and ~114k buildings —
        # and it needs the extra deps laspy[lazrs] + shapely + scikit-image in this venv. The
        # campus base above contributes only the scene<->UTM georef in web/data/manifest.json;
        # these steps fill the ENTIRE city and REPLACE the campus-only buildings/roads. Every
        # step is resumable (LiDAR downloads + OSM responses are cached on disk), so a failure
        # part-way can simply be re-run. Ordering is load-bearing: osm_city writes the AOI bbox
        # ky_lidar queries; ky_lidar --heightmap writes the ground.f32 build_city reads.
        print('\n--- City 1/8: osm_city -> web/data/city.json (service-area bbox + ground plane) ---')
        subprocess.run([sys.executable, '-m', 'tools.osm_city'], cwd=ROOT, check=True)
        print('\n--- City 2/8: ky_lidar -> KYAPED tiles (~8 GB) + ground.f32 (citywide elevation) ---')
        subprocess.run([sys.executable, '-m', 'tools.ky_lidar',
                        '--download-aoi', '--build', '--heightmap'], cwd=ROOT, check=True)
        print('\n--- City 3/8: build_city -> ~114k OSM+LiDAR buildings (merges the manifest) ---')
        subprocess.run([sys.executable, '-m', 'tools.build_city'], cwd=ROOT, check=True)
        print('\n--- City 4/8: pack_buildings -> one packed buffer / one draw call ---')
        subprocess.run([sys.executable, '-m', 'tools.pack_buildings'], cwd=ROOT, check=True)
        print('\n--- City 5/8: osm_roads -> web/data/roads.json ---')
        subprocess.run([sys.executable, '-m', 'tools.osm_roads'], cwd=ROOT, check=True)
        print('\n--- City 6/8: smooth_roads -> smoothed roads.json + signals.json ---')
        subprocess.run([sys.executable, '-m', 'tools.smooth_roads'], cwd=ROOT, check=True)
        # Live-data bakes. Best-effort: a flaky city site / feed warns but never fails the
        # whole city build. lex_cameras snaps cameras to signals.json junctions, so it must
        # run AFTER smooth_roads; --scrape pulls the live camera list off the city map.
        print('\n--- City 7/8: lex_cameras --scrape -> web/data/cameras.json ---')
        try:
            subprocess.run([sys.executable, '-m', 'tools.lex_cameras', '--scrape'], cwd=ROOT, check=True)
        except subprocess.CalledProcessError as e:
            print(f'  [warn] lex_cameras failed ({e}); skipping camera mapping')
        print('\n--- City 8/8: lextran_gtfs -> web/data/transit.json ---')
        try:
            subprocess.run([sys.executable, '-m', 'tools.lextran_gtfs'], cwd=ROOT, check=True)
        except subprocess.CalledProcessError as e:
            print(f'  [warn] lextran_gtfs failed ({e}); skipping transit layer')
    else:
        if not skip_buildings:
            print('\n--- Step 6: Buildings (lidar + OSM -> .bin) ---')
            # Hybrid extractor: OSM footprints split/bound the LiDAR, LiDAR gives
            # shape + height. Run as a module so tools.* imports resolve and the
            # stdlib `inspect` isn't shadowed by tools/inspect.py.
            subprocess.run([sys.executable, '-m', 'tools.extract_buildings_hybrid'],
                           cwd=ROOT, check=True)
            # Re-merge to fold the freshly-extracted buildings into the manifest.
            print('\n--- Step 7: Merge manifests again (fold in buildings) ---')
            merge_manifests()

        # Pack the per-building meshes into ONE buffer for fast loading. Local, no network.
        if not skip_pack and not skip_buildings:
            print('\n--- Step 8: Pack buildings (-> one buffer) ---')
            subprocess.run([sys.executable, '-m', 'tools.pack_buildings'], cwd=ROOT, check=True)

        # Opt-in network layers: city-wide OSM streets/ground + campus road ribbons.
        # Best-effort — a network failure warns but never fails the build.
        if with_city:
            print('\n--- Step 9: City-wide OSM streets + ground plane ---')
            try:
                subprocess.run([sys.executable, '-m', 'tools.osm_city'], cwd=ROOT, check=True)
            except subprocess.CalledProcessError as e:
                print(f'  [warn] osm_city failed ({e}); skipping city layer')
            print('\n--- Step 9b: Road ribbons + signals (OSM -> roads.json + signals.json) ---')
            try:
                subprocess.run([sys.executable, '-m', 'tools.osm_roads'], cwd=ROOT, check=True)
                subprocess.run([sys.executable, '-m', 'tools.smooth_roads'], cwd=ROOT, check=True)
            except subprocess.CalledProcessError as e:
                print(f'  [warn] road extraction failed ({e}); skipping roads/signals layer')
            print('\n--- Step 9c: Traffic cameras (scrape -> cameras.json) ---')
            try:
                # snaps to signals.json junctions, so it runs after smooth_roads above
                subprocess.run([sys.executable, '-m', 'tools.lex_cameras', '--scrape'], cwd=ROOT, check=True)
            except subprocess.CalledProcessError as e:
                print(f'  [warn] lex_cameras failed ({e}); skipping camera mapping')
    if with_transit and not citywide:   # citywide bakes transit itself (City 8/8 above)
        print('\n--- Step 10: Lextran static GTFS -> transit.json ---')
        try:
            subprocess.run([sys.executable, '-m', 'tools.lextran_gtfs'], cwd=ROOT, check=True)
        except subprocess.CalledProcessError as e:
            print(f'  [warn] lextran_gtfs failed ({e}); skipping transit layer')


def merge_manifests():
    """Read per-domain manifests and write the unified manifest."""
    ext = os.path.join(ROOT, 'extracted')

    # Load per-domain manifests
    with open(os.path.join(ext, 'manifest-textures.json')) as f:
        tex_manifest = json.load(f)
    with open(os.path.join(ext, 'manifest-meshes.json')) as f:
        mesh_manifest = json.load(f)
    with open(os.path.join(ext, 'manifest-lidar.json')) as f:
        lidar_manifest = json.load(f)
    with open(os.path.join(ext, 'manifest-scene.json')) as f:
        scene_manifest = json.load(f)

    # Build tile index by name for fast lookup
    mesh_by_name = {t['name']: t for t in mesh_manifest['tiles']}
    tex_by_name = {t['name']: t for t in tex_manifest['textures']}
    scene_by_name = {t['name']: t for t in scene_manifest['tiles']}

    # Build the terrain tiles array
    tiles = []
    for name in sorted(mesh_by_name):
        if name not in scene_by_name:
            continue
        scene = scene_by_name[name]
        tiles.append({
            'name': name,
            'mesh': f"meshes/{name}.bin",
            'texture': f"textures/{name}.jpg",
            'translation_cm': scene.get('translation_cm', [0, 0, 0]),
            'rotation_deg': scene.get('rotation_deg', [0, 0, 0]),
            'scale': scene.get('scale', [1, 1, 1]),
            'visible': scene.get('visible', True),
        })

    # Compute a natural origin
    origin_cm = scene_manifest['origin_node']['translation_cm'] if scene_manifest.get('origin_node') else [0, 0, 0]

    # Build lidar section
    lidar = {
        'offset_cm': lidar_manifest.get('offset_cm', [0, 0, 0]),
        'original_coordinates': lidar_manifest.get('original_coordinates'),
        'chunks': [
            {
                'file': f"lidar/{c['file']}",
                'count': c['count'],
                'bounds_min_cm': c.get('bounds_min_cm'),
                'bounds_max_cm': c.get('bounds_max_cm'),
            }
            for c in lidar_manifest['chunks']
        ],
    }

    # Build buildings section (T024)
    buildings = None
    bld_path = os.path.join(ext, 'manifest-buildings.json')
    if os.path.exists(bld_path):
        with open(bld_path) as f:
            bld_manifest = json.load(f)
        buildings = {'tiles': bld_manifest.get('tiles', [])}
        if 'total_mesh_bytes' in bld_manifest:
            buildings['total_mesh_bytes'] = bld_manifest['total_mesh_bytes']

    manifest = {
        'title': 'Lexington Digital Twin — extracted from UE 4.24.3 editor assets',
        'origin_cm': origin_cm,
        'coordinate_note': scene_manifest.get('coordinate_note',
            'UE world centimeters, Z-up. Converted to Three.js meters Y-up on load.'),
        'terrain': {
            'tiles': tiles,
        },
        'lidar': lidar,
    }

    if buildings:
        manifest['buildings'] = buildings

    manifest['extraction_stats'] = {
        'textures': tex_manifest['count'],
        'meshes': len(mesh_manifest['tiles']),
        'lidar_full_points': lidar_manifest['total_points_full'],
        'lidar_kept_points': lidar_manifest['total_points_kept'],
        'lidar_chunks': len(lidar_manifest['chunks']),
    }
    if buildings:
        manifest['extraction_stats']['buildings'] = len(buildings['tiles'])
        if 'total_mesh_bytes' in buildings:
            manifest['extraction_stats']['building_mesh_bytes'] = buildings['total_mesh_bytes']

    out_path = os.path.join(ROOT, 'web', 'data', 'manifest.json')
    with open(out_path, 'w') as f:
        json.dump(manifest, f, indent=2)
    bl = len(buildings['tiles']) if buildings else 0
    print(f'  wrote {out_path}')
    print(f'  tiles: {len(tiles)}, lidar chunks: {len(lidar["chunks"])}, '
          f'buildings: {bl}, '
          f'origin_cm: [{origin_cm[0]:.1f}, {origin_cm[1]:.1f}, {origin_cm[2]:.1f}]')
    print('\nDone. Run: cd web && python -m http.server 8000')


def verify():
    """Verify data integrity without re-extracting."""
    data = os.path.join(ROOT, 'web', 'data')
    issues = []

    manifest_path = os.path.join(data, 'manifest.json')
    if not os.path.exists(manifest_path):
        issues.append('web/data/manifest.json missing -- run: python tools/build_all.py')
        print('Issues found:')
        for i in issues:
            print(f'  [MISSING] {i}')
        return False

    with open(manifest_path) as f:
        m = json.load(f)

    for t in m['terrain']['tiles']:
        mesh_path = os.path.join(data, t['mesh'])
        tex_path = os.path.join(data, t['texture'])
        if not os.path.exists(mesh_path):
            issues.append(f"missing mesh: {t['mesh']}")
        if not os.path.exists(tex_path):
            issues.append(f"missing texture: {t['texture']}")

    for c in m['lidar']['chunks']:
        chunk_path = os.path.join(data, c['file'])
        if not os.path.exists(chunk_path):
            issues.append(f"missing lidar chunk: {c['file']}")

    # T025: Verify buildings
    if 'buildings' in m:
        for b in m['buildings']['tiles']:
            bpath = os.path.join(data, b['file'])
            if not os.path.exists(bpath):
                issues.append(f"missing building: {b['file']}")
            req_fields = ['name', 'file', 'bounds_min_cm', 'bounds_max_cm',
                          'height_cm', 'footprint_area_m2', 'point_count',
                          'vertex_count', 'index_count']
            for rf in req_fields:
                if rf not in b:
                    issues.append(f"building {b.get('name','?')} missing field: {rf}")

    if issues:
        print('Issues found:')
        for i in issues:
            print(f'  [MISSING] {i}')
        return False

    stats = m['extraction_stats']
    bld = stats.get('buildings', 0)
    bld_mb = stats.get('building_mesh_bytes', 0) / (1024 * 1024)
    print(f'verification OK -- {stats["textures"]} textures, '
          f'{stats["meshes"]} meshes, '
          f'{stats["lidar_kept_points"]/1e6:.1f}M lidar pts in '
          f'{stats["lidar_chunks"]} chunks, '
          f'{bld} buildings ({bld_mb:.1f} MB)')
    return True


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='Full extraction pipeline')
    ap.add_argument('--skip-textures', action='store_true')
    ap.add_argument('--skip-meshes', action='store_true')
    ap.add_argument('--skip-lidar', action='store_true')
    ap.add_argument('--skip-buildings', action='store_true')
    ap.add_argument('--skip-scene', action='store_true',
                    help='skip blueprint/scene extraction (reuse extracted/manifest-scene.json)')
    ap.add_argument('--skip-pack', action='store_true',
                    help='skip packing buildings into one buffer (keep per-building .bins)')
    ap.add_argument('--with-city', action='store_true',
                    help='also build the city-wide OSM context (needs network)')
    ap.add_argument('--with-transit', action='store_true',
                    help='also bake the Lextran transit layer (needs network)')
    ap.add_argument('--citywide', action='store_true',
                    help='build ALL of Lexington (open data), not just campus: after the campus '
                         'georef base, run osm_city -> ky_lidar (~8 GB KYAPED download) -> '
                         'build_city (~114k buildings) -> pack -> osm_roads -> smooth_roads. '
                         'Needs network + laspy[lazrs]/shapely/scikit-image. Replaces campus buildings/roads.')
    ap.add_argument('--verify', action='store_true',
                    help='Verify data integrity without extracting')
    args = ap.parse_args()

    if args.verify:
        ok = verify()
        sys.exit(0 if ok else 1)

    run_extraction(args.skip_textures, args.skip_meshes, args.skip_lidar,
                   args.skip_buildings, args.skip_pack, args.with_city, args.with_transit,
                   skip_scene=args.skip_scene, citywide=args.citywide)