"""Extract UE4 Texture2D aerial-imagery tiles to web-ready JPEGs.

Each texture package stores its TextureSource as a complete PNG file in the
end-of-file bulk region: bytes [bulk_data_start_offset : file_size - 4]
(4-byte package tail magic c1832a9e). Source props (TextureSource struct on
the Texture2D export) give SizeX/SizeY/Format=TSF_BGRA8/bPNGCompressed=True.

Pipeline per file:
  slice PNG -> Pillow decode (MAX_IMAGE_PIXELS disabled) -> verify size vs
  Source props -> optional R<->B swap (--swap) -> downscale to max 4096 px
  LANCZOS -> save JPEG q82 + 512 px thumb. No full-size PNG is kept on disk.

Usage:
  python extract_texture.py <file.uasset> [--swap] [--outdir DIR]
  python extract_texture.py --all [--swap]
  python extract_texture.py --probe <file.uasset>   # props + payload check only
  python extract_texture.py --channel-test <file.uasset>  # save swap/noswap thumbs

Per-file JSON results land in extracted/textures_parts/<NAME>.json;
run with --manifest to merge them into extracted/manifest-textures.json.
"""
import argparse
import gc
import io
import json
import os
import sys
import time

# tools/inspect.py shadows the stdlib 'inspect' module (numpy needs it).
# Preload the real stdlib inspect with the tools dir removed from sys.path,
# then restore the path so we can import uasset.
_TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path
            if os.path.abspath(p or os.getcwd()) != _TOOLS]
import inspect  # noqa: F401,E402  (stdlib, now cached in sys.modules)
import numpy  # noqa: F401,E402
sys.path.insert(0, _TOOLS)
from uasset import Package, Reader  # noqa: E402

import PIL.Image  # noqa: E402
PIL.Image.MAX_IMAGE_PIXELS = None  # tiles up to ~16k x 16k

# Repo root (the CAMPUS/ layout). Was a hardcoded dev-machine path; resolved
# relative to the repo so --all finds textures on any checkout.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEX_DIR = os.path.join(ROOT, 'MESHES', 'DTM_GRID', 'Textures')
OUT_DIR = os.path.join(ROOT, 'web', 'data', 'textures')
PARTS_DIR = os.path.join(ROOT, 'extracted', 'textures_parts')
MAX_DIM = 4096
THUMB_DIM = 512
JPEG_QUALITY = 82


def flatten_props(props, out=None):
    """name -> value for top-level and one nested struct level."""
    if out is None:
        out = {}
    for t in props:
        v = t['value']
        if isinstance(v, list) and v and isinstance(v[0], dict):
            sub = {}
            flatten_props(v, sub)
            out[t['name']] = sub
        else:
            out[t['name']] = v
    return out


def read_texture_info(p):
    """Parse Texture2D export props; return (info dict, reader past props)."""
    e = next(x for x in p.exports if p.class_of(x) == 'Texture2D')
    r = Reader(p.data, e['serial_offset'])
    props = flatten_props(p.read_properties(r))
    guid_flag = r.i32()
    if guid_flag:
        r.guid()
    src = props.get('Source', {})
    info = {
        'name': e['object_name'],
        'size_x': src.get('SizeX'),
        'size_y': src.get('SizeY'),
        'num_mips': src.get('NumMips'),
        'format': src.get('Format'),
        'png_compressed': src.get('bPNGCompressed'),
        'export_serial_end': e['serial_offset'] + e['serial_size'],
    }
    return info, r


def locate_payload(p, r_after_props, serial_end):
    """Return (payload bytes, how). Primary: end-of-file slice. Fallback:
    parse FStripDataFlags + FByteBulkData header from the export tail."""
    payload = p.data[p.bulk_data_start_offset:len(p.data) - 4]
    if payload[:8] == b'\x89PNG\r\n\x1a\n':
        return payload, 'eof-slice'
    # Fallback: strip flags (2 bytes: global+class) then FByteBulkData
    r = r_after_props
    r.read(2)  # FStripDataFlags
    flags, count, payload = p.read_bulkdata(r)
    assert r.tell() <= serial_end, 'bulk header overran export'
    assert payload[:8] == b'\x89PNG\r\n\x1a\n', \
        f'fallback payload not PNG: {payload[:8].hex()}'
    return payload, f'bulkdata-header(flags={flags:#x})'


def channel_stats(im):
    """Per-channel means on a small downscale (cheap, representative)."""
    small = im.convert('RGB')
    small.thumbnail((256, 256), PIL.Image.BILINEAR)
    import numpy as np
    a = np.asarray(small, dtype=np.float64)
    return [round(float(a[..., i].mean()), 2) for i in range(3)]


def extract_one(path, swap, outdir=OUT_DIR, parts_dir=PARTS_DIR):
    t0 = time.time()
    name = os.path.splitext(os.path.basename(path))[0]
    p = Package(path)
    info, r = read_texture_info(p)
    payload, how = locate_payload(p, r, info['export_serial_end'])
    file_size = len(p.data)
    del p
    gc.collect()

    im = PIL.Image.open(io.BytesIO(payload))
    im.load()
    del payload
    gc.collect()
    orig_w, orig_h = im.size
    orig_mode = im.mode
    assert (orig_w, orig_h) == (info['size_x'], info['size_y']), \
        f'size mismatch: PNG {im.size} vs Source {(info["size_x"], info["size_y"])}'

    if im.mode != 'RGB':
        im = im.convert('RGB')
        gc.collect()
    if swap:  # stored BGRA read as RGBA -> swap R and B back
        rch, g, b = im.split()
        im = PIL.Image.merge('RGB', (b, g, rch))
        gc.collect()
    means = channel_stats(im)

    out_w, out_h = orig_w, orig_h
    if max(orig_w, orig_h) > MAX_DIM:
        scale = MAX_DIM / max(orig_w, orig_h)
        out_w = max(1, round(orig_w * scale))
        out_h = max(1, round(orig_h * scale))
        im = im.resize((out_w, out_h), PIL.Image.LANCZOS)
        gc.collect()

    os.makedirs(outdir, exist_ok=True)
    os.makedirs(f'{outdir}/thumbs', exist_ok=True)
    os.makedirs(parts_dir, exist_ok=True)
    jpg_path = f'{outdir}/{name}.jpg'
    im.save(jpg_path, 'JPEG', quality=JPEG_QUALITY)
    thumb = im.copy()
    thumb.thumbnail((THUMB_DIM, THUMB_DIM), PIL.Image.LANCZOS)
    thumb_path = f'{outdir}/thumbs/{name}.jpg'
    thumb.save(thumb_path, 'JPEG', quality=JPEG_QUALITY)

    rec = {
        'name': name,
        'file': f'data/textures/{name}.jpg',
        'thumb': f'data/textures/thumbs/{name}.jpg',
        'orig_w': orig_w, 'orig_h': orig_h,
        'out_w': out_w, 'out_h': out_h,
        'png_mode': orig_mode,
        'format': info['format'],
        'payload_locator': how,
        'channel_swapped': bool(swap),
        'rgb_means': means,
        'uasset_bytes': file_size,
        'jpg_bytes': os.path.getsize(jpg_path),
        'seconds': round(time.time() - t0, 1),
    }
    with open(f'{parts_dir}/{name}.json', 'w') as f:
        json.dump(rec, f, indent=1)
    return rec


def probe(path):
    p = Package(path)
    info, r = read_texture_info(p)
    payload, how = locate_payload(p, r, info['export_serial_end'])
    print(json.dumps({k: v for k, v in info.items()}, indent=1))
    print('payload:', len(payload), 'bytes via', how,
          'magic', payload[:8].hex())


def channel_test(path):
    """Save unswapped and swapped 512px thumbs for visual inspection."""
    name = os.path.splitext(os.path.basename(path))[0]
    p = Package(path)
    info, r = read_texture_info(p)
    payload, _ = locate_payload(p, r, info['export_serial_end'])
    del p
    im = PIL.Image.open(io.BytesIO(payload)).convert('RGB')
    os.makedirs(f'{ROOT}/extracted/tmp', exist_ok=True)
    for tag, img in (('noswap', im),
                     ('swap', PIL.Image.merge('RGB', im.split()[::-1]))):
        t = img.copy()
        t.thumbnail((512, 512), PIL.Image.LANCZOS)
        out = f'{ROOT}/extracted/tmp/{name}_{tag}.jpg'
        t.save(out, 'JPEG', quality=90)
        print(tag, 'rgb_means', channel_stats(img), '->', out)


def build_manifest():
    parts = []
    for fn in sorted(os.listdir(PARTS_DIR)):
        if fn.endswith('.json'):
            with open(f'{PARTS_DIR}/{fn}') as f:
                parts.append(json.load(f))
    manifest = {
        'domain': 'textures',
        'count': len(parts),
        'notes': 'Aerial imagery (DTM_GRID), UE Texture2D PNG sources -> '
                 f'JPEG q{JPEG_QUALITY}, max {MAX_DIM}px. Paths relative to web/.',
        'textures': parts,
    }
    out = f'{ROOT}/extracted/manifest-textures.json'
    with open(out, 'w') as f:
        json.dump(manifest, f, indent=1)
    print('wrote', out, f'({len(parts)} textures)')


def main(args=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('path', nargs='?', help='single .uasset to extract')
    ap.add_argument('--all', action='store_true', help='process every texture')
    ap.add_argument('--swap', action='store_true', help='swap R<->B channels')
    ap.add_argument('--probe', action='store_true', help='inspect only')
    ap.add_argument('--channel-test', action='store_true',
                    help='emit swap/noswap thumbs for visual check')
    ap.add_argument('--manifest', action='store_true',
                    help='merge per-file JSONs into manifest-textures.json')
    ap.add_argument('--outdir', default=OUT_DIR)
    if args is None:                      # CLI use; build_all passes a prebuilt args in-process
        args = ap.parse_args()

    if args.manifest:
        build_manifest()
        return
    if args.probe:
        probe(args.path)
        return
    if args.channel_test:
        channel_test(args.path)
        return

    if args.all:
        files = sorted(f'{TEX_DIR}/{f}' for f in os.listdir(TEX_DIR)
                       if f.endswith('.uasset'))
    else:
        files = [args.path]
    for i, path in enumerate(files, 1):
        try:
            rec = extract_one(path, args.swap, outdir=args.outdir)
            print(f'[{i}/{len(files)}] {rec["name"]}: '
                  f'{rec["orig_w"]}x{rec["orig_h"]} -> '
                  f'{rec["out_w"]}x{rec["out_h"]}, '
                  f'rgb_means={rec["rgb_means"]}, {rec["seconds"]}s',
                  flush=True)
        except Exception as ex:
            print(f'[{i}/{len(files)}] FAILED {path}: {ex!r}', flush=True)
        gc.collect()


if __name__ == '__main__':
    main()
