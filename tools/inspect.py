"""Dump tagged properties of selected exports + hex of trailing custom data."""
import sys
sys.path.insert(0, 'tools')
from uasset import Package, Reader


def hexdump(data, start=0, n=160, abs_off=0):
    for i in range(start, min(start + n, len(data)), 16):
        row = data[i:i + 16]
        hx = ' '.join(f'{b:02x}' for b in row)
        asc = ''.join(chr(b) if 32 <= b < 127 else '.' for b in row)
        print(f'    {abs_off + i:10d}: {hx:<48} {asc}')


def show(path, class_filter=None):
    p = Package(path)
    print(f'== {path} (file={len(p.data)}, bulk_start={p.bulk_data_start_offset})')
    for i, e in enumerate(p.exports):
        cls = p.class_of(e)
        if class_filter and cls not in class_filter:
            continue
        print(f'  export [{i+1}] {cls} {e["object_name"]} '
              f'off={e["serial_offset"]} size={e["serial_size"]}')
        r = Reader(p.data, e['serial_offset'])
        try:
            props = p.read_properties(r)
            for t in props:
                v = t['value']
                if isinstance(v, list) and len(str(v)) > 300:
                    v = str(v)[:300] + '...'
                print(f'    .{t["name"]} ({t["type"]}'
                      f'{"/" + t.get("struct_name", "") if t.get("struct_name") else ""}) = {v}')
        except Exception as ex:
            print(f'    !! property parse failed: {ex}')
        after = r.tell()
        rest = e['serial_offset'] + e['serial_size'] - after
        print(f'    [props end at {after}, {rest} bytes of native data follow]')
        if rest > 0:
            hexdump(p.data, after, min(rest, 256), 0)
    return p


if __name__ == '__main__':
    show(sys.argv[1])
