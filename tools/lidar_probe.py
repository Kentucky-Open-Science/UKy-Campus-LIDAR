"""Probe the LidarPointCloud native serialization region.

Usage: python lidar_probe.py [hexdump_len]
"""
import sys, os, struct
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uasset import Package, Reader

PATH = r'C:/Users/sear234/Desktop/CAMPUS/LIDAR/POINT_CLOUD_2019.uasset'


def hexdump(data, start, n, file_off=0):
    for i in range(start, start + n, 16):
        row = data[i:i+16]
        hx = ' '.join(f'{b:02x}' for b in row)
        asc = ''.join(chr(b) if 32 <= b < 127 else '.' for b in row)
        print(f'{i+file_off:10d}  {hx:<48}  {asc}')


def main():
    p = Package(PATH)
    e = p.exports[0]
    print(f'export: {e["object_name"]} class={p.class_of(e)} '
          f'offset={e["serial_offset"]} size={e["serial_size"]} '
          f'end={e["serial_offset"]+e["serial_size"]}')
    print(f'file_size={len(p.data)} bulk_start={p.bulk_data_start_offset}')
    print('custom versions:')
    for g, v in p.custom_versions:
        print('  ', g, v)

    r = Reader(p.data, e['serial_offset'])
    props = p.read_properties(r)
    for t in props:
        print(f'prop {t["name"]} type={t["type"]} size={t["size"]} '
              f'voff={t["value_offset"]} struct={t.get("struct_name")}')
        if t['name'] == 'OriginalCoordinates':
            rr = Reader(p.data, t['value_offset'])
            print('   doubles:', rr.f64(), rr.f64(), rr.f64())
        if t['name'] == 'ClassificationsImported':
            print('   raw:', p.data[t['value_offset']:t['value_offset']+t['size']].hex())
            print('   value:', t['value'])
    pos = r.tell()
    print(f'props end at {pos}')
    guid_flag = r.i32()
    print(f'guid flag: {guid_flag}')
    if guid_flag:
        print('guid:', r.guid())
    print(f'native data starts at {r.tell()}')
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 512
    hexdump(p.data, r.tell(), n)


if __name__ == '__main__':
    main()
