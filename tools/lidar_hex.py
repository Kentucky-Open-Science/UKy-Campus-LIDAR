"""Hexdump arbitrary windows of the LiDAR uasset. Usage: lidar_hex.py off len [off len ...]"""
import os
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH = os.path.join(_REPO, 'LIDAR', 'POINT_CLOUD_2019.uasset')

def hexdump(data, base, n):
    for i in range(0, n, 16):
        row = data[i:i+16]
        hx = ' '.join(f'{b:02x}' for b in row)
        asc = ''.join(chr(b) if 32 <= b < 127 else '.' for b in row)
        print(f'{base+i:12d}  {hx:<48}  {asc}')

with open(PATH, 'rb') as f:
    args = sys.argv[1:]
    for j in range(0, len(args), 2):
        off, n = int(args[j]), int(args[j+1])
        f.seek(off)
        data = f.read(n)
        print(f'--- offset {off} ---')
        hexdump(data, off, n)
        print()
