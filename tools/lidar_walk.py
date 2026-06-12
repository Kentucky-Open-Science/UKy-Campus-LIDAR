"""Walk the entire LidarPointCloud octree to validate the hypothesized grammar.

Grammar (hypothesis):
  export native data:
    u32 = 0
    FString source path
    u32 = 1
    u32 = 0x000D0001  (custom data version?)
    u32 = 0x000D0001
    u32 = 1
    f32[6] bounds box (min xyz, max xyz)
    u32 = 1
    18 zero bytes
    root node_body
  node_body :=
    u32 nPoints, nPoints * 18B point records
    u32 nExtra,  nExtra  * 18B point records ("padding"?)
    u32 nChildren
    nChildren * { u8 child_idx, f32[3] rel_center, node_body }
  point record (18B) := f32 x, f32 y, f32 z, u8 B, u8 G, u8 R, u8 A, u8 f0, u8 f1
"""
import struct, sys, time

PATH = r'C:/Users/sear234/Desktop/CAMPUS/LIDAR/POINT_CLOUD_2019.uasset'
EXPORT_END = 448664675
ROOT_EXTENT = 170396.5

with open(PATH, 'rb') as f:
    DATA = f.read()

u32 = lambda p: struct.unpack_from('<I', DATA, p)[0]
fv3 = lambda p: struct.unpack_from('<fff', DATA, p)

stats = {
    'nodes': 0, 'points': 0, 'extra': 0, 'max_depth': 0,
    'leafs': 0, 'bad_center': 0, 'children_hist': [0]*9,
    'depth_points': {}, 'depth_nodes': {},
}
sys.setrecursionlimit(10000)


def walk(pos, depth, cx, cy, cz, ext):
    stats['nodes'] += 1
    stats['max_depth'] = max(stats['max_depth'], depth)
    stats['depth_nodes'][depth] = stats['depth_nodes'].get(depth, 0) + 1
    n = u32(pos); pos += 4
    if n > 50_000_000:
        raise ValueError(f'absurd nPoints {n} at {pos-4} depth {depth}')
    stats['points'] += n
    stats['depth_points'][depth] = stats['depth_points'].get(depth, 0) + n
    pos += n * 18
    ne = u32(pos); pos += 4
    if ne > 50_000_000:
        raise ValueError(f'absurd nExtra {ne} at {pos-4} depth {depth}')
    stats['extra'] += ne
    pos += ne * 18
    nc = u32(pos); pos += 4
    if nc > 8:
        raise ValueError(f'absurd nChildren {nc} at {pos-4} depth {depth}')
    stats['children_hist'][nc] += 1
    if nc == 0:
        stats['leafs'] += 1
    half = ext / 2
    for _ in range(nc):
        idx = DATA[pos]; pos += 1
        rx, ry, rz = fv3(pos); pos += 12
        # verify rel center matches idx bits and half extent
        ex = half if (idx & 1) else -half   # guess bit0 = x sign
        ey = half if (idx & 2) else -half
        ez = half if (idx & 4) else -half
        if abs(rx) != half or abs(ry) != half or abs(rz) != half:
            stats['bad_center'] += 1
            if stats['bad_center'] < 5:
                print(f'  center magnitude mismatch at {pos-12}: idx={idx} rel=({rx},{ry},{rz}) half={half}')
        elif (rx, ry, rz) != (ex, ey, ez):
            stats['bad_center'] += 1
            if stats['bad_center'] < 10:
                print(f'  center sign mismatch: idx={idx:#05b} rel=({rx:+},{ry:+},{rz:+}) half={half}')
        pos = walk(pos, depth + 1, cx + rx, cy + ry, cz + rz, half)
    return pos


def main():
    t0 = time.time()
    # locate start: known from probe
    pos = 1918  # bounds box
    box = struct.unpack_from('<6f', DATA, pos)
    print('bounds box:', box)
    pos += 24
    v = u32(pos); pos += 4
    print('u32 after box:', v)
    head18 = DATA[pos:pos+18]
    print('18B head:', head18.hex())
    pos += 18
    end = walk(pos, 0, 0.0, 0.0, 0.0, ROOT_EXTENT)
    dt = time.time() - t0
    print(f'walk finished at {end} (export end {EXPORT_END}, file size {len(DATA)}) in {dt:.1f}s')
    print(f'nodes={stats["nodes"]} leafs={stats["leafs"]} max_depth={stats["max_depth"]}')
    print(f'points={stats["points"]} extra={stats["extra"]} total={stats["points"]+stats["extra"]}')
    print(f'children_hist={stats["children_hist"]}')
    print(f'bad_center={stats["bad_center"]}')
    print('per-depth nodes :', dict(sorted(stats['depth_nodes'].items())))
    print('per-depth points:', dict(sorted(stats['depth_points'].items())))
    rem = EXPORT_END - end
    print(f'remaining bytes to export end: {rem}')
    if 0 <= rem <= 4096:
        print('tail hex:', DATA[end:EXPORT_END].hex())
    print('beyond export end (to file end):', DATA[EXPORT_END:EXPORT_END+64].hex())


if __name__ == '__main__':
    main()
