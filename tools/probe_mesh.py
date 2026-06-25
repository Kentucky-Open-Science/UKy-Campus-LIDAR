"""Exploratory probe of FMeshDescription payload in DTM mesh packages."""
import os
import sys
import struct
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from uasset import Package, Reader, decompress_chunked

_REPO = os.path.dirname(_HERE)
PATH = sys.argv[1] if len(sys.argv) > 1 else \
    os.path.join(_REPO, 'MESHES', 'DTM_GRID', 'Meshes', '15626E185064N.uasset')

p = Package(PATH)
blob = p.data[p.bulk_data_start_offset:len(p.data) - 4]
dec = decompress_chunked(blob)
print('decompressed:', len(dec))

r = Reader(dec)


def hexdump(data, off, n=64):
    for i in range(off, min(off + n, len(data)), 16):
        row = data[i:i + 16]
        print(f'{i:08x}  ' + ' '.join(f'{b:02x}' for b in row) + '  ' +
              ''.join(chr(b) if 32 <= b < 127 else '.' for b in row))


def elem_array(r, label, per_elem_i32s=0):
    """Hypothesis: TMeshElementArray = i32 ArraySize, then raw allocation
    bitmask of ceil(size/32) u32 words, then per-allocated-element payload."""
    pos = r.tell()
    size = r.i32()
    nwords = (size + 31) // 32
    words = struct.unpack_from(f'<{nwords}I', r.d, r.tell())
    r.read(nwords * 4)
    nalloc = sum(bin(w).count('1') for w in words)
    print(f'{label}: at {pos} size={size} words={nwords} allocated={nalloc} '
          f'elem_payload_starts={r.tell()}')
    return size, nalloc


# VertexArray: FMeshVertex serializes NOTHING in new serialization
vsize, valloc = elem_array(r, 'VertexArray')
print('  next bytes after vertex bitmask:')
hexdump(dec, r.tell(), 48)
print()

# VertexInstanceArray: each FMeshVertexInstance serializes i32 VertexID
visize, vialloc = elem_array(r, 'VertexInstanceArray')
print('  first instance VertexIDs:', struct.unpack_from('<8i', r.d, r.tell()))
vi_payload = r.tell()
r.seek(vi_payload + vialloc * 4)
print('  after instance payload at', r.tell())
hexdump(dec, r.tell(), 48)
print()

# EdgeArray: each FMeshEdge serializes 2x i32 VertexIDs
esize, ealloc = elem_array(r, 'EdgeArray')
print('  first edge pairs:', struct.unpack_from('<8i', r.d, r.tell()))
e_payload = r.tell()
r.seek(e_payload + ealloc * 8)
print('  after edge payload at', r.tell())
hexdump(dec, r.tell(), 48)
print()

# PolygonArray: each FMeshPolygon serializes TArray<i32> contour + i32 group
psize, palloc = elem_array(r, 'PolygonArray')
print('  poly payload start:')
hexdump(dec, r.tell(), 64)
# try reading polys as {i32 n, n*i32, i32 group}
ok = True
start = r.tell()
tri_total = 0
for i in range(palloc):
    n = r.i32()
    if not (3 <= n <= 64):
        print(f'  poly[{i}] bad contour count {n} at {r.tell()-4}')
        ok = False
        break
    ids = [r.i32() for _ in range(n)]
    grp = r.i32()
    if i < 3:
        print(f'  poly[{i}]: n={n} ids={ids} group={grp}')
    tri_total += n - 2
if ok:
    print(f'  all {palloc} polys parsed, tri_total={tri_total}, now at {r.tell()}')
print()
hexdump(dec, r.tell(), 64)
print()

# PolygonGroupArray: FMeshPolygonGroup serializes nothing (new)
gsize, galloc = elem_array(r, 'PolygonGroupArray')
print()
hexdump(dec, r.tell(), 256)
