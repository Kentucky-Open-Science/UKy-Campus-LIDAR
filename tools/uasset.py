"""Minimal UE4 (4.24, file version 518) uncooked package parser.

Parses FPackageFileSummary, name table, import table, export table, and
provides helpers for reading tagged properties and FByteBulkData payloads.
"""
import struct
import zlib


class Reader:
    def __init__(self, data, pos=0):
        self.d = data
        self.p = pos

    def seek(self, p):
        self.p = p

    def tell(self):
        return self.p

    def read(self, n):
        b = self.d[self.p:self.p + n]
        self.p += n
        return b

    def i8(self):
        return struct.unpack_from('<b', self.d, self._adv(1))[0]

    def u8(self):
        return struct.unpack_from('<B', self.d, self._adv(1))[0]

    def i16(self):
        return struct.unpack_from('<h', self.d, self._adv(2))[0]

    def u16(self):
        return struct.unpack_from('<H', self.d, self._adv(2))[0]

    def i32(self):
        return struct.unpack_from('<i', self.d, self._adv(4))[0]

    def u32(self):
        return struct.unpack_from('<I', self.d, self._adv(4))[0]

    def i64(self):
        return struct.unpack_from('<q', self.d, self._adv(8))[0]

    def u64(self):
        return struct.unpack_from('<Q', self.d, self._adv(8))[0]

    def f32(self):
        return struct.unpack_from('<f', self.d, self._adv(4))[0]

    def f64(self):
        return struct.unpack_from('<d', self.d, self._adv(8))[0]

    def _adv(self, n):
        p = self.p
        self.p += n
        return p

    def fstring(self):
        n = self.i32()
        if n == 0:
            return ''
        if n < 0:  # UTF-16
            n = -n
            s = self.read(n * 2)[:-2].decode('utf-16-le')
        else:
            s = self.read(n)[:-1].decode('latin-1')
        return s

    def guid(self):
        return self.read(16).hex()


class Package:
    def __init__(self, path):
        with open(path, 'rb') as f:
            self.data = f.read()
        self.path = path
        r = Reader(self.data)
        assert r.u32() == 0x9E2A83C1, 'bad magic'
        self.legacy_ver = r.i32()
        if self.legacy_ver != -4:
            r.i32()  # LegacyUE3Version
        self.ver_ue4 = r.i32()
        self.ver_licensee = r.i32()
        self.custom_versions = []
        if self.legacy_ver <= -2:
            n = r.i32()
            for _ in range(n):
                self.custom_versions.append((r.guid(), r.i32()))
        self.total_header_size = r.i32()
        self.folder_name = r.fstring()
        self.package_flags = r.u32()
        self.name_count = r.i32()
        self.name_offset = r.i32()
        if self.ver_ue4 >= 516:  # VER_UE4_ADDED_PACKAGE_SUMMARY_LOCALIZATION_ID
            self.localization_id = r.fstring()
        if self.ver_ue4 >= 459:
            self.gatherable_text_count = r.i32()
            self.gatherable_text_offset = r.i32()
        self.export_count = r.i32()
        self.export_offset = r.i32()
        self.import_count = r.i32()
        self.import_offset = r.i32()
        self.depends_offset = r.i32()
        if self.ver_ue4 >= 384:
            self.soft_pkg_refs_count = r.i32()
            self.soft_pkg_refs_offset = r.i32()
        if self.ver_ue4 >= 510:
            self.searchable_names_offset = r.i32()
        self.thumbnail_table_offset = r.i32()
        self.package_guid = r.guid()
        if not (self.package_flags & 0x80000000):  # !PKG_FilterEditorOnly
            if self.ver_ue4 >= 518:  # VER_UE4_ADDED_PACKAGE_OWNER
                self.persistent_guid = r.guid()
            if 518 <= self.ver_ue4 < 520:  # < VER_UE4_NON_OUTER_PACKAGE_IMPORT
                self.owner_persistent_guid = r.guid()
        # generations
        gen_count = r.i32()
        self.generations = [(r.i32(), r.i32()) for _ in range(gen_count)]
        if self.ver_ue4 >= 336:
            self.saved_by_engine = self._engine_version(r)
        if self.ver_ue4 >= 444:
            self.compatible_engine = self._engine_version(r)
        self.compression_flags = r.u32()
        n_compressed = r.i32()
        assert n_compressed == 0, 'package-level compression unsupported'
        self.package_source = r.u32()
        n_pkgs = r.i32()
        self.additional_cook_packages = [r.fstring() for _ in range(n_pkgs)]
        if self.ver_ue4 < 224:
            r.i32()
        self.asset_registry_offset = r.i32()
        self.bulk_data_start_offset = r.i64()
        # (world tile info, chunk ids, preload deps follow; not needed)

        self._read_names()
        self._read_imports()
        self._read_exports()

    @staticmethod
    def _engine_version(r):
        return (r.u16(), r.u16(), r.u16(), r.u32(), r.fstring())

    def _read_names(self):
        r = Reader(self.data, self.name_offset)
        self.names = []
        for _ in range(self.name_count):
            s = r.fstring()
            r.read(4)  # precalculated hashes (u16 x2)
            self.names.append(s)

    def fname(self, r):
        idx = r.i32()
        num = r.i32()
        base = self.names[idx]
        return f'{base}_{num - 1}' if num else base

    def _read_imports(self):
        r = Reader(self.data, self.import_offset)
        self.imports = []
        for _ in range(self.import_count):
            class_package = self.fname(r)
            class_name = self.fname(r)
            outer_index = r.i32()
            object_name = self.fname(r)
            self.imports.append({
                'class_package': class_package, 'class_name': class_name,
                'outer_index': outer_index, 'object_name': object_name,
            })

    def _read_exports(self):
        r = Reader(self.data, self.export_offset)
        self.exports = []
        for _ in range(self.export_count):
            e = {}
            e['class_index'] = r.i32()
            e['super_index'] = r.i32()
            if self.ver_ue4 >= 508:
                e['template_index'] = r.i32()
            e['outer_index'] = r.i32()
            e['object_name'] = self.fname(r)
            e['object_flags'] = r.u32()
            if self.ver_ue4 < 511:
                e['serial_size'] = r.i32()
                e['serial_offset'] = r.i32()
            else:
                e['serial_size'] = r.i64()
                e['serial_offset'] = r.i64()
            e['forced_export'] = r.i32()
            e['not_for_client'] = r.i32()
            e['not_for_server'] = r.i32()
            e['package_guid'] = r.guid()
            e['package_flags'] = r.u32()
            if self.ver_ue4 >= 365:
                e['not_always_loaded_for_editor'] = r.i32()
            if self.ver_ue4 >= 485:
                e['is_asset'] = r.i32()
            if self.ver_ue4 >= 507:
                e['first_dep'] = r.i32()
                e['ser_before_ser_deps'] = r.i32()
                e['create_before_ser_deps'] = r.i32()
                e['ser_before_create_deps'] = r.i32()
                e['create_before_create_deps'] = r.i32()
            self.exports.append(e)

    def obj_name(self, package_index):
        """Resolve an FPackageIndex to a printable name."""
        if package_index > 0:
            return self.exports[package_index - 1]['object_name']
        if package_index < 0:
            return self.imports[-package_index - 1]['object_name']
        return 'None'

    def class_of(self, export):
        return self.obj_name(export['class_index'])

    # ---- tagged property parsing -------------------------------------
    def read_properties(self, r, depth=0):
        """Read a tagged property list until 'None'. Returns list of dicts."""
        props = []
        while True:
            name = self.fname(r)
            if name == 'None':
                break
            type_name = self.fname(r)
            size = r.i32()
            array_index = r.i32()
            tag = {'name': name, 'type': type_name, 'size': size,
                   'array_index': array_index}
            self._read_prop_tag_extras(r, tag)
            has_guid = r.u8()
            if has_guid:
                tag['prop_guid'] = r.guid()
            value_start = r.tell()
            tag['value_offset'] = value_start
            tag['value'] = self._read_prop_value(r, tag, depth)
            r.seek(value_start + size)
            props.append(tag)
        return props

    def _read_prop_tag_extras(self, r, tag):
        t = tag['type']
        if t == 'StructProperty':
            tag['struct_name'] = self.fname(r)
            tag['struct_guid'] = r.guid()
        elif t == 'BoolProperty':
            tag['bool_val'] = r.u8()
        elif t in ('ByteProperty', 'EnumProperty'):
            tag['enum_name'] = self.fname(r)
        elif t == 'ArrayProperty':
            tag['inner_type'] = self.fname(r)
        elif t == 'SetProperty':
            tag['inner_type'] = self.fname(r)
        elif t == 'MapProperty':
            tag['key_type'] = self.fname(r)
            tag['value_type'] = self.fname(r)

    def _read_prop_value(self, r, tag, depth):
        t = tag['type']
        if t == 'BoolProperty':
            return bool(tag['bool_val'])
        if t == 'IntProperty':
            return r.i32()
        if t == 'UInt32Property':
            return r.u32()
        if t == 'Int64Property':
            return r.i64()
        if t == 'UInt64Property':
            return r.u64()
        if t == 'FloatProperty':
            return r.f32()
        if t == 'DoubleProperty':
            return r.f64()
        if t in ('ObjectProperty', 'SoftObjectProperty'):
            return r.i32() if t == 'ObjectProperty' else r.fstring()
        if t in ('NameProperty', 'EnumProperty'):
            return self.fname(r)
        if t in ('StrProperty',):
            return r.fstring()
        if t == 'ByteProperty':
            if tag.get('enum_name', 'None') != 'None':
                return self.fname(r)
            return r.u8()
        if t == 'StructProperty':
            return self._read_struct(r, tag.get('struct_name'), tag['size'], depth)
        if t == 'ArrayProperty':
            return self._read_array(r, tag, depth)
        return None  # unknown: caller skips via size

    def _read_struct(self, r, struct_name, size, depth):
        if struct_name == 'Vector':
            return (r.f32(), r.f32(), r.f32())
        if struct_name == 'Vector2D':
            return (r.f32(), r.f32())
        if struct_name == 'Vector4' or struct_name == 'Quat':
            return (r.f32(), r.f32(), r.f32(), r.f32())
        if struct_name == 'Rotator':
            return (r.f32(), r.f32(), r.f32())
        if struct_name == 'Color':
            return tuple(r.read(4))
        if struct_name == 'LinearColor':
            return (r.f32(), r.f32(), r.f32(), r.f32())
        if struct_name == 'Guid':
            return r.guid()
        if struct_name == 'Box':
            v = ((r.f32(), r.f32(), r.f32()), (r.f32(), r.f32(), r.f32()), r.u8())
            return v
        # generic struct: tagged props
        if depth < 8:
            try:
                return self.read_properties(r, depth + 1)
            except Exception:
                return None
        return None

    def _read_array(self, r, tag, depth):
        inner = tag.get('inner_type')
        n = r.i32()
        if inner == 'ObjectProperty':
            return [r.i32() for _ in range(n)]
        if inner == 'IntProperty':
            return [r.i32() for _ in range(n)]
        if inner == 'FloatProperty':
            return [r.f32() for _ in range(n)]
        if inner == 'NameProperty':
            return [self.fname(r) for _ in range(n)]
        if inner == 'StrProperty':
            return [r.fstring() for _ in range(n)]
        if inner == 'StructProperty':
            # inner tag precedes elements
            inner_name = self.fname(r)
            inner_type = self.fname(r)
            inner_size = r.i32()
            r.i32()  # array index
            struct_name = self.fname(r)
            r.guid()
            if r.u8():
                r.guid()
            start = r.tell()
            out = []
            for _ in range(min(n, 64)):
                out.append(self._read_struct(r, struct_name, None, depth + 1))
            r.seek(start + inner_size)
            return {'struct': struct_name, 'count': n, 'items': out}
        return {'count': n, 'inner': inner}

    # ---- bulk data ----------------------------------------------------
    BULKDATA_PayloadAtEndOfFile = 0x0001
    BULKDATA_SerializeCompressedZLIB = 0x0002
    BULKDATA_ForceInlinePayload = 0x0040
    BULKDATA_Size64Bit = 0x2000
    BULKDATA_OptionalPayload = 0x0800
    BULKDATA_PayloadInSeperateFile = 0x0100

    def read_bulkdata(self, r):
        """Read FByteBulkData header at r and return (flags, count, payload bytes)."""
        flags = r.u32()
        if flags & self.BULKDATA_Size64Bit:
            count = r.i64()
            size_on_disk = r.i64()
        else:
            count = r.i32()
            size_on_disk = r.i32()
        offset = r.i64()
        if not (flags & 0x10000):  # BULKDATA_NoOffsetFixUp
            offset += self.bulk_data_start_offset
        if flags & self.BULKDATA_ForceInlinePayload:
            payload = r.read(size_on_disk)
        elif flags & self.BULKDATA_PayloadInSeperateFile:
            payload = None  # .ubulk - not used in editor saves
        else:
            payload = self.data[offset:offset + size_on_disk]
        if payload is not None and flags & self.BULKDATA_SerializeCompressedZLIB:
            payload = decompress_chunked(payload)
        return flags, count, payload


def decompress_chunked(blob):
    """FArchive::SerializeCompressed: FCompressedChunkInfo tag {magic,
    chunk_size} (i64 x2), summary {comp_total, uncomp_total} (i64 x2),
    per-chunk {comp, uncomp} pairs, then zlib streams."""
    r = Reader(blob)
    magic = r.i64()
    assert (magic & 0xFFFFFFFF) == 0x9E2A83C1, hex(magic)
    block_size = r.i64()
    comp_total = r.i64()
    uncomp_total = r.i64()
    n_chunks = (uncomp_total + block_size - 1) // block_size
    sizes = [(r.i64(), r.i64()) for _ in range(n_chunks)]
    out = bytearray()
    for comp_sz, uncomp_sz in sizes:
        out += zlib.decompress(r.read(comp_sz))
    assert len(out) == uncomp_total, (len(out), uncomp_total)
    return bytes(out)


def dump(path, max_props=0):
    p = Package(path)
    print(f'== {path}')
    print(f'  ver_ue4={p.ver_ue4} header={p.total_header_size} '
          f'flags={p.package_flags:#x} bulk_start={p.bulk_data_start_offset} '
          f'file_size={len(p.data)}')
    print(f'  names({p.name_count}): {p.names[:40]}')
    print('  imports:')
    for i, im in enumerate(p.imports):
        print(f'    [{-(i+1)}] {im["class_name"]} {im["object_name"]} (outer {im["outer_index"]})')
    print('  exports:')
    for i, e in enumerate(p.exports):
        print(f'    [{i+1}] class={p.class_of(e)} name={e["object_name"]} '
              f'offset={e["serial_offset"]} size={e["serial_size"]}')
    return p


if __name__ == '__main__':
    import sys
    dump(sys.argv[1])
