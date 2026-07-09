#!/usr/bin/env python3
# dump_triangles.py - parse a PVR VRAM + register dump pair, walk the REGION
# ARRAY -> object lists (OP/PT/TR), and emit every triangle as text. Triangle
# strips are EXPANDED into their individual triangles (with alternating winding),
# triangle arrays into their per-element records, quad arrays into two triangles.
#
# The decode mirrors the RTL exactly:
#   region_array_parser.sv   region entry / ListPointer layout
#   object_list_parser.sv    strip vs tri/quad array classification & fields
#   isp_primitive_iterator.sv strip winding, array record stride, two_volumes
#   peel_core.sv (FV_* FSM)  per-vertex attribute word layout (col/off/u/v)
#
# Dumps are the 32-bit VIEW stored linearly, indexed by (byte_addr & 0x7FFFFF);
# see the pvr-list-decode-conventions memory and tools/scan_arrays.py.
#
# Output: one block per triangle, all values 8-hex-digit words (missing attrs=0):
#     <isp> <tsp> <tcw> odd=<0|1>          # header line
#     <x> <y> <z> <col> <off> <u> <v>      # vertex 0
#     <x> <y> <z> <col> <off> <u> <v>      # vertex 1
#     <x> <y> <z> <col> <off> <u> <v>      # vertex 2
# Blocks are separated by a blank line. --json emits the same data as JSON.
#
# Usage:
#   tools/dump_triangles.py <name>            # dumps/vram_<name>.bin + pvr_regs_<name>.bin
#   tools/dump_triangles.py --vram V --regs R
#   tools/dump_triangles.py --all             # every dumps/vram_*.bin pair
# Text goes to stdout (or --out FILE); --all writes dumps/tris_<name>.txt each.

import struct, glob, os, sys, json, argparse

VMASK = 0x7FFFFF

# register byte offsets (rtl/tsp/gen/pvr_regs_gen.svh)
OFF_PARAM_BASE     = 0x020
OFF_REGION_BASE    = 0x02C
OFF_FPU_SHAD_SCALE = 0x074
OFF_FPU_PARAM_CFG  = 0x07C

# ISP bit positions (tsp_pkg.sv)
ISP_UV16_BIT    = 22
ISP_GOURAUD_BIT = 23   # noqa: F841 (kept for reference; not needed for layout)
ISP_OFFSET_BIT  = 24
ISP_TEXTURE_BIT = 25


def h8(w):
    return f"{w & 0xFFFFFFFF:08x}"


class Dump:
    def __init__(self, regs_path, vram_path):
        self.rg = open(regs_path, 'rb').read()
        self.v  = open(vram_path, 'rb').read()

    def rr(self, o):   # register word
        return struct.unpack_from('<I', self.rg, o)[0]

    def vw(self, a):   # vram 32-bit view word at byte addr
        return struct.unpack_from('<I', self.v, a & VMASK)[0]


def refptr_bytes(x):
    # ListPointer: ptr_in_words = bits[23:2]; byte addr = ptr_in_words * 4.
    return ((x >> 2) & 0x3FFFFF) << 2


# strip winding (isp_primitive_iterator.sv va/vb): even i -> (i, i+1, i+2);
# odd i -> (i+1, i, i+2). is_odd = i & 1.
def strip_verts(i):
    if i & 1:
        return (i + 1, i, i + 2)
    return (i, i + 1, i + 2)


def parse(dump):
    param_base  = dump.rr(OFF_PARAM_BASE) & 0xF00000     # peel_core: PARAM_BASE & 0xF00000
    region_base = dump.rr(OFF_REGION_BASE) & VMASK
    fpu_cfg     = dump.rr(OFF_FPU_PARAM_CFG)
    shad        = dump.rr(OFF_FPU_SHAD_SCALE)
    region_v1   = ((fpu_cfg >> 21) & 1) == 0             # region_header_type==0 -> v1 (20B)
    intensity_shadow = (shad >> 8) & 1                   # FPU_SHAD_SCALE bit 8
    stride_ra   = 20 if region_v1 else 24

    triangles = []

    # ---- decode one param record's vertices (peel_core FV_* FSM) ----
    def read_vertex(vbase, isp):
        texture = (isp >> ISP_TEXTURE_BIT) & 1
        offset  = (isp >> ISP_OFFSET_BIT) & 1
        uv16    = (isp >> ISP_UV16_BIT) & 1
        x = dump.vw(vbase + 0)
        y = dump.vw(vbase + 4)
        z = dump.vw(vbase + 8)
        u = v_ = col = off = 0
        if texture:
            if uv16:
                w = dump.vw(vbase + 12)          # packed: u=hi16<<16, v=lo16<<16
                u  = (w & 0xFFFF0000)
                v_ = (w << 16) & 0xFFFF0000
                col_off = 16
            else:
                u  = dump.vw(vbase + 12)
                v_ = dump.vw(vbase + 16)
                col_off = 20
        else:
            col_off = 12
        col = dump.vw(vbase + col_off)
        if offset:
            off = dump.vw(vbase + col_off + 4)
        # order matches the requested per-line layout: x y z col off u v
        return (x, y, z, col, off, u, v_)

    # ---- walk one object list ----
    def walk_list(byte_base, listname, tile):
        base = byte_base
        for _ in range(4096):
            e = dump.vw(base); base += 4
            if (e >> 31) == 0:
                # ----- triangle STRIP (object_list_parser S_CLASS) -----
                po     = e & 0x1FFFFF                 # param_offs_in_words [20:0]
                skip   = (e >> 21) & 7
                shadow = (e >> 24) & 1
                mask   = (e >> 25) & 0x3F             # mask[30:25]
                emit_strip(po, skip, shadow, mask, tile, listname)
                continue
            typ = (e >> 29) & 7
            if typ == 7:                              # link
                if (e >> 28) & 1:
                    break                             # end_of_list
                base = refptr_bytes(e)
                continue
            if typ in (4, 5):                         # 4=tri array, 5=quad array
                po     = e & 0x1FFFFF
                skip   = (e >> 21) & 7
                shadow = (e >> 24) & 1
                count  = ((e >> 25) & 0xF) + 1
                emit_array(po, skip, shadow, count, typ == 5, tile, listname)
            # other types: ignore (unhandled in RTL too)

    # geometry helpers shared by strip/array emitters -----------------
    def rec_geom(isp, skip, shadow):
        two_vol   = shadow and not intensity_shadow
        stride_w  = 3 + skip * (2 if two_vol else 1)   # words per vertex slot
        hdr_words = 5 if two_vol else 3                # isp,tsp,tcw(,tsp1,tcw1)
        return two_vol, stride_w, hdr_words

    def read_hdr(rec_base):
        return dump.vw(rec_base), dump.vw(rec_base + 4), dump.vw(rec_base + 8)

    def emit_one(rec_base, isp, tsp, tcw, stride_b, vidx, is_odd, tile, listname):
        verts = [read_vertex(rec_base + vi * stride_b, isp) for vi in vidx]
        triangles.append({
            "tile": tile, "list": listname,
            "isp": isp, "tsp": tsp, "tcw": tcw,
            "is_odd": int(is_odd),
            "verts": verts,   # list of (x,y,z,col,off,u,v) int tuples
        })

    def emit_strip(po, skip, shadow, mask, tile, listname):
        rec_base = param_base + po * 4
        isp, tsp, tcw = read_hdr(rec_base)
        two_vol, stride_w, hdr_words = rec_geom(isp, skip, shadow)
        stride_b  = stride_w * 4
        vbase = rec_base + hdr_words * 4               # first vertex
        # triangle i exists iff mask[5-i]; up to 6 triangles (i=0..5)
        for i in range(6):
            if not ((mask >> (5 - i)) & 1):
                continue
            a, b, c = strip_verts(i)
            emit_one(vbase, isp, tsp, tcw, stride_b, (a, b, c), i & 1, tile, listname)

    def emit_array(po, skip, shadow, count, is_quad, tile, listname):
        # PERELEM layout: `count` separate records, each header + N vertices
        # (isp_primitive_iterator S_NEXTREC advances rec_base by rec_bytes).
        cur_po = po
        for _el in range(count):
            rec_base = param_base + cur_po * 4
            isp, tsp, tcw = read_hdr(rec_base)
            two_vol, stride_w, hdr_words = rec_geom(isp, skip, shadow)
            stride_b = stride_w * 4
            vbase = rec_base + hdr_words * 4
            nvtx = 4 if is_quad else 3
            # tri array: one triangle (v0,v1,v2), is_odd=0.
            emit_one(vbase, isp, tsp, tcw, stride_b, (0, 1, 2), 0, tile, listname)
            if is_quad:
                # quad -> second triangle (v0,v2,v3) (fan); is_odd=0.
                emit_one(vbase, isp, tsp, tcw, stride_b, (0, 2, 3), 0, tile, listname)
            # advance to next record: header + nvtx vertex slots
            rec_words = hdr_words + nvtx * stride_w
            cur_po += rec_words

    # ---- walk the region array ----
    base = region_base
    for _ in range(16384):
        ctrl = dump.vw(base + 0)
        opq  = dump.vw(base + 4)
        trn  = dump.vw(base + 12)
        pt   = dump.vw(base + 20) if stride_ra == 24 else 0x80000000
        tx = (ctrl >> 2) & 0x3F
        ty = (ctrl >> 8) & 0x3F
        last = (ctrl >> 31) & 1
        for nm, ptr in (('OP', opq), ('TR', trn), ('PT', pt)):
            if (ptr >> 31) & 1:
                continue                             # empty list
            walk_list(refptr_bytes(ptr), nm, (tx, ty))
        if last:
            break
        base += stride_ra

    return triangles


def render_text(tris):
    out = []
    for t in tris:
        out.append(f"{h8(t['isp'])} {h8(t['tsp'])} {h8(t['tcw'])} odd={t['is_odd']}")
        for (x, y, z, col, off, u, v_) in t['verts']:
            out.append(f"{h8(x)} {h8(y)} {h8(z)} {h8(col)} {h8(off)} {h8(u)} {h8(v_)}")
        out.append("")   # blank separator
    return "\n".join(out)


def render_json(name, tris):
    obj = {"name": name, "triangle_count": len(tris),
           "triangles": [{
               "tile": list(t['tile']), "list": t['list'],
               "isp": h8(t['isp']), "tsp": h8(t['tsp']), "tcw": h8(t['tcw']),
               "is_odd": bool(t['is_odd']),
               "verts": [{"x": h8(x), "y": h8(y), "z": h8(z), "col": h8(c),
                          "off": h8(o), "u": h8(u), "v": h8(v_)}
                         for (x, y, z, c, o, u, v_) in t['verts']],
           } for t in tris]}
    return json.dumps(obj, indent=2)


def process(name, vram_path, regs_path, out_path, as_json):
    dump = Dump(regs_path, vram_path)
    tris = parse(dump)
    text = render_json(name, tris) if as_json else render_text(tris)
    if out_path == '-':
        sys.stdout.write(text + "\n")
    else:
        with open(out_path, 'w') as f:
            f.write(text + "\n")
    return len(tris)


def main():
    ap = argparse.ArgumentParser(description="Dump PVR triangles (region array + object lists) to text/JSON.")
    ap.add_argument('name', nargs='?', help="dump set name (dumps/vram_<name>.bin)")
    ap.add_argument('--vram', help="explicit vram .bin path")
    ap.add_argument('--regs', help="explicit pvr_regs .bin path")
    ap.add_argument('--out', default='-', help="output file (default stdout)")
    ap.add_argument('--json', action='store_true', help="emit JSON instead of the per-line text format")
    ap.add_argument('--all', action='store_true',
                    help="process every dumps/vram_*.bin pair -> dumps/tris_<name>.{txt,json}")
    args = ap.parse_args()

    ext = 'json' if args.json else 'txt'
    d = 'dumps'
    if args.all:
        names = sorted({os.path.basename(p)[len('vram_'):-4]
                        for p in glob.glob(os.path.join(d, 'vram_*.bin'))})
        for nm in names:
            rp = os.path.join(d, f'pvr_regs_{nm}.bin')
            vp = os.path.join(d, f'vram_{nm}.bin')
            if not (os.path.exists(rp) and os.path.exists(vp)):
                continue
            try:
                op = os.path.join(d, f'tris_{nm}.{ext}')
                n = process(nm, vp, rp, op, args.json)
                print(f"  {nm:22s}: {n} triangles -> {op}")
            except Exception as ex:
                print(f"  {nm:22s}: ERROR {ex}")
        return

    if args.vram and args.regs:
        vp, rp = args.vram, args.regs
        nm = args.name or os.path.splitext(os.path.basename(vp))[0]
    elif args.name:
        nm = args.name
        vp = os.path.join(d, f'vram_{nm}.bin')
        rp = os.path.join(d, f'pvr_regs_{nm}.bin')
    else:
        ap.error("give a <name>, or --vram/--regs, or --all")

    n = process(nm, vp, rp, args.out, args.json)
    if args.out != '-':
        print(f"{nm}: {n} triangles -> {args.out}")


if __name__ == '__main__':
    main()
