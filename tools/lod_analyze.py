#!/usr/bin/env python3
# Parse [LODDUMP] lines from frontendtsplp and compute, per texture tag, the
# screen-space texel density and the refsw mip level.
#
# refsw2 (refsw_tile.cpp PixelFlush_tsp):
#   ddx  = U.ddx + V.ddx ; ddy = U.ddy + V.ddy      (interpolation plane deltas)
#   dMip = min(|ddx|,|ddy|) * W * sizeU * MipMapD/4 ; sizeU = 8<<TexU ; W = 1/invW
#   MipLevel = 0; while (dMip>1.5 && MipLevel<11) { MipLevel++; dMip/=2 }
#
# Usage: ./build/obj_frontendtsplp/tb toy_front 2>&1 | python3 tools/lod_analyze.py [tag_hex]

import sys, struct, re
from collections import defaultdict

def f32(h):
    return struct.unpack('<f', struct.pack('<I', h & 0xffffffff))[0]

def refsw_level(dMip):
    lvl = 0
    while dMip > 1.5 and lvl < 11:
        lvl += 1
        dMip /= 2.0
    return lvl, dMip

# filter by tag (arg1) and/or tcw (arg2 as tcw=HEX)
want = None
want_tcw = None
for a in sys.argv[1:]:
    if a.startswith('tcw='):
        want_tcw = int(a[4:], 16)
    else:
        want = int(a, 16)

pat = re.compile(
    r'\[LODDUMP\] tag=([0-9a-f]+) invw=([0-9a-f]+) tsp=([0-9a-f]+) tcw=([0-9a-f]+) '
    r'Ux=([0-9a-f]+) Vx=([0-9a-f]+) Uy=([0-9a-f]+) Vy=([0-9a-f]+)')

# aggregate per (tag) -> list of levels
agg = defaultdict(lambda: defaultdict(int))
n = 0
shown = 0
for line in sys.stdin:
    m = pat.search(line)
    if not m:
        continue
    tag, invw, tsp, tcw, Ux, Vx, Uy, Vy = (int(x, 16) for x in m.groups())
    if want is not None and tag != want:
        continue
    if want_tcw is not None and tcw != want_tcw:
        continue
    n += 1
    W = 1.0 / f32(invw) if f32(invw) != 0 else 1e30
    ddx = f32(Ux) + f32(Vx)
    ddy = f32(Uy) + f32(Vy)
    texu = (tsp >> 3) & 7
    mmd  = (tsp >> 8) & 0xf
    sizeU = 8 << texu
    dens_x = abs(ddx) * W * sizeU     # texels/px along x
    dens_y = abs(ddy) * W * sizeU     # texels/px along y
    dMip_min = min(abs(ddx), abs(ddy)) * W * sizeU * mmd / 4.0
    dMip_max = max(abs(ddx), abs(ddy)) * W * sizeU * mmd / 4.0
    lvl_min, _ = refsw_level(dMip_min)
    lvl_max, _ = refsw_level(dMip_max)
    agg[tag][('min', lvl_min)] += 1
    agg[tag][('max', lvl_max)] += 1
    if (want is not None or want_tcw is not None) and shown < 40:
        shown += 1
        print(f"tag={tag:08x} W={W:9.2f} dens_x={dens_x:7.2f} dens_y={dens_y:7.2f} "
              f"texu={texu} mmd={mmd} dMip_min={dMip_min:6.3f}->L{lvl_min} "
              f"dMip_max={dMip_max:6.3f}->L{lvl_max}")

print(f"\n=== parsed {n} LODDUMP lines ===")
for tag in sorted(agg):
    mins = {lvl: c for (which, lvl), c in agg[tag].items() if which == 'min'}
    maxs = {lvl: c for (which, lvl), c in agg[tag].items() if which == 'max'}
    print(f"tag={tag:08x}  refsw(min) levels={dict(sorted(mins.items()))}  "
          f"(max) levels={dict(sorted(maxs.items()))}")
