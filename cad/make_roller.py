#!/usr/bin/env python3
"""Printable roller for FTC — 350 mm drum in two halves plus a joining collar.

No CAD kernel is needed: the parts are surfaces of revolution with a hex bore,
so the mesh is generated directly and written as binary STL.

Parts
  roller_segment.stl   print two. 175 mm long, 50 mm diameter, 5.2 mm hex bore
  roller_collar.stl    print one. joins the halves, 54 mm outside diameter
"""

import math
import struct

N = 96                     # facets around the circle
SEG_LEN = 175.0            # each half; 2 x 175 = 350 mm
OUTER_R = 25.0             # 50 mm diameter
BORE_AF = 5.2              # across-flats for a 5 mm REV hex shaft
COLLAR_R = 27.0
COLLAR_LEN = 34.0
COLLAR_BORE = 25.2         # slides over the 50 mm roller


def hex_radius(theta, across_flats):
    """Radius to a regular hexagon of the given across-flats, at angle theta."""
    apothem = across_flats / 2.0
    sector = math.radians(60.0)
    a = (theta + sector / 2.0) % sector - sector / 2.0
    return apothem / math.cos(a)


def make_shell(length, outer_r, bore_af, chamfer=1.2):
    """A cylinder with a hexagonal bore, chamfered at both ends."""
    def outer(theta):
        return outer_r

    def bore(theta):
        return hex_radius(theta, bore_af)

    layers = [(0.0, outer_r - chamfer, bore_af + chamfer * 2.0),
              (chamfer, outer_r, bore_af),
              (length - chamfer, outer_r, bore_af),
              (length, outer_r - chamfer, bore_af + chamfer * 2.0)]

    rings = []
    for z, r_out, af in layers:
        top = []
        bottom = []
        for i in range(N):
            theta = 2.0 * math.pi * i / N
            r_in = hex_radius(theta, af)
            top.append((r_out * math.cos(theta), r_out * math.sin(theta), z))
            bottom.append((r_in * math.cos(theta), r_in * math.sin(theta), z))
        rings.append((top, bottom))

    tris = []
    for k in range(len(rings) - 1):
        t0, b0 = rings[k]
        t1, b1 = rings[k + 1]
        for i in range(N):
            j = (i + 1) % N
            tris.append((t0[i], t1[i], t1[j]))
            tris.append((t0[i], t1[j], t0[j]))
            tris.append((b0[i], b1[j], b1[i]))
            tris.append((b0[i], b0[j], b1[j]))

    # end caps: annulus between outer ring and bore ring
    for idx in (0, len(rings) - 1):
        top, bottom = rings[idx]
        flip = idx == 0
        for i in range(N):
            j = (i + 1) % N
            a, b = (top[i], top[j]) if not flip else (top[j], top[i])
            c, d = (bottom[i], bottom[j]) if not flip else (bottom[j], bottom[i])
            tris.append((a, c, d))
            tris.append((a, d, b))
    return tris


def write_stl(path, triangles):
    with open(path, 'wb') as fh:
        fh.write(b'nexus-roller'.ljust(80, b' '))
        fh.write(struct.pack('<I', len(triangles)))
        for a, b, c in triangles:
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            nx = u[1] * v[2] - u[2] * v[1]
            ny = u[2] * v[0] - u[0] * v[2]
            nz = u[0] * v[1] - u[1] * v[0]
            length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            fh.write(struct.pack('<3f', nx / length, ny / length, nz / length))
            for p in (a, b, c):
                fh.write(struct.pack('<3f', *p))
            fh.write(struct.pack('<H', 0))


def check_watertight(triangles):
    """Every edge must be used exactly twice, in opposite directions."""
    edges = {}
    for a, b, c in triangles:
        for p, q in ((a, b), (b, c), (c, a)):
            key = (tuple(round(v, 4) for v in p), tuple(round(v, 4) for v in q))
            rev = (key[1], key[0])
            edges[key] = edges.get(key, 0) + 1
            edges[rev] = edges.get(rev, 0) + 1
    bad = sum(1 for count in edges.values() if count != 2)
    return bad == 0, bad


def main():
    segment = make_shell(SEG_LEN, OUTER_R, BORE_AF)
    collar = make_shell(COLLAR_LEN, COLLAR_R, BORE_AF)

    # the collar is a ring around the roller, not a bored drum: rebuild it with
    # a circular bore of 50.4 mm instead of a hex one
    collar = []
    rings = []
    for z, r_out, r_in in ((0.0, COLLAR_R - 1.0, COLLAR_BORE + 2.0),
                           (1.2, COLLAR_R, COLLAR_BORE),
                           (COLLAR_LEN - 1.2, COLLAR_R, COLLAR_BORE),
                           (COLLAR_LEN, COLLAR_R - 1.0, COLLAR_BORE + 2.0)):
        top, bottom = [], []
        for i in range(N):
            theta = 2.0 * math.pi * i / N
            top.append((r_out * math.cos(theta), r_out * math.sin(theta), z))
            bottom.append((r_in * math.cos(theta), r_in * math.sin(theta), z))
        rings.append((top, bottom))

    for k in range(len(rings) - 1):
        t0, b0 = rings[k]
        t1, b1 = rings[k + 1]
        for i in range(N):
            j = (i + 1) % N
            collar.append((t0[i], t1[i], t1[j]))
            collar.append((t0[i], t1[j], t0[j]))
            collar.append((b0[i], b1[j], b1[i]))
            collar.append((b0[i], b0[j], b1[j]))
    for idx in (0, len(rings) - 1):
        top, bottom = rings[idx]
        flip = idx == 0
        for i in range(N):
            j = (i + 1) % N
            a, b = (top[i], top[j]) if not flip else (top[j], top[i])
            c, d = (bottom[i], bottom[j]) if not flip else (bottom[j], bottom[i])
            collar.append((a, c, d))
            collar.append((a, d, b))

    write_stl('roller_segment.stl', segment)
    write_stl('roller_collar.stl', collar)

    ok_a, bad_a = check_watertight(segment)
    ok_b, bad_b = check_watertight(collar)
    print(f"roller_segment.stl: {len(segment)} triangles, watertight={ok_a} (bad edges {bad_a})")
    print(f"roller_collar.stl:  {len(collar)} triangles, watertight={ok_b} (bad edges {bad_b})")


if __name__ == '__main__':
    main()
