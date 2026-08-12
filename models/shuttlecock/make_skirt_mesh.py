#!/usr/bin/env python3
"""Generate the shuttlecock's conical feather skirt as a binary STL.

sdformat 14 (Gazebo Harmonic) has no <cone> primitive, so the flare is a real
truncated cone mesh instead of the stack of cylinders it would otherwise take.
Dimensions follow BWF spec: the skirt runs from the cork (26 mm dia) out to the
feather-tip circle (65 mm dia) over 60 mm.

Solid and closed, so it stays a convex hull for collision. Units are metres.

Run:  python3 make_skirt_mesh.py
"""

import math
import os
import struct

R_BOTTOM = 0.013     # joins the 26 mm cork
R_TOP = 0.0325       # 65 mm feather-tip circle
HEIGHT = 0.060
SEGMENTS = 48

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes", "skirt.stl")


def tri(a, b, c):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    n = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return struct.pack("<12fH", nx / n, ny / n, nz / n, *a, *b, *c, 0)


def main():
    tris = []
    bottom_c = (0.0, 0.0, 0.0)
    top_c = (0.0, 0.0, HEIGHT)
    for i in range(SEGMENTS):
        t0 = 2 * math.pi * i / SEGMENTS
        t1 = 2 * math.pi * (i + 1) / SEGMENTS
        b0 = (R_BOTTOM * math.cos(t0), R_BOTTOM * math.sin(t0), 0.0)
        b1 = (R_BOTTOM * math.cos(t1), R_BOTTOM * math.sin(t1), 0.0)
        t0p = (R_TOP * math.cos(t0), R_TOP * math.sin(t0), HEIGHT)
        t1p = (R_TOP * math.cos(t1), R_TOP * math.sin(t1), HEIGHT)
        tris.append(tri(b0, b1, t1p))     # flank
        tris.append(tri(b0, t1p, t0p))
        tris.append(tri(bottom_c, b1, b0))  # bottom cap
        tris.append(tri(top_c, t0p, t1p))   # top cap

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "wb") as f:
        f.write(b"shuttlecock feather skirt, truncated cone".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            f.write(t)
    print(f"wrote {OUT}  ({len(tris)} triangles)")
    print(f"  bottom dia {R_BOTTOM * 2 * 1000:.0f} mm, top dia {R_TOP * 2 * 1000:.0f} mm, "
          f"length {HEIGHT * 1000:.0f} mm")


if __name__ == "__main__":
    main()
