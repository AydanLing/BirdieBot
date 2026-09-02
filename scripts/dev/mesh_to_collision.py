#!/usr/bin/env python3
"""Turn a concave claw mesh into axis-aligned collision boxes for URDF.

A concave gripper cannot be used directly as a collision mesh: the physics
engine takes the convex hull, which fills the pocket in and turns a wrap into a
flat paddle. It fails silently -- the object just gets pushed away.

So voxelise the solid and cover the occupied voxels with axis-aligned boxes.
Voxels respect concavity, so the pocket stays empty, and axis-aligned boxes need
no rotation, which avoids the rotated-corner trap that once left the jaws
opening 34.8 mm against a 65 mm target.

The mesh axes are remapped into the finger link frame on the way through:
    finger_x = -mesh_z   (down the tool; mesh mounts at z=0 and hangs to -80)
    finger_y =  mesh_x   (closing direction)
    finger_z = -mesh_y   (along the shuttlecock's axis)

The y sign matters and is easy to get backwards. Measured from the voxels, the
grip pocket of Gripper-Left.stl sits at -X of its material (pocket centroid
X=+13.3 mm, material centroid X=+29.0 mm). finger_y = +mesh_x therefore points
that pocket at the centreline; the opposite sign aims it outward, which shows up
as the left and right claws looking swapped. finger_z is negated so the mapping
stays a proper rotation (determinant +1) rather than a reflection.
"""

import struct
import sys

import numpy as np

MM = 0.001


def load_triangles(path):
    """Triangles from a *binary* STL, checked against the declared count.

    The size check is the whole point. An ASCII STL, or a truncated download,
    puts arbitrary bytes at offset 80, and the old version read that as a
    triangle count and marched off the end of the buffer or allocated something
    absurd. Worse, a count that is merely too small parses cleanly and yields a
    partial mesh, so the boxes below would cover part of the claw and the URDF
    would look plausible while missing geometry -- exactly the kind of quiet
    wrong answer this script exists to prevent.

    A binary STL is 84 + 50n bytes exactly, so the count and the file length
    have to agree. (Testing for a leading "solid" is not sufficient: plenty of
    binary STLs carry that word in their 80-byte header.)
    """
    data = open(path, "rb").read()
    if len(data) < 84:
        raise ValueError(f"{path}: {len(data)} bytes, too short to be an STL")
    n = struct.unpack("<I", data[80:84])[0]
    expected = 84 + 50 * n
    if len(data) != expected:
        hint = (" -- looks like an ASCII STL; convert it to binary first"
                if data[:5].lower() == b"solid" else "")
        raise ValueError(
            f"{path}: header declares {n} triangles, which needs {expected} "
            f"bytes, but the file is {len(data)}{hint}")
    tris = np.empty((n, 3, 3), dtype=np.float64)
    off = 84
    for i in range(n):
        v = struct.unpack("<12f", data[off:off + 48])
        tris[i] = np.array(v[3:12]).reshape(3, 3)
        off += 50
    if not np.isfinite(tris).all():
        raise ValueError(f"{path}: contains non-finite vertex coordinates")
    return tris


def to_finger_frame(v):
    """mesh (x,y,z) -> finger link frame, still in mm."""
    out = np.empty_like(v)
    out[..., 0] = -v[..., 2]
    out[..., 1] = v[..., 0]
    out[..., 2] = -v[..., 1]
    return out


def voxelise(tris, pitch):
    """Occupancy grid by ray parity along +x, one ray per (y,z) cell centre."""
    lo = tris.reshape(-1, 3).min(0) - pitch
    hi = tris.reshape(-1, 3).max(0) + pitch
    dims = np.maximum(((hi - lo) / pitch).astype(int) + 1, 1)

    ys = lo[1] + (np.arange(dims[1]) + 0.5) * pitch
    zs = lo[2] + (np.arange(dims[2]) + 0.5) * pitch
    grid = np.zeros(dims, dtype=bool)

    a, b, c = tris[:, 0, :], tris[:, 1, :], tris[:, 2, :]
    e1, e2 = b - a, c - a
    # Moller-Trumbore, vectorised over triangles for a ray along +x.
    for iy, y in enumerate(ys):
        for iz, z in enumerate(zs):
            origin = np.array([lo[0] - 1.0, y, z])
            d = np.array([1.0, 0.0, 0.0])
            h = np.cross(d, e2)
            det = np.einsum("ij,ij->i", e1, h)
            ok = np.abs(det) > 1e-12
            if not ok.any():
                continue
            inv = np.zeros_like(det)
            inv[ok] = 1.0 / det[ok]
            s = origin - a
            u = np.einsum("ij,ij->i", s, h) * inv
            q = np.cross(s, e1)
            v = np.einsum("j,ij->i", d, q) * inv
            t = np.einsum("ij,ij->i", e2, q) * inv
            hit = ok & (u >= 0) & (u <= 1) & (v >= 0) & (u + v <= 1) & (t > 0)
            if not hit.any():
                continue
            xs = np.sort(origin[0] + t[hit])
            # parity: between crossing 0-1, 2-3, ... is inside
            for k in range(0, len(xs) - 1, 2):
                i0 = int(np.ceil((xs[k] - lo[0]) / pitch - 0.5))
                i1 = int(np.floor((xs[k + 1] - lo[0]) / pitch - 0.5))
                if i1 >= i0:
                    grid[max(i0, 0):i1 + 1, iy, iz] = True
    return grid, lo


def greedy_boxes(grid, lo, pitch):
    """Cover occupied voxels with as few axis-aligned boxes as practical."""
    remaining = grid.copy()
    boxes = []
    while remaining.any():
        idx = np.argwhere(remaining)
        i, j, k = idx[0]
        # grow along each axis while the whole slab stays occupied
        i1 = i
        while i1 + 1 < remaining.shape[0] and remaining[i1 + 1, j, k]:
            i1 += 1
        j1 = j
        while (j1 + 1 < remaining.shape[1]
               and remaining[i:i1 + 1, j1 + 1, k].all()):
            j1 += 1
        k1 = k
        while (k1 + 1 < remaining.shape[2]
               and remaining[i:i1 + 1, j:j1 + 1, k1 + 1].all()):
            k1 += 1
        remaining[i:i1 + 1, j:j1 + 1, k:k1 + 1] = False
        centre = lo + (np.array([i + i1 + 1, j + j1 + 1, k + k1 + 1]) / 2.0) * pitch
        size = (np.array([i1 - i + 1, j1 - j + 1, k1 - k + 1])) * pitch
        boxes.append((centre, size))
    return boxes


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "Gripper-Left.stl"
    pitch = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
    min_vox = int(sys.argv[3]) if len(sys.argv) > 3 else 2

    try:
        tris = to_finger_frame(load_triangles(path))
    except (OSError, ValueError) as e:
        print(f"  {e}")
        sys.exit(1)
    grid, lo = voxelise(tris, pitch)
    filled = int(grid.sum())
    all_boxes = greedy_boxes(grid, lo, pitch)
    boxes = [b for b in all_boxes if np.prod(b[1]) >= min_vox * pitch ** 3]

    print(f"  {path}: {len(tris)} triangles, voxel pitch {pitch:.1f} mm")
    print(f"  filled voxels {filled}  ->  {len(boxes)} boxes "
          f"(dropped slivers under {min_vox} voxels)")
    # Producing no output at all is a plausible-looking result: you paste
    # nothing into the URDF, the jaw ends up with no collision geometry, and the
    # object is pushed away exactly as it is with a hull. So say so, loudly, and
    # exit non-zero rather than printing an empty list.
    if not boxes:
        print(f"  nothing to emit: {filled} filled voxels, "
              f"{len(all_boxes)} raw boxes, all below the {min_vox}-voxel "
              "threshold. Lower the pitch or min_vox; do not paste an empty "
              "collision set into the URDF.")
        sys.exit(1)
    solid = filled * pitch ** 3
    bbox = float(np.prod(tris.reshape(-1, 3).max(0) - tris.reshape(-1, 3).min(0)))
    fill = f"{100*solid/bbox:.0f}% fill" if bbox > 0 else "degenerate bbox"
    print(f"  solid {solid/1000:.1f} cm^3 vs bbox {bbox/1000:.1f} cm^3 "
          f"({fill} -- a hull would report ~100%)")
    print()
    for centre, size in sorted(boxes, key=lambda b: -np.prod(b[1])):
        c = centre * MM
        s = size * MM
        print('      <collision>')
        print(f'        <origin xyz="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}" rpy="0 0 0"/>')
        print(f'        <geometry><box size="{s[0]:.4f} {s[1]:.4f} {s[2]:.4f}"/></geometry>')
        print('      </collision>')


if __name__ == "__main__":
    main()
