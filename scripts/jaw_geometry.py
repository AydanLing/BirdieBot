#!/usr/bin/env python3
"""Report the real jaw geometry straight from the xacro. No simulator needed.

Tuning gripper geometry by eye in RViz is slow and misleading: RViz draws the
visual mesh unless "Collision Enabled" is ticked, and the number that decides
whether a grasp works -- how far the jaws actually open -- is not something you
can eyeball. Rotated collision pieces are the specific trap. A 75 mm pad turned
60 deg moves its corners about 20 mm inboard of where its thickness suggests,
which once left the jaws opening 34.8 mm against a 65 mm target.

Run after editing body.xacro, before rebuilding or launching anything:

    python3 jaw_geometry.py

Prints the opening, the closed gap, floor clearance and the GRASP_OFFSET that
grasp_ball.py should be using.
"""

import itertools
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

TARGET_WIDTH = 0.065        # shuttlecock skirt at its rim
TARGET_CORK = 0.026         # cork base
FINGER_MOUNT_X = 0.0817     # finger joint offset down the tool from link5


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def render():
    share = subprocess.run(["ros2", "pkg", "prefix", "rosbot_description"],
                           capture_output=True, text=True).stdout.strip()
    xacro_path = os.path.join(share, "share", "rosbot_description", "urdf",
                              "rosbot_xl.urdf.xacro")
    comp = os.path.join(share, "share", "rosbot_description", "config",
                        "rosbot_xl", "manipulation_pro.yaml")
    out = subprocess.run(
        ["xacro", xacro_path, f"components_config:={comp}",
         "configuration:=manipulation_pro", "use_sim:=true"],
        capture_output=True, text=True,
    )
    if out.returncode != 0:
        print(out.stderr[:2000])
        sys.exit(1)
    return ET.fromstring(out.stdout)


def corners(collision):
    """Corner offsets of one collision piece, in the finger link frame."""
    origin = collision.find("origin")
    ox, oy, oz = [float(v) for v in (origin.get("xyz") or "0 0 0").split()]
    rot = rpy_matrix(*[float(v) for v in (origin.get("rpy") or "0 0 0").split()])

    box = collision.find(".//box")
    cyl = collision.find(".//cylinder")
    if box is not None:
        hx, hy, hz = [float(v) / 2 for v in box.get("size").split()]
    elif cyl is not None:
        r = float(cyl.get("radius"))
        hx = hy = r
        hz = float(cyl.get("length")) / 2
    else:
        return None                      # a mesh: extents are not in the URDF

    pts = []
    for sx, sy, sz in itertools.product((-1, 1), repeat=3):
        local = (sx * hx, sy * hy, sz * hz)
        pts.append(tuple(
            (ox, oy, oz)[i] + sum(rot[i][k] * local[k] for k in range(3))
            for i in range(3)
        ))
    return pts


def main():
    root = render()
    joint = root.find('.//joint[@name="gripper_left_joint"]')
    mount_y = float(joint.find("origin").get("xyz").split()[1])
    q_open = float(joint.find("limit").get("upper"))
    q_close = float(joint.find("limit").get("lower"))

    link = root.find('.//link[@name="gripper_left_link"]')
    pieces, meshes = [], 0
    for c in link.findall("collision"):
        pts = corners(c)
        if pts is None:
            meshes += 1
        else:
            pieces.append(pts)

    if meshes:
        print(f"  NOTE: {meshes} collision piece(s) are meshes. Their extents are "
              "not in the URDF,\n        so they are not counted below. Check them "
              "in Gazebo with collisions shown.")
    if not pieces:
        print("  No primitive collision geometry found on the jaw.")
        return

    flat = [p for piece in pieces for p in piece]
    inner = min(abs(mount_y + q_open + p[1]) for p in flat)
    inner_c = min(abs(mount_y + q_close + p[1]) for p in flat)
    x_lo = min(p[0] for p in flat)
    x_hi = max(p[0] for p in flat)

    print()
    print(f"  jaw opening (q={q_open:+.3f})   {1000 * 2 * inner:6.1f} mm")
    print(f"  closed gap  (q={q_close:+.3f})   {1000 * 2 * inner_c:6.1f} mm")
    print(f"  grip surfaces span x {1000 * x_lo:.1f} .. {1000 * x_hi:.1f} mm "
          "below the finger mount")
    print()

    ok = True
    if 2 * inner <= TARGET_WIDTH:
        print(f"  FAIL  opens {1000 * 2 * inner:.1f} mm but the skirt is "
              f"{1000 * TARGET_WIDTH:.0f} mm -- the jaws cannot fit around it "
              "and will knock it aside.")
        ok = False
    else:
        print(f"  ok    clears the {1000 * TARGET_WIDTH:.0f} mm skirt by "
              f"{1000 * (2 * inner - TARGET_WIDTH) / 2:.1f} mm a side")
    if 2 * inner_c >= TARGET_WIDTH:
        print(f"  FAIL  closes only to {1000 * 2 * inner_c:.1f} mm, so it can "
              "never clamp the target.")
        ok = False
    else:
        print(f"  ok    closes to {1000 * 2 * inner_c:.1f} mm, tight enough to "
              f"grip (cork is {1000 * TARGET_CORK:.0f} mm)")

    centre_x = 0.5 * (x_lo + x_hi)
    print()
    print(f"  GRASP_OFFSET should be {FINGER_MOUNT_X:.4f} + {centre_x:.4f} "
          f"= {FINGER_MOUNT_X + centre_x:.4f}")
    print("  (set this in grab_sequence/grasp_ball.py; nothing checks it for you)")
    print()
    print("  " + ("all good" if ok else "fix the FAILs before launching"))


if __name__ == "__main__":
    main()
