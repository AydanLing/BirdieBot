#!/usr/bin/env python3
"""Generate the badminton-court world and its AMCL map.

Emits two artefacts that have to agree with each other:

  worlds/badminton_court.sdf   what Gazebo simulates
  maps/badminton_court.{pgm,yaml}   what AMCL localises against

They are generated from the same constants below rather than drawn separately,
because a map that disagrees with the world is the worst kind of bug here: AMCL
converges confidently onto the wrong pose and everything downstream inherits it.

WHY THE COURT IS INSIDE A HALL
------------------------------
A regulation court on its own is nearly invisible to this robot. The lidar sits
at z=0.234 in base_link and the court offers it almost nothing:

  * the lines are paint, with no vertical extent at all
  * the net spans roughly 0.76..1.55 m, so the beam passes underneath it
  * only the two net posts are visible, and two thin posts across 13.4 x 6.1 m
    is far too sparse for AMCL to localise against

So the court sits inside a hall. The walls are what makes localisation work,
and real courts are indoors anyway. The hall is deliberately bigger than the
court so the robot has run-off room to manoeuvre without clipping a wall.

COORDINATES
-----------
The court is centred on the origin, which matters: nav_grasp_trials samples
targets 1..2 m from the origin, so they land on the court near the net rather
than in a corner.
"""

import argparse
import os

# --- regulation badminton dimensions, metres -----------------------------
COURT_L = 13.40          # doubles length, baseline to baseline
COURT_W = 6.10           # doubles width
SINGLES_W = 5.18
SHORT_SERVICE = 1.98     # from the net
DOUBLES_LONG_SERVICE = 0.76   # in from the baseline
NET_POST_H = 1.55
NET_TOP = 1.524          # at the centre
NET_BOTTOM = 0.76
LINE_W = 0.04            # regulation line width

# --- the hall around it --------------------------------------------------
HALL_L = 18.0            # interior, x
HALL_W = 10.0            # interior, y
WALL_T = 0.20
WALL_H = 2.5

# --- map -----------------------------------------------------------------
MAP_RES = 0.05
MAP_PAD = 1.0            # metres of margin beyond the outer wall face


def box(name, sx, sy, sz, x, y, z, rgba, collide=True):
    col = f"""
        <collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>""" if collide else ""  # noqa: E501
    return f"""
      <link name="{name}">
        <pose>{x} {y} {z} 0 0 0</pose>{col}
        <visual name="v">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>
          </material>
        </visual>
      </link>"""


def line(name, sx, sy, x, y):
    """A painted court line. Visual only -- paint is not an obstacle."""
    return box(name, sx, sy, 0.002, x, y, 0.001, "0.95 0.95 0.95 1", collide=False)


def build_world():
    parts = []

    # Floor. A plane, like the original world, so the robot has something to
    # drive on beyond the court itself.
    parts.append(f"""
      <link name="floor">
        <collision name="c">
          <geometry><plane><normal>0 0 1</normal><size>{HALL_L + 4} {HALL_W + 4}</size></plane></geometry>
        </collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>{HALL_L + 4} {HALL_W + 4}</size></plane></geometry>
          <material><ambient>0.35 0.25 0.18 1</ambient><diffuse>0.42 0.30 0.20 1</diffuse></material>
        </visual>
      </link>""")

    # Hall walls. These are the only thing AMCL can see, so they are the
    # functional part of this world, not decoration.
    hx, hy = HALL_L / 2 + WALL_T / 2, HALL_W / 2 + WALL_T / 2
    wall_rgba = "0.55 0.58 0.62 1"
    parts.append(box("wall_xp", WALL_T, HALL_W + 2 * WALL_T, WALL_H, hx, 0, WALL_H / 2, wall_rgba))
    parts.append(box("wall_xn", WALL_T, HALL_W + 2 * WALL_T, WALL_H, -hx, 0, WALL_H / 2, wall_rgba))
    parts.append(box("wall_yp", HALL_L + 2 * WALL_T, WALL_T, WALL_H, 0, hy, WALL_H / 2, wall_rgba))
    parts.append(box("wall_yn", HALL_L + 2 * WALL_T, WALL_T, WALL_H, 0, -hy, WALL_H / 2, wall_rgba))

    # Court surface, a shade distinct from the floor so the GUI reads clearly.
    parts.append(box("court_surface", COURT_L, COURT_W, 0.004, 0, 0, 0.002,
                     "0.10 0.35 0.22 1", collide=False))

    # --- court markings, all visual ---
    hl, hw = COURT_L / 2, COURT_W / 2
    sw = SINGLES_W / 2
    # boundary
    parts.append(line("line_base_p", LINE_W, COURT_W, hl, 0))
    parts.append(line("line_base_n", LINE_W, COURT_W, -hl, 0))
    parts.append(line("line_side_p", COURT_L, LINE_W, 0, hw))
    parts.append(line("line_side_n", COURT_L, LINE_W, 0, -hw))
    # singles sidelines
    parts.append(line("line_singles_p", COURT_L, LINE_W, 0, sw))
    parts.append(line("line_singles_n", COURT_L, LINE_W, 0, -sw))
    # short service lines, either side of the net
    parts.append(line("line_short_p", LINE_W, COURT_W, SHORT_SERVICE, 0))
    parts.append(line("line_short_n", LINE_W, COURT_W, -SHORT_SERVICE, 0))
    # doubles long service lines
    parts.append(line("line_long_p", LINE_W, COURT_W, hl - DOUBLES_LONG_SERVICE, 0))
    parts.append(line("line_long_n", LINE_W, COURT_W, -(hl - DOUBLES_LONG_SERVICE), 0))
    # centre lines, net to short service line on each side
    seg = (hl - SHORT_SERVICE)
    parts.append(line("line_centre_p", seg, LINE_W, SHORT_SERVICE + seg / 2, 0))
    parts.append(line("line_centre_n", seg, LINE_W, -(SHORT_SERVICE + seg / 2), 0))

    # --- net ---
    # The posts ARE lidar-visible and the mesh is not, which is the whole
    # reason the hall exists. Posts get collision; the mesh gets collision too
    # so the arm cannot pass through it, but it sits above the beam.
    post_r = 0.04
    for sign, tag in ((1, "p"), (-1, "n")):
        parts.append(f"""
      <link name="net_post_{tag}">
        <pose>0 {sign * hw} {NET_POST_H / 2} 0 0 0</pose>
        <collision name="c"><geometry><cylinder><radius>{post_r}</radius><length>{NET_POST_H}</length></cylinder></geometry></collision>  # noqa: E501
        <visual name="v">
          <geometry><cylinder><radius>{post_r}</radius><length>{NET_POST_H}</length></cylinder></geometry>
          <material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
        </visual>
      </link>""")
    net_h = NET_TOP - NET_BOTTOM
    parts.append(box("net_mesh", 0.02, COURT_W, net_h, 0, 0,
                     NET_BOTTOM + net_h / 2, "0.85 0.85 0.85 0.45"))

    body = "".join(parts)
    return f"""<?xml version="1.0" ?>
<!-- Generated by scripts/make_badminton_world.py. Edit that, not this. -->
<sdf version="1.8">
  <world name="badminton_court">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact" />
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu" />
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics" />
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster" />
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors" />
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands" />

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="badminton_court">
      <static>true</static>{body}
    </model>
  </world>
</sdf>
"""


def build_map():
    """Occupancy grid of the hall walls, generated from the same constants.

    Drawn rather than SLAMmed on purpose. The wall positions are known exactly,
    so a synthesised grid is perfectly registered to the world; a SLAM map would
    carry the drift of whatever drive produced it, and any error there becomes a
    permanent localisation offset.
    """
    x_min = -(HALL_L / 2 + WALL_T + MAP_PAD)
    x_max = HALL_L / 2 + WALL_T + MAP_PAD
    y_min = -(HALL_W / 2 + WALL_T + MAP_PAD)
    y_max = HALL_W / 2 + WALL_T + MAP_PAD
    w = int(round((x_max - x_min) / MAP_RES))
    h = int(round((y_max - y_min) / MAP_RES))

    FREE, OCC = 254, 0
    grid = [[FREE] * w for _ in range(h)]

    inner_x, inner_y = HALL_L / 2, HALL_W / 2
    outer_x, outer_y = HALL_L / 2 + WALL_T, HALL_W / 2 + WALL_T
    for row in range(h):
        for col in range(w):
            # cell centre in world coords
            wx = x_min + (col + 0.5) * MAP_RES
            wy = y_min + (row + 0.5) * MAP_RES
            in_outer = abs(wx) <= outer_x and abs(wy) <= outer_y
            in_inner = abs(wx) < inner_x and abs(wy) < inner_y
            if in_outer and not in_inner:
                grid[row][col] = OCC

    # PGM rows run top-down; the map frame's +y runs up, so flip.
    rows = [bytes(grid[h - 1 - r]) for r in range(h)]
    pgm = b"P5\n" + f"{w} {h}\n255\n".encode() + b"".join(rows)
    yaml = (f"---\nimage: badminton_court.pgm\nmode: trinary\n"
            f"resolution: {MAP_RES}\norigin: [{x_min}, {y_min}, 0]\n"
            f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return pgm, yaml, w, h, x_min, y_min


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--worlds-dir", default=None)
    ap.add_argument("--maps-dir", default=None)
    a = ap.parse_args()
    base = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..",
                                        "husarion_gz_worlds"))
    worlds = a.worlds_dir or os.path.join(base, "worlds")
    maps = a.maps_dir or os.path.join(base, "maps")

    wpath = os.path.join(worlds, "badminton_court.sdf")
    with open(wpath, "w") as fh:
        fh.write(build_world())
    print(f"  wrote {wpath} ({os.path.getsize(wpath)} bytes)")

    pgm, yaml, w, h, ox, oy = build_map()
    with open(os.path.join(maps, "badminton_court.pgm"), "wb") as fh:
        fh.write(pgm)
    with open(os.path.join(maps, "badminton_court.yaml"), "w") as fh:
        fh.write(yaml)
    print(f"  wrote map {w}x{h} px at {MAP_RES} m/px, origin ({ox}, {oy})")
    print(f"  hall interior {HALL_L} x {HALL_W} m, court {COURT_L} x {COURT_W} m")


if __name__ == "__main__":
    main()
