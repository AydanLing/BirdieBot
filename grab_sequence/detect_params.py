"""Shared vision constants and pixel deprojection.

Split out of grasp_ball.py because grasp_ball imports MoveItPy at module
level. Anything that wants the detection thresholds -- the drive-by scanner,
the dev viewer -- would otherwise have to load the whole motion planner to
read five numbers. Nothing in here imports ROS.

The important content is the two threshold tiers. The same HSV blob feeds two
jobs with very different precision requirements, and using one threshold for
both is what limits detection to arm's length:

  grasp-grade   The robot is stopped at a standoff and about to close jaws with
                ~10 mm of clearance. Every millimetre matters, so the blob must
                be big enough that the median depth across it is trustworthy.
                MIN_VALID_PX = 50 is the binding constraint here, not area.

  search-grade  The robot is driving past and only needs to know that something
                yellow is roughly over there, well enough to come back to it
                later and take a proper look. Centimetres are fine.

Measured on the simulated ZED at 640x360, fx = 224, against shuttlecocks on the
court floor from the parked-arm camera height of 0.449 m:

    range   blob area   valid depth px
    1.24 m     72.5           88          passes both tiers
    1.98 m     31.0           44          search only
    2.99 m     10.5           17          search only
    3.02 m     11.0           18          search only
    3.99 m      4.0           10          too few pixels to be anything

So grasp-grade reaches ~1.5 m and search-grade ~3 m. Past 4 m a shuttlecock
stops producing a usable blob at all, whatever the thresholds say -- that is
optics, not tuning, and no amount of relaxation recovers it.
"""

import math

import numpy as np

BASE_FRAME = "base_link"
MAP_FRAME = "map"
RGB_TOPIC = "/zed/zed_node/rgb/image_rect_color"
DEPTH_TOPIC = "/zed/zed_node/depth"
CAMERA_INFO_TOPIC = "/zed/zed_node/rgb/camera_info"

LOWER_YELLOW = np.array([20, 100, 100])
UPPER_YELLOW = np.array([35, 255, 255])

# --- grasp-grade: stopped at a standoff, about to close the jaws -----------
# Pixel area, so it scales with the square of camera resolution. The ZED
# dropped from 1280x720 to 640x360 (see stereolabs_zed.urdf.xacro) to make
# the depth cloud usable as a nav2 costmap source, so this went 80 -> 20.
MIN_CONTOUR_AREA = 20.0
# Depth pixels needed before the median across the blob is worth trusting.
MIN_VALID_PX = 50

# --- search-grade: driving past, banking a coordinate for later ------------
# Both floors exist to keep the median depth honest, and a search fix does not
# need an honest median -- it needs to be within a car's length so the robot can
# come back and look properly. Dropping them from (20, 50) to (8, 10) is what
# takes the sensing radius from ~1.5 m to ~3 m. Not lower: at 4 m a shuttlecock
# is 4 px with 10 valid depth samples, which is indistinguishable from noise,
# and admitting it would fill the map with ghosts.
SEARCH_MIN_CONTOUR_AREA = 8.0
SEARCH_MIN_VALID_PX = 10

# Nothing closer than this is an object in the world rather than part of the
# robot. Shared floor for both tiers.
#
# During the carry to the hopper the gripper holds a shuttlecock roughly 100 mm
# from the lens, where it is by far the largest yellow blob in frame and would
# otherwise be treated as a target sitting inside the robot.
#
# The ceiling on this value is set by the pick, not by the carry, and it is much
# lower than it looks. At the arm's search pose the camera rides on link5 with
# the target almost underneath it: measured with a shuttlecock at a normal grasp
# distance, the range reads 0.33-0.38 m. A floor at 0.40 m therefore rejects the
# object the pick exists to find -- which it duly did, turning a working pick
# into "No reachable shuttlecock found" on every attempt, with a clean sweep of
# all five search bearings and not one sighting. 0.20 m sits comfortably between
# the 0.10 m carried object and the 0.33 m nearest real target.
MIN_DETECT_RANGE = 0.20

# The scanner's own floor, which can afford to be much higher.
#
# It only ever wants floor targets to drive to, and something 300 mm from the
# lens while the robot is moving is either part of the robot or already too
# close to be worth mapping.
SEARCH_MIN_RANGE = 0.40

# Beyond this a blob cannot be a shuttlecock the camera genuinely resolved.
MAX_DETECT_RANGE = 4.00


def deproject(k, u, v, d, ray_offset=0.0):
    """Pixel (u, v) at depth d -> a point in the camera's own link frame.

    `k` is the 9-element row-major intrinsic matrix from CameraInfo.

    rgb/depth carry frame_id "zed_camera_center", the physical link
    (REP-103: X forward, Y left, Z up), not an optical frame -- hence the sign
    flips rather than the usual optical-frame convention.

    ray_offset pushes the point further along the camera ray, used by the pick
    to aim past the near surface of an object at its centre.
    """
    fx, fy = k[0], k[4]
    cx, cy = k[2], k[5]

    x_cam = d
    y_cam = -(u - cx) / fx * d
    z_cam = -(v - cy) / fy * d

    r = math.sqrt(x_cam**2 + y_cam**2 + z_cam**2)
    if r < 1e-6:
        return None
    s = (r + ray_offset) / r
    return (x_cam * s, y_cam * s, z_cam * s)
