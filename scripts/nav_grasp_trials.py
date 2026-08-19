#!/usr/bin/env python3
"""Navigate-then-grasp: drive to a shuttlecock across the room, then pick it up.

Every grasp harness before this one teleported the object into the arm's reach
band and never moved the base, so the two halves of the robot were only ever
tested apart. This joins them: place the shuttlecock 1-2 m away, navigate to a
standoff pose, then hand off to the existing vision-based pick.

Three stages, because one is not enough:

  1. nav2 goal. Coarse. general_goal_checker runs xy_goal_tolerance 0.25 m,
     while the arm's whole reach band is 0.050..0.218 m -- 0.168 m wide. nav2
     can therefore report SUCCEEDED with the object completely out of reach.
     Treating the nav goal as sufficient would score the arm down for a
     navigation tolerance.

  2. Fine approach. Closes the residual on the AMCL estimate (map->base_link),
     driving /cmd_vel until the base is within FINE_TOL of the standoff pose.
     Deliberately uses AMCL rather than Gazebo ground truth -- ground truth
     would make the harness measure something the robot cannot actually do.
     AMCL measured ~25 mm error against truth today, well inside the band.

  3. The existing grasp sequence, unchanged, which closes the last centimetres
     on vision.

The base is parked STANDOFF m from the target along the approach heading, which
puts the object at (ARM_X + STANDOFF_REACH, 0) in base_link -- the middle of the
radius band tipped_trials.py samples, inside the narrow forward cone the claw
allows (it spans ~180 mm open, so side bearings foul the wheels and body).

Run:  python3 nav_grasp_trials.py [n_trials] [seed]
"""

import math
import os
import re
import subprocess
import sys
import time

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from repeatability_test import (ARM_X, ensure_shuttlecock, model_pose,  # noqa: E402
                                park_arm, run_grasp, set_pose, skirt_centre)

# Where the object should sit in base_link once parked: ARM_X + this. Matches
# the radius band tipped_trials.py samples (0.148..0.157).
STANDOFF_REACH = 0.152
STANDOFF = ARM_X + STANDOFF_REACH

FINE_TOL = 0.030          # m, how close the fine approach drives the base
FINE_YAW_TOL = 0.05       # rad
FINE_TIMEOUT = 40.0       # s
FINE_V = 0.12             # m/s during fine approach
FINE_W = 0.5              # rad/s
LIFT = 0.030              # m rise that counts as picked up

NAV_TIMEOUT = 120.0

# --- vision-based final correction ---------------------------------------
# The AMCL-based fine approach is not enough on its own. Measured over 10
# trials: it reported converging to 14..30 mm every time, yet four runs left
# the object 0.24..0.41 m from the arm -- outside the 0.050..0.218 m reach
# band -- and the error was lateral, putting the target at ~72 deg bearing.
# Vision and Gazebo ground truth agreed on that to within 7 mm while the fine
# approach disagreed, which means the map-frame pose it converged to was
# itself wrong. This stage never consults map: it measures the object with the
# camera and moves the base by an odom-tracked relative displacement, so
# whatever the localisation is doing cannot affect it.
VISION_TOL = 0.025        # m, accept when the object is this close to the aim point
VISION_ITERS = 3          # detect/move rounds before giving up
VISION_SETTLE = 4.0       # s; the arm trajectory is 3 s and the camera rides on it,
                          # so a shorter wait pairs a mid-swing image with a settled TF.
                          # Measured: 1.5 s gave 9 mm detection error, 4.0 s gave 5 mm.
VISION_SAMPLES = 6        # synced frames per detection, median-combined
MOVE_V = 0.10             # m/s during a relative correction
MOVE_W = 0.5              # rad/s while turning to face the target
MOVE_TIMEOUT = 20.0


def _const(name, cast=float):
    """Read a constant out of grasp_ball.py rather than keeping a second copy.

    repeatability_test._arm_x already does this for ARM_X, for the reason that
    a duplicate silently went stale once and every trial then scored the robot
    down for a harness bug. Same risk applies to the detection thresholds.
    """
    src = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "grab_sequence", "grasp_ball.py")
    with open(src) as fh:
        body = fh.read()
    m = re.search(rf"^{name}\s*=\s*(.+)$", body, re.M)
    if not m:
        raise RuntimeError(f"{name} not found in grasp_ball.py")
    return cast(m.group(1).strip())


def _hsv(name):
    m = re.search(r"\[([^\]]+)\]", _const(name, str))
    return np.array([int(v) for v in m.group(1).split(",")])


LOWER_YELLOW = _hsv("LOWER_YELLOW")
UPPER_YELLOW = _hsv("UPPER_YELLOW")
MIN_CONTOUR_AREA = _const("MIN_CONTOUR_AREA")
SEARCH_J234 = tuple(float(v) for v in
                    re.search(r"\(([^)]+)\)", _const("SEARCH_J234", str)).group(1).split(","))
SEARCH_BEARINGS = [math.radians(d) for d in (0.0, 35.0, -35.0, 70.0, -70.0)]

RGB_TOPIC = "/zed/zed_node/rgb/image_rect_color"
DEPTH_TOPIC = "/zed/zed_node/depth"
CAMERA_INFO_TOPIC = "/zed/zed_node/rgb/camera_info"


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class NavGrasp(Node):
    def __init__(self):
        super().__init__("nav_grasp_trials")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.cmd = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.initpose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # vision
        self.bridge = CvBridge()
        self.info = None
        self.frames = []
        self.collecting = False
        self.arm = self.create_publisher(
            JointTrajectory, "/manipulator_controller/joint_trajectory", 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC,
                                 self._info_cb, 10)
        rgb = message_filters.Subscriber(self, Image, RGB_TOPIC,
                                         qos_profile=qos_profile_sensor_data)
        dep = message_filters.Subscriber(self, Image, DEPTH_TOPIC,
                                         qos_profile=qos_profile_sensor_data)
        message_filters.ApproximateTimeSynchronizer(
            [rgb, dep], 10, 0.1).registerCallback(self._img_cb)

    def _info_cb(self, msg):
        self.info = msg

    def _img_cb(self, rgb, dep):
        if not self.collecting or self.info is None:
            return
        self.frames.append((rgb, dep))

    def set_arm(self, j1, j234=SEARCH_J234, secs=3):
        t = JointTrajectory()
        t.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        p = JointTrajectoryPoint()
        p.positions = [float(j1)] + [float(v) for v in j234]
        p.time_from_start.sec = secs
        t.points = [p]
        for _ in range(15):
            self.arm.publish(t)
            self.spin(0.05)

    def _deproject(self, rgb_msg, dep_msg):
        """One frame -> object position in base_link, or None.

        Mirrors grasp_ball's detector: largest yellow contour, whole-blob
        centroid, median depth across it, then deproject. rgb and depth are the
        same resolution by construction (grasp_ball indexes depth with RGB
        pixel coordinates), so the centroid transfers directly.
        """
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(dep_msg, desired_encoding="32FC1")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
        cont, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cont:
            return None
        big = max(cont, key=cv2.contourArea)
        if cv2.contourArea(big) < MIN_CONTOUR_AREA:
            return None
        blob = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(blob, [big], -1, 255, cv2.FILLED)
        valid = (blob > 0) & np.isfinite(depth) & (depth > 0.0)
        if not valid.any():
            return None
        ys, xs = np.nonzero(valid)
        u, v = float(xs.mean()), float(ys.mean())
        d = float(np.median(depth[valid]))
        k = self.info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        ps = PointStamped()
        ps.header = dep_msg.header
        ps.point.x = d
        ps.point.y = -(u - cx) / fx * d
        ps.point.z = -(v - cy) / fy * d
        try:
            tf = self.buf.lookup_transform(
                "base_link", dep_msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return None
        b = do_transform_point(ps, tf)
        return b.point.x, b.point.y

    def detect(self):
        """Median object position in base_link over VISION_SAMPLES frames."""
        self.frames = []
        self.collecting = True
        end = time.time() + 4.0
        while rclpy.ok() and len(self.frames) < VISION_SAMPLES and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.collecting = False
        pts = [p for p in (self._deproject(r, d) for r, d in self.frames) if p]
        if len(pts) < 2:
            return None
        return (float(np.median([p[0] for p in pts])),
                float(np.median([p[1] for p in pts])))

    def find_object(self):
        """Sweep joint1 until the object is seen. Returns base_link (x, y).

        A single forward look is not enough: the failures this stage exists to
        fix left the target near 72 deg bearing, past the camera's half-FOV, so
        the sweep is what makes them recoverable at all.
        """
        for b in SEARCH_BEARINGS:
            self.set_arm(b)
            self.spin(VISION_SETTLE)
            p = self.detect()
            if p is not None:
                return p
        return None

    def odom_pose(self):
        try:
            t = self.buf.lookup_transform("odom", "base_link", rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _drive(self, vx, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.linear.x = float(vx)
        m.twist.angular.z = float(wz)
        self.cmd.publish(m)

    def rotate_by(self, dtheta):
        """Turn dtheta radians, closed on odom yaw."""
        p0 = self.odom_pose()
        if p0 is None:
            return False
        goal = p0[2] + dtheta
        end = time.time() + MOVE_TIMEOUT
        while rclpy.ok() and time.time() < end:
            p = self.odom_pose()
            if p is None:
                self.spin(0.1)
                continue
            e = wrap(goal - p[2])
            if abs(e) < 0.02:
                break
            self._drive(0.0, math.copysign(min(MOVE_W, max(0.15, abs(e) * 1.2)), e))
            self.spin(0.05)
        self.stop()
        return True

    def drive_forward(self, dist):
        """Drive dist metres along +x, closed on odom position."""
        p0 = self.odom_pose()
        if p0 is None:
            return False
        gx = p0[0] + dist * math.cos(p0[2])
        gy = p0[1] + dist * math.sin(p0[2])
        end = time.time() + MOVE_TIMEOUT
        while rclpy.ok() and time.time() < end:
            p = self.odom_pose()
            if p is None:
                self.spin(0.1)
                continue
            ex, ey = gx - p[0], gy - p[1]
            # signed along the current heading, so overshoot reverses
            along = ex * math.cos(p[2]) + ey * math.sin(p[2])
            if abs(along) < 0.010:
                break
            self._drive(math.copysign(min(MOVE_V, max(0.04, abs(along) * 0.8)), along), 0.0)
            self.spin(0.05)
        self.stop()
        return True

    def move_polar(self, px, py):
        """Put the object at (ARM_X + STANDOFF_REACH, 0) by turning then driving.

        Deliberately avoids strafing. Measured on this base: a commanded +x of
        0.25 m produced +0.259 m, and rotation tracked correctly in both
        directions, but a commanded +y of 0.25 m (left) produced -0.109 m --
        wrong direction and wrong magnitude. The lateral channel of the mecanum
        controller cannot be trusted for open-loop corrections, so this uses
        only the two that behave.

        Rotation is about base_link's origin, so the heading that brings the
        object onto the +x axis is atan2(py, px) -- not measured from the arm
        mount. The arm sits on that same axis at (ARM_X, 0), so aligning to it
        aligns to the arm.
        """
        theta = math.atan2(py, px)
        rng = math.hypot(px, py)
        self.rotate_by(theta)
        self.drive_forward(rng - ARM_X - STANDOFF_REACH)
        return True

    def visual_servo(self):
        """Put the object at (ARM_X + STANDOFF_REACH, 0) in base_link.

        Returns (final_error_m, iterations, seen) for reporting.
        """
        seen = False
        err = float("nan")
        for i in range(VISION_ITERS):
            p = self.find_object()
            if p is None:
                return err, i, seen
            seen = True
            ex = p[0] - (ARM_X + STANDOFF_REACH)
            ey = p[1]
            err = math.hypot(ex, ey)
            if err < VISION_TOL:
                return err, i + 1, seen
            self.move_polar(p[0], p[1])
        return err, VISION_ITERS, seen

    def spin(self, sec):
        end = time.time() + sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def base_pose(self):
        """(x, y, yaw) of base_link in map, from AMCL. None if TF is not ready."""
        try:
            t = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def seed_amcl(self, x, y, yaw):
        """Re-seed the filter after a teleport.

        set_pose moves the body in Gazebo without producing any wheel odometry,
        so AMCL's motion model never sees the jump and settles into a wrong
        local minimum. Every trial teleports the base back to the origin, so
        without this the second trial onward navigates from a bad estimate.
        """
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.15
        cov[35] = 0.05
        m.pose.covariance = cov
        for _ in range(10):
            m.header.stamp = self.get_clock().now().to_msg()
            self.initpose.publish(m)
            self.spin(0.2)
        self.spin(2.0)

    def send_nav_goal(self, x, y, yaw):
        """Blocking NavigateToPose. Returns 'SUCCEEDED' / 'FAILED' / 'TIMEOUT'."""
        if not self.nav.wait_for_server(timeout_sec=15.0):
            return "NO_SERVER"
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = x
        g.pose.pose.position.y = y
        g.pose.pose.orientation.z = math.sin(yaw / 2)
        g.pose.pose.orientation.w = math.cos(yaw / 2)

        fut = self.nav.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if not fut.done() or fut.result() is None or not fut.result().accepted:
            return "REJECTED"
        res = fut.result().get_result_async()
        end = time.time() + NAV_TIMEOUT
        while rclpy.ok() and not res.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not res.done():
            return "TIMEOUT"
        return "SUCCEEDED" if res.result().status == 4 else "FAILED"

    def stop(self):
        for _ in range(5):
            self.cmd.publish(TwistStamped())
            self.spin(0.05)

    def fine_approach(self, gx, gy, gyaw):
        """Drive the residual nav2 left behind, closing on the AMCL estimate.

        Holonomic base, so this corrects x, y and yaw together rather than
        turning to face and driving in. Returns the final (dist, dyaw) error.
        """
        end = time.time() + FINE_TIMEOUT
        dist = dyaw = float("inf")
        while rclpy.ok() and time.time() < end:
            p = self.base_pose()
            if p is None:
                self.spin(0.1)
                continue
            bx, by, byaw = p
            ex, ey = gx - bx, gy - by
            dist = math.hypot(ex, ey)
            dyaw = wrap(gyaw - byaw)
            if dist < FINE_TOL and abs(dyaw) < FINE_YAW_TOL:
                break
            # rotate the map-frame error into the base frame
            c, s = math.cos(-byaw), math.sin(-byaw)
            fx, fy = ex * c - ey * s, ex * s + ey * c
            t = TwistStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "base_link"
            if dist > FINE_TOL:
                n = max(dist, 1e-6)
                t.twist.linear.x = FINE_V * fx / n
                t.twist.linear.y = FINE_V * fy / n
            if abs(dyaw) > FINE_YAW_TOL:
                t.twist.angular.z = math.copysign(min(FINE_W, abs(dyaw) * 1.5), dyaw)
            self.cmd.publish(t)
            self.spin(0.05)
        self.stop()
        return dist, dyaw


# --- obstacles ------------------------------------------------------------
# Deliberately absent from husarion_world.yaml, so nav2 has to discover them
# from live sensors rather than plan around known map geometry.
#
# Placed at 0.85..1.15 m, inside the annulus targets are drawn from, so they
# sit between the robot and a good fraction of the targets. They are NOT
# placed nearer the targets than OBST_CLEAR: inflation_radius is 0.70 m, so an
# obstacle close to a target would swallow the standoff pose in inflated cost
# and make the goal unplannable -- that would test nav2's tolerance for
# impossible goals, not obstacle avoidance.
#
# The last entry floats at z 0.30..0.60: invisible to the lidar plane at
# z=0.07 and detectable only through the ZED depth layer added to the local
# costmap. It is the one obstacle that exercises that layer.
OBSTACLES = [
    # x,     y,     sx,   sy,   sz,   z_centre
    (0.95,  0.30,  0.30, 0.30, 0.60, 0.30),
    (-0.30, 1.00,  0.30, 0.30, 0.60, 0.30),
    (-1.00, -0.45, 0.30, 0.30, 0.60, 0.30),
    (0.35, -1.00,  0.30, 0.30, 0.60, 0.30),
    (1.05, -0.55,  0.40, 0.60, 0.30, 0.45),   # overhang: lidar cannot see it
]
OBST_CLEAR = 0.85         # m, minimum target-to-obstacle spacing
OBST_START_CLEAR = 0.75   # m, keep the robot's start pose clear


def spawn_obstacles():
    """Put the pillars in the world, replacing any left from a previous run."""
    import tempfile
    for i, (x, y, sx, sy, sz, zc) in enumerate(OBSTACLES):
        name = f"obstacle_{i}"
        subprocess.run(
            ["gz", "service", "-s", "/world/husarion_world/remove",
             "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
             "--timeout", "4000", "--req", f'name: "{name}", type: 2'],
            capture_output=True, text=True, timeout=15)
        sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8"><model name="{name}"><static>true</static><link name="l">
<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
<material><ambient>0.7 0.25 0.2 1</ambient><diffuse>0.7 0.25 0.2 1</diffuse></material>
</visual></link></model></sdf>"""
        path = os.path.join(tempfile.gettempdir(), f"{name}.sdf")
        with open(path, "w") as fh:
            fh.write(sdf)
        subprocess.run(
            ["gz", "service", "-s", "/world/husarion_world/create",
             "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
             "--timeout", "8000", "--req",
             f'sdf_filename: "{path}", name: "{name}", '
             f"pose: {{position: {{x: {x}, y: {y}, z: {zc}}}}}"],
            capture_output=True, text=True, timeout=20)
        time.sleep(1)


def clear_of_obstacles(x, y):
    if math.hypot(x, y) < OBST_START_CLEAR:
        return False
    return all(math.hypot(x - ox, y - oy) >= OBST_CLEAR
               for ox, oy, *_ in OBSTACLES)


def target_pose(rng):
    """A spot 1-2 m from the origin, lying on its side at a random heading."""
    for _ in range(200):
        bearing = rng.uniform(-math.pi, math.pi)
        radius = rng.uniform(1.0, 2.0)
        x = radius * math.cos(bearing)
        y = radius * math.sin(bearing)
        if clear_of_obstacles(x, y):
            break
    else:
        raise RuntimeError("no clear target spot found")
    yaw = rng.uniform(-math.pi, math.pi)
    h = math.pi / -4.0                       # half of -90 deg about X: lay it down
    qx_l, qw_l = math.sin(h), math.cos(h)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return x, y, 0.033, (cy * qx_l, sy * qx_l, sy * qw_l, cy * qw_l)


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    import random
    rng = random.Random(int(sys.argv[2]) if len(sys.argv) > 2 else 11)

    rclpy.init()
    node = NavGrasp()
    ensure_shuttlecock()
    spawn_obstacles()
    results = []

    for i in range(1, n + 1):
        set_pose("rosbot", 0, 0, 0)
        time.sleep(2)
        node.seed_amcl(0.0, 0.0, 0.0)
        park_arm()
        time.sleep(2)

        tx, ty, tz, q = target_pose(rng)
        set_pose("shuttlecock", tx, ty, tz, *q)
        time.sleep(6)                        # it slides before settling

        before = model_pose("shuttlecock")
        if before is None:
            print(f"  trial {i}: fail  [shuttlecock missing]", flush=True)
            results.append(False)
            continue

        # Park STANDOFF back along the approach heading, facing the object.
        aim = skirt_centre(before)
        approach = math.atan2(aim[1], aim[0])
        gx = aim[0] - STANDOFF * math.cos(approach)
        gy = aim[1] - STANDOFF * math.sin(approach)

        t0 = time.time()
        nav = node.send_nav_goal(gx, gy, approach)
        nav_dt = time.time() - t0

        after_nav = node.base_pose()
        nav_err = (math.hypot(after_nav[0] - gx, after_nav[1] - gy)
                   if after_nav else float("nan"))

        fine_d, fine_y = node.fine_approach(gx, gy, approach)

        # Final correction on vision. Everything up to here trusts the map
        # frame; this does not.
        vis_err, vis_iters, vis_seen = node.visual_servo()
        park_arm()
        time.sleep(1)

        # Where the object actually sits relative to the arm now.
        base_truth = model_pose("rosbot")
        reach = float("nan")
        if base_truth:
            dx, dy = aim[0] - base_truth[0], aim[1] - base_truth[1]
            byaw = base_truth[5]
            c, s = math.cos(-byaw), math.sin(-byaw)
            rx, ry = dx * c - dy * s, dx * s + dy * c
            reach = math.hypot(rx - ARM_X, ry)

        log = run_grasp(f"/tmp/navgrasp_{i}.log")
        after = model_pose("shuttlecock")
        lifted = bool(after and (after[2] - before[2]) > LIFT)
        results.append(lifted)

        why = ""
        if "No reachable shuttlecock found" in log:
            why = " [not detected/unreachable]"
        elif "Arm move failed" in log:
            why = " [arm move refused]"
        vis = (f"{vis_err:.3f}m x{vis_iters}" if vis_seen else "NOT SEEN")
        print(f"  trial {i:2d}: {'PASS' if lifted else 'fail'}  "
              f"target ({tx:+.2f},{ty:+.2f}) d={math.hypot(tx,ty):.2f}m  "
              f"nav {nav:<9s} err {nav_err:.3f}  "
              f"fine {fine_d:.3f}m  vis {vis:<12s} "
              f"reach {reach:.3f}m  z {before[2]:.3f}->{after[2] if after else 0:.3f}"
              f"{why}", flush=True)

    print(f"\n  {sum(results)}/{len(results)} picked up after navigating")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
