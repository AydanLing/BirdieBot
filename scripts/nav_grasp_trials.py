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

import atexit
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
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
from repeatability_test import (ARM_X, UNKNOWN, GzQueryFailed,  # noqa: E402
                                WorldMismatch, classify, ensure_shuttlecock,
                                gz_run, model_list, park_arm, probe_pose,
                                remove_model, run_grasp, set_pose, skirt_centre,
                                spawn_model, summarise)

# Where the object should sit in base_link once parked: ARM_X + this. Matches
# the radius band tipped_trials.py samples (0.148..0.157).
STANDOFF_REACH = 0.152
STANDOFF = ARM_X + STANDOFF_REACH

FINE_TOL = 0.030          # m, how close the fine approach drives the base
FINE_YAW_TOL = 0.05       # rad
FINE_TIMEOUT = 40.0       # s
# Raised from 0.12. The fine approach only has to cover nav2's residual, which
# lands at 0.245..0.250 m almost every trial (its xy_goal_tolerance), so this is
# a fixed ~0.25 m of driving per trial and the speed is pure overhead.
FINE_V = 0.25             # m/s during fine approach
FINE_W = 1.0              # rad/s
# The lift threshold and the PASS/FAIL/INDETERMINATE rules live in
# repeatability_test.classify(), shared rather than duplicated here. A local
# copy of LIFT already existed and could drift from the one the other harnesses
# score against, which is the same trap _arm_x() exists to avoid.

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
VISION_SETTLE = 2.0       # s to let the arm stop before believing a frame.
                          # Was 4.0, and it is the single largest cost in a
                          # trial: every detection attempt pays one arm move
                          # plus one settle, for up to 5 search bearings and up
                          # to VISION_ITERS rounds.
                          #
                          # Measured detection error against settle time:
                          # 1.5 s -> 9 mm, 4.0 s -> 5 mm. VISION_TOL is 25 mm,
                          # so 4 s was buying 4 mm of accuracy inside a budget
                          # that never needed it. 2.0 s keeps a margin over the
                          # (now 1.5 s) arm trajectory so a mid-swing image is
                          # still never paired with a settled TF.
VISION_SAMPLES = 6        # synced frames per detection, median-combined
MOVE_V = 0.22             # m/s during a relative correction
MOVE_W = 1.0              # rad/s while turning to face the target
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

    def set_arm(self, j1, j234=SEARCH_J234, secs=2):
        """Command the arm. secs must stay BELOW VISION_SETTLE.

        The camera rides on link5, so a frame captured mid-swing paired with a
        settled TF lookup deprojects to the wrong place. Trajectory 2 s against
        a 2.0 s settle leaves the margin thin but real; do not raise this
        without raising VISION_SETTLE with it.
        """
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

        Turns to face the goal, drives to it, then turns to the goal heading.
        It does NOT strafe, even though the base is nominally holonomic.

        This used to command linear.y directly, on the reasoning that a mecanum
        base can correct x and y at once. Measured against ground truth with
        /cmd_vel confirmed silent at idle, that lateral term is nearly inert:

            commanded +x 0.15 m/s for 6 s  ->  travelled 0.839 m   (93%)
            commanded +y 0.15 m/s for 6 s  ->  travelled 0.024 m   (3%)

        Nothing is misconfigured. The controller emits textbook strafe wheel
        velocities (FL -3.061, FR +3.061, RL +3.061, RR -3.061 rad/s), and the
        URDF and converted SDF both carry the mecanum surface friction, all four
        fdir1 roller diagonals intact. The friction direction and mu2 are ODE
        parameters, and gz-sim runs DART by default, whose contact model does
        not apply them -- so friction ends up isotropic, the four wheels' lateral
        components cancel, and the base slips instead of strafing. A real ROSbot
        XL has physical rollers and would strafe; this is a simulation limit.

        So the lateral command was not correcting anything, and convergence came
        from the forward and rotational terms regardless. move_polar next door
        already turns and drives for the same reason, which is why the vision
        stage works. Returns the final (dist, dyaw) error.
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

            t = TwistStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "base_link"

            if dist >= FINE_TOL:
                # Face the goal, then drive at it. If it is behind, back up
                # rather than spinning 180 degrees for a few centimetres.
                heading_err = wrap(math.atan2(ey, ex) - byaw)
                reverse = abs(heading_err) > math.pi / 2
                if reverse:
                    heading_err = wrap(heading_err + math.pi)
                if abs(heading_err) > 0.15:
                    t.twist.angular.z = math.copysign(
                        min(FINE_W, max(0.15, abs(heading_err) * 1.5)), heading_err)
                else:
                    v = min(FINE_V, max(0.04, dist * 0.8))
                    t.twist.linear.x = -v if reverse else v
                    t.twist.angular.z = heading_err * 1.0   # trim while driving
            else:
                # In position; settle the final heading.
                t.twist.angular.z = math.copysign(
                    min(FINE_W, max(0.15, abs(dyaw) * 1.5)), dyaw)

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

# Plan-view radius of the footprint, matching robot_radius in both costmaps and
# ROBOT_RADIUS in cmd_vel_guard.py.
ROBOT_RADIUS = 0.22
# Height of the tallest thing on the robot with the arm parked. Not measured --
# it is a rough envelope, used only to decide whether the floating obstacle is
# in the way at all. The contact sensors below are the authority on whether the
# robot actually hit something; this is a cross-check, not a substitute.
ROBOT_TOP_Z = 0.35


def obstacle_names():
    return [f"obstacle_{i}" for i in range(len(OBSTACLES))]


def obstacle_sdf(name, sx, sy, sz):
    """Static box. Collision evidence comes from ClearanceMonitor, not a sensor.

    These obstacles are <static>true</static>, so an earlier check that
    compared their poses before and after a run and concluded "unmoved,
    therefore no collision" could not ever have failed: a static body cannot be
    pushed, so it reports "no collision" whether the robot grazed it, drove
    into it, or never went near it. That check is gone.

    It was replaced by a contact sensor on this collision, which ALSO did not
    work, for a different reason: a contact sensor on a model spawned at
    RUNTIME is never instantiated, so its topic never appears in `gz topic -l`.
    Verified static and non-static, with and without an explicit <topic>, in a
    world that does load gz-sim-contact-system. The sensor has been removed
    rather than left in looking functional.

    ClearanceMonitor instead samples the robot's ground-truth pose from
    /world/<world>/dynamic_pose/info for the whole trial and reports the
    minimum footprint-to-box gap, which also catches near misses.
    """
    return f"""<?xml version="1.0" ?>
<sdf version="1.8"><model name="{name}"><static>true</static><link name="l">
<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
<material><ambient>0.7 0.25 0.2 1</ambient><diffuse>0.7 0.25 0.2 1</diffuse></material>
</visual>
</link></model></sdf>"""


def spawn_obstacles(force=True, present=None):
    """Put the pillars in the world and verify each one actually arrived.

    The old version fired remove and create at gz and never looked at the
    result. Both calls report success unconditionally -- a remove with the
    wrong entity type no-ops, and a create naming a missing sdf spawns nothing
    -- so a run could and did measure an empty scene while printing obstacle
    avoidance results. remove_model/spawn_model read the world back with
    `gz model --list` and raise if it does not match.

    force=False only replaces the obstacles that are missing, which is the
    per-trial mode.

    Returns the list of names it had to (re)spawn.
    """
    names = model_list() if present is None else present
    respawned = []
    for i, (x, y, sx, sy, sz, zc) in enumerate(OBSTACLES):
        name = f"obstacle_{i}"
        if not force and name in names:
            continue
        path = os.path.join(tempfile.gettempdir(), f"{name}.sdf")
        with open(path, "w") as fh:
            fh.write(obstacle_sdf(name, sx, sy, sz))
        remove_model(name)
        spawn_model(name, path, x, y, zc, settle=1.5)
        respawned.append(name)
    return respawned


def obstacle_clearance(bx, by):
    """Smallest plan-view gap between the footprint and any obstacle box.

    Negative means the footprint overlaps a box that the robot cannot be
    passing over or under, i.e. the base is inside something. Cheap
    cross-check on the contact sensors: it uses Gazebo ground truth for the
    base, so unlike anything AMCL-derived it cannot be fooled by localisation
    error, but it only sees the poses it is asked about rather than the whole
    trajectory.

    Obstacles whose z span misses the robot's [0, ROBOT_TOP_Z] envelope are
    skipped -- the floating one at z 0.30..0.60 is meant to be avoided by the
    ZED costmap layer, and counting a plan-view overlap with it as contact
    would flag runs where the robot legitimately passed underneath.
    """
    best = float("inf")
    for ox, oy, sx, sy, sz, zc in OBSTACLES:
        if zc - sz / 2 >= ROBOT_TOP_Z or zc + sz / 2 <= 0.0:
            continue
        # distance from a point to an axis-aligned box, in plan view
        dx = max(abs(bx - ox) - sx / 2, 0.0)
        dy = max(abs(by - oy) - sy / 2, 0.0)
        best = min(best, math.hypot(dx, dy) - ROBOT_RADIUS)
    return best


class ClearanceMonitor:
    """Streams the robot's ground-truth pose and tracks its closest approach.

    This replaces a contact-sensor monitor that never worked. The sensors were
    valid, the world loads gz-sim-contact-system, and an explicit <topic> was
    tried -- but a contact sensor attached to a model spawned at RUNTIME is
    never instantiated, so the topic simply never appears in `gz topic -l`.
    Verified both static and non-static, and with an explicit topic name. The
    old monitor therefore reported "unavailable" on every run, which was honest
    but useless.

    /world/<world>/dynamic_pose/info does work: it streams gz.msgs.Pose_V for
    every moving entity, `rosbot` included, for as long as the simulator runs.
    The obstacles are static and absent from it, but their poses are known --
    this harness placed them.

    Sampling that stream the whole trial gives the MINIMUM clearance between
    the robot's footprint and any obstacle box, which is strictly more useful
    than a contact boolean: it distinguishes a clean run from a near miss, and
    a negative value is a genuine overlap.

    Still three-state for the same reason as before. If the stream produces no
    usable samples, result() returns None -- unavailable -- rather than a
    comfortable number. A check that cannot fail is worse than no check, which
    is the trap the original static-pose comparison fell into.
    """

    TOPIC = "/world/husarion_world/dynamic_pose/info"

    def __init__(self, names):
        self.names = list(names)
        self.proc = None
        self.path = None
        self.fh = None
        self.reason = ""
        self.samples = 0
        atexit.register(self.stop)

    @property
    def available(self):
        return True

    def start(self):
        try:
            fd, self.path = tempfile.mkstemp(prefix="clearance_", suffix=".txt")
            self.fh = os.fdopen(fd, "w")
            self.proc = subprocess.Popen(
                ["stdbuf", "-oL", "gz", "topic", "-e", "-t", self.TOPIC],
                stdout=self.fh, stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            return True
        except FileNotFoundError:
            self.reason = "`gz topic` or `stdbuf` is not on PATH"
            self.proc = None
            return False

    def stop(self):
        """Returns the minimum clearance in metres, or None if unavailable."""
        if self.proc is None:
            return None
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                self.proc.wait(timeout=5)
        except (ProcessLookupError, PermissionError):
            pass
        self.proc = None
        try:
            self.fh.close()
        except Exception:
            pass
        try:
            with open(self.path, errors="ignore") as fh:
                body = fh.read()
            os.unlink(self.path)
        except OSError:
            self.reason = "clearance stream could not be read back"
            return None
        return self._min_clearance(body)

    def _min_clearance(self, body):
        """Smallest footprint-to-box gap seen across the streamed poses.

        gz prints Pose_V as repeated `pose { name: ... position { x: y: z: } }`
        blocks, and the robot's LINKS appear alongside the model, so this keys
        strictly on the model name and takes the x/y that follow it.
        """
        best = None
        lines = body.splitlines()
        for i, line in enumerate(lines):
            if line.strip() != 'name: "rosbot"':
                continue
            x = y = None
            for probe in lines[i + 1:i + 12]:
                t = probe.strip()
                if t.startswith("x:") and x is None:
                    x = float(t.split(":", 1)[1])
                elif t.startswith("y:") and y is None:
                    y = float(t.split(":", 1)[1])
                elif t.startswith("orientation"):
                    break
            if x is None or y is None:
                continue
            self.samples += 1
            c = obstacle_clearance(x, y)
            if c is not None and (best is None or c < best):
                best = c
        if not self.samples:
            self.reason = ("no `rosbot` poses parsed from the stream; the "
                           "simulator may have been paused or restarted")
        return best


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


NAV2_NODES = ("bt_navigator", "planner_server", "controller_server",
              "behavior_server", "velocity_smoother")


def _activate_nav2_nodes(names):
    """Drive stalled nav2 nodes to active, returning the ones that would not go.

    nav2's lifecycle manager regularly gives up part way through bringup and
    leaves a split state -- observed on a freshly rebooted machine at load ~17
    with controller_server and smoother_server active while bt_navigator,
    behavior_server, planner_server and velocity_smoother sat inactive. The
    node logs show why: a node's change_state response does not arrive in time
    ("failed to send response to /planner_server/change_state (timeout)"), the
    manager treats the transition as failed, and the sequence stops there.

    It is recoverable every time. `ros2 lifecycle set /<node> activate` brought
    all four up without relaunching anything. Doing that by hand cost two full
    ten-trial runs to notice, once as NO_SERVER and once as REJECTED, so it is
    done here instead.

    A node in `unconfigured` needs configure before activate; one in `inactive`
    needs only activate. Both are attempted in order and the state is re-read
    rather than trusting the set command's exit code.
    """
    stuck = []
    for name in names:
        for transition in ("configure", "activate"):
            state = _lifecycle_state(name)
            if _is_active(state):
                break
            if transition == "configure" and "unconfigured" not in state:
                continue
            subprocess.run(["ros2", "lifecycle", "set", f"/{name}", transition],
                           capture_output=True, text=True, timeout=30)
        if not _is_active(_lifecycle_state(name)):
            stuck.append(f"{name}({_lifecycle_state(name) or 'no reply'})")
    return stuck


def _lifecycle_state(name):
    """The bare state word from `ros2 lifecycle get`, e.g. "active"."""
    try:
        out = subprocess.run(["ros2", "lifecycle", "get", f"/{name}"],
                             capture_output=True, text=True, timeout=15)
        return out.stdout.strip()
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return ""


def _is_active(state):
    """True only for the active state.

    Emphatically NOT `"active" in state`. `ros2 lifecycle get` prints
    "inactive [2]" for a stalled node, and "active" is a substring of
    "inactive", so the obvious membership test reports every stalled node as
    healthy. That bug shipped in the first version of this check and made it
    useless: the harness ran against a stack with bt_navigator, behavior_server
    and velocity_smoother all inactive, reported nothing, and every goal came
    back REJECTED.
    """
    return state.split()[0] == "active" if state.split() else False


def _inactive_nav2_nodes():
    """Names of the nav2 lifecycle nodes this harness needs that are not active.

    Uses the CLI rather than lifecycle service clients: it is a once-per-run
    check, and shelling out keeps this free of extra rclpy plumbing.
    """
    bad = []
    for name in NAV2_NODES:
        try:
            out = subprocess.run(["ros2", "lifecycle", "get", f"/{name}"],
                                 capture_output=True, text=True, timeout=15)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            bad.append(f"{name}(unreachable)")
            continue
        if not _is_active(out.stdout.strip()):
            bad.append(f"{name}({out.stdout.strip() or 'no reply'})")
    return bad


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    import random
    rng = random.Random(int(sys.argv[2]) if len(sys.argv) > 2 else 11)

    rclpy.init()
    node = NavGrasp()

    # Pre-flight: refuse to start without the navigation action server.
    #
    # nav2's lifecycle manager can leave bt_navigator and behavior_server stuck
    # "unconfigured" when transitions time out under load, and the failure is
    # quiet -- planner_server and controller_server can report active while
    # /navigate_to_pose is not advertised at all. A full 10-trial run was spent
    # that way: every goal returned NO_SERVER, every trial scored
    # INDETERMINATE, and roughly fifteen minutes of simulator time measured
    # nothing. The per-trial handling was correct, but it should not take ten
    # trials to learn the stack is not up.
    #
    # Checking planner_server is NOT sufficient; check the server this harness
    # actually calls.
    # wait_for_server is necessary but NOT sufficient. bt_navigator advertises
    # /navigate_to_pose while still inactive and then rejects every goal with
    # "Action server is inactive. Rejecting the goal." -- a whole run was spent
    # that way, all ten trials REJECTED. Check the lifecycle state too.
    inactive = _inactive_nav2_nodes()
    if inactive:
        # Recover rather than abort. This state is normal on this machine and
        # always recoverable; aborting just moves the manual step to the user.
        print(f"  nav2 nodes not active: {', '.join(inactive)} -- activating",
              flush=True)
        inactive = _activate_nav2_nodes(NAV2_NODES)
    if inactive:
        print(f"  ABORT: nav2 nodes would not activate: {', '.join(inactive)}.\n"
              "    The lifecycle manager stalls partway under load and leaves\n"
              "    the action server advertised but inactive, which passes a\n"
              "    wait_for_server check and then rejects every goal.\n"
              "    Fix with:  ros2 lifecycle set /<node> activate\n"
              "    or relaunch navigation_launch.py on a quieter machine.",
              flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1

    if not node.nav.wait_for_server(timeout_sec=20.0):
        print("  ABORT: /navigate_to_pose is not available after 20 s.\n"
              "    Check `ros2 lifecycle get /bt_navigator` -- if it reports\n"
              "    unconfigured or inactive, the lifecycle manager did not\n"
              "    finish. Relaunch navigation_launch.py once the machine is\n"
              "    quieter and wait for /navigate_to_pose in `ros2 action list`.",
              flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1

    ensure_shuttlecock(force=True)
    spawn_obstacles(force=True)

    contacts = ClearanceMonitor(obstacle_names())
    if not contacts.available:
        print(f"  ! collision check UNAVAILABLE: {contacts.reason}\n"
              "    trials will report closest as 'n/a'. Do not read that as a "
              "clean run.", flush=True)

    results = []          # (verdict, why) per trial, for summarise()
    contact_trials = []   # trials whose closest approach was an overlap

    for i in range(1, n + 1):
        try:
            # Re-verify the whole world every trial, not once per run. The old
            # code called ensure_shuttlecock() once at startup, so anything
            # that destroyed the object mid-run turned every remaining trial
            # into a scored robot failure. One model list serves both checks.
            present = model_list()
            if ensure_shuttlecock(force=False, present=present):
                print(f"    ! trial {i}: shuttlecock was missing, respawned",
                      flush=True)
            regrown = spawn_obstacles(force=False, present=present)
            if regrown:
                print(f"    ! trial {i}: obstacles missing, respawned {regrown}",
                      flush=True)

            # 0.30 m: loose enough for settling, tight enough to catch a
            # set_pose that silently did nothing and left the base where the
            # last trial parked it. This one readback also stands in for the
            # object's placement below -- it proves the set_pose service is
            # answering this trial at all, which is the failure worth catching;
            # the object's own placement is then read back as `before` anyway.
            set_pose("rosbot", 0, 0, 0, tol=0.30, settle=2.0)
            node.seed_amcl(0.0, 0.0, 0.0)
            if not park_arm():
                raise GzQueryFailed("park_arm publishes failed; arm pose unknown")
            time.sleep(2)

            tx, ty, tz, q = target_pose(rng)
            set_pose("shuttlecock", tx, ty, tz, *q)
            time.sleep(6)                        # it slides before settling

            before = probe_pose("shuttlecock")
            if before is UNKNOWN:
                raise GzQueryFailed("could not read the object's start pose")
            if before is None:
                raise WorldMismatch(
                    "object absent right after set_pose placed it")
        except (GzQueryFailed, WorldMismatch) as e:
            print(f"  trial {i:2d}: INDETERMINATE  infra: {e}", flush=True)
            results.append(("INDETERMINATE", f"infra: {e}"))
            continue

        # Park STANDOFF back along the approach heading, facing the object.
        aim = skirt_centre(before)
        approach = math.atan2(aim[1], aim[0])
        gx = aim[0] - STANDOFF * math.cos(approach)
        gy = aim[1] - STANDOFF * math.sin(approach)

        contacts.start()
        t_trial = time.time()
        nav = node.send_nav_goal(gx, gy, approach)
        t_nav = time.time() - t_trial

        after_nav = node.base_pose()
        nav_err = (math.hypot(after_nav[0] - gx, after_nav[1] - gy)
                   if after_nav else float("nan"))

        _t = time.time()
        fine_d, _fine_y = node.fine_approach(gx, gy, approach)
        t_fine = time.time() - _t

        # Final correction on vision. Everything up to here trusts the map
        # frame; this does not.
        _t = time.time()
        vis_err, vis_iters, vis_seen = node.visual_servo()
        t_vis = time.time() - _t
        park_arm()
        time.sleep(1)

        # Minimum footprint-to-obstacle gap over the whole trial, not just at
        # the parked pose: a run that clipped an obstacle mid-navigation and
        # then parked clear would otherwise look spotless.
        min_clear = contacts.stop()
        if min_clear is not None and min_clear <= 0.0:
            contact_trials.append(i)

        # Where the object actually sits relative to the arm now, and how close
        # the parked base ended up to an obstacle.
        base_truth = probe_pose("rosbot")
        reach = float("nan")
        clear = float("nan")
        if isinstance(base_truth, tuple):
            dx, dy = aim[0] - base_truth[0], aim[1] - base_truth[1]
            byaw = base_truth[5]
            c, s = math.cos(-byaw), math.sin(-byaw)
            rx, ry = dx * c - dy * s, dx * s + dy * c
            reach = math.hypot(rx - ARM_X, ry)
            clear = obstacle_clearance(base_truth[0], base_truth[1])

        _t = time.time()
        log, grasp_status = run_grasp(f"/tmp/navgrasp_{i}.log")
        t_grasp = time.time() - _t
        t_total = time.time() - t_trial
        after = probe_pose("shuttlecock")
        m = re.search(r"gripper (?:at|stalled at) ([-\d.]+)", log)

        verdict, why = classify(log, before, after,
                                float(m.group(1)) if m else None, grasp_status)
        # classify() only knows about the pick. Navigation faults are scored
        # here, and as INDETERMINATE rather than as failures: a nav server that
        # was never up, or a goal that was rejected before the robot moved,
        # says nothing about whether the robot can pick the object up.
        if verdict != "INDETERMINATE" and nav in ("NO_SERVER", "REJECTED"):
            verdict, why = "INDETERMINATE", f"nav2 goal {nav} (infra)"
        elif verdict == "FAIL":
            if not why:
                why = "no lift"
            if "Arm move failed" in log:
                why = f"{why}; arm move refused"
        results.append((verdict, why))

        # Minimum gap seen at any point in the trial. "n/a" means the stream
        # gave nothing, NOT that the run was clean.
        if min_clear is None:
            closest = "n/a"
        elif min_clear <= 0.0:
            closest = f"OVERLAP {min_clear:+.3f}"
        else:
            closest = f"{min_clear:+.3f}m"
        vis = (f"{vis_err:.3f}m x{vis_iters}" if vis_seen else "NOT SEEN")
        after_z = f"{after[2]:.3f}" if isinstance(after, tuple) else "  ?  "
        print(f"  trial {i:2d}: {verdict:13s} "
              f"target ({tx:+.2f},{ty:+.2f}) d={math.hypot(tx,ty):.2f}m  "
              f"nav {nav:<9s} err {nav_err:.3f}  "
              f"fine {fine_d:.3f}m  vis {vis:<12s} "
              f"reach {reach:.3f}m  parked {clear:+.3f}m  closest {closest:<12s} "
              f"z {before[2]:.3f}->{after_z}  "
              f"[nav {t_nav:.0f} fine {t_fine:.0f} vis {t_vis:.0f} grasp {t_grasp:.0f} = {t_total:.0f}s]"
              f"{'  ' + why if why else ''}", flush=True)

    summarise(results, label="picked up after navigating")
    if not contacts.available:
        print("  obstacle contact:  NOT CHECKED -- "
              f"{contacts.reason}")
    else:
        print(f"  obstacle contact:  {len(contact_trials)} trial(s) touched an "
              f"obstacle{': ' + str(contact_trials) if contact_trials else ''}  "
              f"[{contacts.stream_bytes} bytes streamed from "
              f"{len(contacts.topics)} sensor(s)]")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
