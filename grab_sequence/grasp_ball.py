"""Find a shuttlecock on the floor with OpenCV and pick it up with MoveIt.

The ZED is mounted on link5 (see manipulation_pro.yaml: parent_link: link5),
so it is an eye-in-hand camera -- moving the arm moves the view. That is what
makes this work: the arm is first sent to a bird's-eye "search" pose where the
camera looks down at the floor, and the patch it sees is the same patch the arm
can reach. No base motion is needed.

Geometry that fixes the numbers below (all verified against live TF, and the
forward kinematics here reproduce link4/link5/EE to under a millimetre):

  * joint2 sits 0.2095 m up and the shoulder->wrist chain is 0.254 m long, so
    with the gripper held perpendicular to the floor the grasp point can only
    be placed at a radius of 0.099..0.157 m from the arm axis, which is at the
    chassis front (see ARM_X).
  * from the search pose the camera sits ~0.42 m up looking ~79 deg down, with
    its axis meeting the floor at radius ~0.198 m; the 110 deg field of view
    covers the whole grasp band comfortably.

joint1 simply rotates both the view and the reach together, so the robot can
sweep it to look around itself for the ball.
"""

import math
import statistics
import time

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import CameraInfo, Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_geometry_msgs import do_transform_point

BASE_FRAME = "base_link"
RGB_TOPIC = "/zed/zed_node/rgb/image_rect_color"
DEPTH_TOPIC = "/zed/zed_node/depth"
CAMERA_INFO_TOPIC = "/zed/zed_node/rgb/camera_info"

LOWER_YELLOW = np.array([20, 100, 100])
UPPER_YELLOW = np.array([35, 255, 255])
# Pixel area, so it scales with the square of camera resolution. The ZED
# dropped from 1280x720 to 640x360 (see stereolabs_zed.urdf.xacro) to make
# the depth cloud usable as a nav2 costmap source, so this went 80 -> 20.
MIN_CONTOUR_AREA = 20.0

# --- target object: to-scale badminton shuttlecock ----------------------
# Cork base 26 mm dia (z 0..0.025), feather skirt 65 mm dia (z 0.025..0.085).
# The jaws span 30.2 mm closed and 82.2 mm open, so the 26 mm cork is too
# narrow to clamp -- the jaws bottom out on their own stop before touching it.
# The 65 mm skirt sits inside that range, so the grasp goes there.
# The jaw gap is not constant along the finger. Measured off the meshes, as a
# depth below the link5 origin (mesh position + the 0.0817 m joint offset):
#     81.7- 91.7 mm  ~52 mm gap (mounting bracket, narrowest)
#     91.7-101.7 mm  ~70 mm
#    101.7-111.7 mm  ~76 mm
#    111.7-131.7 mm  ~86 mm (widest, the flat pad faces)
# The 65 mm skirt has to sit in that last band or its top rim fouls the bracket
# and forces the fingers onto their hard stop. Putting the skirt top (z=0.085)
# at depth 111.7 mm puts link5 at 0.197, and the pad centre at 0.075 -- the
# pads then close on the top 20 mm of the skirt with ~10 mm clearance a side.
TARGET_GRASP_Z = 0.075
# Seen from almost straight above, the depth ray lands on the top of the skirt
# and its centroid is already on the object's axis, so no radial correction is
# needed (a sphere would need +radius here to reach its centre).
TARGET_RAY_OFFSET = 0.0

# Height above which the shuttlecock is standing on its cork, rather than lying.
# Re-measured 2026-08-21 at the search pose, 8+ frames per sample, three target
# distances each:
#     upright   0.0851 .. 0.0852     (its standing height is 0.085)
#     lying     0.0412 .. 0.0436
# so 0.075 clears upright by 10 mm and lying by 32 mm. The earlier note here
# predicted 0.052..0.065 for lying, which is too high; the separation is wider
# than advertised, not narrower.
#
# This is compared against ball[2], which is built from the whole-blob centroid
# with the median depth across it. That LOOKS like a volumetric centre, and it
# was mistaken for one -- a volumetric centre could never reach 0.075 and the
# test would be broken for upright objects. It is not. The blob is the object's
# VISIBLE surface, and from the bird's-eye search pose the visible surface is
# its top, so the deprojected centroid lands on the top face. The measurement
# and the threshold are in the same units after all.
#
# If you ever see this read ~0.024 for a lying shuttlecock, that is the cork
# top with the skirt mesh missing (see ensure_shuttlecock in the harness), not
# a classification failure.
UPRIGHT_TOP_Z = 0.075

# --- arm geometry, from open_manipulator/body.xacro ----------------------
# joint1 axis in base_link. Must match the arm mount in rosbot_xl.urdf.xacro;
# the arm sits at the chassis front so the working zone is ahead of the robot.
ARM_X = 0.065
SHOULDER_H = 0.2095     # joint2 height
V1 = (0.024, 0.128)     # joint2 -> joint3, in the arm plane
L2 = math.hypot(*V1)
PSI1 = math.atan2(V1[0], V1[1])
L3 = 0.124              # joint3 -> joint4 (= link5 origin)
# link5 origin -> middle of the finger pads, measured along the tool axis.
# Two parts, and leaving out the first was a real bug: the finger joints mount
# 0.0817 m down the tool from link5 (body.xacro, gripper_*_joint origin), and
# the pads sit a further distance along the finger. Using only the second term
# put the pads 0.082 m too low -- through the floor -- so the fingers splayed
# against the ground plane and never touched the target.
# The second term is where along the claw the object is aimed to nest. The CAD
# claw's pocket runs its whole length (measured void span 2..78 mm below the
# finger mount, with no dead material below it), so the target can sit anywhere
# along that wrap. Aim the lower part of it, not the middle: the tip is at
# 78 mm, so aiming at 40 mm leaves 38 mm of claw under the grip point and the
# tip hits the floor on anything lower than 38 mm. Aiming at 68 mm leaves 10 mm
# and reaches the ~21 mm a lying shuttlecock needs.
# Note jaw_geometry.py reports the midpoint of all collision geometry, which is
# NOT this number -- it has no idea where the pocket is.
GRASP_OFFSET = 0.0817 + 0.068

# Lowest point of the claw below link5, measured from the CAD mesh (material
# spans 2..78 mm below the finger mount). The gap between this and GRASP_OFFSET
# is how much claw sits under the grip point, and so the lowest a grasp can go
# before the tip is through the floor.
CLAW_TIP_BELOW_LINK5 = 0.0817 + 0.078
MIN_GRASP_Z = CLAW_TIP_BELOW_LINK5 - GRASP_OFFSET

JOINT_LIMITS = [
    (-4.0 / 5.0 * math.pi, math.pi),   # joint1
    (-math.pi / 2, math.pi / 2),       # joint2
    (-1.5, 1.4),                       # joint3
    (-1.7, 1.97),                      # joint4
]

# Bird's-eye pose for joints 2-4: camera ~0.42 m up, looking ~79 deg down.
# joint4 is kept off its 1.97 limit on purpose.
SEARCH_J234 = (-0.5834, -0.0914, 1.8486)

# joint1 values to sweep while looking for the target. With the arm moved to the
# chassis front these face forward, where the floor is clear: the chassis ends at
# x=+0.161 and is 0.135 wide either side, so at the arm's reach a bearing beyond
# roughly +-75 deg lands the grasp point under the robot itself. Straight ahead
# is tried first since that is where a shuttlecock the robot drove up to will be.
SEARCH_BEARINGS = [
    math.radians(0.0),
    math.radians(35.0),
    math.radians(-35.0),
    math.radians(70.0),
    math.radians(-70.0),
]

HOVER = 0.06


def ik_planar(s, h, pitch_sum, elbow):
    """Place the link5 origin at (s, h) in the arm plane. pitch_sum fixes the
    tool direction: pi/2 means pointing straight down at the floor."""
    ds, dh = s, h - SHOULDER_H
    dist = math.hypot(ds, dh)
    if dist > L2 + L3 or dist < abs(L2 - L3):
        return None
    cos_d = max(-1.0, min(1.0, (dist * dist - L2 * L2 - L3 * L3) / (2 * L2 * L3)))
    delta = elbow * math.acos(cos_d)
    a = math.atan2(ds, dh) - math.atan2(L3 * math.sin(delta), L2 + L3 * math.cos(delta))
    t2 = a - PSI1
    t3 = delta - math.pi / 2 + PSI1
    return t2, t3, pitch_sum - t2 - t3


def grasp_point_of(joints, tool_offset=GRASP_OFFSET):
    """Forward kinematics: where the middle of the finger pads actually is,
    in base_link, for a given (j1, j2, j3, j4)."""
    j1, t2, t3, t4 = joints
    a = PSI1 + t2
    p3s, p3h = L2 * math.sin(a), SHOULDER_H + L2 * math.cos(a)
    b = math.pi / 2 + t2 + t3
    p4s, p4h = p3s + L3 * math.sin(b), p3h + L3 * math.cos(b)
    c = math.pi / 2 + t2 + t3 + t4
    gs = p4s + tool_offset * math.sin(c)
    gh = p4h + tool_offset * math.cos(c)
    return ARM_X + gs * math.cos(j1), gs * math.sin(j1), gh


def arm_joints_for(x, y, z, tool_offset=GRASP_OFFSET):
    """Closed-form 4-DOF solution putting the grasp point at base_link (x, y, z)
    with the gripper perpendicular to the floor. None if unreachable.

    Solved analytically rather than as a MoveIt pose goal: with only 4 joints a
    full 6-DOF pose goal is over-constrained, and OMPL's goal sampler just fails
    with "Unable to sample any valid states for goal tree"."""
    j1 = math.atan2(y, x - ARM_X)
    s = math.hypot(x - ARM_X, y)
    for elbow in (+1, -1):
        sol = ik_planar(s, z + tool_offset, math.pi / 2, elbow)
        if sol is None:
            continue
        joints = (j1,) + sol
        if all(lo <= q <= hi for q, (lo, hi) in zip(joints, JOINT_LIMITS)):
            return list(joints)
    return None


class BallGrasper(Node):
    def __init__(self):
        super().__init__("ball_grasper")
        # "x,y,z" in base_link skips detection and grasps there directly.
        self.declare_parameter("ball_xyz", "")
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_info = None
        self.samples = []
        self.axis_from_pca_frames = 0
        self.axis_from_depth_frames = 0
        self.collecting = False
        self.collect_start = None

        self.joint_positions = {}
        self.gripper_pub = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10
        )
        self.roll_pub = self.create_publisher(
            JointTrajectory, "/wrist_roll_controller/joint_trajectory", 10
        )
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._info_cb, 10)
        rgb_sub = message_filters.Subscriber(self, Image, RGB_TOPIC)
        depth_sub = message_filters.Subscriber(self, Image, DEPTH_TOPIC)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.05
        )
        self._sync.registerCallback(self._image_cb)

    def _info_cb(self, msg):
        self.camera_info = msg

    def _joint_cb(self, msg):
        self.joint_positions.update(zip(msg.name, msg.position))

    def arm_joints_now(self):
        names = ["joint1", "joint2", "joint3", "joint4"]
        if not all(n in self.joint_positions for n in names):
            return None
        return [self.joint_positions[n] for n in names]

    def _image_cb(self, rgb_msg, depth_msg):
        if self.camera_info is None or not self.collecting:
            return
        # Ignore anything captured before the arm finished moving, so a frame
        # from mid-swing is never paired with the parked arm pose.
        stamp = rclpy.time.Time.from_msg(depth_msg.header.stamp)
        if self.collect_start is not None and stamp < self.collect_start:
            return
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < MIN_CONTOUR_AREA:
            return
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")

        # Whole-blob centroid with the median depth across it.
        #
        # This previously used only pixels within 15 mm of the nearest depth, on
        # the theory that the mask also catches the skirt's near side wall and
        # would drag a plain centroid off-axis. Measured against ground truth,
        # that patch drops below 20 pixels at ordinary poses -- every frame then
        # gets rejected here and the object is never seen at all. The whole blob
        # is far more robust and, once the camera is aimed at the target (see the
        # re-aim step in main), lands within about 6 mm.
        blob = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(blob, [largest], -1, 255, cv2.FILLED)
        valid = (blob > 0) & np.isfinite(depth_img) & (depth_img > 0.0)
        if int(valid.sum()) < 50:
            return
        ys, xs = np.nonzero(valid)
        u, v = float(xs.mean()), float(ys.mean())
        d = float(np.median(depth_img[valid]))

        # Long axis of the silhouette, for aligning the wrist roll. Taken from
        # the whole blob rather than just the top face, since a shuttlecock
        # lying on its side is elongated over its full outline.
        axis_px, half_px = None, 0.0
        pts = largest.reshape(-1, 2).astype(np.float32)
        if len(pts) >= 5:
            _, eigvec, eigval = cv2.PCACompute2(pts, mean=None)
            l1, l2 = float(eigval[0][0]), float(eigval[1][0])
            # Standing upright the outline is near-circular and its "long axis"
            # is just noise, so only trust a clearly elongated blob.
            if l1 > 1e-6 and math.sqrt(max(l2, 0.0) / l1) < 0.75:
                axis_px = eigvec[0]
                half_px = math.sqrt(l1)
                # Which end is the skirt? PCA gives a line, not a direction, but
                # the tapered jaw is not symmetric -- its gap narrows toward one
                # end, and that end has to be the cork. Resolve it by width: the
                # skirt half of the silhouette is measurably fatter than the
                # cork half. Point the axis at the fat end.
                rel = pts - pts.mean(axis=0)
                along = rel @ axis_px
                across = np.abs(rel @ eigvec[1])
                fwd, bwd = along > 0, along < 0
                if fwd.any() and bwd.any() and across[bwd].mean() > across[fwd].mean():
                    axis_px = -axis_px

        # Latest TF is the right transform here because detection only runs with
        # the arm parked. Asking for the capture-time transform instead would
        # deadlock: TF trails the image stream by a few ms, and blocking for it
        # inside this callback stops the single-threaded executor from ever
        # receiving the TF that would unblock it. Freshness is enforced by
        # dropping pre-settle frames above, so latest == capture-time pose.
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, depth_msg.header.frame_id, rclpy.time.Time()
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}", throttle_duration_sec=2.0)
            return

        centre = self._to_base(u, v, d, depth_msg.header, tf, TARGET_RAY_OFFSET)
        if centre is None:
            return

        # Turn the pixel-space axis into a direction on the floor by projecting
        # both ends at the target's depth and differencing them in base_link.
        # Doing it this way avoids having to reason about image-axis sign and
        # rotation conventions. The stored angle is signed, pointing cork to
        # skirt, so it is averaged as a plain unit vector.
        c2 = s2 = None
        # Linear pixel measure, so it halves with the resolution drop (2.0 -> 1.0).
        if axis_px is not None and half_px > 1.0:
            a = self._to_base(u - axis_px[0] * half_px, v - axis_px[1] * half_px,
                              d, depth_msg.header, tf, TARGET_RAY_OFFSET)
            b = self._to_base(u + axis_px[0] * half_px, v + axis_px[1] * half_px,
                              d, depth_msg.header, tf, TARGET_RAY_OFFSET)
            if a is not None and b is not None:
                ang = math.atan2(b[1] - a[1], b[0] - a[0])
                c2, s2 = math.cos(ang), math.sin(ang)

        if c2 is None:
            ang = self._axis_from_depth(valid, depth_img, depth_msg.header, tf)
            if ang is not None:
                c2, s2 = math.cos(ang), math.sin(ang)
                self.axis_from_depth_frames += 1
        elif c2 is not None:
            self.axis_from_pca_frames += 1

        self.samples.append((centre[0], centre[1], centre[2], c2, s2))

    def _axis_from_depth(self, valid, depth_img, header, tf):
        """Recover the object's axis from the slope of its top surface.

        The silhouette PCA above needs a clearly elongated outline, and a
        shuttlecock lying down does not reliably have one. Measured at four
        headings, its eccentricity ran 0.53..0.78 against a 0.75 gate, so the
        axis resolved on some headings and not others -- roughly half of trials
        came back "upright (no usable axis)" and got no wrist alignment at all,
        even though the height test correctly called them "on its side".

        The depth image does not have that problem, because the object is a
        cone. Lying down, its visible top rises from the cork end (~26 mm) to
        the skirt end (~65 mm) over about 85 mm of length, so the surface has a
        pronounced tilt whose downhill-to-uphill direction IS the cork-to-skirt
        axis. Fitting z = ax + by + c over the blob's deprojected points and
        taking atan2(b, a) recovers it, and the gradient carries the sense for
        free -- uphill is the fat end -- which the PCA had to infer separately
        by comparing silhouette widths.

        Measured against Gazebo ground truth at four headings:

            truth  +89.7   depth +105.0   slope 0.53   err 15.3 deg
            truth +124.4   depth +125.9   slope 0.42   err  1.5 deg
            truth -179.3   depth +172.2   slope 0.46   err  8.5 deg
            truth   -5.7   depth   +7.1   slope 0.46   err 12.9 deg   (PCA failed here)

        So it always resolves, to within about 15 deg. That is looser than a
        good PCA fit, which is why this runs only as a FALLBACK: when the
        silhouette really is elongated the existing path is kept untouched, and
        this fills in the headings that used to get nothing. A 15 deg error on
        an 85 mm object displaces its ends by about 11 mm, well inside what the
        claw's wrap tolerates.

        MIN_SLOPE rejects a flat fit. A shuttlecock standing on its cork
        presents a roughly level top, so its gradient direction is noise, and
        that is exactly the case that must NOT produce an axis.
        """
        MIN_SLOPE = 0.20
        MIN_POINTS = 50
        ys, xs = np.nonzero(valid)
        if len(xs) < MIN_POINTS:
            return None
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]
        ds = depth_img[valid]

        # Deproject in bulk. Doing this per-pixel through _to_base would mean
        # thousands of PointStamped transforms per frame inside the callback.
        x_cam = ds
        y_cam = -(xs - cx) / fx * ds
        z_cam = -(ys - cy) / fy * ds
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        cp, sp = math.cos(pitch), math.sin(pitch)
        cyw, syw = math.cos(yaw), math.sin(yaw)
        # rotate by pitch then yaw; roll is ~0 for this mount
        xr = x_cam * cp + z_cam * sp
        zr = -x_cam * sp + z_cam * cp
        bx = tf.transform.translation.x + xr * cyw - y_cam * syw
        by = tf.transform.translation.y + xr * syw + y_cam * cyw
        bz = tf.transform.translation.z + zr

        A = np.c_[bx - bx.mean(), by - by.mean(), np.ones(len(bx))]
        try:
            coef, *_ = np.linalg.lstsq(A, bz - bz.mean(), rcond=None)
        except np.linalg.LinAlgError:
            return None
        if math.hypot(coef[0], coef[1]) < MIN_SLOPE:
            return None
        return math.atan2(coef[1], coef[0])

    def _to_base(self, u, v, d, header, tf, ray_offset=0.0):
        """Deproject a pixel at depth d and express it in base_link."""
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]

        # rgb/depth carry frame_id "zed_camera_center", the physical link
        # (REP-103: X forward, Y left, Z up), not an optical frame.
        x_cam = d
        y_cam = -(u - cx) / fx * d
        z_cam = -(v - cy) / fy * d

        r = math.sqrt(x_cam**2 + y_cam**2 + z_cam**2)
        if r < 1e-6:
            return None
        k = (r + ray_offset) / r

        point = PointStamped()
        point.header = header
        point.point.x, point.point.y, point.point.z = x_cam * k, y_cam * k, z_cam * k
        p = do_transform_point(point, tf)
        return (p.point.x, p.point.y, p.point.z)

    def set_gripper(self, position, seconds=3.0):
        """Command the gripper straight at its JointTrajectoryController.

        Not routed through MoveIt on purpose. Brushing the target pushes the
        fingers onto their 0.019 hard stop, and MoveIt's CheckStartStateBounds
        then refuses to plan at all ("Start state out of bounds"), so the close
        fails exactly when contact means it matters most. Publishing directly
        is also what joy2servo does in this repo. gripper_right_joint mimics
        the left, so only the left is commanded."""
        msg = JointTrajectory()
        msg.joint_names = ["gripper_left_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [position]
        pt.time_from_start = Duration(sec=1, nanosec=0)
        msg.points = [pt]

        # Publish once, not repeatedly: every new trajectory replaces the one in
        # flight and restarts its 1 s motion, so a stream of them leaves the
        # gripper permanently re-starting and never arriving.
        while rclpy.ok() and self.gripper_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.gripper_pub.publish(msg)

        end = time.monotonic() + seconds
        last, still_since = None, None
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            q = self.joint_positions.get("gripper_left_joint")
            if q is None:
                continue
            # Reached the commanded position.
            if abs(q - position) < 0.0005:
                return
            # Or stopped moving, which is what closing onto the object looks
            # like. Waiting out the full timeout after the jaws have already
            # stalled was costing several seconds on every pick.
            now = time.monotonic()
            if last is not None and abs(q - last) < 0.0002:
                if still_since is None:
                    still_since = now
                elif now - still_since > 0.4:
                    self.get_logger().info(
                        f"gripper stalled at {q:.4f} (target {position:.4f})"
                    )
                    return
            else:
                still_since = None
            last = q
        q = self.joint_positions.get("gripper_left_joint")
        if q is not None:
            self.get_logger().info(f"gripper at {q:.4f} (target {position:.4f})")

    def set_wrist_roll(self, angle, seconds=3.0):
        """Rotate the jaws about the tool axis. joint5 is simulation-only and
        has its own controller, so it is commanded directly like the gripper."""
        msg = JointTrajectory()
        msg.joint_names = ["joint5"]
        pt = JointTrajectoryPoint()
        pt.positions = [angle]
        pt.time_from_start = Duration(sec=1, nanosec=0)
        msg.points = [pt]
        while rclpy.ok() and self.roll_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.roll_pub.publish(msg)
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            q = self.joint_positions.get("joint5")
            if q is not None and abs(q - angle) < 0.01:
                return

    def settle(self, seconds):
        """Spin (rather than sleep) so queued frames from while the arm was
        moving are drained instead of piling up for the next detection."""
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def locate_ball(self, num_samples=10, timeout_sec=5.0):
        """Average several detections into one (x, y, z) in base_link.

        The deadline is wall-clock: under use_sim_time this node's ROS clock
        reads 0 until its first spin, so a ROS-time deadline looks pre-expired."""
        self.samples.clear()
        self.collect_start = self.get_clock().now()
        self.collecting = True
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and len(self.samples) < num_samples:
            if time.monotonic() > deadline:
                self.collecting = False
                return None, None
            rclpy.spin_once(self, timeout_sec=0.2)
        self.collecting = False
        xs, ys, zs, c2s, s2s = zip(*self.samples)

        # Median, not mean, and over many samples. The simulated ZED carries
        # stddev_error=0.03 (stereolabs_zed.urdf.xacro), i.e. 30 mm of Gaussian
        # depth noise, against roughly 10 mm of jaw clearance. Averaging 15
        # samples left ~17 mm of error with 40 mm outliers, which was the single
        # biggest cause of missed grasps. The median ignores the outliers and
        # the larger sample count shrinks the rest as 1/sqrt(n).
        pos = (statistics.median(xs), statistics.median(ys), statistics.median(zs))

        # Average the direction as a unit vector rather than averaging angles,
        # which would break across the +-pi wrap.
        good = [(c, s) for c, s in zip(c2s, s2s) if c is not None]
        axis = None
        if len(good) >= max(3, len(self.samples) // 3):
            mc = sum(c for c, _ in good) / len(good)
            ms = sum(s for _, s in good) / len(good)
            if math.hypot(mc, ms) > 0.5:      # directions agreed with each other
                axis = math.atan2(ms, mc)
        return pos, axis


def main():
    rclpy.init()
    node = BallGrasper()

    # Wait for move_group to actually appear rather than sleeping a fixed 10 s.
    # On a warm system it is up in a second or two, and the old fixed sleeps
    # were 13 s of the ~43 s a pick used to take.
    deadline = time.monotonic() + 30.0
    while rclpy.ok() and time.monotonic() < deadline:
        if any("move_group" in n for n in node.get_node_names()):
            break
        rclpy.spin_once(node, timeout_sec=0.2)
    node.settle(1.0)

    moveit = MoveItPy(node_name="grasp_ball_moveit")
    arm = moveit.get_planning_component("manipulator")
    robot_model = moveit.get_robot_model()

    # Wait for the controller's action server, not just for move_group to exist.
    # move_group appearing says nothing about whether MoveIt's controller action
    # clients have connected, and firing the first move too early aborts it with
    # "Action client not connected to action server". That killed whole runs at
    # the first search pose, intermittently, in about half of them.
    ready = ActionClient(node, FollowJointTrajectory,
                         "/manipulator_controller/follow_joint_trajectory")
    if not ready.wait_for_server(timeout_sec=30.0):
        node.get_logger().error("manipulator_controller action server never appeared")
        return
    # No fixed wait here any more.
    #
    # MoveIt's SimpleControllerManager builds its OWN action client, and that
    # one can still be unconnected when the first move goes out, which aborts it
    # with "Action client not connected to action server" and used to kill about
    # half of all runs at the first search pose. Our own client connecting says
    # nothing about MoveIt's, and there is no way to observe MoveIt's from here,
    # so this was handled by sleeping 3 s and hoping -- which every run paid for,
    # including the majority that never needed it.
    #
    # run() now retries instead. The cost moves onto the runs that actually hit
    # the race, and a healthy start pays nothing. This also covers the case the
    # fixed sleep never could: a manager that takes longer than 3 s.

    def run(component, attempts=4, backoff=1.0):
        """Plan and execute, retrying while MoveIt's controller client connects.

        Only the FIRST move of a run is realistically affected -- once the
        manager has connected it stays connected -- but retrying every move is
        harmless and costs nothing when they succeed.
        """
        for attempt in range(attempts):
            plan = component.plan()
            if plan and moveit.execute(plan.trajectory, controllers=[]):
                return True
            if attempt < attempts - 1:
                node.get_logger().warn(
                    f"arm move failed (attempt {attempt + 1}/{attempts}); "
                    "MoveIt's controller client may still be connecting")
                node.settle(backoff)
        return False

    # Open is the SRDF "Open" state. Close drives to the CAD claw's lower stop
    # rather than the SRDF's -0.009, which left a 26 mm gap between the claw
    # tips -- wide enough for the shuttlecock's 26 mm cork to slip through.
    # See gripper_lower in body.xacro; keep the two in step.
    GRIPPER_OPEN = 0.017
    GRIPPER_CLOSE = -0.023

    def set_gripper(state):
        node.set_gripper(GRIPPER_OPEN if state == "Open" else GRIPPER_CLOSE)

    def move_to_point(x, y, z, label, tries=3, tol=0.004):
        """Put the finger pads at base_link (x, y, z), correcting for the arm's
        steady-state error.

        joint2 carries the whole arm and settles ~0.2 rad short of its command
        under gravity, which lands the gripper ~30 mm high -- enough to clip the
        shuttlecock instead of closing around it. So aim, measure where the pads
        actually ended up, and re-aim by the leftover error."""
        goal = [x, y, z]
        for attempt in range(tries):
            joints = arm_joints_for(*goal)
            if joints is None:
                node.get_logger().error(f"No IK for adjusted goal during {label}")
                return False
            if not move_joints(joints, f"{label} (try {attempt + 1})"):
                return False
            node.settle(0.2)
            actual = node.arm_joints_now()
            if actual is None:
                return True
            ax, ay, az = grasp_point_of(actual)
            ex, ey, ez = x - ax, y - ay, z - az
            err = math.sqrt(ex * ex + ey * ey + ez * ez)
            node.get_logger().info(
                f"{label}: pads at ({ax:.3f}, {ay:.3f}, {az:.3f}), error {err * 1000:.1f} mm"
            )
            if err < tol:
                return True
            goal = [goal[0] + ex, goal[1] + ey, goal[2] + ez]
        return True

    def move_joints(joints, label):
        goal = RobotState(robot_model)
        goal.set_joint_group_positions("manipulator", list(joints))
        goal.update()
        arm.set_start_state_to_current_state()
        arm.set_goal_state(robot_state=goal)
        if not run(arm):
            node.get_logger().error(f"Arm move failed: {label}")
            return False
        time.sleep(0.15)
        return True

    try:
        raw = (node.get_parameter("ball_xyz").value or "").strip()
        override = [float(v) for v in raw.split(",")] if raw else []

        target_axis = None
        if len(override) == 3:
            ball = tuple(override)
            node.get_logger().info(
                f"Using given ball position ({ball[0]:.3f}, {ball[1]:.3f}, {ball[2]:.3f})"
            )
            set_gripper("Open")
        else:
            set_gripper("Open")
            ball = None
            # Sweep joint1, checking the floor patch under the camera at each
            # bearing. View and reach rotate together, so anything seen here is
            # a candidate for grasping.
            for bearing in SEARCH_BEARINGS:
                node.get_logger().info(f"Searching at joint1 = {math.degrees(bearing):.0f} deg")
                if not move_joints((bearing,) + SEARCH_J234, "search pose"):
                    continue
                # joint2 sags for a while after the trajectory reports done, and
                # the camera rides on link5 -- detecting too early pairs an
                # image with a pose the arm has already crept away from.
                node.settle(1.0)
                found, axis = node.locate_ball()
                if found is None:
                    continue
                node.get_logger().info(
                    f"Shuttlecock seen at base_link ({found[0]:.3f}, {found[1]:.3f})"
                    + (f", long axis {math.degrees(axis):.0f} deg"
                       f" [pca {node.axis_from_pca_frames}/depth "
                       f"{node.axis_from_depth_frames} frames]" if axis is not None
                       else ", upright (no usable axis)")
                )
                if arm_joints_for(found[0], found[1], TARGET_GRASP_Z) is not None:
                    # Re-aim and re-measure before committing.
                    #
                    # The sweep accepts whichever bearing first sees the object,
                    # which is usually not the bearing pointing at it, and the
                    # camera's own axis meets the floor at radius ~0.198. An
                    # object seen well off that axis measures badly: at bearing
                    # 135 seen from 170 the error was 17.8 mm, and 5.6 mm once
                    # joint1 was turned to face it. Only joint1 moves here, so
                    # unlike lifting the camera overhead there is no reach or
                    # joint-limit problem.
                    aim = math.atan2(found[1], found[0] - ARM_X)
                    lo, hi = JOINT_LIMITS[0]
                    delta = math.atan2(math.sin(aim - bearing), math.cos(aim - bearing))
                    if abs(delta) > math.radians(4.0) and lo <= aim <= hi:
                        node.get_logger().info(
                            f"Re-aiming joint1 {math.degrees(bearing):.0f} -> "
                            f"{math.degrees(aim):.0f} deg and re-measuring"
                        )
                        if move_joints((aim,) + SEARCH_J234, "re-aim"):
                            node.settle(1.0)
                            refined, refined_axis = node.locate_ball()
                            if refined is not None:
                                found = refined
                                if refined_axis is not None:
                                    axis = refined_axis
                                node.get_logger().info(
                                    f"Refined to ({found[0]:.3f}, {found[1]:.3f})"
                                )
                    ball = found
                    target_axis = axis
                    break
                s = math.hypot(found[0] - ARM_X, found[1])
                j1 = math.atan2(found[1], found[0] - ARM_X)
                lo, hi = JOINT_LIMITS[0]
                why = (
                    f"bearing {math.degrees(j1):.0f} deg is outside joint1's "
                    f"{math.degrees(lo):.0f}..{math.degrees(hi):.0f} deg"
                    if not (lo <= j1 <= hi)
                    else f"radius {s:.3f} m is outside 0.050-0.218 m"
                )
                node.get_logger().warn(f"Seen but not reachable ({why}); continuing search")
            if ball is None:
                node.get_logger().error("No reachable shuttlecock found")
                return

        # Standing or lying is decided by the measured height of the top
        # surface, not by how elongated the blob looks.
        #
        # A shuttlecock on its cork measures ~0.085 and one on its side ~0.052
        # to 0.065, which separate cleanly. Blob eccentricity does not: over a
        # 12-trial sweep it disagreed with the measured height on 5 of 10
        # detections, in both directions. Upright shuttlecocks read as elongated
        # because the camera sees the skirt wall from 11 degrees off vertical,
        # and tilted ones read as round when foreshortened. Since this branch
        # picks the grasp height, getting it wrong put the jaws tens of mm off.
        bx, by = ball[0], ball[1]
        top_z = ball[2]
        upright = top_z > UPRIGHT_TOP_Z
        if upright:
            # Grip the top of the skirt, just under its rim.
            bz = max(0.02, top_z - 0.010)
            roll_axis = None
        else:
            # On its side the body is a cylinder, widest across the middle, so
            # close there rather than near the top where it narrows away.
            bz = max(0.02, 0.5 * top_z)
            roll_axis = target_axis
        node.get_logger().info(
            f"Top surface at z={top_z:.3f} -> {'upright' if upright else 'on its side'}, "
            f"grasping at z={bz:.3f}"
        )
        target_axis = roll_axis

        # Refuse grasps that would drive the claw through the floor.
        #
        # The claw hangs CLAW_TIP_BELOW_LINK5 down the tool while its grip
        # surface is only GRASP_OFFSET down, so the grip point cannot go below
        # the difference without the tip going underground. Without this check
        # the arm simply jams into the ground, the object gets nudged away and
        # the jaws close on air, with nothing in the log saying why.
        if bz < MIN_GRASP_Z - 1e-6:
            node.get_logger().error(
                f"Grasp at z={bz:.3f} would put the claw tip "
                f"{1000 * (MIN_GRASP_Z - bz):.0f} mm below the floor. This claw "
                f"cannot pick anything needing a grip below z={MIN_GRASP_Z:.3f} "
                "-- it reaches 0.075 for an upright shuttlecock but not the "
                "~0.021 a lying one needs. Shorten the claw below its grip "
                "surface, or grip the object higher up."
            )
            return
        joints_grasp = arm_joints_for(bx, by, bz)
        joints_hover = arm_joints_for(bx, by, bz + HOVER)
        if joints_grasp is None or joints_hover is None:
            s = math.hypot(bx - ARM_X, by)
            node.get_logger().error(
                f"No IK for ({bx:.3f}, {by:.3f}, {bz:.3f}) -- radius {s:.3f} m "
                "is outside the 0.050-0.218 m band; the base would have to move."
            )
            return

        # Descend in small steps rather than one hover->grasp move. A single
        # move lets the planner pick any joint-space path between the two, and
        # the gripper swings sideways into the ball on the way down; stepping
        # keeps every segment short so the tool tracks a near-vertical line.
        def vertical_move(z_from, z_to, label, steps=4):
            for i in range(1, steps + 1):
                z = z_from + (z_to - z_from) * i / steps
                if not move_to_point(bx, by, z, f"{label} z={z:.3f}"):
                    return False
            return True

        if not move_to_point(bx, by, bz + HOVER, "hover"):
            return

        # Align the jaws to the target before descending.
        #
        # With the tool pointing straight down, link5's x is (0,0,-1) and its y
        # (the jaw line) is the arm's pitch axis, at bearing joint1+90. So
        # z = x cross y puts the jaw's z at bearing -joint1, and rolling by
        # theta about a downward axis turns bearings by -theta:
        #     jaw_z_bearing = -joint1 - theta
        # The V is not symmetric end to end -- its gap narrows toward +z -- so
        # the cork, the narrow end, has to sit at +z. target_axis points cork to
        # skirt, hence cork_bearing = target_axis + pi and
        #     theta = -joint1 - target_axis - pi
        # This needs the full +-pi range, not the +-pi/2 a bare jaw line would:
        # getting it 180 deg out points the taper the wrong way and is worse
        # than no taper at all. Skipped when the shuttlecock is standing, where
        # it is round from above and every roll is equivalent.
        if target_axis is not None:
            j1 = math.atan2(by, bx - ARM_X)
            roll = -j1 - target_axis - math.pi
            roll = math.atan2(math.sin(roll), math.cos(roll))
            lo, hi = -math.pi, math.pi
            roll = max(lo, min(hi, roll))
            node.get_logger().info(
                f"Object axis (cork->skirt) {math.degrees(target_axis):.0f} deg, "
                f"joint1 {math.degrees(j1):.0f} deg -> wrist roll {math.degrees(roll):.0f} deg"
            )
            node.set_wrist_roll(roll)
        else:
            node.set_wrist_roll(0.0)

        if not vertical_move(bz + HOVER, bz, "descend"):
            return
        set_gripper("Close")
        vertical_move(bz, bz + HOVER, "lift")
        node.get_logger().info("Grasp sequence complete")
    finally:
        # MoveItPy holds C++ state tied to the rclpy context; shut down once here
        # rather than from an early return.
        rclpy.shutdown()


if __name__ == "__main__":
    main()
