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
    be placed at a radius of 0.099..0.157 m from the arm axis at (-0.110, 0).
  * from the search pose the camera sits ~0.42 m up looking ~79 deg down, with
    its axis meeting the floor at radius ~0.198 m; the 110 deg field of view
    covers the whole grasp band comfortably.

joint1 simply rotates both the view and the reach together, so the robot can
sweep it to look around itself for the ball.
"""

import math
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
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import CameraInfo, Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_geometry_msgs import do_transform_point

BASE_FRAME = "base_link"
RGB_TOPIC = "/zed/zed_node/rgb/image_rect_color"
DEPTH_TOPIC = "/zed/zed_node/depth"
CAMERA_INFO_TOPIC = "/zed/zed_node/rgb/camera_info"

LOWER_YELLOW = np.array([20, 100, 100])
UPPER_YELLOW = np.array([35, 255, 255])
MIN_CONTOUR_AREA = 80.0

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

# --- arm geometry, from open_manipulator/body.xacro ----------------------
ARM_X = -0.110          # joint1 axis in base_link
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
# The second term is pad_centre_x from body.xacro, the centre of the tapered
# V pads; keep the two in step if that changes.
GRASP_OFFSET = 0.0817 + 0.045

JOINT_LIMITS = [
    (-4.0 / 5.0 * math.pi, math.pi),   # joint1
    (-math.pi / 2, math.pi / 2),       # joint2
    (-1.5, 1.4),                       # joint3
    (-1.7, 1.97),                      # joint4
]

# Bird's-eye pose for joints 2-4: camera ~0.42 m up, looking ~79 deg down.
# joint4 is kept off its 1.97 limit on purpose.
SEARCH_J234 = (-0.5834, -0.0914, 1.8486)

# joint1 values to sweep while looking for the ball. Rear and side bearings
# put the floor patch clear of the chassis and wheels; joint1's upper limit is
# pi, so 170 deg is used rather than a true 180.
SEARCH_BEARINGS = [
    math.radians(170.0),
    math.radians(135.0),
    math.radians(-135.0),
    math.radians(90.0),
    math.radians(-90.0),
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

        # Use only the object's top face, not the whole silhouette. The camera
        # looks down about 11 degrees off vertical, so the mask also catches the
        # near side wall of the skirt and a plain centroid gets pulled ~14 mm
        # off the axis -- more than the ~10 mm of clearance the jaws have.
        # Keeping just the closest surface leaves the top disc, whose centroid
        # is the axis.
        blob = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(blob, [largest], -1, 255, cv2.FILLED)
        valid = (blob > 0) & np.isfinite(depth_img) & (depth_img > 0.0)
        if not valid.any():
            return
        d_min = float(depth_img[valid].min())
        top = valid & (depth_img <= d_min + 0.015)
        ys, xs = np.nonzero(top)
        if len(xs) < 20:
            return
        u, v = float(xs.mean()), float(ys.mean())
        d = float(depth_img[top].mean())

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
        if axis_px is not None and half_px > 2.0:
            a = self._to_base(u - axis_px[0] * half_px, v - axis_px[1] * half_px,
                              d, depth_msg.header, tf, TARGET_RAY_OFFSET)
            b = self._to_base(u + axis_px[0] * half_px, v + axis_px[1] * half_px,
                              d, depth_msg.header, tf, TARGET_RAY_OFFSET)
            if a is not None and b is not None:
                ang = math.atan2(b[1] - a[1], b[0] - a[0])
                c2, s2 = math.cos(ang), math.sin(ang)

        self.samples.append((centre[0], centre[1], centre[2], c2, s2))

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
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            q = self.joint_positions.get("gripper_left_joint")
            # Stop early once it arrives, or once it stalls against the object
            # (closing onto something is a success, not a failure).
            if q is not None and abs(q - position) < 0.0005:
                return
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

    def locate_ball(self, num_samples=15, timeout_sec=6.0):
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
        pos = (sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs))

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

    time.sleep(10.0)
    moveit = MoveItPy(node_name="grasp_ball_moveit")
    arm = moveit.get_planning_component("manipulator")
    robot_model = moveit.get_robot_model()
    time.sleep(3.0)

    def run(component):
        plan = component.plan()
        return bool(plan) and bool(moveit.execute(plan.trajectory, controllers=[]))

    # Joint values matching the SRDF "Open"/"Close" group states.
    GRIPPER_OPEN = 0.017
    GRIPPER_CLOSE = -0.009

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
            node.settle(0.6)
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
        time.sleep(0.5)
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
                node.settle(3.0)
                found, axis = node.locate_ball()
                if found is None:
                    continue
                node.get_logger().info(
                    f"Shuttlecock seen at base_link ({found[0]:.3f}, {found[1]:.3f})"
                    + (f", long axis {math.degrees(axis):.0f} deg" if axis is not None
                       else ", upright (no usable axis)")
                )
                if arm_joints_for(found[0], found[1], TARGET_GRASP_Z) is not None:
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

        # Vision supplies where the object is (x, y) and how high its top
        # surface sits; the grasp height comes off that rather than a constant,
        # because a shuttlecock standing on its cork is 85 mm tall while one
        # lying on its side is only about 65 mm, and the right place to close
        # the jaws differs in each case.
        bx, by = ball[0], ball[1]
        top_z = ball[2]
        if target_axis is not None:
            # Lying down: the body is a cylinder on its side, widest across the
            # middle, so close there rather than near the top where it narrows.
            bz = max(0.02, 0.5 * top_z)
        else:
            # Standing: grip the top of the skirt, just under its rim.
            bz = max(0.02, top_z - 0.010)
        node.get_logger().info(f"Top surface at z={top_z:.3f}, grasping at z={bz:.3f}")
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
