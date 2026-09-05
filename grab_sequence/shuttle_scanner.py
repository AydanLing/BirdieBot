"""Opportunistic shuttlecock detection from a moving base.

The pick's own detector (grasp_ball.BallGrasper._image_cb) assumes a stationary
robot in three separate places, and each one has to be undone here:

  1. It looks up the LATEST transform rather than the one at capture time,
     because asking for capture-time TF inside a single-threaded executor
     deadlocks -- the callback blocks waiting for a TF that the same thread
     would have to spin to receive. Parked, latest and capture-time are the
     same thing and the shortcut is free. Driving, the error is velocity times
     image age, and image age on this stack measures at 12 ms median but 334 ms
     worst case. At 0.285 m/s that is a 95 mm error on a bad frame; spinning at
     1 rad/s it is 19 degrees of bearing, which throws a 3 m target by most of
     a metre.

     Fixed here without touching the executor: the TF lookup is attempted
     non-blocking, and any detection whose transform has not arrived yet is
     parked in a queue and retried on the next frame. TF trails the image
     stream by milliseconds, so in practice a detection resolves on its first
     or second attempt, and the callback never blocks for any of it.

  2. It accumulates samples in base_link and takes a median. base_link is
     attached to the moving robot, so ten samples taken while driving are ten
     measurements of ten different points -- the median is of a smear. Every
     detection here is transformed into `map` before it is stored, which is
     what makes samples from different moments, and different passes, additive
     rather than contradictory. See target_map.TargetMap.

  3. It keeps only the largest contour, so one frame yields at most one object.
     Driving past a scattered court, the interesting frames are exactly the
     ones with several shuttlecocks in view. Every qualifying blob is reported.

Accuracy here is centimetres, not the millimetres a grasp needs. That is the
intended division of labour: this populates the target list, and the robot
stops and re-detects with grasp-grade thresholds before it closes the jaws.
"""

import collections

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_geometry_msgs import do_transform_point

from .detect_params import (CAMERA_INFO_TOPIC, deproject, DEPTH_TOPIC,
                            LOWER_YELLOW, MAP_FRAME, MAX_DETECT_RANGE,
                            RGB_TOPIC, SEARCH_MIN_CONTOUR_AREA,
                            SEARCH_MIN_RANGE, SEARCH_MIN_VALID_PX,
                            UPPER_YELLOW)

# Yaw rate above which frames are discarded.
#
# This is the quantitative version of "only scan while driving straight". The
# rgb/depth synchroniser accepts up to 50 ms of skew between the two images.
# Translating during that gap is harmless -- 14 mm at full speed -- but rotating
# is not, because it moves the whole image: at fx = 224, one radian per second
# smears the pair by 11 px, and a 3 m blob is only about 6 px across. Anything
# above ~20 deg/s is rejected. Gentle MPPI heading corrections pass; a
# spin-in-place at a waypoint does not.
MAX_SCAN_OMEGA = 0.35

# Joints whose motion swings the camera. The camera is mounted on link5, so
# every one of these rotates the optical axis.
ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5")

# Joint rate above which frames are discarded, rad/s.
#
# MAX_SCAN_OMEGA covers the base turning. It does not cover the arm, and the
# arm is the bigger problem: the base is stationary for most of an arm move, so
# a gate that only watches odometry sees a perfectly still robot while the
# camera sweeps through a 2 rad arc in three seconds. Frames from the middle of
# that arc are smeared exactly like frames from a spin, and they are also
# pointed at the sky, the hopper, and the robot's own chassis on the way past.
#
# `park_arm_async` returns immediately by design, so a short hop finishes long
# before the arm reaches the parked pose -- meaning most of a short drive is
# scanned mid-swing unless this gate stops it.
#
# At rest the controllers report about 1e-13 rad/s, so anything above a
# hundredth of a radian per second is real motion. 0.05 rad/s over the 50 ms
# synchroniser skew is 0.56 px at fx = 224, comfortably below a pixel.
MAX_SCAN_JOINT_RATE = 0.05

# How long an unresolved detection waits for its capture-time transform before
# giving up and using the latest one instead.
#
# This is a staleness budget, not a patience setting, and it wants to be short.
# Measured on this stack, capture-time lookups fail with "extrapolation into
# the future" -- the camera's stamps run ahead of the latest TF, so the
# transform for a given frame does not exist yet and the only question is how
# long to wait for it. AMCL post-dates map->odom by a full second
# (transform_tolerance), so the gap is downstream of that: the EKF publishes
# odom->base_link at its configured 25 Hz with no post-dating at all, so a
# stamp landing past the newest sample has nothing to interpolate against.
#
# Waiting two seconds meant the fallback, when it fired, used a transform two
# seconds after capture -- worse than the problem it was working around. One
# and a half frame periods gives TF a couple of chances to catch up and caps
# the fallback error at roughly 100 mm even at full speed.
PENDING_TTL = 0.35


class ShuttleScanner:
    """Watches the camera and files what it sees into a TargetMap.

    Composed onto an existing node rather than being one, so it can ride along
    inside a harness that already owns a node, a TF buffer and an executor.
    """

    def __init__(self, node, target_map, tf_buffer=None):
        self.node = node
        self.map = target_map
        self.tf_buffer = tf_buffer or tf2_ros.Buffer()
        self._owns_listener = tf_buffer is None
        if self._owns_listener:
            self._tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self.bridge = CvBridge()
        self.camera_info = None
        self.enabled = False
        self.omega = 0.0
        self.arm_rate = 0.0

        self.pending = collections.deque()
        self.frames_seen = 0
        self.frames_spun = 0      # rejected for turning too fast
        self.frames_armed = 0     # rejected for the arm being in motion
        self.blobs_filed = 0
        self.blobs_stale = 0      # transform never arrived
        self.blobs_late = 0       # resolved against the latest transform instead
        self.tf_reason = None     # why capture-time lookups are failing
        self.blobs_suppressed = 0  # landed on a retired target

        node.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._info_cb, 10)
        node.create_subscription(Odometry, "/odometry/filtered", self._odom_cb, 10)
        node.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        rgb = message_filters.Subscriber(node, Image, RGB_TOPIC)
        depth = message_filters.Subscriber(node, Image, DEPTH_TOPIC)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb, depth], queue_size=5, slop=0.05
        )
        self._sync.registerCallback(self._image_cb)

    # --- control ---------------------------------------------------------

    def enable(self):
        """Start filing detections. Safe to call when already enabled."""
        self.enabled = True

    def divert(self, target_map):
        """File into a different map for a while. Returns the previous one.

        For the stationary re-look, which wants its detections kept apart from
        the running map until it has decided which cluster is the target it
        went looking for. Unlike retarget this leaves the counters alone, so
        the per-trial scanner statistics still cover the whole trial.
        """
        previous = self.map
        self.map = target_map
        self.pending.clear()
        return previous

    def retarget(self, target_map):
        """Point the scanner at a fresh map and reset the counters.

        Constructing a second scanner instead would attach a second set of
        subscriptions to the same node, since the scanner is composed onto a
        node it does not own. Across a multi-trial run that stacks up a
        duplicate image pipeline per trial -- every frame decoded and
        thresholded N times over -- which is invisible until the trial count
        gets high enough to matter.
        """
        self.map = target_map
        self.pending.clear()
        self.enabled = False
        self.frames_seen = self.frames_spun = self.frames_armed = 0
        self.blobs_filed = self.blobs_stale = self.blobs_suppressed = 0
        self.blobs_late = 0

    def disable(self):
        """Stop filing detections, and abandon anything still queued.

        Called around the carry to the hopper. The queued detections are
        dropped rather than kept because they were captured before whatever
        made the caller stop scanning.
        """
        self.enabled = False
        self.pending.clear()

    @property
    def arm_settled(self):
        """True when the arm is still enough for a frame to be worth using."""
        return self.arm_rate <= MAX_SCAN_JOINT_RATE

    def stats(self):
        return (f"frames {self.frames_seen} "
                f"(spun {self.frames_spun}, arm {self.frames_armed}), "
                f"filed {self.blobs_filed} (late {self.blobs_late}), "
                f"stale {self.blobs_stale}, "
                f"suppressed {self.blobs_suppressed}"
                + (f", tf: {self.tf_reason}" if self.tf_reason else ""))

    # --- callbacks -------------------------------------------------------

    def _info_cb(self, msg):
        self.camera_info = msg

    def _odom_cb(self, msg):
        self.omega = msg.twist.twist.angular.z

    def _joint_cb(self, msg):
        rate = 0.0
        for name, vel in zip(msg.name, msg.velocity):
            if name in ARM_JOINTS:
                rate = max(rate, abs(vel))
        self.arm_rate = rate

    def _image_cb(self, rgb_msg, depth_msg):
        # Drain first, unconditionally. A detection queued just before the
        # scanner was disabled still deserves its transform, and draining costs
        # nothing when the queue is empty.
        self._drain()
        if not self.enabled or self.camera_info is None:
            return
        self.frames_seen += 1
        if abs(self.omega) > MAX_SCAN_OMEGA:
            self.frames_spun += 1
            return
        if self.arm_rate > MAX_SCAN_JOINT_RATE:
            self.frames_armed += 1
            return

        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV),
                           LOWER_YELLOW, UPPER_YELLOW)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")

        for contour in contours:
            if cv2.contourArea(contour) < SEARCH_MIN_CONTOUR_AREA:
                continue
            measured = self._measure(contour, depth_img)
            if measured is None:
                continue
            u, v, d = measured
            point = deproject(self.camera_info.k, u, v, d)
            if point is None:
                continue
            # Queued rather than transformed inline: the transform for this
            # exact stamp may not have arrived yet, and blocking for it is the
            # deadlock this whole module exists to avoid.
            self.pending.append((depth_msg.header, point, d))

        self._drain()

    # --- internals -------------------------------------------------------

    def _measure(self, contour, depth_img):
        """(u, v, range) for one blob, or None if it fails the search tier.

        Cropped to the contour's bounding box rather than masking the full
        frame. This runs on every blob of every frame while the robot drives,
        and a scattered court puts several in view at once; allocating a
        640x360 buffer per contour was measurable.
        """
        x, y, w, h = cv2.boundingRect(contour)
        sub = depth_img[y:y + h, x:x + w]
        blob = np.zeros((h, w), np.uint8)
        cv2.drawContours(blob, [contour - (x, y)], -1, 255, cv2.FILLED)

        valid = (blob > 0) & np.isfinite(sub) & (sub > 0.0)
        if int(valid.sum()) < SEARCH_MIN_VALID_PX:
            return None
        ys, xs = np.nonzero(valid)
        d = float(np.median(sub[valid]))
        if not (SEARCH_MIN_RANGE <= d <= MAX_DETECT_RANGE):
            return None
        return float(xs.mean()) + x, float(ys.mean()) + y, d

    def _drain(self):
        """Resolve queued detections whose capture-time transform has landed."""
        if not self.pending:
            return
        now = self.node.get_clock().now()
        keep = collections.deque()

        while self.pending:
            header, point, rng = self.pending.popleft()
            try:
                tf = self.tf_buffer.lookup_transform(
                    MAP_FRAME, header.frame_id,
                    rclpy.time.Time.from_msg(header.stamp)
                )
            except tf2_ros.TransformException as ex:
                if self.tf_reason is None:
                    self.tf_reason = str(ex)[:150]
                age = (now - rclpy.time.Time.from_msg(header.stamp)).nanoseconds / 1e9
                if age < PENDING_TTL:
                    keep.append((header, point, rng))
                    continue
                # Out of patience. Rather than bin the detection, fall back to
                # the latest transform available.
                #
                # This is the very shortcut the module exists to avoid, so it is
                # worth being clear about why it is acceptable *here*. A frame
                # only reaches this point if it already passed both motion
                # gates, so the base is turning slower than 0.35 rad/s and the
                # arm is still. What remains is translation during the image's
                # age, which at full speed is 95 mm on the worst frame measured
                # and a few millimetres on a typical one -- comfortably inside
                # search-grade tolerance, and far better than the alternative of
                # discarding the detection outright. Discarding was costing
                # roughly a third of all sightings, which showed up as
                # shuttlecocks confirmed on the bare minimum three hits, or not
                # confirmed at all.
                try:
                    tf = self.tf_buffer.lookup_transform(
                        MAP_FRAME, header.frame_id, rclpy.time.Time())
                except tf2_ros.TransformException:
                    self.blobs_stale += 1
                    continue
                self.blobs_late += 1

            stamped = PointStamped()
            stamped.header = header
            stamped.point.x, stamped.point.y, stamped.point.z = point
            world = do_transform_point(stamped, tf)
            hit = self.map.observe(world.point.x, world.point.y,
                                   world.point.z, rng)
            if hit is None:
                self.blobs_suppressed += 1
            else:
                self.blobs_filed += 1

        self.pending = keep
