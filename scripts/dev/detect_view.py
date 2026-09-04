#!/usr/bin/env python3
"""Draw boxes on whatever the shuttlecock detector currently sees.

A tuning aid for choosing an arm pose to search from. It runs the SAME
threshold and the SAME area gate as the robot, importing both rather than
copying them, so what you see here is what the pick would actually detect. A
box drawn in green would be accepted; amber is a blob the eye can see and the
detector throws away for being under MIN_CONTOUR_AREA.

Publishes an annotated image rather than opening a window, because this session
is Wayland and cv2.imshow from a background process is unreliable there. View
it with an Image display in RViz:

    /shuttle_detect/image

Each box is labelled with its blob area in pixels and, when the depth image has
a reading there, the range. The range is what tells you whether a pose is
useful: a shuttlecock that is visible but under the gate at 3 m means the pose
sees further than the detector does, which is the thing worth knowing before
building a search around it.

Usage:
    ros2 run grab_sequence detect_view.py
    ros2 run grab_sequence detect_view.py --min-area 8    # try a looser gate
"""

import argparse
import importlib.util
import math
import os
import sys

import cv2
from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped


def _constants():
    """Read the detector's constants from the module the robot itself reads.

    A copy here would drift the moment the real thresholds were retuned, and
    this tool would then be showing something the robot does not do. This used
    to regex them out of grasp_ball.py's source, because importing grasp_ball
    loads MoveItPy. detect_params exists precisely so that is no longer needed.
    """
    spec = importlib.util.find_spec("grab_sequence.detect_params")
    if spec is None:
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    from grab_sequence import detect_params
    return detect_params


class DetectView(Node):

    def __init__(self, min_area):
        super().__init__("detect_view")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        c = _constants()
        self.lo, self.hi = c.LOWER_YELLOW, c.UPPER_YELLOW
        self.gate = c.MIN_CONTOUR_AREA if min_area is None else min_area
        self.bridge = CvBridge()
        self.info = None
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)  # noqa: F841
        self.pub = self.create_publisher(Image, "/shuttle_detect/image", 1)
        self.create_subscription(CameraInfo, c.CAMERA_INFO_TOPIC,
                                 self._info_cb, 1)
        rgb = message_filters.Subscriber(self, Image, c.RGB_TOPIC)
        dep = message_filters.Subscriber(self, Image, c.DEPTH_TOPIC)
        message_filters.ApproximateTimeSynchronizer(
            [rgb, dep], 10, 0.1).registerCallback(self._cb)
        self.get_logger().info(
            f"HSV {self.lo.tolist()}..{self.hi.tolist()}, area gate {self.gate:.0f} px^2; "
            f"annotated stream on /shuttle_detect/image")

    def _info_cb(self, msg):
        self.info = msg

    def _range_of(self, depth, mask_pixels, u, v):
        """Median depth over the blob, and the deprojected range in base_link."""
        vs, us = mask_pixels
        d = depth[vs, us]
        d = d[np.isfinite(d) & (d > 0.0)]
        if d.size == 0 or self.info is None:
            return None, None
        z = float(np.median(d))
        k = self.info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        ps = PointStamped()
        ps.header.frame_id = self.info.header.frame_id
        ps.point.x = z
        ps.point.y = -(u - cx) / fx * z
        ps.point.z = -(v - cy) / fy * z
        try:
            tf = self.buf.lookup_transform("base_link", ps.header.frame_id,
                                           rclpy.time.Time())
            b = do_transform_point(ps, tf)
            return z, math.hypot(b.point.x, b.point.y)
        except Exception:
            return z, None

    def _cb(self, rgb_msg, dep_msg):
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(dep_msg, desired_encoding="32FC1")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lo, self.hi)
        cont, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        kept = dropped = 0
        for c in cont:
            area = cv2.contourArea(c)
            if area < 1.0:
                continue                       # single-pixel speckle, not worth drawing
            passes = area >= self.gate
            x, y, w, h = cv2.boundingRect(c)
            blob = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(blob, [c], -1, 255, cv2.FILLED)
            px = np.nonzero((blob > 0) & np.isfinite(depth) & (depth > 0.0))
            u, v = (float(px[1].mean()), float(px[0].mean())) if px[0].size else (
                x + w / 2.0, y + h / 2.0)
            _z, rng = self._range_of(depth, px, u, v)

            colour = (0, 220, 0) if passes else (0, 170, 255)
            pad = 6
            cv2.rectangle(frame, (x - pad, y - pad), (x + w + pad, y + h + pad), colour, 2)
            label = f"{area:.0f}px" + (f" {rng:.2f}m" if rng else "")
            cv2.putText(frame, label, (x - pad, max(12, y - pad - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, colour, 1, cv2.LINE_AA)
            kept += passes
            dropped += not passes

        banner = f"gate {self.gate:.0f}px^2   accepted {kept}   too small {dropped}"
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 22), (0, 0, 0), -1)
        cv2.putText(frame, banner, (6, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)
        out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = rgb_msg.header
        self.pub.publish(out)
        self.get_logger().info(f"accepted {kept}, too small {dropped}",
                               throttle_duration_sec=2.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--min-area", type=float, default=None,
                    help="override MIN_CONTOUR_AREA, to see what a looser gate would catch")
    args, _ = ap.parse_known_args(
        sys.argv[1:sys.argv.index("--ros-args")] if "--ros-args" in sys.argv else sys.argv[1:])
    rclpy.init()
    node = DetectView(args.min_area)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
