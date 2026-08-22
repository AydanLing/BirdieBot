#!/usr/bin/env python3
"""Drop laser returns that land on the robot itself, republish the rest.

The lidar sees the arm. Measured by capturing /scan from four different robot
poses and keeping only the beams that saw something closer than 0.45 m from
EVERY pose -- anything that survives that is mounted on the robot, not in the
world:

    104 beams, at 0.090..0.198 m, spanning +-174..180 deg in the laser frame

The angles are misleading until you check the frame. base_link -> laser is
yaw 3.142, so the laser frame is rotated 180 deg and its "+-180 deg" is DEAD
AHEAD in base_link, not behind. Those returns land at x -0.035..+0.073, which
is the arm mount at x=+0.065. So the robot blanks a ~12 deg wedge directly in
front of itself, in the direction it drives.

config/rosbot_xl/manipulation.yaml raises the lidar 0.07 specifically to clear
"the arm's base column, which spans z 0.133..0.193". The lidar sits at
z=0.234, so it does clear the column -- but the parked arm folds back over
itself and some of it reaches lidar height anyway.

Why this matters: the costmap's scan source runs obstacle_min_range 0.0, so
those returns are marked as lethal obstacles a few centimetres in front of the
robot, permanently, and they travel with it. nav2's MPPI then reports
collisions on trajectories that are actually clear.

Why a footprint test rather than a minimum range: a blanket obstacle_min_range
would blind the costmap in every direction, including toward real obstacles the
robot is about to hit. Every self-return endpoint falls inside the chassis
rectangle, so testing the endpoint against that rectangle removes exactly the
self-hits and keeps everything outside the body -- in any direction, at any
range, and without hardcoding a sector that would go stale if the arm or the
lidar moved again.

Publishes /scan_filtered. Point the costmap observation sources at that.
"""

import math

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# Chassis extent in base_link, from rosbot_xl_macro.urdf.xacro and the
# manipulation.yaml comments: the body ends at x=+0.161 and is 0.135 wide
# either side, with the rear edge at -0.167. MARGIN covers the arm and the
# mounts that overhang the bare box slightly.
BODY_X_MIN = -0.167
BODY_X_MAX = 0.161
BODY_Y_ABS = 0.135
MARGIN = 0.030

# Beyond this there is nothing of the robot to hit, so the footprint test is
# skipped entirely and a distant return can never be discarded by a numerical
# accident in the transform.
MAX_SELF_RANGE = 0.45


class ScanSelfFilter(Node):
    def __init__(self):
        super().__init__("scan_self_filter")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.tf = None          # base_link <- laser, cached; it is static
        self.dropped = 0
        self.kept = 0
        # RELIABLE, matching the gz bridge's /scan, so /scan_filtered is a
        # drop-in replacement. Publishing BEST_EFFORT here silently starved
        # every RELIABLE subscriber: cmd_vel_guard reported "New publisher
        # discovered on topic '/scan_filtered', offering incompatible QoS. No
        # messages will be received from it." and then sat with no scan at all.
        # A RELIABLE publisher satisfies BEST_EFFORT subscribers too, so nav2's
        # SensorDataQoS costmap readers are still fine.
        self.pub = self.create_publisher(LaserScan, "/scan_filtered", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            f"filtering /scan -> /scan_filtered; body x {BODY_X_MIN}..{BODY_X_MAX}, "
            f"|y| < {BODY_Y_ABS}, margin {MARGIN}")

    def _lookup(self, frame):
        """Cache base_link <- laser. It is a static transform; one success is enough."""
        if self.tf is not None:
            return self.tf
        try:
            t = self.buf.lookup_transform("base_link", frame, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f"TF base_link <- {frame}: {type(exc).__name__}",
                                   throttle_duration_sec=5.0)
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.tf = (t.transform.translation.x, t.transform.translation.y,
                   math.cos(yaw), math.sin(yaw))
        self.get_logger().info(
            f"laser is at ({self.tf[0]:+.3f}, {self.tf[1]:+.3f}) yaw "
            f"{math.degrees(yaw):+.1f} deg in base_link")
        return self.tf

    def on_scan(self, msg):
        tf = self._lookup(msg.header.frame_id)
        if tf is None:
            # Republish untouched rather than dropping the scan. A costmap with
            # no scan at all is worse than one with the self-hits still in it.
            self.pub.publish(msg)
            return
        tx, ty, c, s = tf

        out = list(msg.ranges)
        ang = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and msg.range_min < r < MAX_SELF_RANGE:
                lx, ly = r * math.cos(ang), r * math.sin(ang)
                bx = tx + lx * c - ly * s
                by = ty + lx * s + ly * c
                if (BODY_X_MIN - MARGIN <= bx <= BODY_X_MAX + MARGIN
                        and abs(by) <= BODY_Y_ABS + MARGIN):
                    out[i] = float("inf")
                    self.dropped += 1
                else:
                    self.kept += 1
            ang += msg.angle_increment

        msg.ranges = out
        self.pub.publish(msg)

    def _report(self):
        total = self.dropped + self.kept
        if total:
            self.get_logger().info(
                f"near returns: {self.dropped} dropped as self, {self.kept} kept "
                f"({100.0 * self.dropped / total:.0f}% dropped)")
        self.dropped = self.kept = 0


def main():
    rclpy.init()
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
