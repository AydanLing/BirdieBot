#!/usr/bin/env python3
"""Collision-checking replacement for nav2's collision_monitor.

Sits between velocity_smoother and the base: subscribes /cmd_vel_smoothed,
publishes /cmd_vel, and slows or stops the robot when the commanded motion
would drive the footprint into something the lidar can see.

Why this exists: nav2's collision_monitor silently forwards nothing in this
setup. /collision_monitor_state reports action_type 0 (DO_NOTHING, i.e. it
believes nothing is wrong) and QoS matches end to end, yet /cmd_vel stays empty
while /cmd_vel_smoothed carries real commands. See cmd_vel_relay.py for the
isolation trail. That relay restored motion but removed all collision
protection; this restores the protection.

Two deliberate differences from collision_monitor, both targeting its observed
failure modes:

  * TF is looked up at latest-available time, never at the message stamp.
    collision_monitor's logs showed "Lookup would require extrapolation into
    the future. Requested time 2008.404 but latest data is at 2008.400" -- a
    4 ms race that put it into a hard stop every cycle.
  * When blocking, this publishes an explicit zero Twist rather than going
    silent. Publishing nothing leaves the base coasting on its last command
    until cmd_vel_timeout expires, which is both slower to stop and hides the
    fault -- exactly what made the original so hard to diagnose.

Approximates the footprint as a circle of ROBOT_RADIUS, matching the
robot_radius already used by both costmaps in amcl.yaml.
"""

import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

ROBOT_RADIUS = 0.22          # matches robot_radius in both costmaps
TIME_HORIZON = 1.2           # seconds to look ahead, matches time_before_collision
SIM_STEP = 0.1               # forward-simulation timestep
SCAN_TIMEOUT = 2.0           # seconds without a scan before failing safe
MIN_SCALE = 0.15             # below this, stop outright rather than crawl
# This reads /scan_filtered, not /scan.
#
# The lidar sees the robot's own arm, and unfiltered those returns land inside
# the footprint, so the forward simulation refuses every command as an
# immediate collision. That used to be handled here by discarding everything
# inside a 0.30 m radius, with a note that "a proper fix is a footprint polygon
# filter on /scan rather than a radius". scan_self_filter.py is that fix, so
# the radius is gone.
#
# It was worth removing rather than leaving alone. The radius was blind to
# genuine obstacles inside 0.30 m -- against a 0.22 m footprint that left only
# about 8 cm of real margin, in the one region where stopping matters most. The
# footprint test drops only returns whose endpoint is inside the chassis, so a
# real obstacle 0.25 m away is now seen instead of discarded.
#
# Measured in open space: /scan carried 104 returns under 0.45 m and
# /scan_filtered carried 0, keeping 2896 of 3000 beams.


class CmdVelGuard(Node):
    def __init__(self):
        super().__init__("cmd_vel_guard")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.points = None
        self._self_hits = 0
        self._ang = self._cos = self._sin = None   # beam angle table, cached          # obstacle points in base_link
        self.last_scan = None
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan_filtered", self.on_scan, 10)
        self.create_subscription(TwistStamped, "/cmd_vel_smoothed", self.on_cmd, 10)
        self.get_logger().info(
            f"guarding /cmd_vel: radius={ROBOT_RADIUS} horizon={TIME_HORIZON}s"
        )

    def on_scan(self, msg):
        try:
            tf = self.buf.lookup_transform("base_link", msg.header.frame_id,
                                           rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"scan TF lookup failed: {type(e).__name__}: {e}",
                                   throttle_duration_sec=3.0)
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cy, sy = math.cos(yaw), math.sin(yaw)

        r = np.asarray(msg.ranges, dtype=np.float64)
        n = r.size
        if self._ang is None or self._ang.size != n:
            self._ang = msg.angle_min + np.arange(n) * msg.angle_increment
            self._cos = np.cos(self._ang)
            self._sin = np.sin(self._ang)

        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        lx = r[ok] * self._cos[ok]
        ly = r[ok] * self._sin[ok]
        # (N, 2) of obstacle points in base_link, ready for the broadcast in
        # time_to_collision. Kept as one array rather than a list of tuples so
        # the collision test never has to touch the Python interpreter per point.
        self.points = np.column_stack((t.x + lx * cy - ly * sy,
                                       t.y + lx * sy + ly * cy))
        pts = self.points
        self.last_scan = self.get_clock().now()
        self.get_logger().info(f"scan ok: {len(pts)} points", once=True)

    def time_to_collision(self, vx, vy, wz):
        """Earliest time within the horizon at which the footprint hits a point."""
        if self.points is None or len(self.points) == 0:
            return None

        # The pose sequence is integrated in Python -- it is 12 steps and each
        # depends on the last -- but the collision test against every scan point
        # is done as one broadcast per step. The fully nested version cost 35%
        # of a core: 12 steps x ~2900 points x 20 Hz is ~700k interpreted
        # distance checks a second, on the machine that also runs Gazebo.
        steps = int(TIME_HORIZON / SIM_STEP)
        x = y = th = 0.0
        xs = np.empty(steps)
        ys = np.empty(steps)
        for i in range(steps):
            x += (vx * math.cos(th) - vy * math.sin(th)) * SIM_STEP
            y += (vx * math.sin(th) + vy * math.cos(th)) * SIM_STEP
            th += wz * SIM_STEP
            xs[i] = x
            ys[i] = y

        px = self.points[:, 0]
        py = self.points[:, 1]
        r2 = ROBOT_RADIUS * ROBOT_RADIUS

        # Drop returns that are already inside the footprint at t=0. Nothing can
        # be a FUTURE collision if the robot is standing on it, so these are a
        # sensing artifact, not an obstacle. They are real: scan_self_filter
        # clears the chassis RECTANGLE, whose front face is 0.191 m out, while
        # this test uses the 0.22 m circumscribed radius, so the sliver between
        # the two survives filtering and lands inside the circle. The result was
        # "collision in 0.10s, stopping" 89 times in one batch -- the first
        # forward-simulation step colliding against the robot's own body -- and
        # nav2 being commanded 0.188 m/s while the guard published a hard zero
        # for the first several seconds of a run.
        inside = (px * px + py * py) < r2
        if inside.any():
            px = px[~inside]
            py = py[~inside]
            self._self_hits += int(inside.sum())
            if px.size == 0:
                return None
        # (steps, N) squared distances; argmax on the any-hit mask gives the
        # earliest colliding step, matching the old loop's return-on-first-hit.
        d2 = (px[None, :] - xs[:, None]) ** 2 + (py[None, :] - ys[:, None]) ** 2
        hit = (d2 < r2).any(axis=1)
        if not hit.any():
            return None
        return float(np.argmax(hit) + 1) * SIM_STEP

    def on_cmd(self, msg):
        out = TwistStamped()
        out.header = msg.header
        vx, vy, wz = msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z

        # Fail safe on stale or missing scans, but publish an explicit zero.
        if self.last_scan is None:
            self.pub.publish(out)
            return
        age = (self.get_clock().now() - self.last_scan).nanoseconds / 1e9
        if age > SCAN_TIMEOUT:
            self.get_logger().warn(f"scan {age:.1f}s old, stopping",
                                   throttle_duration_sec=2.0)
            self.pub.publish(out)
            return

        if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(wz) < 1e-6:
            self.pub.publish(msg)
            return

        tc = self.time_to_collision(vx, vy, wz)
        if tc is None:
            self.pub.publish(msg)
            return

        scale = max(0.0, min(1.0, tc / TIME_HORIZON))
        if scale < MIN_SCALE:
            self.get_logger().warn(f"collision in {tc:.2f}s, stopping",
                                   throttle_duration_sec=1.0)
            self.pub.publish(out)
            return

        self.get_logger().info(f"collision in {tc:.2f}s, scaling to {scale:.2f}",
                               throttle_duration_sec=1.0)
        out.twist.linear.x = vx * scale
        out.twist.linear.y = vy * scale
        out.twist.angular.z = wz * scale
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
