#!/usr/bin/env python3
"""Seed AMCL at a pose and verify map->base_link actually appears.

Usage: seed_amcl.py [x] [y] [yaw_deg]   (defaults 0 0 0)

Two things here are deliberate.

ZERO STAMP. The obvious implementation stamps the pose with the current sim
time, and AMCL then needs a TF at exactly that instant. Straight after a
set_pose teleport the odom TF the teleport invalidated has not caught up, and
AMCL rejects every publish with "Failed to transform initial pose in time
(Lookup would require extrapolation)". A zero stamp asks TF for the latest
available transform instead, which is what this actually wants.

VERIFY, DO NOT ASSUME. An unseeded AMCL publishes no map->base_link at all.
Nothing downstream announces this: nav2's global_costmap simply refuses to
activate, which fails planner_server, which makes the lifecycle manager
abandon the rest of the bringup -- that is the origin of every half-active
stack in this project, controller_server active while the other five sit
inactive. Worse, a harness that computes goals from a stale pose runs a whole
trial with the objects metres from where the robot believes they are.
"""
import math
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def main():
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    yaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0

    rclpy.init()
    n = Node("amcl_seeder")
    n.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

    q = QoSProfile(depth=1)
    q.durability = DurabilityPolicy.TRANSIENT_LOCAL
    pub = n.create_publisher(PoseWithCovarianceStamped, "/initialpose", q)

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)

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

    for attempt in range(1, 5):
        for _ in range(8):
            m.header.stamp = rclpy.time.Time().to_msg()   # 0 == latest available
            pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.25)
        deadline = time.time() + 8.0
        while time.time() < deadline:
            rclpy.spin_once(n, timeout_sec=0.3)
            try:
                t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
                p = t.transform.translation
                print(f"seed_amcl: map->base_link OK at ({p.x:+.3f}, {p.y:+.3f}) "
                      f"after attempt {attempt}")
                rclpy.try_shutdown()
                return 0
            except Exception:
                pass
        print(f"seed_amcl: no transform after attempt {attempt}, retrying")

    print("seed_amcl: FAILED -- AMCL never published map->base_link. "
          "Do not launch navigation; its costmaps will not activate.")
    rclpy.try_shutdown()
    return 1


if __name__ == "__main__":
    sys.exit(main())
