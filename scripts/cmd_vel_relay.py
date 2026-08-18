#!/usr/bin/env python3
"""Bridge /cmd_vel_smoothed straight to /cmd_vel, bypassing collision_monitor.

nav2's collision_monitor is broken in this setup: /collision_monitor_state
reports action_type: 0 (DO_NOTHING, i.e. it believes nothing is wrong) and QoS
between velocity_smoother and collision_monitor matches exactly (both
RELIABLE/VOLATILE), yet it forwards zero messages to /cmd_vel regardless.
Isolated by tracing values through the full chain during a live nav2 goal:

  /cmd_vel_nav       (controller_server, MPPI raw output) -- real, ~0.18 m/s
  /cmd_vel_smoothed  (velocity_smoother output)            -- identical, passes through
  /cmd_vel           (collision_monitor output)             -- zero messages, always

With collision_monitor deactivated and this relay running instead, the same
goal drove the robot from (0,0) to (1.80,-0.06) against a (2.0,0.0) target in
one continuous run. Not yet root-caused inside collision_monitor itself; this
is a workaround, not a fix, and it means the safety-stop layer is off. Do not
use this on hardware or anywhere collision_monitor's slowdown/stop behaviour
matters until the real cause is found.

Run after deactivating collision_monitor:
    ros2 lifecycle set /collision_monitor deactivate
    python3 cmd_vel_relay.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class Relay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(TwistStamped, '/cmd_vel_smoothed', self.cb, 10)
    def cb(self, msg):
        self.pub.publish(msg)

rclpy.init()
rclpy.spin(Relay())
