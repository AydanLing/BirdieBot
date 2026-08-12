import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy


def main():
    rclpy.init()
    moveit = MoveItPy(node_name="probe_reach_moveit")
    arm = moveit.get_planning_component("manipulator")
    time.sleep(3.0)

    candidates = [
        (x, z, pitch_deg)
        for x in (0.20, 0.30, 0.40)
        for z in (0.03,)
        for pitch_deg in (30, 45, 60, 75)
    ]

    for x, z, pitch_deg in candidates:
        pitch = math.radians(pitch_deg)
        sp, cp = math.sin(pitch / 2.0), math.cos(pitch / 2.0)
        qx, qy, qz, qw = 0.0, sp, 0.0, cp

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = 0.0
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        arm.set_start_state_to_current_state()
        arm.set_goal_state(pose_stamped_msg=pose, pose_link="end_effector_link")
        t0 = time.monotonic()
        result = arm.plan()
        dt = time.monotonic() - t0
        print(
            f"PROBE x={x:.2f} z={z:.2f} pitch={pitch_deg}deg -> {'OK' if result else 'FAIL'} ({dt:.1f}s)",
            flush=True,
        )

    rclpy.shutdown()


if __name__ == "__main__":
    main()
