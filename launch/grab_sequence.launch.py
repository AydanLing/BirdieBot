import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rosbot_xl", package_name="rosbot_moveit")
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("grab_sequence"), "config", "moveit_cpp.yaml"
            )
        )
        .to_moveit_configs()
    )
    # manipulation_pro.yaml, NOT manipulation.yaml. The simulation is brought up
    # with configuration:=manipulation_pro, and pointing MoveIt at the plain
    # config gives it a different robot: the lidar in its old rear-deck spot at
    # (-0.125, 0, 0.07) and no ZED on link5 at all.
    #
    # The cost of that divergence was days of it. Every deposit into the hopper
    # failed with "Found a contact between 'rplidar_link' and
    # 'gripper_left_link'", the lidar was moved four times to get away from it,
    # and none of it helped, because the obstacle was a phantom in MoveIt's
    # model sitting where the hopper now is. Proof: parking the real lidar a
    # metre in the air left the sim reporting it at z +1.133 while the planner
    # went on colliding with it.
    components_config = PathJoinSubstitution(
        [FindPackageShare("rosbot_description"), "config", "rosbot_xl", "manipulation_pro.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("rosbot_description"), "urdf", "rosbot_xl.urdf.xacro"]),
            " components_config:=", components_config,
            " configuration:='manipulation_pro'",
        ]
    )
    moveit_config.robot_description = {"robot_description": robot_description_content}

    grab_sequence_node = Node(
        package="grab_sequence",
        executable="grab_sequence",
        output="screen",
        parameters=[moveit_config.to_dict(),  {"use_sim_time": True}],
    )
    return LaunchDescription([grab_sequence_node])
