import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
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
    components_config = PathJoinSubstitution(
        [FindPackageShare("rosbot_description"), "config", "rosbot_xl", "manipulation.yaml"]
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

    ball_xyz_arg = DeclareLaunchArgument(
        "ball_xyz",
        default_value="",
        description='Optional "x,y,z" in base_link: skip detection and base motion, grasp there.',
    )

    grasp_ball_node = Node(
        package="grab_sequence",
        executable="grasp_ball",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True, "ball_xyz": LaunchConfiguration("ball_xyz")},
        ],
    )
    return LaunchDescription([ball_xyz_arg, grasp_ball_node])
