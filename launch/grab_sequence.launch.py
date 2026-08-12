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

    grab_sequence_node = Node(
        package="grab_sequence",
        executable="grab_sequence",
        output="screen",
        parameters=[moveit_config.to_dict(),  {"use_sim_time": True}],
    )
    return LaunchDescription([grab_sequence_node])