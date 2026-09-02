"""Bring the whole stack up on the badminton court and run grasp trials.

Replaces scripts/ops/bringup.sh, which did the same thing but reached into the
source tree by absolute path and could only be run from a shell in this
workspace.

The ordering is the entire point of this file, not the node list:

    gazebo -> scan filter + guard -> localization -> SEED AMCL -> navigation

The seed MUST land before navigation starts. nav2's global_costmap refuses to
activate without map->base_link ("Failed to activate global_costmap because
transform..."), that failure takes planner_server with it, and the lifecycle
manager then abandons the rest of the bringup. Seeding after launching
navigation is the intuitive order and it races the costmap's activation
timeout; it is the cause of every half-active stack this project has produced.

Both nav2 launches run with autostart:=false and are started afterwards by
arm_lifecycle.py. bond_timeout is read when a bond is CREATED, at activation,
so with autostart the managers bond before anything can change it and a later
`ros2 param set` is accepted but inert. The managers keep the heartbeat they
were born with and later declare a healthy server dead under load.

Sequencing is by TimerAction rather than process events. The gates that matter
are "is Gazebo actually simulating" and "has AMCL converged", neither of which
is an exit code or a startup line, and the delays below are the ones bringup.sh
arrived at empirically. seed_amcl.py verifies its own postcondition and fails
loudly, which is the real check.

Usage:
    ros2 launch grab_sequence grasp_trial.launch.py
    ros2 launch grab_sequence grasp_trial.launch.py headless:=False trials:=3
    ros2 launch grab_sequence grasp_trial.launch.py run_trial:=false
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Seconds after launch start. Cumulative, matching the sleeps bringup.sh used.
T_FILTER = 60.0     # gazebo needs this long before /scan is worth filtering
T_LOCAL = 72.0
T_ARM_LOCAL = 107.0
T_SEED = 115.0
# Measured from the seed's exit rather than from launch, since navigation is
# chained to it.
T_NAV_ARM_DELAY = 35.0
T_NAV_TRIAL_DELAY = 50.0

NAV_NODES = ("controller_server,smoother_server,planner_server,behavior_server,"
             "bt_navigator,waypoint_follower,velocity_smoother")


def generate_launch_description():
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    trials = LaunchConfiguration("trials")
    run_trial = LaunchConfiguration("run_trial")

    worlds_share = FindPackageShare("husarion_gz_worlds")
    params = PathJoinSubstitution([FindPackageShare("grab_sequence"),
                                   "config", "amcl.yaml"])
    map_yaml = PathJoinSubstitution([worlds_share, "maps",
                                     [world, ".yaml"]])

    sim = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("rosbot_gazebo"), "launch", "simulation.yaml"])),
        launch_arguments={
            "robot_model": "rosbot_xl",
            "configuration": "manipulation_pro",
            "gz_world": world,
            "gz_headless_mode": headless,
        }.items(),
    )

    filt = Node(package="grab_sequence", executable="scan_self_filter.py",
                name="scan_self_filter", output="screen",
                parameters=[{"use_sim_time": True}])
    guard = Node(package="grab_sequence", executable="cmd_vel_guard.py",
                 name="cmd_vel_guard", output="screen",
                 parameters=[{"use_sim_time": True}])

    localization = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("nav2_bringup"), "launch",
             "localization_launch.py"])),
        launch_arguments={
            "use_sim_time": "true",
            "map": map_yaml,
            "params_file": params,
            "autostart": "false",
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("nav2_bringup"), "launch",
             "navigation_launch.py"])),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": params,
            "autostart": "false",
        }.items(),
    )

    # Node rather than ExecuteProcess: these install to lib/grab_sequence, which
    # is not on PATH, and Node is what knows how to find a package executable.
    # They are one-shot rclpy programs rather than long-lived nodes, which is
    # fine; launch does not require a node to outlive its actions.
    arm_local = Node(
        package="grab_sequence", executable="arm_lifecycle.py",
        name="arm_localization", output="screen",
        arguments=["lifecycle_manager_localization", "map_server,amcl", "25"])
    seed = Node(
        package="grab_sequence", executable="seed_amcl.py",
        name="seed_amcl", output="screen", arguments=["0", "0", "0"])
    arm_nav = Node(
        package="grab_sequence", executable="arm_lifecycle.py",
        name="arm_navigation", output="screen",
        arguments=["lifecycle_manager_navigation", NAV_NODES, "25"])
    trial = Node(
        package="grab_sequence", executable="nav_grasp_trials.py",
        name="nav_grasp_trials", output="screen",
        arguments=[trials], condition=IfCondition(run_trial))

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="badminton_court",
                              description="Gazebo world and AMCL map basename."),
        DeclareLaunchArgument(
            "headless", default_value="True",
            description=("Run Gazebo without its GUI. True by default: this "
                         "machine is 8-core and has crashed outright at load "
                         "33.7, and a full stack plus a trial batch already "
                         "sits at 20-38 without one.")),
        DeclareLaunchArgument("trials", default_value="5",
                              description="Number of navigate-then-grasp trials."),
        DeclareLaunchArgument("run_trial", default_value="true",
                              description="Set false to bring the stack up and stop."),

        LogInfo(msg="grasp_trial: 1/5 gazebo"),
        sim,
        TimerAction(period=T_FILTER, actions=[
            LogInfo(msg="grasp_trial: 2/5 scan filter + cmd_vel guard"),
            filt, guard]),
        TimerAction(period=T_LOCAL, actions=[
            LogInfo(msg="grasp_trial: 3/5 localization (autostart off)"),
            localization]),
        TimerAction(period=T_ARM_LOCAL, actions=[arm_local]),
        TimerAction(period=T_SEED, actions=[
            LogInfo(msg="grasp_trial: 4/5 seeding AMCL, must precede navigation"),
            seed]),
        # Navigation waits for the seed to actually finish, and only starts if
        # it succeeded. A non-zero exit means AMCL never published
        # map->base_link, and nav2's global_costmap cannot activate without it.
        RegisterEventHandler(OnProcessExit(
            target_action=seed,
            on_exit=lambda event, context: (
                [LogInfo(msg="grasp_trial: 5/5 navigation (autostart off)"),
                 navigation,
                 TimerAction(period=T_NAV_ARM_DELAY, actions=[arm_nav]),
                 TimerAction(period=T_NAV_TRIAL_DELAY, actions=[trial])]
                if event.returncode == 0 else
                [LogInfo(msg="grasp_trial: ABORT, AMCL would not seed; not "
                             "launching navigation"),
                 EmitEvent(event=Shutdown(reason="AMCL seed failed"))]))),
    ])
