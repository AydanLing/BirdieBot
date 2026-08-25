#!/bin/bash
# Bring the whole stack up, in the one order that works.
#
#     gazebo -> scan filter + guard -> localization -> SEED AMCL -> navigation
#
# The seed MUST land before navigation launches. nav2's global_costmap refuses
# to activate without map->base_link ("Failed to activate global_costmap because
# transform..."), that fails planner_server, and the lifecycle manager then
# abandons the rest of the bringup. Seeding after launching navigation is the
# intuitive order and it races the costmap's activation timeout; it is the
# cause of every half-active stack this project has produced.
#
# Usage: bringup.sh [world] [map]      defaults: badminton_court
# -e but deliberately NOT -u: /opt/ros/jazzy/setup.bash references
# AMENT_TRACE_SETUP_FILES unset, so `set -u` kills the script on the first
# source line. Found by running this script rather than by reading it.
set -e

WS=/home/aydan-ling/rosbot_ws
WORLD=${1:-badminton_court}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP=${2:-$WS/install/husarion_gz_worlds/share/husarion_gz_worlds/maps/$WORLD.yaml}
PARAMS=$WS/src/grab_sequence/config/amcl.yaml
LOGS=${BRINGUP_LOGS:-/tmp/rosbot_bringup}
mkdir -p "$LOGS"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export RCUTILS_LOGGING_SEVERITY=${RCUTILS_LOGGING_SEVERITY:-INFO}

# Headless by default. The machine is 8-core and has crashed outright at load
# 33.7; a full stack plus a trial batch already sits at 20-38 without a GUI.
HEADLESS=${GZ_HEADLESS:-True}

say() { echo "bringup: $*"; }

say "1/5 gazebo ($WORLD, headless=$HEADLESS)"
export GZ_SIM_RESOURCE_PATH=$WS/install/husarion_gz_worlds/share/husarion_gz_worlds/models:${GZ_SIM_RESOURCE_PATH:-}
setsid ros2 launch rosbot_gazebo simulation.yaml \
  robot_model:=rosbot_xl configuration:=manipulation_pro \
  gz_world:="$WORLD" gz_headless_mode:="$HEADLESS" \
  > "$LOGS/sim.log" 2>&1 < /dev/null &
sleep 60
pgrep -f "[g]z sim" > /dev/null || { say "gazebo did not start, see $LOGS/sim.log"; exit 1; }

say "2/5 scan filter + cmd_vel guard"
setsid python3 "$WS/src/grab_sequence/scripts/scan_self_filter.py" \
  --ros-args -p use_sim_time:=true > "$LOGS/filter.log" 2>&1 < /dev/null &
setsid python3 "$WS/src/grab_sequence/scripts/cmd_vel_guard.py" \
  --ros-args -p use_sim_time:=true > "$LOGS/guard.log" 2>&1 < /dev/null &
sleep 12

say "3/5 localization"
# autostart:=false on purpose. bond_timeout is read when a bond is CREATED,
# which happens at activation, so with autostart the managers bond before
# anything can change the parameter and a later set is accepted but inert.
setsid ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=true map:="$MAP" params_file:="$PARAMS" autostart:=false \
  > "$LOGS/localization.log" 2>&1 < /dev/null &
sleep 35
python3 "$HERE/arm_lifecycle.py" lifecycle_manager_localization map_server,amcl 25 \
  || { say "localization would not start"; exit 1; }
sleep 8

say "4/5 seeding AMCL (must succeed before navigation)"
if ! python3 "$HERE/seed_amcl.py" 0 0 0; then
  say "AMCL would not seed; not launching navigation. See $LOGS/localization.log"
  exit 1
fi

say "5/5 navigation"
setsid ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true params_file:="$PARAMS" autostart:=false \
  > "$LOGS/navigation.log" 2>&1 < /dev/null &
sleep 35
python3 "$HERE/arm_lifecycle.py" lifecycle_manager_navigation controller_server,smoother_server,planner_server,behavior_server,bt_navigator,waypoint_follower,velocity_smoother 25 \
  || say "WARNING: navigation did not start cleanly"
sleep 8


say "logs in $LOGS -- load $(cut -d' ' -f1-2 /proc/loadavg)"
say "done. Verify with: ros2 lifecycle get /controller_server"
