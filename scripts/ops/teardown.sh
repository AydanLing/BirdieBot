#!/bin/bash
# Stop everything this workspace starts, completely.
#
# SIGKILL, not SIGINT. SIGINT was tried and measured: roughly 28 children
# survive it, and a half-dead stack behaves in ways that look like robot bugs.
#
# The pattern list matters more than it looks. Four processes are named after
# their packages rather than after ros2/nav2/gz, so an obvious list misses all
# of them:
#
#     ekf_node (robot_localization)  joy_node  teleop_node  opennav_docking
#
# The EKF is the one that costs you. It has survived several supposedly clean
# restarts carrying a diverged state -- /odometry/filtered reading
# (-2239, -8783) with the robot sitting on the origin -- which feeds AMCL a
# garbage motion model and leaves it lost with covariance 93. Two separate
# debugging sessions went into measuring that instead of the actual bug.
#
# Note the shell trap too: `pkill -f <pattern>` matches the command line of the
# shell running it, so a naive call kills itself and reports exit 143/144 with
# no output. Everything here collects numeric PIDs first and skips its own
# process tree.
set -u

SELF=$$
PATTERNS=(
  # harnesses
  nav_grasp_trials collect_trials repeatability_test tipped_trials
  # this package's helper nodes
  scan_self_filter cmd_vel_guard grab_sequence
  # simulator
  "gz sim" gz-sim gz_sim ruby parameter_bridge
  # ros2 / nav2 / moveit
  nav2_ ros2 rviz move_group moveit robot_state_publisher
  controller_manager spawner twist_mux
  # the ones a naive pattern list misses
  ekf_node robot_localization joy_node joy2servo teleop_twist_joy teleop_node
  opennav_docking docking_server
)

for pat in "${PATTERNS[@]}"; do
  for p in $(ps -eo pid,args | grep -F -- "$pat" | grep -v grep | awk '{print $1}'); do
    [ "$p" = "$SELF" ] && continue
    [ "$p" = "$PPID" ] && continue
    kill -9 "$p" 2>/dev/null
  done
done

sleep 4

# Only after every process is dead. Orphaned DDS segments block port
# allocation, and the symptom is "Failed init_port fastrtps_portNNNN" with
# topics apparently flowing while lifecycle queries return nothing at all.
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

# Re-check rather than accuse on the first look. A process caught mid-exit still
# appears in ps, and a spurious "NOT CLEAN" sends you hunting for a survivor
# that is already gone.
check() {
  ps -eo args | grep -cE '[g]z sim|[n]av2_|[e]kf_node|[m]ove_group|[s]can_self|[c]md_vel_guard'
}
left=$(check)
for _ in 1 2 3; do
  [ "$left" -eq 0 ] && break
  sleep 2
  left=$(check)
done
shm=$(ls /dev/shm/ 2>/dev/null | grep -c fastrtps)
echo "teardown: $left processes left, $shm shm segments left, load $(cut -d' ' -f1-2 /proc/loadavg)"
if [ "$left" -eq 0 ] && [ "$shm" -eq 0 ]; then
  echo "teardown: clean"
else
  echo "teardown: NOT CLEAN -- survivors below, rerun"
  ps -eo pid,args | grep -E '[g]z sim|[n]av2_|[e]kf_node|[m]ove_group|[s]can_self|[c]md_vel_guard' | cut -c1-120
fi
