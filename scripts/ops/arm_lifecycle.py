#!/usr/bin/env python3
"""Replace a nav2 lifecycle manager with one whose bond heartbeat is off.

Usage: arm_lifecycle.py <manager_name> <node1,node2,...>

bond_timeout cannot be changed at runtime. nav2's lifecycle manager reads it
into a member when the node is CONSTRUCTED, and there is no parameter callback,
so `ros2 param set .../bond_timeout 0.0` is accepted, reports success, and
changes nothing. The proof is in the manager's own log: it still prints
"Creating bond timer...", which it only does when bond_timeout > 0.

nav2_bringup's launch files do not forward params_file to the manager -- they
pass only {autostart} and {node_names} -- so there is no way to set it there
either. The only remaining option is to not use their manager: launch nav2 with
autostart:=false so its manager bonds to nothing, kill it, and run our own with
bond_timeout:=0.0, then start the stack with that.

Why bother: under load this machine starves map_server's thread for longer than
the 4 s default, the manager declares a healthy server dead --

    CRITICAL FAILURE: SERVER map_server IS DOWN after not receiving a heartbeat
    Deactivating amcl

-- and AMCL stops publishing map->base_link, after which every trial fails in
ways that look like localisation bugs. That cascade has been misdiagnosed
repeatedly in this project.
"""
import subprocess
import sys
import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node


def kill_existing(mgr):
    n = 0
    for p in subprocess.run(["ps", "-eo", "pid,args"], capture_output=True,
                            text=True).stdout.splitlines():
        if "lifecycle_manager" in p and mgr in p and "arm_lifecycle" not in p:
            pid = p.split()[0]
            subprocess.run(["kill", "-9", pid], capture_output=True)
            n += 1
    return n


def _argv():
    """sys.argv without ROS's own arguments.

    launch_ros appends "--ros-args ..." to every node it starts, so positional
    parsing has to stop there or it reads a flag as a value.
    """
    a = sys.argv
    return a[:a.index("--ros-args")] if "--ros-args" in a else a


def main():
    mgr = _argv()[1]
    nodes = _argv()[2].split(",")
    timeout = float(_argv()[3]) if len(_argv()) > 3 else 25.0

    killed = kill_existing(mgr)
    print(f"arm_lifecycle: replaced {killed} stock {mgr}")
    time.sleep(2)

    node_list = "[" + ",".join(f"'{n}'" for n in nodes) + "]"
    proc = subprocess.Popen(
        ["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager",
         "--ros-args", "-r", f"__node:={mgr}",
         "-p", "use_sim_time:=true",
         "-p", "autostart:=false",
         "-p", "bond_timeout:=0.0",          # only honoured at construction
         "-p", f"node_names:={node_list}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    time.sleep(8)
    if proc.poll() is not None:
        print(f"arm_lifecycle: {mgr} exited immediately")
        return 1

    rclpy.init()
    n = Node("lifecycle_armer")
    cli = n.create_client(ManageLifecycleNodes, f"/{mgr}/manage_nodes")
    if not cli.wait_for_service(timeout_sec=timeout):
        print(f"arm_lifecycle: {mgr} never offered manage_nodes")
        rclpy.try_shutdown()
        return 1
    fut = cli.call_async(
        ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP))
    # STARTUP configures and activates every managed node in turn; the costmaps
    # alone take tens of seconds on a loaded machine.
    rclpy.spin_until_future_complete(n, fut, timeout_sec=timeout * 6)
    r = fut.result()
    ok = r is not None and r.success
    print(f"arm_lifecycle: {mgr} bond disabled, startup {'ok' if ok else 'FAILED'}")
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
