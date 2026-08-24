#!/usr/bin/env python3
"""Clear a court of shuttlecocks: start in one corner, collect all of them.

Different task from nav_grasp_trials.py, which teleports the base back to the
origin and places one object per trial. Here the robot starts in the bottom
left corner and works its way through a scattered field without ever being
put back, so errors accumulate across a whole trial the way they would on a
real court.

Three things had to change from the single-object harness rather than being
reused as-is:

  * Approach heading. nav_grasp_trials computes it as atan2(aim.y, aim.x),
    the bearing from the WORLD ORIGIN to the object, which is only correct
    because the base always starts there. Here the base is wherever the last
    pick left it, so the heading is taken from the base's own pose.

  * AMCL seeding. The single-object harness reseeds every trial because
    set_pose teleports the base without generating odometry. This one
    teleports once per trial, so AMCL is seeded once and then left to track
    normally for the whole sweep -- which is also the more honest test.

  * Object identity. classify() scores one named model. With sixteen in the
    world each is spawned under its own name and scored individually, and
    the pick is only credited to the shuttle the robot was actually sent to.

Order is nearest-first from the base's current pose. That is the obvious
policy for this task and it keeps the hops short, which matters because the
navigation phase dominates the clock.

A shuttle is removed from the world after its attempt either way, collected
or missed. Leaving a missed one in place would either be retried forever or
sit in the field as a distractor for the pick that follows, and neither
produces a number worth reading. Missed ones are reported separately.
"""

import glob
import math
import os
import subprocess
import re
import sys
import time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_grasp_trials import (ARM_X, STANDOFF, ClearanceMonitor,  # noqa: E402
                              NavGrasp, _activate_nav2_nodes,
                              _inactive_nav2_nodes, _wait_for_transform,
                              clear_of_obstacles, obstacle_clearance,
                              obstacle_names, spawn_obstacles)
from repeatability_test import (UNKNOWN, GzQueryFailed,  # noqa: E402
                                WorldMismatch, classify, park_arm, probe_pose,
                                remove_model, resolved_shuttlecock_sdf,
                                run_grasp, set_pose, skirt_centre, spawn_model)

N_SHUTTLES = 16

# Bottom left corner of the court. The court is 13.4 x 6.1 centred on the
# origin, so its corner is (-6.70, -3.05); this sits inside that with room for
# the 0.22 m footprint and the wall inflation behind it.
START_X, START_Y = -6.00, -2.60

# Field bounds, matching the single-object harness so the two are comparable.
FIELD_X, FIELD_Y = 5.60, 2.30

# Two shuttles closer than this risk both landing in the camera's view during
# the final visual correction, and the pick has no way to say which one it was
# sent to. 51.5 m^2 over 16 objects averages 3.2 m^2 each, so this is not a
# tight constraint in practice.
MIN_SEP = 0.90

# Keep the field off the robot's own start, or the first pick begins with the
# object already under the arm and the navigation phase is untested.
START_CLEAR = 1.20

SHUTTLE_Z = 0.033

# Below this, drive with fine_approach and never involve nav2.
#
# nav2 is built to cross the court, not to shuffle a metre sideways, and on this
# field it visibly is not built for it. Measured over the first 9 picks of a
# collection run, the two hops that went to nav2 at 0.71 m and 1.46 m both came
# back nav TIMEOUT after burning the full 120 s allowance, and those two picks
# alone took 428 s of the 1199 s the nine cost together -- 36% of the clock for
# 22% of the work. fine_approach turns, drives and turns closed on AMCL, which
# is exactly the manoeuvre a short hop needs, and at FINE_V 0.25 m/s it covers
# 2 m in about 8 s.
SHORT_HOP = 2.00

# fine_approach drives a straight line and knows nothing about obstacles, so the
# bypass is only taken when that line is actually clear. nav2 keeps the job
# whenever it is not, however short the hop.
PATH_MARGIN = 0.12


def segment_clear(x0, y0, x1, y1, margin=PATH_MARGIN):
    """True if a footprint can sweep the straight line without touching anything."""
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.05))
    for i in range(n + 1):
        f = i / n
        if obstacle_clearance(x0 + (x1 - x0) * f, y0 + (y1 - y0) * f) < margin:
            return False
    return True


def seed_amcl_latest(node, x, y, yaw, tries=4):
    """Seed AMCL with a zero-stamped pose, verifying the transform appears.

    NavGrasp.seed_amcl stamps with the current sim time, which is fine when the
    base is teleported back to the origin it is already localised near. Sending
    it to the corner is a 6.5 m jump immediately after set_pose, and AMCL then
    rejected every publish with "Failed to transform initial pose in time
    (Lookup would require extrapolation)" -- the stamp is ahead of the odom TF
    the teleport just invalidated. A zero stamp asks TF for the latest available
    transform instead of one at an exact instant, which is what this needs.

    Verifies rather than assumes: an unseeded AMCL publishes no map->base_link,
    every hop is then computed from a stale pose, and the trial silently runs
    with the objects metres from where the robot believes they are.
    """
    from geometry_msgs.msg import PoseWithCovarianceStamped
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
    for _ in range(tries):
        for _ in range(8):
            m.header.stamp = rclpy.time.Time().to_msg()   # 0 == latest available
            node.initpose.publish(m)
            node.spin(0.25)
        node.spin(2.0)
        if _wait_for_transform("map", "base_link", timeout=8.0):
            return True
    return False


def sweep_orphan_shm():
    """Delete DDS segments no live process still maps. -> count removed.

    Every pick spawns a fresh grasp launch, so a 16-shuttle trial creates and
    tears down sixteen sets of DDS participants and a five-trial run creates
    eighty. FastDDS does not always reclaim the shared memory, and the debris
    accumulates until port allocation starts failing.

    That is not theoretical. One run reached 259 segments partway through the
    fourth trial and then fell over: four picks in a row came back "grasp launch
    never exited", nav2 began rejecting goals, and eventually every lifecycle
    node stopped answering and map->base_link disappeared, with the processes
    still running but unable to talk to each other.

    Only unmapped files are touched, and only between trials, never while a
    grasp launch is in flight. Sweeping mid-trial on that failed run did not
    recover it and may well have disturbed participants that were still live.
    """
    live = set()
    for m in glob.glob("/proc/[0-9]*/maps"):
        try:
            with open(m) as f:
                for line in f:
                    if "/dev/shm/" in line:
                        live.add(line.rsplit("/dev/shm/", 1)[1].strip())
        except (IOError, OSError):
            continue
    gone = 0
    for path in (glob.glob("/dev/shm/fastrtps_*")
                 + glob.glob("/dev/shm/sem.fastrtps_*")):
        if os.path.basename(path) in live:
            continue
        try:
            os.unlink(path)
            gone += 1
        except OSError:
            pass
    return gone


def scatter(rng):
    """Place N_SHUTTLES over the court, spaced and clear of obstacles."""
    pts = []
    for _ in range(20000):
        if len(pts) == N_SHUTTLES:
            break
        x = rng.uniform(-FIELD_X, FIELD_X)
        y = rng.uniform(-FIELD_Y, FIELD_Y)
        if math.hypot(x - START_X, y - START_Y) < START_CLEAR:
            continue
        if not clear_of_obstacles(x, y):
            continue
        if any(math.hypot(x - px, y - py) < MIN_SEP for px, py in pts):
            continue
        pts.append((x, y))
    if len(pts) < N_SHUTTLES:
        raise RuntimeError(f"only placed {len(pts)} of {N_SHUTTLES} shuttles")
    return pts


def nearest(pose, remaining):
    """Index of the closest un-collected shuttle to the base's current pose."""
    bx, by = pose[0], pose[1]
    return min(remaining, key=lambda k: math.hypot(remaining[k][0] - bx,
                                                   remaining[k][1] - by))


def main():
    n_trials = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    seed = int(os.environ.get("COLLECT_SEED", "7"))
    import random
    rng = random.Random(seed)

    # Before the preflight checks, not after: _wait_for_transform builds its own
    # probe node and rclpy refuses to create one otherwise.
    rclpy.init()

    # Same runtime bond_timeout override nav_grasp_trials does, and for the same
    # reason: nav2's launch files pass only {autostart} and {node_names} to the
    # lifecycle managers, so a lifecycle_manager_* block in amcl.yaml is read by
    # nobody. Omitting it here is what killed two attempts at this run. A trial
    # is 16 grasp launches back to back, which is heavy enough that a manager
    # eventually misses a heartbeat and declares a healthy server dead --
    # "CRITICAL FAILURE: SERVER map_server IS DOWN" -- after which AMCL stops
    # publishing map->base_link, every hop is computed from a stale pose, and
    # the picks fail with the object metres from where the robot thinks it is.
    for mgr in ("lifecycle_manager_navigation", "lifecycle_manager_localization"):
        try:
            subprocess.run(["ros2", "param", "set", f"/{mgr}", "bond_timeout", "0.0"],
                           capture_output=True, text=True, timeout=45)
        except subprocess.TimeoutExpired:
            # The CLI's own discovery is slow on a loaded machine and this is a
            # best-effort hardening step, not a precondition. Letting the
            # exception out killed a run before a single trial started.
            print(f"  ! bond_timeout set on {mgr} timed out; continuing",
                  flush=True)

    dead = _inactive_nav2_nodes()
    if dead:
        print(f"  activating nav2: {', '.join(dead)}", flush=True)
        _activate_nav2_nodes(dead)
    if not _wait_for_transform("map", "base_link"):
        print("  ABORT: no map->base_link transform. Seed AMCL first.")
        return 1

    sdf = resolved_shuttlecock_sdf()
    node = NavGrasp()
    contacts = ClearanceMonitor(obstacle_names())
    spawn_obstacles(force=True)

    names = [f"shuttle_{k:02d}" for k in range(N_SHUTTLES)]
    totals = []

    for t in range(1, n_trials + 1):
        swept = sweep_orphan_shm()
        if swept:
            print(f"  swept {swept} orphaned DDS segments", flush=True)
        for nm in names:
            remove_model(nm)
        # The single-object harness leaves a model called "shuttlecock" wherever
        # its last trial ended. It is not part of this field, but it is the same
        # mesh in the same colour, so leaving it in place puts an unscored
        # distractor on the court for the visual correction to find.
        remove_model("shuttlecock")
        set_pose("rosbot", START_X, START_Y, 0.0, tol=0.30, settle=2.0)
        yaw = math.atan2(-START_Y, -START_X)      # face the middle of the court
        # Seeded twice with a gap. A single publish straight after the teleport
        # was rejected with "Failed to transform initial pose in time (Lookup
        # would require extrapolation)": the stamp beats the odom TF that the
        # teleport invalidated, AMCL drops it, and the whole trial then runs on a
        # pose five metres from where the robot is.
        if not seed_amcl_latest(node, START_X, START_Y, yaw):
            print("    ! AMCL would not accept the corner pose; aborting trial",
                  flush=True)
            continue
        park_arm()
        time.sleep(2)

        field = scatter(rng)
        for nm, (x, y) in zip(names, field):
            spawn_model(nm, sdf, x, y, SHUTTLE_Z, settle=0.6)
        time.sleep(4)

        remaining = {}
        for nm, (x, y) in zip(names, field):
            p = probe_pose(nm)
            if isinstance(p, tuple):
                remaining[nm] = (p[0], p[1])
        print(f"\n  trial {t}: {len(remaining)} shuttles on court, "
              f"robot at ({START_X:+.2f},{START_Y:+.2f})", flush=True)

        collected, missed, indet = 0, 0, 0
        t_trial = time.time()
        contacts.start()

        while remaining:
            pose = node.base_pose()
            if pose is None:
                print("    ! lost base pose, aborting trial", flush=True)
                break
            nm = nearest(pose, remaining)
            k = len(field) - len(remaining) + 1

            before = probe_pose(nm)
            if not isinstance(before, tuple):
                remaining.pop(nm)
                indet += 1
                continue

            aim = skirt_centre(before)
            # Heading from the BASE to the object, not from the world origin.
            approach = math.atan2(aim[1] - pose[1], aim[0] - pose[0])
            gx = aim[0] - STANDOFF * math.cos(approach)
            gy = aim[1] - STANDOFF * math.sin(approach)
            hop = math.hypot(gx - pose[0], gy - pose[1])

            t0 = time.time()
            if hop < SHORT_HOP and segment_clear(pose[0], pose[1], gx, gy):
                nav = "SKIPPED"
            else:
                nav = node.send_nav_goal(gx, gy, approach)
            node.fine_approach(gx, gy, approach)
            vis_err, _vi, vis_seen = node.visual_servo()
            park_arm()
            time.sleep(1)

            base_truth = probe_pose("rosbot")
            reach = float("nan")
            if isinstance(base_truth, tuple):
                dx, dy = aim[0] - base_truth[0], aim[1] - base_truth[1]
                c, s = math.cos(-base_truth[5]), math.sin(-base_truth[5])
                rx, ry = dx * c - dy * s, dx * s + dy * c
                reach = math.hypot(rx - ARM_X, ry)

            log, status = run_grasp(f"/tmp/collect_{t}_{k}.log")
            after = probe_pose(nm)
            m = re.search(r"gripper (?:at|stalled at) ([-\d.]+)", log)
            verdict, why = classify(log, before, after,
                                    float(m.group(1)) if m else None, status)
            if verdict != "INDETERMINATE" and nav in ("NO_SERVER", "REJECTED"):
                verdict, why = "INDETERMINATE", f"nav2 goal {nav} (infra)"

            if verdict == "PASS":
                collected += 1
            elif verdict == "INDETERMINATE":
                indet += 1
            else:
                missed += 1

            el = time.time() - t0
            print(f"    {k:2d}/{len(field)} {nm} {verdict:13s} hop {hop:4.2f}m "
                  f"nav {nav:9s} vis {'seen' if vis_seen else 'NOT SEEN':8s} "
                  f"reach {reach:5.3f}m  {el:5.1f}s"
                  f"{'  ' + why if why and verdict != 'PASS' else ''}",
                  flush=True)

            remove_model(nm)
            remaining.pop(nm)

        min_clear = contacts.stop()
        dur = time.time() - t_trial
        totals.append((collected, missed, indet, dur))
        mc = f"{min_clear:+.3f}m" if min_clear is not None else "n/a"
        print(f"  trial {t}: collected {collected}/{len(field)} "
              f"(missed {missed}, indeterminate {indet}) in {dur/60:.1f} min, "
              f"closest obstacle {mc}", flush=True)

    print()
    scorable = [(c, m) for c, m, _i, _d in totals]
    got = sum(c for c, _ in scorable)
    att = sum(c + m for c, m in scorable)
    print(f"  {got}/{att} shuttles collected across {len(totals)} trials "
          f"({100.0 * got / att:.0f}%)" if att else "  no scorable attempts")
    for i, (c, m, ind, d) in enumerate(totals, 1):
        print(f"    trial {i}: {c} collected, {m} missed, {ind} indeterminate, "
              f"{d/60:.1f} min")
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
