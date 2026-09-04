#!/usr/bin/env python3
"""Court clearing with no prior knowledge of where the shuttlecocks are.

collect_trials.py answers "how well does the robot pick and navigate?" by
handing it the ground-truth position of every object. This harness answers the
harder question -- "can it find them itself?" -- by never telling it anything.
Ground truth is still read here, but only after an attempt, and only to score
it. Nothing the robot decides is derived from it.

The loop, which is the whole point:

    1. Survey from a standstill until something is found.
    2. Drive to the nearest known target, WITH THE CAMERA RUNNING, so the trip
       banks whatever else happens to pass through view.
    3. Pick it, deposit it, retire it from the map.
    4. Go to the nearest remaining known target. Only survey again when the
       map runs dry.

Step 2 is what makes this cheaper than it sounds. A survey costs 15-20 s of
standing still, and paying that before every pick would dominate the trial.
Driving to one shuttlecock sweeps a corridor roughly 6 m wide through the
court, so most of the time the next target is already in the map by the time
the current one is in the hopper, and the survey is never run at all.

What limits it is sensing range, which is far shorter than intuition suggests.
Measured on this stack, a shuttlecock stops producing a usable blob at about
4 m and stops being reliably measurable at about 3 m -- see detect_params for
the numbers. So the map is always a local picture, the survey grid exists to
walk that 3 m window over a 4.8 x 4.6 m half-court, and a trial ends when the
grid is exhausted rather than when the court is provably clear.

    ros2 run grab_sequence search_trials.py 3       # 3 trials
    COLLECT_N=8 ros2 run grab_sequence search_trials.py 1
"""

import math
import os
import signal
import re
import sys
import time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_grasp_trials import (_activate_nav2_nodes, _inactive_nav2_nodes,  # noqa: E402,I202
                              _wait_for_transform, ClearanceMonitor, NavGrasp,
                              obstacle_names, spawn_obstacles)
from repeatability_test import (classify, park_arm, probe_pose,  # noqa: E402
                                remove_model, resolved_shuttlecock_sdf,
                                run_grasp, set_pose, spawn_model)
from collect_trials import (_on_alarm, APPROACH_STANDOFF,  # noqa: E402
                            APPROACH_UNDERSHOOT, FIELD_X, FIELD_Y, lying_quat,
                            N_SHUTTLES, NET_CLEAR, NET_X, PICK_DEADLINE,
                            PickTimeout, scatter, seed_amcl_latest,
                            segment_clear, SHORT_HOP, SHUTTLE_Z, SIDE,
                            START_X, START_Y)

from grab_sequence.shuttle_scanner import ShuttleScanner  # noqa: E402
from grab_sequence.target_map import TargetMap  # noqa: E402

# --- surveying -------------------------------------------------------------

# Stops in one full rotation.
#
# The camera's horizontal field of view is 110 degrees, so three stops would
# technically close the circle. Four is used because the useful part of the
# view is narrower than the nominal one -- a shuttlecock at the extreme edge of
# frame is both foreshortened and at the worst part of the depth image -- and
# because the overlap gives every object two chances to clear MIN_HITS.
SURVEY_STOPS = 4

# Seconds held still at each stop.
#
# Depth arrives at about 4.2 Hz with gaps up to 0.8 s under simulator load, so
# this is roughly 6 frames: comfortably above the 3 that TargetMap needs to
# confirm, with margin for the frames lost to the rotation still settling.
SURVEY_DWELL = 1.5

# Spacing of the survey waypoint grid.
#
# Set from the ~3 m search-grade detection radius, not from the court. A spin
# survey sees a 3 m disc, so waypoints 2.4 m apart overlap by a comfortable
# margin -- enough that a shuttlecock in the gap between two of them is seen
# from both rather than neither.
SURVEY_SPACING = 2.40

# How far off a nominal waypoint still counts as having visited it.
SURVEY_ARRIVED = 0.60

# --- re-look before committing -------------------------------------------

# Seconds to allow the arm to reach the scan pose before trusting a frame.
#
# park_arm sends a 3 s trajectory. Dwelling for less than that means the whole
# re-look happens mid-swing, and the scanner's arm gate then correctly rejects
# every frame -- producing "gone" for a target that is sitting right there.
ARM_SETTLE_TIMEOUT = 5.0

# Seconds held still while re-measuring a target before driving to it.
#
# Longer than a survey stop because this measurement replaces the target's
# entire history and there is no second opinion to average against.
REFINE_DWELL = 2.0

# How far a freshly measured cluster can be from the believed position and
# still be considered the same object.
#
# Deliberately much wider than the map's 0.35 m merge radius: the whole reason
# for re-looking is that the believed position may be badly wrong, and a
# tolerance tight enough to reject a 0.55 m error would reject exactly the
# cases this exists to fix. Still inside the 0.90 m scatter separation, so it
# cannot lock onto the neighbour instead.
REFINE_MATCH = 0.80

# Beyond this, failing to see the target proves nothing -- it is simply out of
# search-grade range -- so the old estimate is kept and no miss is recorded.
REFINE_MAX_RANGE = 3.20

# Bearing error worth turning to correct before re-looking.
#
# A target near the edge of a 110 degree frame is foreshortened and sits in the
# worst part of the depth image. Turning to centre it costs a second or two and
# is the difference between a usable re-measurement and another bad one.
REFINE_BEARING = math.radians(20)

# Ground-truth object within this distance of a map target is "the one we were
# aiming at", for scoring only.
#
# Wider than the map's own merge radius because it has to absorb the drive-by
# fix error AND the distance the object may have been nudged. Still well inside
# the 0.90 m minimum separation of the scatter, so it can never match the
# neighbour instead.
SCORE_RADIUS = 0.60


def survey_waypoints():
    """Coarse grid over the robot's half of the court, nearest corner first.

    Generated from the same FIELD bounds the scatter uses, so the search area
    and the object distribution cannot drift apart.
    """
    lo_x, hi_x = sorted((SIDE * FIELD_X, NET_X + SIDE * NET_CLEAR))
    pts = []
    nx = max(1, int(round((hi_x - lo_x) / SURVEY_SPACING)))
    ny = max(1, int(round((2 * FIELD_Y) / SURVEY_SPACING)))
    for i in range(nx):
        x = lo_x + (hi_x - lo_x) * (i + 0.5) / nx
        row = [(-FIELD_Y + (2 * FIELD_Y) * (j + 0.5) / ny) for j in range(ny)]
        # Serpentine, so consecutive waypoints are adjacent and the robot is
        # not sent back across the court between rows.
        pts.extend((x, y) for y in (row if i % 2 == 0 else reversed(row)))
    pts.sort(key=lambda p: math.hypot(p[0] - START_X, p[1] - START_Y))
    return pts


def spin_survey(node, scanner):
    """Rotate on the spot, pausing to look. Returns frames actually scanned.

    Rotating and scanning at the same time does not work: the rgb/depth
    synchroniser tolerates 50 ms of skew, and at survey rotation speeds that is
    several pixels of smear across a blob only six pixels wide. ShuttleScanner
    rejects frames above MAX_SCAN_OMEGA for exactly this reason, so a
    continuous spin would be scanned almost entirely in vain. Stopping at each
    heading costs a couple of seconds and is the only version that sees
    anything.
    """
    before = scanner.blobs_filed
    park_arm()
    _wait_for_arm(node, scanner)
    scanner.enable()
    for _ in range(SURVEY_STOPS):
        node.spin(SURVEY_DWELL)
        node.rotate_by(2 * math.pi / SURVEY_STOPS)
    node.stop()
    node.spin(0.3)
    return scanner.blobs_filed - before


def drive_to(node, gx, gy, yaw, pose):
    """Navigate to a pose, scanning the whole way. Returns the nav verdict.

    Same short-hop bypass as collect_trials: nav2 is built to cross the court,
    not to shuffle a metre sideways, and on this field a 1 m hop routinely
    burned the full nav timeout.
    """
    hop = math.hypot(gx - pose[0], gy - pose[1])
    if hop < SHORT_HOP and segment_clear(pose[0], pose[1], gx, gy):
        nav = "SKIPPED"
    else:
        nav = node.send_nav_goal(gx, gy, yaw)
    node.fine_approach(gx, gy, yaw)
    return nav, hop


def acquire(node, scanner, tmap, waypoints, visited):
    """Get at least one confirmed target into the map, or give up.

    Surveys where it stands first -- free, and usually enough after the first
    pick because the drive banked something. Only when that finds nothing does
    it start walking the grid, which is the expensive path.
    """
    pose = node.base_pose()
    if pose is None:
        return False
    if spin_survey(node, scanner) and tmap.nearest(pose[0], pose[1]):
        return True

    while True:
        pose = node.base_pose()
        if pose is None:
            return False
        todo = [p for i, p in enumerate(waypoints) if i not in visited]
        if not todo:
            return False
        nxt = min(todo, key=lambda p: math.hypot(p[0] - pose[0], p[1] - pose[1]))
        visited.add(waypoints.index(nxt))
        if math.hypot(nxt[0] - pose[0], nxt[1] - pose[1]) > SURVEY_ARRIVED:
            yaw = math.atan2(nxt[1] - pose[1], nxt[0] - pose[0])
            scanner.enable()
            drive_to(node, nxt[0], nxt[1], yaw, pose)
            scanner.disable()
        spin_survey(node, scanner)
        pose = node.base_pose()
        if pose and tmap.nearest(pose[0], pose[1]):
            return True


def _wait_for_arm(node, scanner):
    """Spin until the arm has stopped moving, or the timeout expires.

    Polling the measured joint rate rather than sleeping a fixed interval: the
    trajectory duration is a request, not a guarantee, and under simulator load
    a 3 s arm move can take noticeably longer. Waiting on the actual rate also
    means this costs nothing when the arm is already where it should be.
    """
    end = time.time() + ARM_SETTLE_TIMEOUT
    while time.time() < end:
        node.spin(0.1)
        if scanner.arm_settled:
            return True
    return False


def refine_target(node, scanner, tmap, target):
    """Re-measure one target from a standstill before driving to it.

    A position banked while driving past is worth about half a metre. The same
    object measured from a stationary robot is worth about forty millimetres --
    the difference is not the detector but the three static-base assumptions
    described in shuttle_scanner, none of which can be undone for free while
    the wheels are turning. Since the robot is stopped anyway between the
    deposit and the next hop, the better measurement is nearly free, and it is
    taken before the approach is planned rather than after.

    Detections go into a scratch map first. Until the re-look has finished
    there is no way to know which of the clusters it produces is the target it
    went looking for, and folding them straight into the running map would
    merge that decision into data that is hard to unpick. Whatever is not the
    target is replayed into the running map afterwards, so a second shuttlecock
    that happened to be in frame is still banked.

    Returns (status, correction_metres).
    """
    # Park the arm first, and wait for it.
    #
    # The camera rides on link5. This function is called immediately after a
    # deposit, which leaves the arm wherever the carry ended -- up over the
    # hopper, pointing the camera at the sky. Re-looking from there sees
    # nothing, reports the target gone, and charges it a miss. That is exactly
    # what happened: the first pick of a trial refined correctly and passed,
    # because setup had parked the arm, and every pick after it reported
    # "gone" -- including one whose mapped position was accurate to 74 mm.
    park_arm()
    _wait_for_arm(node, scanner)

    pose = node.base_pose()
    if pose is None:
        return "nopose", 0.0
    believed = target.position
    bearing = math.atan2(believed[1] - pose[1], believed[0] - pose[0])
    reach = math.hypot(believed[0] - pose[0], believed[1] - pose[1])

    error = math.atan2(math.sin(bearing - pose[2]), math.cos(bearing - pose[2]))
    if abs(error) > REFINE_BEARING:
        node.rotate_by(error)
        node.stop()

    scratch = TargetMap()
    previous = scanner.divert(scratch)
    scanner.enable()
    node.spin(REFINE_DWELL)
    scanner.disable()
    scanner.divert(previous)

    match, best = None, REFINE_MATCH
    for cluster in scratch.confirmed:
        d = math.hypot(cluster.position[0] - believed[0],
                       cluster.position[1] - believed[1])
        if d < best:
            match, best = cluster, d

    # Anything the re-look saw that was not the target is still a real
    # detection and belongs in the running map.
    for cluster in scratch.targets:
        if cluster is match:
            continue
        for x, y, z, rng in cluster.observations:
            tmap.observe(x, y, z, rng)

    if match is None:
        if reach <= REFINE_MAX_RANGE:
            # Well inside the range where it should have been visible, and it
            # was not. Weak evidence of a ghost -- weak, because occlusion and
            # a bad viewing angle both look like this, which is why it costs a
            # miss rather than an immediate retirement.
            tmap.record_miss(target)
            return "gone", 0.0
        return "far", 0.0

    target.replace(match.observations)
    return "refined", best


def score_match(aim, remaining):
    """Which real shuttlecock, if any, the robot was actually aiming at.

    Ground truth, and therefore strictly off-limits to anything that steers the
    robot. Used only to label the attempt afterwards, and to tell a genuine
    miss apart from a pick aimed at a target that was never there.
    """
    best, best_d = None, SCORE_RADIUS
    for nm, (x, y) in remaining.items():
        d = math.hypot(x - aim[0], y - aim[1])
        if d < best_d:
            best, best_d = nm, d
    return best


def main():
    n_trials = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    seed = int(os.environ.get("COLLECT_SEED", "7"))
    import random
    rng = random.Random(seed)

    rclpy.init()

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
    # One scanner for the whole run, re-pointed at a fresh map each trial. See
    # ShuttleScanner.retarget for why it is not rebuilt per trial.
    scanner = ShuttleScanner(node, TargetMap(), tf_buffer=node.buf)
    grid = survey_waypoints()
    print(f"  survey grid: {len(grid)} waypoints at {SURVEY_SPACING:.2f} m "
          f"spacing", flush=True)
    totals = []

    for t in range(1, n_trials + 1):
        for nm in names:
            remove_model(nm)
        remove_model("shuttlecock")
        set_pose("rosbot", START_X, START_Y, 0.0, tol=0.30, settle=2.0)
        yaw = math.atan2(-START_Y, -START_X)
        if not seed_amcl_latest(node, START_X, START_Y, yaw):
            print("    ! AMCL would not accept the corner pose; aborting trial",
                  flush=True)
            continue
        park_arm()
        time.sleep(2)

        field = scatter(rng)
        for nm, (x, y) in zip(names, field):
            spawn_model(nm, sdf, x, y, SHUTTLE_Z, settle=0.6)
            set_pose(nm, x, y, SHUTTLE_Z, *lying_quat(rng), settle=0.3)
        time.sleep(8)

        # Ground truth, for scoring only. The robot never reads this.
        remaining = {}
        for nm, (x, y) in zip(names, field):
            p = probe_pose(nm)
            if isinstance(p, tuple):
                remaining[nm] = (p[0], p[1])
        placed = len(remaining)

        # A fresh map every trial. Carrying one over would hand the robot the
        # previous field's answers, which is the exact thing being tested.
        tmap = TargetMap()
        scanner.retarget(tmap)
        visited = set()

        print(f"\n  trial {t}: {placed} shuttles on court, robot at "
              f"({START_X:+.2f},{START_Y:+.2f}), map empty", flush=True)

        collected, missed, ghosts, indet, surveys = 0, 0, 0, 0, 0
        t_trial = time.time()
        contacts.start()
        k = 0

        while True:
            pose = node.base_pose()
            if pose is None:
                print("    ! lost base pose, aborting trial", flush=True)
                break

            target = tmap.nearest(pose[0], pose[1])
            if target is None:
                surveys += 1
                scanner.disable()
                if not acquire(node, scanner, tmap, grid, visited):
                    print(f"    survey {surveys}: grid exhausted, "
                          f"nothing left in view", flush=True)
                    break
                pose = node.base_pose()
                target = tmap.nearest(pose[0], pose[1])
                conf, pend, ret = tmap.counts()
                print(f"    survey {surveys}: map now {conf} confirmed, "
                      f"{pend} pending, {ret} retired", flush=True)
                if target is None:
                    break

            k += 1
            # Re-measure before planning the approach, not after: the standoff
            # point is computed from the target position, so a correction that
            # arrives later has already been baked into where the robot parked.
            fix, moved = refine_target(node, scanner, tmap, target)
            if fix == "gone" and not target.confirmed:
                print(f"    {k:2d} target {target.id} DROPPED       "
                      f"re-look found nothing within "
                      f"{REFINE_MAX_RANGE:.1f}m", flush=True)
                ghosts += 1
                continue
            pose = node.base_pose()
            if pose is None:
                break

            aim = target.position
            approach = math.atan2(aim[1] - pose[1], aim[0] - pose[0])
            d_obj = math.hypot(aim[0] - pose[0], aim[1] - pose[1])
            back = min(APPROACH_STANDOFF + APPROACH_UNDERSHOOT, d_obj)
            gx = aim[0] - back * math.cos(approach)
            gy = aim[1] - back * math.sin(approach)

            node.park_arm_async()
            t0 = time.time()
            signal.signal(signal.SIGALRM, _on_alarm)
            signal.setitimer(signal.ITIMER_REAL, PICK_DEADLINE)
            try:
                # The camera runs for the whole approach. This is step 2 of the
                # loop and the reason surveys stay rare.
                scanner.enable()
                nav, hop = drive_to(node, gx, gy, approach, pose)
                # Off for the pick itself: the arm swings the camera through
                # arbitrary poses and then holds a shuttlecock in front of the
                # lens, none of which is a view of the floor.
                scanner.disable()
                _err, _vi, vis_seen = node.visual_servo(aim_map=(aim[0], aim[1]))
                park_arm()
                time.sleep(1)
            except PickTimeout:
                signal.setitimer(signal.ITIMER_REAL, 0)
                scanner.disable()
                node.stop()
                print(f"    {k:2d} target {target.id} TIMEOUT      "
                      f"approach exceeded {PICK_DEADLINE:.0f}s", flush=True)
                tmap.retire(target, "timeout")
                indet += 1
                continue
            finally:
                signal.setitimer(signal.ITIMER_REAL, 0)

            # Only now, after the robot has committed, is ground truth read.
            nm = score_match(aim, remaining)
            before = probe_pose(nm) if nm else None

            signal.setitimer(signal.ITIMER_REAL, PICK_DEADLINE)
            try:
                log, status = run_grasp(f"/tmp/search_{t}_{k}.log")
                after = probe_pose(nm) if nm else None
            except PickTimeout:
                print(f"    {k:2d} target {target.id} TIMEOUT      "
                      f"grasp exceeded {PICK_DEADLINE:.0f}s", flush=True)
                if nm:
                    remove_model(nm)
                    remaining.pop(nm)
                tmap.retire(target, "timeout")
                indet += 1
                continue
            finally:
                signal.setitimer(signal.ITIMER_REAL, 0)

            if nm is None:
                # Nothing real was within SCORE_RADIUS of where the robot
                # believed a shuttlecock was. The map invented it.
                verdict, why = "GHOST", "no object within scoring radius"
                ghosts += 1
                tmap.record_miss(target)
                tmap.retire(target, "ghost")
            else:
                m = re.search(r"gripper (?:at|stalled at) ([-\d.]+)", log)
                verdict, why = classify(log, before, after,
                                        float(m.group(1)) if m else None, status)
                if verdict == "PASS":
                    collected += 1
                    remove_model(nm)
                    remaining.pop(nm)
                    tmap.retire(target, "collected")
                elif verdict == "INDETERMINATE":
                    indet += 1
                    remove_model(nm)
                    remaining.pop(nm)
                    tmap.retire(target, "indeterminate")
                else:
                    missed += 1
                    # Not removed from the world. A missed shuttlecock is still
                    # on the court, and the honest behaviour is to let the robot
                    # come back to it -- which it will, because record_miss
                    # leaves the target live until it has failed twice.
                    if tmap.record_miss(target):
                        remove_model(nm)
                        remaining.pop(nm)

            err = math.hypot(before[0] - aim[0], before[1] - aim[1]) \
                if isinstance(before, tuple) else float("nan")
            el = time.time() - t0
            print(f"    {k:2d} target {target.id} {verdict:13s} "
                  f"relook {fix:7s} {moved:5.3f}m  "
                  f"hop {hop:4.2f}m nav {nav:9s} "
                  f"vis {'seen' if vis_seen else 'NOT SEEN':8s} "
                  f"map err {err:5.3f}m  {el:5.1f}s"
                  f"{'  ' + why if why and verdict != 'PASS' else ''}",
                  flush=True)

        min_clear = contacts.stop()
        dur = time.time() - t_trial
        left = len(remaining)
        totals.append((collected, missed, ghosts, indet, left, surveys, dur))
        mc = f"{min_clear:+.3f}m" if min_clear is not None else "n/a"
        print(f"  trial {t}: collected {collected}/{placed} "
              f"(missed {missed}, ghosts {ghosts}, indeterminate {indet}, "
              f"never found {left}) in {dur/60:.1f} min, "
              f"{surveys} surveys, closest obstacle {mc}", flush=True)
        print(f"    scanner: {scanner.stats()}", flush=True)

    print()
    got = sum(c for c, *_ in totals)
    put = sum(c + m + g + i + lf for c, m, g, i, lf, _s, _d in totals)
    if put:
        print(f"  {got}/{put} shuttles collected across {len(totals)} trials "
              f"({100.0 * got / put:.0f}%), no prior knowledge of positions")
    for i, (c, m, g, ind, lf, s, d) in enumerate(totals, 1):
        print(f"    trial {i}: {c} collected, {m} missed, {g} ghosts, "
              f"{ind} indeterminate, {lf} never found, {s} surveys, "
              f"{d/60:.1f} min")
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
