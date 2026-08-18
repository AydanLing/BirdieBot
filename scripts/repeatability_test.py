#!/usr/bin/env python3
"""Repeatability harness for the shuttlecock pick.

Each trial resets the base, places the shuttlecock at a randomised pose inside
the reachable band, runs the full detect-align-grasp sequence, and scores it
against Gazebo ground truth.

Every "it works" result in this project so far rests on a single run, and
several of them turned out to be confounded (base rotating under ground strike,
the model origin sitting at the cork tip rather than its centre, world vs
base_link frame mixups). This exists to replace anecdotes with a success rate
and a breakdown of how it fails.

Run:  python3 repeatability_test.py [n_trials]
"""

import math
import os
import random
import re
import subprocess
import sys
import time

WORLD = "husarion_world"


def _arm_x():
    """Read ARM_X from grasp_ball.py rather than keeping a second copy.

    A duplicate here silently went stale when the arm moved to the chassis
    front, and every trial then placed the target behind the robot where it
    cannot be reached -- scoring the robot down for a harness bug.
    """
    src = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "grab_sequence", "grasp_ball.py")
    with open(src) as fh:
        m = re.search(r"^ARM_X\s*=\s*([-\d.]+)", fh.read(), re.M)
    if not m:
        raise RuntimeError("ARM_X not found in grasp_ball.py")
    return float(m.group(1))


ARM_X = _arm_x()
LIFT_THRESHOLD = 0.030      # m above start z to count as picked up
GRIPPER_OPEN = 0.017
# The CAD claw closes to -0.023 (gripper_lower in body.xacro); reaching it
# means the jaws met with nothing between them.
GRIPPER_FREE_CLOSE = -0.023


def gz(args, timeout=20):
    return subprocess.run(args, capture_output=True, text=True, timeout=timeout).stdout


def model_pose(name):
    """(x, y, z, roll, pitch, yaw) from Gazebo, or None.

    Keeps only bracket groups holding exactly three floats. `gz model -p` also
    prints a `Model: [13]` entity id, and warnings can add their own brackets,
    so positional indexing into every match is not safe.
    """
    out = gz(["gz", "model", "-m", name, "-p"])
    triples = []
    for group in re.findall(r"\[([-\d.e+ ]+)\]", out):
        parts = group.split()
        if len(parts) != 3:
            continue
        try:
            triples.append([float(v) for v in parts])
        except ValueError:
            continue
    if len(triples) < 2:
        return None
    return (*triples[0], *triples[1])


def skirt_centre(pose):
    """Where the detector should actually aim, in world coords.

    The shuttlecock's model origin is at the cork tip, not its centre, so
    comparing a detection against it reads ~60 mm off on a tipped object even
    when the detection is perfect. The skirt spans z 0.025..0.085 in the model
    frame, so its centre is 0.055 up the body axis.
    """
    x, y, z, roll, pitch, yaw = pose
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # third column of Rz(yaw) @ Ry(pitch) @ Rx(roll): the model's local +z
    ax = cy * sp * cr + sy * sr
    ay = sy * sp * cr - cy * sr
    az = cp * cr
    return (x + 0.055 * ax, y + 0.055 * ay, z + 0.055 * az)


def set_pose(name, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    gz([
        "gz", "service", "-s", f"/world/{WORLD}/set_pose",
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--timeout", "8000",
        "--req",
        f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
        f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}",
    ])


def pub_once(topic, msgtype, payload, times=15):
    subprocess.run(
        ["ros2", "topic", "pub", "--times", str(times), "--rate", "10",
         topic, msgtype, payload],
        capture_output=True, text=True, timeout=30,
    )


def park_arm():
    pub_once(
        "/manipulator_controller/joint_trajectory",
        "trajectory_msgs/msg/JointTrajectory",
        "{joint_names: [joint1,joint2,joint3,joint4], points: [{positions: "
        "[0.0,-1.0,0.7,0.3], time_from_start: {sec: 3}}]}",
        times=20,
    )
    pub_once(
        "/gripper_controller/joint_trajectory",
        "trajectory_msgs/msg/JointTrajectory",
        "{joint_names: [gripper_left_joint], points: [{positions: [0.017], "
        "time_from_start: {sec: 1}}]}",
    )
    pub_once(
        "/wrist_roll_controller/joint_trajectory",
        "trajectory_msgs/msg/JointTrajectory",
        "{joint_names: [joint5], points: [{positions: [0.0], "
        "time_from_start: {sec: 1}}]}",
    )


def ensure_shuttlecock(x=0.215, y=0.0, z=0.033):
    """Spawn the shuttlecock, resolving its skirt mesh to an absolute path.

    model.sdf refers to the skirt as model://shuttlecock/meshes/skirt.stl, which
    Gazebo resolves by searching GZ_SIM_RESOURCE_PATH. Nothing in this workspace
    puts grab_sequence/models on that path, so spawning the model by absolute
    sdf_filename loads the cork cylinder and silently drops the skirt.

    That failure is quiet and it wrecks the pick in four separate ways, all of
    which look like unrelated perception bugs:
      * the yellow blob falls from ~1018 px to 169 px,
      * it becomes a perfect circle (PCA eigenvalues equal), so the long axis
        never resolves and the wrist-roll alignment never runs,
      * the centroid lands on the cork, ~42 mm from the skirt centre the
        harness scores against,
      * the skirt collision is missing too, so the object rolls like a bare
        cylinder and leaves the reachable band while settling.

    Rewriting the URI to an absolute file:// path sidesteps the search entirely.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    model_dir = os.path.join(here, "..", "models", "shuttlecock")
    model_dir = os.path.abspath(model_dir)
    with open(os.path.join(model_dir, "model.sdf")) as fh:
        sdf = fh.read()
    sdf = sdf.replace("model://shuttlecock/", f"file://{model_dir}/")
    tmp = "/tmp/shuttlecock_resolved.sdf"
    with open(tmp, "w") as fh:
        fh.write(sdf)

    gz(["gz", "service", "-s", f"/world/{WORLD}/remove",
        "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
        "--timeout", "5000", "--req", 'name: "shuttlecock", type: 2'])
    time.sleep(2)
    gz(["gz", "service", "-s", f"/world/{WORLD}/create",
        "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
        "--timeout", "10000", "--req",
        f'sdf_filename: "{tmp}", name: "shuttlecock", '
        f"pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}"])
    time.sleep(4)


def sample_target(rng):
    """A pose inside the reachable band, avoiding joint1's blind sector.

    joint1 spans -144..180 deg, so bearings just past -144 are unreachable no
    matter how close the object is. Radius stays inside 0.09..0.17 m, comfortably
    within the 0.050..0.218 band at grasp height.
    """
    # Forward sector, matching the arm's new mount at the chassis front. Beyond
    # about +-70 deg the grasp point falls under the chassis, so sampling there
    # would only measure the robot bumping into itself.
    bearing = math.radians(rng.uniform(-65.0, 65.0))
    radius = rng.uniform(0.09, 0.17)
    x = ARM_X + radius * math.cos(bearing)
    y = radius * math.sin(bearing)

    if rng.random() < 0.5:
        return "upright", x, y, 0.001, (0.0, 0.0, 0.0, 1.0)

    # Lying on its side, rolled to a random compass heading. Rotate -90 deg
    # about X to lay it down, then yaw by a random angle.
    yaw = rng.uniform(-math.pi, math.pi)
    h = math.pi / -4.0  # half of -90 deg
    qx_l, qw_l = math.sin(h), math.cos(h)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    # quaternion product: Rz(yaw) * Rx(-90)
    qx = cy * qx_l
    qy = sy * qx_l
    qz = sy * qw_l
    qw = cy * qw_l
    return "tipped", x, y, 0.033, (qx, qy, qz, qw)


def run_grasp(log_path):
    with open(log_path, "w") as fh:
        subprocess.run(
            ["ros2", "launch", "grab_sequence", "grasp_ball.launch.py"],
            stdout=fh, stderr=subprocess.STDOUT, timeout=300,
        )
    return open(log_path, errors="ignore").read()


def classify(log, before, after, gripper):
    """Why a trial ended the way it did."""
    lifted = after is not None and before is not None and (after[2] - before[2]) > LIFT_THRESHOLD
    if lifted:
        return "PASS", ""
    if "No reachable shuttlecock found" in log:
        return "FAIL", "never detected / unreachable"
    if "not reachable" in log:
        return "FAIL", "seen but out of reach"
    if gripper is not None and gripper <= GRIPPER_FREE_CLOSE + 0.002:
        return "FAIL", "jaws closed on air (missed)"
    if gripper is not None and gripper >= GRIPPER_OPEN + 0.001:
        return "FAIL", "jaws forced open (fouled object)"
    moved = (before and after and math.hypot(after[0] - before[0], after[1] - before[1]) > 0.02)
    return "FAIL", "knocked object aside" if moved else "no lift, cause unclear"


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 12
    rng = random.Random(20260812)
    results = []

    ensure_shuttlecock()

    for i in range(1, n + 1):
        set_pose("rosbot", 0, 0, 0)
        time.sleep(2)
        park_arm()
        time.sleep(2)

        kind, x, y, z, q = sample_target(rng)
        set_pose("shuttlecock", x, y, z, *q)
        # Dropped in, it slides several cm before coming to rest. Reading ground
        # truth too early records a pose it has already left, which scores a
        # correct detection as a miss.
        time.sleep(8)

        before = model_pose("shuttlecock")
        base_before = model_pose("rosbot")
        log = run_grasp(f"/tmp/rep_trial_{i}.log")
        after = model_pose("shuttlecock")
        base_after = model_pose("rosbot")

        m = re.search(r"gripper (?:at|stalled at) ([-\d.]+)", log)
        gripper = float(m.group(1)) if m else None
        seen = re.search(r"seen at base_link \(([-\d.]+), ([-\d.]+)\)", log)
        det_err = None
        if seen and before:
            aim = skirt_centre(before)
            det_err = math.hypot(float(seen.group(1)) - aim[0],
                                 float(seen.group(2)) - aim[1])
        base_yaw = (abs(base_after[5] - base_before[5])
                    if base_before and base_after else None)

        verdict, why = classify(log, before, after, gripper)
        results.append((i, kind, verdict, why, det_err, gripper, base_yaw))
        print(f"  trial {i:2d}  {kind:8s} {verdict}  {why}", flush=True)

    print("\n" + "=" * 78)
    passes = sum(1 for r in results if r[2] == "PASS")
    print(f"  {passes}/{len(results)} picked up")
    for kind in ("upright", "tipped"):
        sub = [r for r in results if r[1] == kind]
        if sub:
            k = sum(1 for r in sub if r[2] == "PASS")
            print(f"    {kind:8s} {k}/{len(sub)}")
    errs = [r[4] for r in results if r[4] is not None]
    if errs:
        print(f"  detection error: mean {1000*sum(errs)/len(errs):.1f} mm, "
              f"worst {1000*max(errs):.1f} mm")
    yaws = [r[6] for r in results if r[6] is not None]
    if yaws:
        print(f"  base yaw drift:  mean {math.degrees(sum(yaws)/len(yaws)):.2f} deg, "
              f"worst {math.degrees(max(yaws)):.2f} deg")
    fails = {}
    for r in results:
        if r[2] == "FAIL":
            fails[r[3]] = fails.get(r[3], 0) + 1
    if fails:
        print("  failure modes:")
        for why, count in sorted(fails.items(), key=lambda kv: -kv[1]):
            print(f"    {count:2d}x {why}")


if __name__ == "__main__":
    main()
