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

It also has to survive its own plumbing. See the "talking to Gazebo" block
below: a harness that cannot tell a failed query from a real answer reports
robot failures that never happened, which is worse than reporting nothing.

Run:  python3 repeatability_test.py [n_trials]
"""

import math
import os
import random
import re
import signal
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


# --- talking to Gazebo without being lied to -------------------------------
#
# Every gz CLI call here is treated as unreliable until proven otherwise. All
# three of these were observed, and the first two each silently corrupted a
# whole run:
#
#   * `gz model -p` times out under load and prints nothing. The old parser
#     returned None for that and the harness read None as "object absent",
#     which it scored as a failed grasp. In one 10-trial navigate-then-grasp
#     run that cost two trials -- one printed "[shuttlecock missing]", the
#     other recorded a final z of 0.000 -- while the object was present and had
#     been lifted to z=0.060, with a clean detection in that trial's own grasp
#     log. The run reported 8/10 for a robot that had gone 10/10.
#
#   * `gz service` create/remove reply gz.msgs.Boolean and that reply looks the
#     same whether or not anything happened. Two specific traps: a remove with
#     `type: 1` is LIGHT, not MODEL, and removes nothing while looking like a
#     success (MODEL is `type: 2`); and a create whose sdf_filename does not
#     exist -- e.g. a /tmp file wiped by a reboot -- also reports success and
#     spawns nothing. Ten minutes of "measurements" were once taken of a scene
#     that was empty for that second reason.
#
#   * `gz` itself exits 255 with its error text on *stdout* (not stderr) when
#     GZ_CONFIG_PATH is unset, so an unsourced shell yields empty parses rather
#     than an obvious crash. Checking the return code is the only way to see it.
#
# The rules that follow from that, applied throughout this module:
#   1. a reply message is never accepted as evidence of a world change;
#   2. silence is never evidence of absence -- only Gazebo's own
#      "No model named <x> was found" is;
#   3. every spawn and remove is verified by reading the world back with
#      `gz model --list`, and mismatches raise rather than warn;
#   4. a query that never answered raises GzQueryFailed, and the trial loop
#      turns that into INDETERMINATE rather than into a robot failure.

GZ_ATTEMPTS = 4             # tries per query
GZ_BACKOFF = 1.0            # s before the first retry, doubled each time
                            # (1+2+4 = 7 s worst case, which is short next to
                            # the ~8 s settle each trial already waits out)


class GzQueryFailed(RuntimeError):
    """Gazebo did not answer. The world state is unknown -- which is not empty."""


class WorldMismatch(RuntimeError):
    """Gazebo answered and the world is not what the harness asked for."""


# Sentinel for "the query failed, so this is unknown". Deliberately distinct
# from None, which here means Gazebo positively reported the model as absent.
UNKNOWN = "UNKNOWN"


def gz_run(args, timeout=20):
    """Run a gz CLI command. -> (rc, stdout). rc is None if it never returned.

    Both the return code and stdout matter: gz reports "I cannot find any
    available 'gz' command" on stdout with rc 255, so stdout alone cannot tell
    a broken environment from an empty answer.
    """
    try:
        p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None, ""
    except FileNotFoundError:
        raise GzQueryFailed(
            "`gz` is not on PATH -- source /opt/ros/jazzy/setup.bash first")
    return p.returncode, p.stdout


def gz(args, timeout=20):
    """stdout only, for calls whose result is verified some other way."""
    return gz_run(args, timeout)[1]


def _parse_pose(out):
    """(x, y, z, roll, pitch, yaw) from `gz model -p` output, or None.

    Keeps only bracket groups holding exactly three floats. `gz model -p` also
    prints a `Model: [13]` entity id, and warnings can add their own brackets,
    so positional indexing into every match is not safe.
    """
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


def model_pose(name, attempts=GZ_ATTEMPTS):
    """(x, y, z, roll, pitch, yaw), or None if the model is genuinely absent.

    Raises GzQueryFailed when Gazebo never gave a usable answer, which is the
    whole point: the previous version returned None for both cases and the
    caller scored "the query timed out" as "the robot dropped the object".

    Absence is only ever concluded from Gazebo's own words. Asking for a model
    that does not exist prints, on a successful query:
        No model named <no_such_model> was found
    whereas a query that timed out prints the "Requesting state for world"
    banner and nothing else.
    """
    delay = GZ_BACKOFF
    detail = "no output"
    for attempt in range(attempts):
        rc, out = gz_run(["gz", "model", "-m", name, "-p"])
        if rc == 0:
            if f"No model named <{name}> was found" in out:
                return None
            pose = _parse_pose(out)
            if pose is not None:
                return pose
            detail = "answered but no pose in the output"
        elif rc is None:
            detail = "call timed out"
        else:
            detail = f"exit {rc}: {' '.join(out.split())[:120]}"
        if attempt + 1 < attempts:
            time.sleep(delay)
            delay *= 2
    raise GzQueryFailed(f"pose query for '{name}' failed {attempts}x ({detail})")


def model_list(attempts=GZ_ATTEMPTS):
    """Set of model names currently in the world. Raises GzQueryFailed if none.

    Output shape, checked against the live world:

        Requesting state for world [husarion_world]...

        Available models:
            - husarion_world
            - rosbot

    The "Available models:" header is the marker that the query answered at
    all. Keying on the list being empty instead would read a timed-out call as
    an empty world, and an empty world is exactly the state this function
    exists to detect -- so it must not be the default answer when nothing came
    back.
    """
    delay = GZ_BACKOFF
    detail = "no output"
    for attempt in range(attempts):
        rc, out = gz_run(["gz", "model", "--list"])
        if rc == 0 and "Available models:" in out:
            names = set()
            for line in out.split("Available models:", 1)[1].splitlines():
                m = re.match(r"\s*-\s*(\S.*?)\s*$", line)
                if m:
                    names.add(m.group(1))
            return names
        if rc is None:
            detail = "call timed out"
        elif rc != 0:
            detail = f"exit {rc}: {' '.join(out.split())[:120]}"
        else:
            detail = "no 'Available models:' header in the reply"
        if attempt + 1 < attempts:
            time.sleep(delay)
            delay *= 2
    raise GzQueryFailed(f"model list query failed {attempts}x ({detail})")


def in_world(pose, floor=-0.20, span=25.0):
    """Is this pose somewhere an object can still be interacted with?

    `gz model --list` keeps listing a model that has fallen through the floor
    or been flung out of the room, so presence in the list is not enough to
    conclude the object is usable.
    """
    return pose[2] > floor and abs(pose[0]) < span and abs(pose[1]) < span


def set_pose(name, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0, tol=None, settle=0.0):
    """Teleport a model. With tol set, verify by reading the pose back.

    The service reply is not evidence -- see the block at the top of this file
    -- so the only check that counts is reading the pose back out of Gazebo.

    tol has to be loose: a dropped object slides several cm before settling, so
    this catches a request that did nothing at all (the model stays wherever
    the last trial left it) rather than a few cm of settling. Raises
    WorldMismatch if the model did not move to roughly where it was told to,
    and propagates GzQueryFailed if the readback itself could not answer.
    """
    gz([
        "gz", "service", "-s", f"/world/{WORLD}/set_pose",
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--timeout", "8000",
        "--req",
        f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
        f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}",
    ])
    if tol is None:
        return None
    if settle:
        time.sleep(settle)
    pose = model_pose(name)
    if pose is None:
        raise WorldMismatch(f"set_pose named '{name}', which is not in the world")
    if math.hypot(pose[0] - x, pose[1] - y) > tol:
        raise WorldMismatch(
            f"set_pose('{name}') asked for ({x:.3f}, {y:.3f}) but it is at "
            f"({pose[0]:.3f}, {pose[1]:.3f}) -- the request did not take")
    return pose


def pub_once(topic, msgtype, payload, times=15, timeout=30):
    """ros2 topic pub, checked. -> True if the publisher ran to completion.

    Unchecked, this is a silent single point of failure for a whole run: if
    `ros2` is missing, the workspace is unsourced, or the controller topic has
    been renamed, park_arm does nothing, every trial starts from whatever pose
    the last one ended in, and the results look like erratic robot behaviour.
    """
    try:
        p = subprocess.run(
            ["ros2", "topic", "pub", "--times", str(times), "--rate", "10",
             topic, msgtype, payload],
            capture_output=True, text=True, timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        print(f"    ! ros2 topic pub {topic} timed out after {timeout}s", flush=True)
        return False
    except FileNotFoundError:
        print("    ! `ros2` is not on PATH -- source the workspace first", flush=True)
        return False
    if p.returncode != 0:
        err = " ".join((p.stderr or p.stdout or "").split())[:160]
        print(f"    ! ros2 topic pub {topic} exited {p.returncode}: {err}", flush=True)
        return False
    return True


def park_arm():
    """Return the arm to the neutral pose. -> True if every publish succeeded.

    Callers should treat False as an infrastructure fault for the trial rather
    than run the pick from an unknown arm pose.
    """
    ok = pub_once(
        "/manipulator_controller/joint_trajectory",
        "trajectory_msgs/msg/JointTrajectory",
        "{joint_names: [joint1,joint2,joint3,joint4], points: [{positions: "
        "[0.0,-1.0,0.7,0.3], time_from_start: {sec: 3}}]}",
        times=20,
    )
    ok &= pub_once(
        "/gripper_controller/joint_trajectory",
        "trajectory_msgs/msg/JointTrajectory",
        "{joint_names: [gripper_left_joint], points: [{positions: [0.017], "
        "time_from_start: {sec: 1}}]}",
    )
    ok &= pub_once(
        "/wrist_roll_controller/joint_trajectory",
        "trajectory_msgs/msg/JointTrajectory",
        "{joint_names: [joint5], points: [{positions: [0.0], "
        "time_from_start: {sec: 1}}]}",
    )
    return ok


def remove_model(name, attempts=3):
    """Remove a model and verify from `gz model --list` that it is really gone.

    `type: 2` is MODEL in gz.msgs.Entity. `type: 1` is LIGHT, removes nothing,
    and replies exactly like a success -- that one cost a day. The verification
    below, not the request, is the reason this function exists: a remove that
    quietly no-ops leaves a stale model behind, and the next create then either
    fails or produces a duplicate.

    Returns True if a removal actually happened, False if it was already absent.
    """
    if name not in model_list():
        return False
    for k in range(attempts):
        gz(["gz", "service", "-s", f"/world/{WORLD}/remove",
            "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
            "--timeout", "5000", "--req", f'name: "{name}", type: 2'])
        time.sleep(2.0 + k)          # removal is asynchronous: the entity is
                                     # deleted on a later simulation step
        if name not in model_list():
            return True
    raise WorldMismatch(
        f"'{name}' is still in the world after {attempts} remove requests. "
        "Check the request used type: 2 (MODEL); type: 1 is LIGHT and no-ops.")


def spawn_model(name, sdf_path, x, y, z, settle=4.0, attempts=2):
    """Create a model from an sdf file and verify it appeared in the world.

    The sdf_filename existence check is not paranoia: a create request naming a
    file that is not there replies success and spawns nothing. /tmp is the
    usual culprit because a reboot empties it and the resolved sdf written by
    an earlier session is gone.
    """
    if not os.path.isfile(sdf_path):
        raise WorldMismatch(
            f"cannot spawn '{name}': {sdf_path} does not exist. Gazebo would "
            "have replied success and spawned nothing.")
    for _ in range(attempts):
        gz(["gz", "service", "-s", f"/world/{WORLD}/create",
            "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
            "--timeout", "10000", "--req",
            f'sdf_filename: "{sdf_path}", name: "{name}", '
            f"pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}"])
        time.sleep(settle)
        if name in model_list():
            return True
    raise WorldMismatch(
        f"'{name}' is not in the world after {attempts} create requests from "
        f"{sdf_path} -- the reply said nothing useful, the model list did.")


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


def resolved_shuttlecock_sdf(tmp="/tmp/shuttlecock_resolved.sdf"):
    """Write model.sdf with its mesh URI resolved to an absolute path. -> path.

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

    Every file:// path the rewrite produces is checked for existence here. A
    create request pointing at a missing mesh -- or a missing sdf -- does not
    fail loudly; it reports success and gives you a scene that is missing the
    thing you are measuring.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    model_dir = os.path.abspath(os.path.join(here, "..", "models", "shuttlecock"))
    src = os.path.join(model_dir, "model.sdf")
    if not os.path.isfile(src):
        raise WorldMismatch(f"shuttlecock model.sdf missing at {src}")
    with open(src) as fh:
        sdf = fh.read()
    sdf = sdf.replace("model://shuttlecock/", f"file://{model_dir}/")
    missing = [p for p in re.findall(r"file://([^<>\s\"']+)", sdf)
               if not os.path.isfile(p)]
    if missing:
        raise WorldMismatch(
            "shuttlecock sdf points at files that do not exist, so it would "
            f"spawn without its skirt mesh: {missing}")
    with open(tmp, "w") as fh:
        fh.write(sdf)
    if not os.path.isfile(tmp) or os.path.getsize(tmp) == 0:
        raise WorldMismatch(f"failed to write the resolved sdf to {tmp}")
    return tmp


def ensure_shuttlecock(x=0.215, y=0.0, z=0.033, force=True, present=None):
    """Guarantee a usable shuttlecock exists in the world. -> True if spawned.

    force=True removes and respawns unconditionally. Use it once at the start
    of a run: a shuttlecock put there by anything other than this function (a
    launch file, a previous session) may be the mesh-less variant described in
    resolved_shuttlecock_sdf, and nothing in the model list can tell you which
    one you have.

    force=False respawns only when the object is missing or has left the world.
    That is the per-trial mode. nav_grasp_trials.py used to call this once per
    run, so anything that destroyed or ejected the object mid-run turned every
    remaining trial into a scored robot failure.

    `present` accepts an already-fetched model_list() so a caller checking
    several models at once does not pay for a second query.
    """
    sdf = resolved_shuttlecock_sdf()
    if not force:
        names = model_list() if present is None else present
        if "shuttlecock" in names:
            pose = model_pose("shuttlecock")
            if pose is not None and in_world(pose):
                return False
    remove_model("shuttlecock")
    spawn_model("shuttlecock", sdf, x, y, z)
    return True


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


# Lines the grasp node logs from inside its own sequence. At least one of them
# has to be in the log before a trial can be scored at all: an empty log means
# the node never got as far as looking, which is a launch/controller problem,
# not a robot failure. Read as "never detected" it silently deflates the rate.
GRASP_RAN_MARKERS = (
    "Searching at joint1",
    "Shuttlecock seen at base_link",
    "No reachable shuttlecock found",
    "Grasp sequence complete",
)
# Markers that positively identify an infrastructure fault even if the node did
# start logging.
GRASP_INFRA_MARKERS = (
    "manipulator_controller action server never appeared",
)


def run_grasp(log_path, timeout=300):
    """Run the pick sequence. -> (log_text, status).

    status is "ran" | "timeout" | "never_started". Anything other than "ran" is
    an infrastructure fault and must not be scored against the robot:

      * "timeout" -- the launch never exited. This used to raise
        TimeoutExpired straight out of the trial loop, which threw away every
        result collected so far in the run.
      * "never_started" -- the launch returned without the grasp node logging a
        single line of its own (package not built, workspace unsourced, MoveIt
        config missing, controllers not up). The old code handed that empty log
        to classify(), which read it as "no lift, cause unclear" and charged it
        to the robot.
    """
    cmd = ["ros2", "launch", "grab_sequence", "grasp_ball.launch.py"]
    status = "ran"
    with open(log_path, "w") as fh:
        try:
            # start_new_session so the whole launch tree can be signalled. A
            # bare kill of `ros2 launch` leaves its nodes orphaned, and the next
            # trial then runs against a second live copy of the grasp node.
            proc = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        except FileNotFoundError:
            fh.write("ros2 not on PATH\n")
            return "ros2 not on PATH\n", "never_started"
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            status = "timeout"
            _kill_tree(proc)

    log = open(log_path, errors="ignore").read()
    if status == "ran" and not any(m in log for m in GRASP_RAN_MARKERS):
        status = "never_started"
    if any(m in log for m in GRASP_INFRA_MARKERS):
        status = "never_started"
    return log, status


def _kill_tree(proc, grace=15):
    """SIGINT the process group, then SIGKILL what is left."""
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGKILL, 10)):
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            return
        try:
            proc.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            continue


def classify(log, before, after, gripper, grasp_status="ran"):
    """Why a trial ended the way it did. -> (PASS|FAIL|INDETERMINATE, reason).

    INDETERMINATE exists because this harness used to have no way to say "I do
    not know". A ground-truth query that timed out returned None, None read as
    "the object is not there", and that scored as a failed grasp. In one
    10-trial run it turned a 10/10 into a reported 8/10 -- one trial printed
    "[shuttlecock missing]" and another logged a final z of 0.000, both while
    the object was in the gripper at z=0.060.

    An infrastructure fault must not move the success rate in either direction,
    so these come out of the numerator *and* the denominator rather than being
    guessed at from the grasp log. The log is reported alongside so a human can
    adjudicate, but the harness does not score on it.
    """
    if grasp_status == "timeout":
        return "INDETERMINATE", "grasp launch never exited (infra)"
    if grasp_status == "never_started":
        return "INDETERMINATE", "grasp node never ran (infra)"
    if before is UNKNOWN or after is UNKNOWN:
        hint = ("grasp log reports a completed sequence"
                if "Grasp sequence complete" in log
                else "grasp log has no completion line")
        return "INDETERMINATE", f"ground-truth query failed; {hint}"
    if before is None:
        return "INDETERMINATE", "object absent before the trial started (infra)"
    if after is None:
        return "INDETERMINATE", "object vanished from the world mid-trial (infra)"

    if (after[2] - before[2]) > LIFT_THRESHOLD:
        return "PASS", ""
    if "No reachable shuttlecock found" in log:
        return "FAIL", "never detected / unreachable"
    if "not reachable" in log:
        return "FAIL", "seen but out of reach"
    if gripper is not None and gripper <= GRIPPER_FREE_CLOSE + 0.002:
        return "FAIL", "jaws closed on air (missed)"
    if gripper is not None and gripper >= GRIPPER_OPEN + 0.001:
        return "FAIL", "jaws forced open (fouled object)"
    moved = math.hypot(after[0] - before[0], after[1] - before[1]) > 0.02
    return "FAIL", "knocked object aside" if moved else "no lift, cause unclear"


def probe_pose(name):
    """model_pose with a failed query flattened to the UNKNOWN sentinel.

    For call sites that want to carry on and mark the trial INDETERMINATE
    rather than abort the run.
    """
    try:
        return model_pose(name)
    except GzQueryFailed as e:
        print(f"    ! {e}", flush=True)
        return UNKNOWN


def summarise(results, label="picked up"):
    """Print a scoreboard whose denominator excludes indeterminate trials.

    results: iterable of (verdict, why) pairs. Printing "8/10" when two of the
    ten never produced a valid measurement is the specific misreport this whole
    change exists to prevent, so the indeterminate count is always shown, even
    when it is zero.
    """
    rows = list(results)
    passes = sum(1 for v, _ in rows if v == "PASS")
    fails = sum(1 for v, _ in rows if v == "FAIL")
    indet = [w for v, w in rows if v == "INDETERMINATE"]
    scored = passes + fails
    if scored:
        print(f"\n  {passes}/{scored} {label}  "
              f"({len(indet)} indeterminate, excluded from the rate)")
    else:
        print(f"\n  0 scorable trials out of {len(rows)} -- "
              "every trial hit an infrastructure fault, so this run measured nothing")
    if indet:
        print("  indeterminate:")
        counts = {}
        for why in indet:
            counts[why] = counts.get(why, 0) + 1
        for why, count in sorted(counts.items(), key=lambda kv: -kv[1]):
            print(f"    {count:2d}x {why}")
    fail_modes = {}
    for verdict, why in rows:
        if verdict == "FAIL":
            fail_modes[why] = fail_modes.get(why, 0) + 1
    if fail_modes:
        print("  failure modes:")
        for why, count in sorted(fail_modes.items(), key=lambda kv: -kv[1]):
            print(f"    {count:2d}x {why}")


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 12
    rng = random.Random(20260812)
    results = []

    ensure_shuttlecock(force=True)

    for i in range(1, n + 1):
        kind = "?"
        try:
            # Re-verify the world every trial rather than once per run. Losing
            # the object mid-run used to turn every remaining trial into a
            # scored failure.
            if ensure_shuttlecock(force=False):
                print(f"    ! trial {i}: shuttlecock was missing, respawned", flush=True)

            # 0.30 m tolerance: loose enough for the slide-and-settle the drop
            # causes, tight enough to catch a set_pose that did nothing.
            set_pose("rosbot", 0, 0, 0, tol=0.30, settle=2.0)
            if not park_arm():
                raise GzQueryFailed("park_arm publishes failed; arm pose unknown")
            time.sleep(2)

            kind, x, y, z, q = sample_target(rng)
            set_pose("shuttlecock", x, y, z, *q)
            # Dropped in, it slides several cm before coming to rest. Reading
            # ground truth too early records a pose it has already left, which
            # scores a correct detection as a miss.
            time.sleep(8)

            before = probe_pose("shuttlecock")
            base_before = probe_pose("rosbot")
            log, grasp_status = run_grasp(f"/tmp/rep_trial_{i}.log")
            after = probe_pose("shuttlecock")
            base_after = probe_pose("rosbot")
        except (GzQueryFailed, WorldMismatch) as e:
            print(f"  trial {i:2d}  {kind:8s} INDETERMINATE  infra: {e}", flush=True)
            results.append((i, kind, "INDETERMINATE", f"infra: {e}", None, None, None))
            continue

        m = re.search(r"gripper (?:at|stalled at) ([-\d.]+)", log)
        gripper = float(m.group(1)) if m else None
        seen = re.search(r"seen at base_link \(([-\d.]+), ([-\d.]+)\)", log)
        det_err = None
        if seen and isinstance(before, tuple):
            aim = skirt_centre(before)
            det_err = math.hypot(float(seen.group(1)) - aim[0],
                                 float(seen.group(2)) - aim[1])
        base_yaw = (abs(base_after[5] - base_before[5])
                    if isinstance(base_before, tuple) and isinstance(base_after, tuple)
                    else None)

        verdict, why = classify(log, before, after, gripper, grasp_status)
        results.append((i, kind, verdict, why, det_err, gripper, base_yaw))
        print(f"  trial {i:2d}  {kind:8s} {verdict}  {why}", flush=True)

    print("\n" + "=" * 78)
    for kind in ("upright", "tipped"):
        sub = [r for r in results if r[1] == kind]
        if sub:
            k = sum(1 for r in sub if r[2] == "PASS")
            scored = sum(1 for r in sub if r[2] in ("PASS", "FAIL"))
            print(f"    {kind:8s} {k}/{scored} "
                  f"({len(sub) - scored} indeterminate)")
    errs = [r[4] for r in results if r[4] is not None]
    if errs:
        print(f"  detection error: mean {1000*sum(errs)/len(errs):.1f} mm, "
              f"worst {1000*max(errs):.1f} mm")
    yaws = [r[6] for r in results if r[6] is not None]
    if yaws:
        print(f"  base yaw drift:  mean {math.degrees(sum(yaws)/len(yaws)):.2f} deg, "
              f"worst {math.degrees(max(yaws)):.2f} deg")
    summarise([(r[2], r[3]) for r in results])


if __name__ == "__main__":
    main()
