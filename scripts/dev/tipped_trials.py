#!/usr/bin/env python3
"""Run N picks with the shuttlecock lying on its side, in the forward zone.

Separate from repeatability_test.py, which mixes upright and tipped. Lying flat
is the harder case: the object is only ~65 mm tall at the skirt and ~26 mm at the
cork, it rolls when touched, and the wrist roll has to align across its axis.

Scoring, world verification and the gz plumbing all come from
repeatability_test so the two harnesses cannot drift apart in what they call a
pass -- see the "talking to Gazebo" block there for why none of it takes a gz
reply at face value.
"""

import math
import os
import random
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from repeatability_test import (ARM_X, GzQueryFailed,  # noqa: E402
                                WorldMismatch, classify, ensure_shuttlecock,
                                park_arm, probe_pose, run_grasp, set_pose,
                                skirt_centre, summarise)


def tipped_pose(rng):
    """A spot in the forward reachable band, lying on its side at a random heading."""
    # Narrow forward cone. The CAD claw spans ~180 mm across when open, so at
    # side bearings it sweeps into the front wheels and body -- MoveIt refuses
    # those poses outright (gripper_right_link vs fr_wheel_link / body_link).
    # Straight ahead at longer radius puts the whole claw past the chassis front.
    # Pushed out to the far edge of the straight-down reach band (0.099..0.157),
    # to keep the claw well clear of the chassis and wheels.
    bearing = math.radians(rng.uniform(-10.0, 10.0))
    radius = rng.uniform(0.148, 0.157)
    x = ARM_X + radius * math.cos(bearing)
    y = radius * math.sin(bearing)
    yaw = rng.uniform(-math.pi, math.pi)
    h = math.pi / -4.0                      # half of -90 deg about X: lay it down
    qx_l, qw_l = math.sin(h), math.cos(h)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return x, y, 0.033, (cy * qx_l, sy * qx_l, sy * qw_l, cy * qw_l)


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    rng = random.Random(int(sys.argv[2]) if len(sys.argv) > 2 else 7)
    results = []

    # force=True: guarantees the skirt mesh actually loaded, which presence in
    # the model list cannot tell you. See ensure_shuttlecock.
    ensure_shuttlecock(force=True)

    for i in range(1, n + 1):
        try:
            # Re-verified per trial, not once per run: an object lost or
            # destroyed mid-run otherwise turns every remaining trial into a
            # scored robot failure.
            if ensure_shuttlecock(force=False):
                print(f"    ! trial {i}: shuttlecock was missing, respawned", flush=True)

            set_pose("rosbot", 0, 0, 0, tol=0.30, settle=2.0)
            if not park_arm():
                raise GzQueryFailed("park_arm publishes failed; arm pose unknown")
            time.sleep(2)

            x, y, z, q = tipped_pose(rng)
            set_pose("shuttlecock", x, y, z, *q)
            time.sleep(8)                        # it slides a few cm before settling

            before = probe_pose("shuttlecock")
            t0 = time.time()
            log, grasp_status = run_grasp(f"/tmp/tipped_{i}.log")
            dt = time.time() - t0
            after = probe_pose("shuttlecock")
        except (GzQueryFailed, WorldMismatch) as e:
            print(f"  trial {i}: INDETERMINATE  infra: {e}", flush=True)
            results.append(("INDETERMINATE", f"infra: {e}"))
            continue

        grip = re.search(r"gripper (?:at|stalled at) ([-\d.]+)", log)
        seen = re.search(r"seen at base_link \(([-\d.]+), ([-\d.]+)\)", log)
        roll = re.search(r"wrist roll ([-\d.]+) deg", log)
        err = None
        if seen and isinstance(before, tuple):
            aim = skirt_centre(before)
            err = 1000 * math.hypot(float(seen.group(1)) - aim[0],
                                    float(seen.group(2)) - aim[1])

        verdict, why = classify(log, before, after,
                                float(grip.group(1)) if grip else None,
                                grasp_status)
        results.append((verdict, why))
        # Every field below has to survive an unreadable pose. The old version
        # indexed `before` and `after` unconditionally, so a single timed-out
        # `gz model -p` raised TypeError mid-run and discarded the whole run's
        # results rather than marking one trial unknown.
        placed = (f"({before[0]:+.3f},{before[1]:+.3f})"
                  if isinstance(before, tuple) else "(   ?   ,   ?   )")
        z0 = f"{before[2]:.3f}" if isinstance(before, tuple) else "  ?  "
        z1 = f"{after[2]:.3f}" if isinstance(after, tuple) else "  ?  "
        print(f"  trial {i}: {verdict:13s} "
              f"placed {placed}  "
              f"det_err {f'{err:.0f}mm' if err is not None else '--':>6}  "
              f"roll {roll.group(1) if roll else '--':>5}deg  "
              f"grip {grip.group(1) if grip else 'closed on air':>13}  "
              f"z {z0}->{z1}  {dt:.0f}s"
              f"{'  ' + why if why else ''}", flush=True)

    summarise(results, label="lifted")


if __name__ == "__main__":
    main()
