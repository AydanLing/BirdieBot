"""A running list of where the robot believes shuttlecocks are.

Detections arrive one frame at a time, from a moving base, at whatever range
the object happened to be when it crossed the camera. Any single one of them is
worth a few centimetres at best. This module is what turns that stream into a
list of distinct objects worth driving to.

Three jobs, none of which the per-frame detector can do:

  merge      The same shuttlecock is seen on dozens of consecutive frames and
             again on a later pass from a different angle. Those are one
             object, not fifty. Detections within MERGE_RADIUS of a known
             target are folded into it.

  confirm    A single frame is not evidence. Yellow noise, a glint off the
             floor, or a half-occluded blob all produce one-frame ghosts.
             A target is only offered up for a pick once MIN_HITS separate
             frames have agreed on it.

  retire     A collected shuttlecock is gone, but the map does not know that
             and would happily send the robot back to the empty patch of floor
             where it used to be. Retiring blacklists the location so a stale
             detection still in flight cannot resurrect it.

Deliberately free of ROS imports. Everything here is arithmetic on tuples,
which means it can be tested in milliseconds without a simulator, and the
clustering rules can be argued about without standing up a robot.

All coordinates are in the `map` frame. That is the whole point -- see
shuttle_scanner.ShuttleScanner for why detections have to leave `base_link`
before they can be accumulated at all.
"""

import itertools
import math
import statistics

# Detections closer together than this are treated as the same object.
#
# Bounded below by measurement error and above by object separation. At the 3 m
# edge of detection range a blob is ~11 px, so its centroid is uncertain by a
# pixel or two (13 mm/px at that range) on top of the depth sensor's 30 mm
# sigma -- call it 50 mm typical and 150 mm on a bad frame. The scatter in
# collect_trials keeps shuttlecocks MIN_SEP = 0.90 m apart. 0.35 m sits clear of
# both: loose enough that one object never splits into two entries, tight enough
# that two objects never collapse into one.
MERGE_RADIUS = 0.35

# Frames that must agree before a target is worth driving to.
#
# Driving past at 0.285 m/s with depth arriving at ~4.2 Hz gives roughly 15
# frames of an object inside the 3 m detection radius, so three is cheap. It is
# the single most effective filter against one-frame yellow noise, which by
# definition cannot repeat in the same place.
MIN_HITS = 3

# How far a retired target keeps suppressing new detections.
#
# Slightly wider than MERGE_RADIUS so a detection that lands just outside the
# original cluster still gets absorbed, and still well inside MIN_SEP so
# retiring one shuttlecock never blinds the robot to its neighbour.
RETIRE_RADIUS = 0.40

# Failed pick attempts before a target is written off.
#
# Not one. A drive-by fix is worth centimetres, and the grasp-grade detector
# re-runs from a standoff on arrival, so the first failure is more likely to
# mean "the estimate was off" than "there is nothing here". Two consecutive
# failures, the second from close range with a fresh look, is a ghost.
MAX_MISSES = 2

# Observations averaged into a target's position estimate.
#
# Kept to the closest few rather than all of them. Error scales with range, so
# a handful of 1 m looks carry far more information than fifty 3 m looks, and
# letting the distant ones vote just drags the estimate around.
POSITION_SAMPLES = 5


class Target:
    """One believed shuttlecock, and the detections backing that belief."""

    _ids = itertools.count(1)

    def __init__(self, x, y, z, rng):
        self.id = next(Target._ids)
        # (x, y, z, range_at_detection), newest last.
        self.observations = [(x, y, z, rng)]
        self.misses = 0
        self.retired = False
        self.retire_reason = None

    @property
    def hits(self):
        return len(self.observations)

    @property
    def confirmed(self):
        return self.hits >= MIN_HITS and not self.retired

    @property
    def position(self):
        """Best (x, y, z) estimate: median over the closest observations.

        Median rather than mean for the same reason locate_ball uses one -- the
        simulated ZED's depth noise is Gaussian with occasional large outliers,
        and a single 40 mm flier in a five-sample mean moves the answer by
        8 mm, which is most of the jaw clearance.
        """
        best = sorted(self.observations, key=lambda o: o[3])[:POSITION_SAMPLES]
        return (statistics.median([o[0] for o in best]),
                statistics.median([o[1] for o in best]),
                statistics.median([o[2] for o in best]))

    @property
    def best_range(self):
        """Closest range this target has ever been seen from."""
        return min(o[3] for o in self.observations)

    def observe(self, x, y, z, rng):
        self.observations.append((x, y, z, rng))

    def replace(self, observations):
        """Throw away the history and adopt a fresh set of measurements.

        Used only by a deliberate stationary re-look. Appending would not work:
        `position` medians the closest few observations, and a drive-by pass
        contributes hundreds of them, so a handful of better ones taken from a
        standstill would be outvoted by the very data they are meant to
        correct. A stationary measurement is categorically better information
        than anything gathered while moving, so it supersedes rather than
        joins.
        """
        self.observations = list(observations)

    def __repr__(self):
        px, py, _pz = self.position
        state = self.retire_reason if self.retired else (
            "confirmed" if self.confirmed else f"{self.hits}/{MIN_HITS}")
        return f"<Target {self.id} ({px:+.2f},{py:+.2f}) {state}>"


class TargetMap:
    """Every shuttlecock the robot currently believes exists."""

    def __init__(self, merge_radius=MERGE_RADIUS, retire_radius=RETIRE_RADIUS):
        self.merge_radius = merge_radius
        self.retire_radius = retire_radius
        self.targets = []

    # --- writing ---------------------------------------------------------

    def observe(self, x, y, z, rng):
        """Fold one detection in. Returns the Target it landed on, or None.

        None means the detection was inside the blacklist radius of something
        already retired, i.e. it is a stale sighting of a shuttlecock that is
        already in the hopper.
        """
        nearest, best = None, float("inf")
        for t in self.targets:
            tx, ty, _tz = t.position
            d = math.hypot(tx - x, ty - y)
            if d < best:
                nearest, best = t, d

        if nearest is not None and nearest.retired and best <= self.retire_radius:
            return None
        if nearest is not None and not nearest.retired and best <= self.merge_radius:
            nearest.observe(x, y, z, rng)
            return nearest

        fresh = Target(x, y, z, rng)
        self.targets.append(fresh)
        return fresh

    def retire(self, target, reason):
        """Mark a target gone. Its location keeps suppressing detections."""
        target.retired = True
        target.retire_reason = reason

    def record_miss(self, target):
        """A pick attempt found nothing. Retire once it has happened enough."""
        target.misses += 1
        if target.misses >= MAX_MISSES:
            self.retire(target, "ghost")
        return target.retired

    # --- reading ---------------------------------------------------------

    @property
    def confirmed(self):
        return [t for t in self.targets if t.confirmed]

    def nearest(self, x, y):
        """Closest confirmed target to (x, y), or None if the queue is dry."""
        live = self.confirmed
        if not live:
            return None
        return min(live, key=lambda t: math.hypot(t.position[0] - x,
                                                  t.position[1] - y))

    def counts(self):
        """(confirmed, pending, retired) for logging."""
        conf = sum(1 for t in self.targets if t.confirmed)
        ret = sum(1 for t in self.targets if t.retired)
        return conf, len(self.targets) - conf - ret, ret
