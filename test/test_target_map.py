"""Clustering rules for the drive-by target map.

These are the rules that decide where the robot drives next, and every one of
them is a judgement call that could plausibly have gone the other way. Pinning
them here means retuning a radius is a deliberate act with a failing test
attached, rather than a silent behaviour change discovered mid-trial.

No ROS, no simulator -- target_map is deliberately pure arithmetic, so the
whole file runs in milliseconds.
"""

from grab_sequence.target_map import MAX_MISSES, MIN_HITS, TargetMap


def _saturate(tmap, x, y, z=0.033, rng=1.5, n=MIN_HITS):
    """Observe the same point enough times to confirm it."""
    for _ in range(n):
        hit = tmap.observe(x, y, z, rng)
    return hit


def test_single_detection_is_not_confirmed():
    """One frame is never enough -- this is the whole defence against noise."""
    tmap = TargetMap()
    tmap.observe(1.0, 2.0, 0.033, 1.5)
    assert tmap.confirmed == []


def test_repeated_detections_confirm_one_target():
    tmap = TargetMap()
    _saturate(tmap, 1.0, 2.0)
    assert len(tmap.confirmed) == 1
    assert len(tmap.targets) == 1


def test_nearby_detections_merge():
    """Scatter within the merge radius is one shuttlecock, not several."""
    tmap = TargetMap()
    for dx in (0.0, 0.10, -0.08):
        tmap.observe(1.0 + dx, 2.0, 0.033, 1.5)
    assert len(tmap.targets) == 1
    assert len(tmap.confirmed) == 1


def test_distant_detections_stay_separate():
    """0.90 m is the minimum separation the scatter enforces."""
    tmap = TargetMap()
    _saturate(tmap, 1.0, 2.0)
    _saturate(tmap, 1.9, 2.0)
    assert len(tmap.confirmed) == 2


def test_position_prefers_close_observations():
    """Error scales with range, so a 1 m look must outvote 3 m looks."""
    tmap = TargetMap()
    for _ in range(4):
        tmap.observe(1.30, 2.0, 0.033, 3.0)   # far and wrong
    for _ in range(4):
        tmap.observe(1.00, 2.0, 0.033, 1.0)   # close and right
    x, _y, _z = tmap.confirmed[0].position
    assert abs(x - 1.00) < 0.02


def test_position_ignores_a_single_outlier():
    tmap = TargetMap()
    for _ in range(4):
        tmap.observe(1.0, 2.0, 0.033, 1.0)
    tmap.observe(1.30, 2.0, 0.033, 1.0)       # 300 mm flier
    x, _y, _z = tmap.confirmed[0].position
    assert abs(x - 1.0) < 0.01


def test_retired_target_leaves_the_queue():
    tmap = TargetMap()
    target = _saturate(tmap, 1.0, 2.0)
    tmap.retire(target, "collected")
    assert tmap.confirmed == []
    assert tmap.nearest(0.0, 0.0) is None


def test_retired_location_suppresses_stale_detections():
    """A frame still in flight must not resurrect a shuttlecock in the hopper."""
    tmap = TargetMap()
    target = _saturate(tmap, 1.0, 2.0)
    tmap.retire(target, "collected")
    assert tmap.observe(1.05, 2.0, 0.033, 1.5) is None
    assert tmap.confirmed == []


def test_retirement_does_not_blind_the_neighbour():
    """Blacklist radius must stay well inside the 0.90 m scatter separation."""
    tmap = TargetMap()
    target = _saturate(tmap, 1.0, 2.0)
    tmap.retire(target, "collected")
    _saturate(tmap, 1.9, 2.0)
    assert len(tmap.confirmed) == 1


def test_one_failed_pick_does_not_write_a_target_off():
    """A drive-by fix is worth centimetres; the first miss is likely the fix."""
    tmap = TargetMap()
    target = _saturate(tmap, 1.0, 2.0)
    assert tmap.record_miss(target) is False
    assert target in tmap.confirmed


def test_repeated_failures_retire_the_target():
    tmap = TargetMap()
    target = _saturate(tmap, 1.0, 2.0)
    for _ in range(MAX_MISSES):
        tmap.record_miss(target)
    assert tmap.confirmed == []
    assert target.retire_reason == "ghost"


def test_nearest_is_measured_from_the_base():
    tmap = TargetMap()
    _saturate(tmap, 5.0, 0.0)
    near = _saturate(tmap, 1.0, 0.0)
    assert tmap.nearest(0.0, 0.0) is near
    assert tmap.nearest(6.0, 0.0) is not near


def test_replace_supersedes_a_drive_by_history():
    """A stationary re-look must win against hundreds of moving samples.

    This is the whole point of `replace`. Appending would leave the fresh
    measurements outvoted by the very data they exist to correct.
    """
    tmap = TargetMap()
    target = None
    for _ in range(200):
        target = tmap.observe(1.55, 2.0, 0.033, 3.0)   # driving past, 550 mm out
    assert abs(target.position[0] - 1.55) < 0.01

    fresh = TargetMap()
    for _ in range(6):
        good = fresh.observe(1.0, 2.0, 0.033, 1.8)     # stopped, looking at it
    target.replace(good.observations)
    assert abs(target.position[0] - 1.00) < 0.01
    assert target.confirmed


def test_replace_keeps_the_target_usable():
    """Identity and retirement state survive a re-measurement."""
    tmap = TargetMap()
    target = _saturate(tmap, 1.0, 2.0)
    ident = target.id
    target.replace([(1.2, 2.1, 0.033, 1.0)] * MIN_HITS)
    assert target.id == ident
    assert tmap.nearest(0.0, 0.0) is target


def test_counts_split_confirmed_pending_and_retired():
    tmap = TargetMap()
    _saturate(tmap, 1.0, 0.0)
    tmap.observe(3.0, 0.0, 0.033, 2.0)               # pending, one hit
    tmap.retire(_saturate(tmap, 5.0, 0.0), "collected")
    assert tmap.counts() == (1, 1, 1)
