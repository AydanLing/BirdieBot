# Birdie Bot

Navigate-then-grasp for a Husarion ROSbot XL with an OpenMANIPULATOR-X arm and
an eye-in-hand ZED camera, in Gazebo Harmonic under ROS 2 Jazzy. The robot
drives to a shuttlecock somewhere on a badminton court, aligns on vision, and
picks it up.

Two harnesses:

| script | task |
|---|---|
| `scripts/nav_grasp_trials.py N` | N single-object trials; base teleports home between each |
| `scripts/collect_trials.py N` | N court-clearing trials; corner start, 16 shuttles, no teleport home |

Best measured results: **10/10** single-object at 3.84–6.02 m; mean trial time
94 s after the MPPI `temperature` fix (144 s before it). **93%** (42/45) on the 16-shuttle collection task over three trials.

## Running it

```bash
scripts/ops/bringup.sh                 # ~3.5 min, headless
python3 scripts/nav_grasp_trials.py 10
scripts/ops/teardown.sh
```

`bringup.sh` takes a world name and map path if you want something other than
`badminton_court`. `GZ_HEADLESS=False` gives you the GUI, at the cost of about
a core — see the load note below.

## Things that will cost you a day if you rediscover them

**Launch order is not negotiable.**

```
gazebo -> scan filter + guard -> localization -> SEED AMCL -> navigation
```

nav2's `global_costmap` will not activate without `map->base_link`. That fails
`planner_server`, and the lifecycle manager then abandons the rest of the
bringup. Seeding *after* launching navigation is the intuitive order and it
races the costmap's activation timeout. Every half-active stack this project
produced — `controller_server` active while the other five sat `inactive` —
came from getting this wrong. `bringup.sh` refuses to launch navigation if the
seed did not take.

**`bond_timeout` cannot be set from yaml.** nav2's launch files pass only
`{autostart}` and `{node_names}` to the lifecycle managers, so a
`lifecycle_manager_*` block in `config/amcl.yaml` is read by nobody. It has to
be set at runtime. Without it, sustained load makes a manager miss a heartbeat
and declare a healthy server dead (`CRITICAL FAILURE: SERVER map_server IS
DOWN`), which takes `map->base_link` down with it.

**Teardown misses four processes if you guess the patterns.** `ekf_node`,
`joy_node`, `teleop_node` and `opennav_docking` are named after their packages,
not after `ros2`/`nav2`/`gz`. The EKF is the expensive one: it has survived
several supposedly clean restarts carrying a diverged state
(`/odometry/filtered` reading `-2239, -8783` with the robot on the origin),
which feeds AMCL a garbage motion model. Use `scripts/ops/teardown.sh`, and
note it reports whether it actually got everything.

**SIGINT does not stop this stack** — about 28 children survive it. SIGKILL,
then remove `/dev/shm/fastrtps_*` *after* everything is dead. Orphaned DDS
segments block port allocation, and the symptom is misleading: topics appear to
flow while lifecycle queries return nothing.

**`pkill -f <pattern>` matches the shell running it.** Killing your own shell
shows up as exit 143/144 with no output. Collect numeric PIDs first.

**The machine is the bottleneck.** 8 cores, and it has crashed at load 33.7.
A full stack plus a batch runs at 20–38. Run Gazebo headless, do not leave
extra instrumentation streaming, and never leave a second Gazebo alive. Under
load the `ros2` CLI's own discovery times out and reports healthy nodes as
`<no response>` or `unreachable` — re-query before concluding anything died.

**A long run leaks DDS segments.** Each grasp spawns a launch, so a 16-shuttle
trial creates and tears down 16 sets of DDS participants and a 5-trial run
creates 80. `collect_trials.py` sweeps unmapped segments between trials. Do not
sweep mid-trial; it does not recover a failing run and may disturb live
participants.

## Configuration worth knowing about

`config/amcl.yaml` carries AMCL, both costmaps and MPPI. The comments there
record what was measured rather than assumed — notably why the depth costmap's
marking ceiling is 0.70 (the badminton net's lower edge sits at 0.760 and was
painting a phantom barrier across the court), why `visualize` is off, and why
`prune_distance` is 3.0.

`use_servo` defaults to **false** in `rosbot_controller/launch/manipulator.yaml`.
Nothing in the autonomous stack drives `moveit_servo`; it was costing 64% of a
core to serve a gamepad nobody was holding. Set it true for teleop.

## Known open issues

- **MPPI speed: largely solved, headroom remains.** The cap was `temperature`,
  the softmax selectivity: at 0.3 the optimiser averaged good and mediocre
  samples together and its output was dragged toward the middle of the batch.
  0.05 raised peak commanded vx from ~0.19 to ~0.31 m/s and cut trial time 35%
  (nav 68.3 -> 34.0 s). It still asks for well under `vx_max` 0.8 while the base
  does 0.601 on a direct command, so there is more to find.
- **Reach bias in collection runs.** Picks land at 0.023–0.166 m where the
  single-object harness gives 0.104–0.180, and failures cluster at the low end.
  Correlates with hop length.
- **Bond-heartbeat instability under sustained load** is mitigated, not solved.
  It is what stopped the 4th and 5th collection trials.
- Ground-obstacle raytrace clearing, the `taper_jaws` naming, and some dead
  xacro were all triaged as not worth fixing.
