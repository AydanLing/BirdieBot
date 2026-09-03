# Birdie Bot

https://github.com/user-attachments/assets/33159f59-d129-48f2-8405-3e28f053adb7

A ROSbot XL with an OpenMANIPULATOR-X arm clears a badminton court of
shuttlecocks. It drives to each one, aligns on an eye-in-hand depth camera,
picks it up, carries it back and drops it into an onboard hopper — then goes
looking for the next.

ROS 2 Jazzy, Gazebo Harmonic, nav2, MoveIt 2.

<!-- Replace with a recording: see docs/media/README.md -->
![Birdie Bot clearing one half of the court](docs/media/demo.gif)

```bash
ros2 launch grab_sequence grasp_trial.launch.py
```

Four shuttlecocks on the robot's half of the net, cleared nearest-first without
teleporting the base between picks.

## Results

| task | result |
|---|---|
| Court clearing, 4 shuttlecocks, one half | **4/4**, 6.3 min, deposited in the hopper |
| Court clearing, 16 shuttlecocks, full court | **14/16**, 34.7 min |
| Navigate-then-grasp, single object, 3.8–6.0 m | **10/10** |

Measured against Gazebo ground truth rather than the robot's own report, which
matters more than it sounds: the robot will happily log a clean pick while the
shuttlecock is lying on the floor behind it.

## How it works

Three layers of control, separate because each fails differently:

1. **nav2** crosses the court. Works in the map frame, trusts AMCL.
2. **`fine_approach`** closes the last stretch — still on AMCL, but turn-then-drive
   rather than a sampled optimiser.
3. **`visual_servo`** does the final correction from the camera alone. Trusts
   nothing in the map frame, which is what makes the whole thing robust to
   localisation error.

Then the arm picks the object up, and carries it to the hopper in staged moves —
rise, tilt, traverse, release — because doing it as one diagonal sweep clips the
chassis and flicks the shuttlecock out of the jaws.

## Reading the code

**[`docs/DESIGN.md`](docs/DESIGN.md)** is the substance of this repo: why each
component is the way it is, which decisions were forced by something real, and
which numbers came from measurement rather than taste. It opens with a glossary
if the ROS vocabulary is unfamiliar.

A few things it covers that took a while to learn:

- The robot drove at a fifth of its configured speed because the **local costmap
  was smaller than the controller's planning horizon** — every fast trajectory
  ran off the edge of the map and was discarded. 3×3 m → 6×6 m took it from
  0.175 m/s to 0.285 m/s and eliminated the optimiser failures entirely.
- A collision with a lidar that **wasn't there**: MoveIt was planning against a
  different robot description than the simulator was running. Confirmed by
  parking the real lidar a metre in the air and watching the planner go on
  colliding with it.
- The pick was assumed to be torque-bound. Measured, the arm servos peak at
  **4–15%** of available torque; the gripper's effort limit was set to 1 N on a
  prismatic joint — about the weight of a 100 g object.

## Building it

Needs three forked packages on non-default branches:

```bash
mkdir -p ~/rosbot_ws/src && cd ~/rosbot_ws/src
vcs import < https://raw.githubusercontent.com/AydanLing/BirdieBot/main/birdiebot.repos
cd ~/rosbot_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

The forks carry the hopper and lidar placement (`rosbot_ros`), the badminton
court world and its map (`husarion_gz_worlds`), and a camera resolution the
depth cloud can actually be used as a costmap source at
(`husarion_components_description`).

## Running it

```bash
ros2 launch grab_sequence grasp_trial.launch.py                    # 4 shuttles, headless
ros2 launch grab_sequence grasp_trial.launch.py headless:=False    # watch it
ros2 launch grab_sequence grasp_trial.launch.py shuttles:=8 trials:=2
ros2 launch grab_sequence grasp_trial.launch.py run_trial:=false   # stack only
```

Bringup takes about 3 minutes. `scripts/ops/teardown.sh` stops everything —
use it rather than Ctrl-C, for reasons in the next section.

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
came from getting this wrong. The launch file chains navigation to the seed's
process exit rather than a timer, and aborts if the seed fails.

**`bond_timeout` cannot be set from yaml, or at runtime.** nav2's launch files
pass only `{autostart}` and `{node_names}` to the lifecycle managers, so a
`lifecycle_manager_*` block in `config/amcl.yaml` is read by nobody. It is also
read when a bond is *created*, at activation, so setting it afterwards is
accepted and inert. The stack launches `autostart:=false` and
`scripts/ops/arm_lifecycle.py` substitutes a manager built with it disabled.
Without that, sustained load makes a manager miss a heartbeat and declare a
healthy server dead, which takes `map->base_link` down with it.

**Teardown misses four processes if you guess the patterns.** `ekf_node`,
`joy_node`, `teleop_node` and `opennav_docking` are named after their packages,
not after `ros2`/`nav2`/`gz`. The EKF is the expensive one: it has survived
several supposedly clean restarts carrying a diverged state
(`/odometry/filtered` reading `-2239, -8783` with the robot on the origin),
which feeds AMCL a garbage motion model.

**SIGINT does not stop this stack** — about 28 children survive it. SIGKILL,
then remove `/dev/shm/fastrtps_*` *after* everything is dead. Orphaned DDS
segments block port allocation, and the symptom is misleading: topics appear to
flow while lifecycle queries return nothing.

**`pkill -f <pattern>` matches the shell running it.** Killing your own shell
shows up as exit 143/144 with no output. Collect numeric PIDs first.

**The machine is the bottleneck.** 8 cores, and it has crashed at load 33.7. A
full stack plus a batch runs at 20–38. Run Gazebo headless, and under load the
`ros2` CLI's own discovery times out and reports healthy nodes as
`<no response>` — re-query before concluding anything died.

## Layout

```
launch/grasp_trial.launch.py   one command: stack + trials, in the order that works
grab_sequence/grasp_ball.py    the pick: detect, align, grasp, carry, deposit
scripts/collect_trials.py      court-clearing harness
scripts/nav_grasp_trials.py    single-object navigate-then-grasp harness
scripts/cmd_vel_guard.py       last-resort collision guard between nav2 and the wheels
scripts/scan_self_filter.py    drops laser returns that land on the robot itself
scripts/ops/                   bringup, teardown, AMCL seeding, lifecycle arming
config/amcl.yaml               nav2 tuning, with the measurements behind it
docs/DESIGN.md                 why everything is the way it is
```

## License

Apache 2.0. See [LICENSE](LICENSE).
