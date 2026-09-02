# Commit message archive

The full original commit messages, kept because the git history was
rewritten to one-line subjects and these bodies carry the measurements
behind most of the constants in this repo.

SHAs below are the ORIGINAL ones, from before the rewrite. They no longer
resolve; the mapping to the current SHAs is at the bottom of this file.

## ea0a24c  2026-09-02  Give the carry stages reach margin so the hopper deposit actually runs

Both sat exactly on their IK limits after the standoff moved out, so move_to_point's
re-aim pushed past; hopper drops to 150 mm to buy the headroom.

---
## 8564a76  2026-09-02  Launch four shuttlecocks on the robot's own half of the net

Field is bounded to the side START_X is on; the launch now runs collect_trials.

---
## 8e31892  2026-09-02  Find grasp_ball and the shuttlecock model via the package, not a relative source path

Installed scripts live in lib/grab_sequence, where '../grab_sequence/grasp_ball.py'
resolves to nothing; this is why the launch initialised but never grasped.

---
## 7ed2697  2026-09-02  Document the reasoning behind the code

docs/DESIGN.md. The code says what it does and its comments say why a
given line is the way it is; this is the layer above that. What each
component is for, which decisions were forced by something real, and
which numbers came from measurement rather than taste.

Opens with a glossary, because the terms are load-bearing and several of
them mean something specific here. AMCL, seeding, races, bonds, costmaps,
tool pitch, RTF, QoS, DDS shared memory. Someone reading this code
without ROS background otherwise has to reverse-engineer the vocabulary
before they can read the reasoning.

Organised so the ordering constraints come first, since bringup order is
the single thing most likely to be broken by someone tidying up, and it
fails in ways that look like unrelated subsystem bugs.

Every constant with a story has the measurement next to it: the 3x3 local
costmap capping MPPI at 0.175 m/s against a base that does 0.601 on
command; 30 degrees of object rotation carrying 2/2 where 90 dropped 2/2;
1 N of grip force on a prismatic joint; arm servos peaking at 4 to 15
percent of their available torque while the pick was assumed to be
torque-bound. Changing those numbers without repeating the measurements
is how the bugs come back.

Closes with a symptom-to-cause table for the traps that cost the most
time. Most of them present as a fault in whatever subsystem you happen to
be working on rather than where they actually are: a stale planning model
reads as a collision bug, a leaked shared memory segment reads as a
localisation bug, a failed build that leaves a stale install reads as a
change that did nothing.

Verified the constants quoted here against the source rather than writing
them from memory, which given the error rate in arriving at them seemed
like the minimum.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 9763c1c  2026-09-02  One launch file for the whole badminton-court grasp trial

    ros2 launch grab_sequence grasp_trial.launch.py

replaces scripts/ops/bringup.sh, which did the same job but reached into
the source tree by absolute path and only worked from a shell in this
workspace.

The ordering is the content of this file, not the node list:

    gazebo -> scan filter + guard -> localization -> SEED AMCL -> navigation

Navigation is chained to the seed's process EXIT rather than to a timer,
and that is not tidiness. On a timer it raced and lost: the seed needed a
second attempt, reported "map->base_link OK" after navigation had already
started, and the ordering the file exists to enforce held only by luck.
nav2's global_costmap will not activate without that transform, the
failure takes planner_server with it, and the lifecycle manager then
abandons the rest of the bringup, which is where every half-active stack
in this project came from. A non-zero seed now shuts the launch down
rather than handing nav2 something it cannot activate.

Both nav2 launches run autostart:=false and are started afterwards by
arm_lifecycle.py, because bond_timeout is read when a bond is created, at
activation. With autostart the managers bond before anything can change
it and a later set is accepted but inert.

Supporting changes:

setup.py installs the ops scripts and amcl.yaml. They were only ever read
out of the source tree. repeatability_test.py and collect_trials.py go
with them: nav_grasp_trials imports the first as a sibling, and an
install missing it fails at runtime with ModuleNotFoundError that nothing
surfaces until the trial starts.

The scripts ignore argv from "--ros-args" on, since launch_ros appends it
to every node it starts and positional parsing otherwise reads a flag as
a value.

Also fixes a pre-existing crash in nav_grasp_trials' summary, which
referenced contacts.stream_bytes and contacts.topics. Both belonged to
the contact-sensor ClearanceMonitor that was replaced when contact
sensors turned out never to instantiate on runtime-spawned models. Every
otherwise-successful run printed its results and then died with
AttributeError, exiting non-zero.

Verified end to end: 2/2 picked up, 0 obstacle contacts over 11763
clearance samples, from a single command on an empty machine.

Note the two "lifecycle_manager ... exit code -9" errors in the log are
expected. That is arm_lifecycle.py killing the stock manager to put its
own in place.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## bf9e6e2  2026-08-31  Change project name to Birdie Bot

Updated project title and description in README.
---
## c55cd67  2026-08-26  Park further out from the chassis; make the shuttle count settable

STANDOFF_REACH 0.152 -> 0.195. The old value put the object at x 0.217 in
base_link, 56 mm beyond the chassis front edge at 0.161, and each claw
mesh reaches 70.6 mm to one side, so on descents where the wrist roll
pointed a jaw inboard it clipped the body: "Found a contact between
'body_link' and 'gripper_right_link'" on descend z=0.020, in 2 of 5
trials. At 0.195 the object sits at 0.260, 99 mm clear.

The IK reaches x 0.300 at grasp height, so this is well inside the
envelope. The 0.099..0.157 radius band in the module docstring describes
a narrower case than arm_joints_for actually solves, and reading it as
the limit is what kept the standoff short.

Flagging honestly: a later 5-trial run showed "Arm move failed: search
pose" and an IK rejection at radius 0.144 that had not appeared before
this change. The bench used for the hopper work places the object
directly and does not exercise the search, so the regression is not
characterised. Worth a nav_grasp_trials batch before trusting it.

collect_trials reads COLLECT_N so a short validation run does not need a
code edit; the field geometry is unchanged by the count.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 2573edf  2026-08-26  Carry the shuttlecock to the hopper and release it

Adds the deposit: after the lift the arm takes the object to the hopper
and drops it in, instead of the pick ending with the object held.

The carry is staged, and that is the whole trick. It began as one
move_to_point from the grasp pose to the release pose, which translated
340 mm backwards, climbed 170 mm and rotated the tool all at once. The
claw took a diagonal swipe through the chassis and flicked the
shuttlecock out on the way, which read as a clean run in the logs: the
arm arrived over the mouth within 3 mm, logged its release, and the
object was already lying on the floor behind the robot. It is now rise,
tilt, traverse, release, and nothing moves laterally until the claw is
clear of the deck.

Two supporting changes were needed.

arm_joints_for grew a pitch argument. It always passed pi/2 to ik_planar
and the tool therefore always pointed down, which matters because the
jaws cradle the object ACROSS the tool axis: a vertical tool can only
hold a shuttlecock horizontal, and it lands across the mouth rather than
dropping through it. Tilting also lifts the ceiling a long way. Over the
hopper, tool-down tops out at z 0.215 while 45 degrees reaches 0.345,
which is the difference between a 57 mm tray and a 180 mm tube. The tool
offset now applies along the tool axis rather than straight down; the old
form was correct only at pi/2 and would silently mislocate the pads at
any other pitch. At pi/2 the new expression reduces to the old one.

The deposit tolerance is 20 mm rather than the 4 mm used for grasping.
move_to_point re-aims until it is inside tolerance, and demanding 4 mm
over a 100 mm opening meant it kept re-aiming after it had arrived: four
of five deposits reached the mouth at 4.1 to 7.9 mm and then died with
"No IK for adjusted goal during over hopper".

45 degrees for the traverse is a measured compromise. A taller hopper
needs a flatter tool to clear the rim, and a flatter tool turns the
object further from the attitude it was grasped in. 30 degrees of
rotation carried 2/2 and 90 degrees dropped 2/2, so the height is capped
by the grip rather than by reach. Fixing the grip is what would lift it
further: the jaws stall 9 mm short of their commanded close, pinching the
cork instead of enclosing it.

Measured: 2/2 deposited at 126 mm, and 2/2 again at 180 mm.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## fba5421  2026-08-26  Plan against the robot that is actually running

All three launches built MoveIt's model from manipulation.yaml while the
simulation is brought up with configuration:=manipulation_pro. Those are
different robots: the plain config puts the lidar on the rear deck at
(-0.125, 0, 0.07) and carries no ZED on link5 at all.

Every collision check in this project has therefore been run against a
robot that has not existed for weeks, and it was not harmless. Deposits
into the hopper failed with "Found a contact between 'rplidar_link' and
'gripper_left_link'" while the hopper occupied exactly the spot the
phantom lidar sat in. The lidar was physically moved four times chasing
that contact and none of it changed anything, because the obstacle was in
the model rather than the world.

What settled it: the real lidar was parked a metre in the air, the sim
reported rplidar_link at z +1.133, and the planner went on colliding with
it. Two robots, one of them imaginary.

Worth keeping in mind that a stale planning model fails quietly. It does
not error; it just refuses paths, or permits ones it should not, and the
symptoms look like whatever subsystem you happen to be working on.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 71ac6ae  2026-08-25  Give the local costmap enough reach for MPPI's horizon

The local costmap was a 3 x 3 m rolling window, so the robot could see
1.5 m in any direction. MPPI's horizon is time_steps x model_dt =
56 x 0.05 = 2.8 s, which at vx_max 0.8 sweeps 2.24 m. Every candidate
trajectory quick enough to be worth having ran off the edge of the
costmap and was discarded, so the optimiser could only commit to speeds
whose entire horizon stayed inside 1.5 m.

That is the missing explanation for the speed cap that three earlier
attempts failed to find. The guard, the drivetrain, prune_distance and
the critic weights were all ruled out by measurement -- commanded, guard
output and achieved velocity agreed to within 0.01 m/s, and a direct
0.60 m/s command was met exactly -- but nothing had looked at how far the
optimiser could actually see.

Measured on a 10 m straight run across the court:

    3 x 3 costmap   0.175 m/s, and in the worst state 0.048 m/s with the
                    robot travelling 0.08 m in 35 s while the controller
                    logged "Optimizer fail to compute path" 18 times
    6 x 6 costmap   0.285 m/s mean, 0.351 peak, the full 10 m covered,
                    and zero optimizer failures

Six metres gives 3 m of reach, comfortably past the 2.24 m vx_max needs.

This is a real gain but not the whole story: 0.285 is still well short of
the 0.8 the base is configured for and demonstrably capable of, so
something else is limiting it too. Worth noting that a 6 x 6 costmap at
0.05 resolution is four times the cells of a 3 x 3, which is not free on
a machine that is already the bottleneck; if that hurts, coarsening the
resolution is cheaper than shrinking the window back.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 64063c4  2026-08-25  Approach forwards from a metre out, and rehome the arm on the move

Four changes to the approach, all aimed at the same thing: the base
should be driving forwards at something it can see, and should not be
waiting on anything it could be doing already.

Never reverse. fine_approach used to back up whenever the goal sat more
than 90 degrees behind, on the reasoning that spinning 180 for a few
centimetres is wasteful. On a collection sweep that is the wrong trade --
the base reverses toward an object it cannot see, because the camera
rides on the arm and is pointed the other way, and reverse is capped at
vx_min 0.35 against 0.8 forwards. The turn now happens up front, gated at
FINE_TURN_FIRST 0.09 rad, and the drive is always positive.

Stand off a metre rather than 0.22. Parking almost on top of the object
left nothing between "arrived" and "too far", so every overshoot had to
be reversed out of. Aiming 0.12 m beyond the metre means the expected
overshoot still lands short, and the vision stage closes the gap going
forwards. The standoff sits on the line from base to object, so once the
turn is done the drive arrives already facing it with no closing
rotation. Clamped so it is never behind the base -- picks are
nearest-first and the next object is often inside a metre already.

Rehome the arm during the drive. park_arm shells out to `ros2 topic pub`
twenty times and blocks for seconds, and it was being called on arrival,
so the base sat at the standoff waiting for a move that could have
happened en route. park_arm_async publishes the same pose on the node's
own publisher and returns.

Stop sweeping once the object is found. The bearing sweep costs a 2 s arm
move plus a 2 s settle per bearing, and since the neighbour-lock fix it
was running all five every time to refine a match the vision stage then
iterates on anyway -- about 20 s a pick. It now stops at the first
sighting within VISION_ACCEPT 0.25 m, with VISION_GATE still discarding
neighbouring objects.

Detection at the longer range is fine: the skirt is ~133 px^2 at 1.12 m
against a MIN_CONTOUR_AREA of 20.

Note fine_approach is shared with nav_grasp_trials, so the no-reverse
change lands there too. Low risk, since nav2 hands that harness a small
residual with the goal ahead, but it is not yet re-validated at 10/10.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 59a131c  2026-08-25  Pick the blob nearest the target, not the biggest in frame

With one object about, "largest yellow contour, first bearing that sees
anything" is a fine detector. With sixteen it is not: the robot kept
driving to a neighbour. Reaches in a tipped 16-shuttle run came back
0.967 and 0.968 m against a 0.152 m aim point, and MIN_SEP between
shuttles is 0.90 -- it had gone to the wrong shuttle and knocked the
right one aside on the way.

Two things caused it and both are fixed. _deproject took the largest
contour anywhere in frame, and a lying shuttle side-on presents a bigger
blob than a nearer one seen end-on, so bigger did not mean closer.
find_object returned the first bearing in its sweep that saw anything at
all, so a neighbour off to one side won simply by being looked at first.

Callers that know where their target should be now say so. _deproject_all
returns every blob, detect(expect) takes the nearest to the prediction
and ignores anything beyond VISION_GATE = 0.45 m, and find_object(expect)
sweeps every bearing and keeps the best match rather than the first
sighting. The gate sits between the two error scales either side of it:
AMCL plus the fine approach land within about 0.2 m, while neighbours are
0.90 m away.

visual_servo recomputes the prediction from the target's map position on
every iteration, because the base moves between them and a prediction
computed once is stale by the second look.

The single-object path passes no prediction and is bytewise unchanged in
behaviour -- _deproject is still largest-blob, find_object still returns
on first sighting. Not yet validated on a run; the collection harness has
not completed a trial since the field went to sixteen.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 8dd124c  2026-08-24  Fix three faults that made collection runs unrunnable

All three were mine, and each one masked the next.

The shm sweep was destroying live DDS segments. It decided a segment was
orphaned by testing /proc/*/maps, which is not a liveness test: a process
can hold a shm file open without it being mapped at the instant the sweep
looks, and FastDDS maps on demand. The damage was plain once looked at
directly -- a freshly brought-up stack seeded AMCL and localised fine,
the harness started, swept 150 segments, and from that moment the gz->ROS
bridges relayed nothing. /clock went silent on the ROS side while Gazebo
was still stepping at RTF 0.57 and gz-side /scan still carried data. With
no scans reaching AMCL it never published map->base_link again and every
trial aborted at the corner seed, which is where three separate
diagnoses went looking. The function is retired in place as a record.

bond_timeout was never actually being disabled. It is read into a member
when the lifecycle manager is CONSTRUCTED and there is no parameter
callback, so setting it at runtime is accepted, reports success, and
changes nothing -- the manager's own log still printed "Creating bond
timer...", which it only does when the timeout is non-zero. nav2_bringup
does not forward params_file to its manager either, so the only way is
to launch with autostart:=false, replace the stock manager with one
constructed with bond_timeout:=0.0, and start the stack with that.
scripts/ops/arm_lifecycle.py does this and the bond timer count is now 0.

A pick could hang unboundedly. One ran for 10968 s -- three hours on a
single shuttle in a run that managed seven picks in three and a half.
Every individual call is bounded, so the stall was in something that
waits without a deadline; a SIGALRM watchdog around the whole attempt
catches that class of fault rather than one member of it.

Also: shuttles now spawn lying on their side at a random heading, the
same construction nav_grasp_trials.target_pose uses. They were being
spawned at the default orientation, standing on their cork, which is
both the easy case and not what a struck shuttle does.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## af59319  2026-08-24  Drop set -u from bringup.sh; ROS setup files trip it

/opt/ros/jazzy/setup.bash reads AMENT_TRACE_SETUP_FILES while unset, so
the script died on its own first source line with "unbound variable"
before starting anything. Found by running it, not by reading it.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## cda0ec3  2026-08-24  Drop MPPI temperature to 0.05; trials get 35% faster

This was the throttle behind the long-standing puzzle of nav2 commanding
about a fifth of vx_max while the base itself does 0.601 m/s on a direct
command with the guard passing it through untouched.

temperature is the softmax selectivity. MPPI weights its 1000 sampled
trajectories by exp(-cost/temperature) and averages them, so at 0.3 the
good samples and the mediocre ones were being blended and the output was
pulled toward the middle of the batch rather than toward the best of it.
Peak commanded vx, over 45 s runs the length of the court:

    temperature 0.3    0.198, 0.190, 0.189 m/s
    temperature 0.05   0.310, 0.304 m/s

measured on two separate stacks. Raising vx_std does the opposite, since
wider sampling widens the average too: 0.2 -> 0.5 took the mean down from
0.170 to 0.135.

Validated on a 5-trial batch rather than on the velocity figure alone,
because a sharper weighting could plausibly have made the controller
twitchier and the speed is worth nothing if it costs the 10/10. Every
trial returned nav SUCCEEDED and 4/4 scorable picks passed. Against the
previous 10-trial baseline, excluding one trial lost to the unrelated
"grasp launch never exited" infrastructure fault:

    nav    68.3 -> 34.0 s   -50%
    fine   18.4 ->  8.2 s   -55%
    grasp  27.5 -> 20.5 s   -26%
    total 144.0 -> 93.8 s   -35%

fine halves as a consequence of nav, which now hands the base over closer
to the standoff pose than it used to.

There is a floor somewhere below this: 0.01 produced no motion at all.
0.10 also produced a dead run, but the stack was degrading by then and
that is not a clean measurement, so 0.05 is simply the only value under
0.3 that has been seen to work, twice. n is 4 scorable trials against a
baseline of 10, so the size of the win is better established than its
precision.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## e03da20  2026-08-24  Add a runbook and the operational scripts it describes

Everything in here was being carried in a session scratchpad under /tmp,
which is exactly as durable as it sounds -- the scripts that brought this
stack up and tore it down all session evaporated with the session, and
the launch-order and teardown traps they encoded live nowhere but in a
comment inside nav_grasp_trials.py.

scripts/ops/bringup.sh runs the one order that works, gazebo -> filters ->
localization -> seed -> navigation, and refuses to launch navigation if
the AMCL seed did not take. That refusal is the point: nav2's
global_costmap will not activate without map->base_link, which fails
planner_server, and the lifecycle manager then abandons the rest of the
bringup. Every half-active stack this project produced came from seeding
after launching navigation and racing the costmap's activation timeout.

scripts/ops/teardown.sh SIGKILLs, because SIGINT measurably leaves about
28 children alive, and it names ekf_node, joy_node, teleop_node and
opennav_docking explicitly because they are named after their packages
rather than after ros2/nav2/gz and an obvious pattern list misses all
four. The EKF is the one that costs you: it has survived several
supposedly clean restarts carrying a diverged state, feeding AMCL a
garbage motion model. It also collects numeric PIDs rather than using
pkill -f, which matches the shell running it, and it re-checks before
reporting a survivor so a process caught mid-exit does not send you
hunting for something already gone.

scripts/ops/seed_amcl.py stamps the pose with zero rather than the
current sim time. AMCL rejected timestamped seeds with "Lookup would
require extrapolation" whenever the base had just been teleported, since
the stamp beats the odom TF the teleport invalidated. It verifies the
transform appeared instead of assuming, because an unseeded AMCL fails
silently and a harness then runs a whole trial computing goals from a
stale pose.

The README records the rest: the 8-core load ceiling and the crash at
33.7, why bond_timeout cannot come from yaml, the DDS segment leak across
long runs, and the open issues -- chiefly that MPPI commands 0.175 m/s
while the drivetrain does 0.601 on request.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 3a8d436  2026-08-23  Add a court-clearing harness: one corner start, sixteen shuttles

Different task from nav_grasp_trials.py, which teleports the base back to
the origin and places one object per trial. Here the robot starts in the
bottom left corner and works a scattered field of sixteen without ever
being put back, so error accumulates across a whole sweep the way it
would on a real court. Order is nearest-first, which keeps the hops short.

Three things could not be reused as-is. The approach heading in the
single-object harness is atan2(aim.y, aim.x), the bearing from the world
ORIGIN, which is only correct because the base always starts there; here
it comes from the base's own pose. AMCL is seeded once per trial rather
than once per object, which is also the more honest test. And sixteen
models are scored individually, so a pick is only credited to the shuttle
the robot was actually sent to.

nav2 is bypassed below 2 m, provided the straight line is clear of
obstacles. It is built to cross a court, not to shuffle a metre sideways,
and the measurements were emphatic: over the first nine picks the two
hops that went to nav2, at 0.71 m and 1.46 m, both came back nav TIMEOUT
after burning the full 120 s, and those two picks alone took 428 s of the
1199 s the nine cost together. With the bypass, mean pick time fell from
133 s to 72 s and the 0.71 m and 0.93 m hops that had failed both passed.
Across a full trial the split was 13 picks at 72 s via fine_approach
against 2 at 206 s via nav2.

Also here because a sixteen-object trial spawns sixteen grasp launches
and a run spawns eighty, which the single-object harness never stressed:
orphaned DDS segments are swept between trials, the lifecycle managers
get the runtime bond_timeout override nav2's launch files make
impossible to set from yaml, and the corner seed is zero-stamped and
verified. Each of those was a run that died. The bond timeout declared a
healthy map_server dead mid-sweep and took map->base_link with it; the
timestamped seed was rejected with "Lookup would require extrapolation"
because the stamp beat the odom TF the teleport had just invalidated,
and the trial then ran with the objects metres from where the robot
believed they were.

Measured: 15/16, 15/16 and 12/16 over three complete trials.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 8d51c9f  2026-08-23  Stop the guard braking on its own chassis, and unthrottle MPPI

Two separate costs, found by measuring the velocity chain rather than
guessing at it. Commanded, guard-output and achieved velocity were logged
together: nav2 asked for 0.157 m/s, the guard passed it through unchanged
and the base delivered it exactly, while a direct 0.60 m/s command on the
same path was met to within 0.001 m/s. So the drivetrain and the guard
were both innocent of the slow driving and nav2 was the throttle.

The guard was guilty of something else. scan_self_filter clears the
chassis RECTANGLE, whose front face is 0.191 m out, while the guard tests
the 0.22 m circumscribed circle, so the sliver between the two survived
filtering and landed inside the collision radius. Every such return made
the first forward-simulation step collide, which read as "collision in
0.10s, stopping": 89 hard stops in one batch, and nav2 being commanded
0.188 m/s while the guard published zero for the opening seconds of a
run. Returns already inside the footprint at t=0 are now dropped. Nothing
can be a future collision if the robot is standing on it, and treating
them as one meant the robot could not even reverse out. 89 events -> 0.

On the MPPI side: visualize was publishing the whole candidate trajectory
bundle every cycle to nobody, prune_distance 1.7 was shorter than the
2.24 m the optimiser's own 2.8 s horizon covers at vx_max so it could
never see far enough ahead to commit to speed, PreferForwardCritic at 5.0
was losing to PathAlignCritic at 14.0 badly enough that the robot reversed
5 m to goals behind it at 0.103 m/s, and PathAlign was pinning it to the
path hard enough that any fast candidate looked worse than a slow one.

Measured over 10 trials against the previous 5, at targets 3.84..6.02 m:
10/10 with every trial nav SUCCEEDED, nav phase 93.0 -> 68.3 s (-27%),
total 169 -> 144 s (-15%), or -11% per metre once the slightly shorter
mean distance is accounted for. That was achieved at load 27-38 against
the earlier run's 21, so the margin is if anything understated.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## a4b7d61  2026-08-23  Spread the obstacles across the court instead of around the spawn

The five boxes sat within ~1.05 m of the origin, which suited
husarion_world where targets were 1-2 m away. On a 13.4 x 6.1 m court
with targets at 4-5 m they had become a cage around the robot's start,
and one pair formed a trap it could not escape.

The target at (+3.28,-1.87) failed in four consecutive batches. Logging
Gazebo ground truth against /amcl_pose through a live run showed why: the
straight line to it at -29.7 deg passes through the overhang box, which
spans z 0.30..0.60 while the lidar sits at 0.234 and passes underneath,
so the costmap saw clear floor. The robot drove in and wedged between two
boxes with 8 mm and 20 mm of clearance and sat there for over a minute.
It never had a chance: that corridor is 0.35 m wide and the robot is
0.44 m. AMCL was not at fault, which is where I first looked. Its error
stayed under 0.35 m until well after the robot was stuck, and the
1.0 m drift that followed is what a wedged base with turning wheels does
to odometry.

The new layout keeps the overhang, since a box the lidar cannot see is
exactly what the depth costmap source is for, but gives it room. Every
obstacle is now at least 1.87 m clear of the spawn and the narrowest
corridor anywhere is 1.08 m. Positions are spread over both halves rather
than tuned to block the seeded targets, which would only overfit one draw
of the RNG.

Measured after the change: 5/5, every trial nav SUCCEEDED with err ~0.25
and reach 0.104..0.180 m. Driving the old failing target directly now
crosses the trap zone with AMCL error under 0.30 m and covariance falling
0.42 -> 0.15, stopping 0.49 m short only because that target is 0.77 m
from an obstacle and its standoff pose lands in the inflation zone --
which is why OBST_CLEAR at 0.85 now rejects it.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 6797c83  2026-08-23  Vectorise the scan filter and the guard's collision test

Both nodes looped over ~2900 beams in pure Python on every scan. The
guard was the worse of the two: 12 forward-simulation steps x 2896 points
per /cmd_vel message at 20 Hz is roughly 700k interpreted distance checks
a second, on the machine that also runs Gazebo.

This does NOT save measurable CPU, and the honest reason is recorded here
so nobody re-does the experiment. The two nodes sit at ~30% of a core
each, but that cost is /clock, not the loops: a node that does literally
nothing measures 23.7% with use_sim_time true and 0.0% with it false,
because gz publishes /clock at ~659 Hz and rclpy runs a Python callback
on every tick. The filter's own arithmetic is 0.47 ms per scan at ~7 Hz,
which is half a percent of a core.

Keeping the change anyway: it is behaviour-preserving, and the array form
states the geometry more directly than the index bookkeeping did.
Verified equivalent against the original implementations over 3000
randomised collision cases and 300 randomised scans, with zero
mismatches.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 28d451c  2026-08-23  Stop marking the badminton net as an obstacle

The net panel spans z 0.760..1.524 across the whole court width at x=0,
and the robot starts on that line. The depth costmap's marking sources
ran a 0.10..1.00 height band, so the net's lower 24 cm sat inside it and
the eye-in-hand camera painted a lethal barrier along x=0 whenever the
arm happened to look that way.

Measured over two 5-trial batches with a fixed seed: every target at +x
timed out (4/4) while every target at -x succeeded (6/6) with err ~0.245.
After dropping the marking ceiling to 0.70, the +x target at (+4.75,-0.16)
went from nav TIMEOUT to nav SUCCEEDED with err 0.249.

The barrier was a phantom. The robot is 0.30 m tall and drives under the
net without touching it, so nothing above 0.70 can collide with the base
and marking it only invents obstacles. The clearing-only sources keep
their 1.00 ceiling, since a higher ceiling there clears more, which is
what we want.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 83270ac  2026-08-23  Scatter shuttlecocks across the court instead of a 1-2 m ring

Targets were drawn from an annulus 1..2 m around the origin, which suited the
old open world. On the badminton court it wastes the space: everything landed
within a couple of metres of the net, and the robot barely had to navigate.

Now sampled over a RECTANGLE covering the court, at least 2.5 m out. The shape
change is not cosmetic -- the court is long and narrow, 13.4 x 6.1 m, so a
radius large enough to be interesting along x would put the target through the
sideline and into the wall along y. An annulus cannot cover this shape.

    TARGET_X_ABS 5.6    court half-length 6.70 less margin
    TARGET_Y_ABS 2.3    court half-width  3.05 less margin
    TARGET_MIN_R 2.5

The margins have to cover the robot radius (0.22) plus the standoff pose it
parks at (0.217 short of the target) plus room to turn, or nav2 gets handed
goals sitting inside the wall's inflation and refuses them.

Sampled with the usual seed, targets now land 3.0..4.8 m out rather than
1.0..2.0. Obstacle clearance is unchanged and now essentially free: the
obstacles sit ~1 m from the origin and targets start at 2.5 m.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## a88b28e  2026-08-23  Add a badminton court world, and stop hardcoding the world name

The court is generated by scripts/make_badminton_world.py rather than
hand-written, because the world and the AMCL map have to agree exactly and are
built from one set of constants. A map that disagrees with the world is the
worst failure available here: AMCL converges confidently on a wrong pose and
everything downstream inherits it silently.

THE COURT IS INSIDE A HALL, and that is the load-bearing design decision. A
regulation court on its own is nearly invisible to this robot:

  * the lines are paint, with no vertical extent
  * the net spans ~0.76..1.55 m and the lidar sits at z=0.234, so the beam
    passes underneath it
  * only the two net posts are lidar-visible, and two thin posts across
    13.4 x 6.1 m is far too sparse for AMCL

A bare court would have broken navigation outright. The 18 x 10 m hall gives
AMCL structure in every direction, and real courts are indoors anyway.

Verified in the running world:
  * lidar returns 3000/3000 beams, max range 10.43 m, which matches the hall
    diagonal sqrt(9^2 + 5^2) = 10.30 m
  * AMCL localises to 25 mm / 0.2 deg, the same quality as the old world
  * a nav2 goal across the court SUCCEEDED, (1.05,0.44) -> (3.90,1.83)
    against a target of (4.0,2.0)

The map is synthesised from the wall constants rather than SLAMmed. The wall
positions are known exactly, so the grid is perfectly registered; a SLAM map
would bake in the drift of whatever drive produced it and that error would
become a permanent localisation offset.

Court is centred on the origin deliberately: nav_grasp_trials samples targets
1..2 m from the origin, so they land on the court near the net rather than in a
corner.

Separately, the world name is no longer hardcoded. Three scripts embedded
"husarion_world" in gz service paths, which would have silently broken every
spawn, removal and pose query the moment the world changed -- and gz reports
success for a bad world name, which is exactly the failure mode this harness
keeps getting caught by. repeatability_test.WORLD now asks gz which world is
running, overridable with GZ_WORLD, falling back to the old default so the
module still imports without a simulator. Confirmed auto-detecting
"badminton_court".

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## cd0c878  2026-08-22  Stop the lifecycle managers tearing down nav2 mid-run over a missed heartbeat

This is the cause of the half-active nav2 stacks that have disrupted every
session in this project, and it is not a nav2 defect.

Each lifecycle manager holds a bond heartbeat with every node it manages and
shuts the whole group down when one is missed. The default window is 4000 ms,
and on this machine under ordinary trial load that fires:

    Have not received a heartbeat from smoother_server.
    CRITICAL FAILURE: SERVER smoother_server IS DOWN after not receiving a
    heartbeat for 4000 ms. Shutting down related nodes.

The node really did miss its window. The machine was saturated: gz sim server
at 127% plus the GUI at 79% on 8 cores at load 18, with MPPI missing its own
control loop badly enough to log "Current loop rate is 7.57" against a 20 Hz
target. So the manager was working exactly as designed and killing a healthy
stack.

This explains behaviour I had previously misattributed twice. The recurring
"planner_server inactive while controller_server is active" is this, not the
AMCL ordering fixed in 5699816 -- that ordering bug is real and separate, and
both had to be fixed. It also explains why nodes I activated by hand were
inactive again minutes later, which made no sense under any theory of a
one-time startup race.

bond_timeout 0.0 disables the monitoring. The bond exists to notice a genuinely
crashed node and stop the rest driving blind, which matters on hardware; here
the simulator is the thing under test and a false positive costs an entire run.
Raise it to a few seconds instead if crash detection is wanted.

It has to be set at RUNTIME. A lifecycle_manager_* section in amcl.yaml does
nothing: nav2's launch files construct those nodes with
parameters=[{autostart}, {node_names}] and never pass configured_params.
Verified by writing it into the yaml and reading back the unchanged default of
4.0, then setting it with `ros2 param set` and reading back 0.0. The yaml now
carries that explanation instead of a config block that looks effective and is
not.

Also in this commit: MPPI batch_size 2000 -> 1000, halving its share of the
contended CPU. Not independently measured -- the control loop rate could not be
re-sampled before the stack was torn down again by the very bond timeout this
commit fixes.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 5699816  2026-08-22  Find the cause of the nav2 half-bringup: AMCL must be seeded first

Every session in this project has hit nav2 coming up half-active --
controller_server active while bt_navigator, planner_server, behavior_server
and velocity_smoother sat inactive or unreachable. I built an auto-recovery for
it (1514e99), treated it as load-related lifecycle timeouts, and it kept
recurring on idle machines. The auto-recovery treats the symptom.

The actual error was visible in the navigation launch output all along:

    [planner_server] Failed to activate global_costmap because transform...
    [lifecycle_manager] Failed to change state for node: planner_server
    [lifecycle_manager] Failed to bring up all requested nodes. Aborting.

global_costmap will not activate without map->base_link. That transform comes
from AMCL, and AMCL does not publish it until it receives an initial pose. So
launching navigation before seeding /initialpose races the costmap's activation
timeout, planner_server fails, and the lifecycle manager abandons everything
after it in the list.

The order that works:

    gazebo -> scan filter -> localization -> SEED /initialpose -> navigation

Seeding first brought all six nodes to active [3] on the first attempt, with no
manual recovery, where every previous run this session needed intervention. The
subsequent batch was 5/5.

The harness now refuses to start without map->base_link and says why, rather
than running and reporting every trial as an infrastructure fault. The
auto-recovery stays: it is still the right response to a genuine timeout, and
it is now a fallback rather than the primary mechanism.

Worth recording alongside this: the OTHER failure mode, where nodes are alive
but nothing can query them, is not this. That one is DDS -- stale
/dev/shm/fastrtps_* segments left by SIGKILLed processes eventually block port
allocation ("Failed init_port fastrtps_port7020: open_and_lock_file failed"),
and it presents as topics still flowing while `ros2 node list` returns zero.
Cleaning /dev/shm with nothing running fixes it; a reboot fixes it completely.
Tearing the stack down with SIGINT rather than SIGKILL is what prevents it.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## cc27cce  2026-08-22  Drop the fixed 3 s arm wait, and stop the nav2 recovery from crashing the run

Two independent fixes, both replacing a blocking wait with something that only
costs time when it is actually needed.

grasp_ball.py: the run started with node.settle(3.0), a fixed pause where the
arm sat still doing nothing. It guarded a real race -- MoveIt's
SimpleControllerManager builds its OWN action client, which can still be
unconnected when the first move goes out, aborting it with "Action client not
connected to action server" and killing about half of all runs at the first
search pose. Our own client connecting says nothing about MoveIt's and there is
no way to observe MoveIt's from here, so the original fix slept and hoped. That
had already been cut from 13 s to 3 s.

run() now retries the plan/execute up to 4 times with a 1 s backoff instead.
A healthy start pays nothing, the cost lands only on runs that hit the race,
and it also covers the case a fixed sleep never could: a manager taking longer
than 3 s.

nav_grasp_trials.py: the lifecycle auto-recovery added in 1514e99 crashed the
run it was supposed to save. `ros2 lifecycle set /planner_server activate` hung
and raised TimeoutExpired, which propagated out of _activate_nav2_nodes and
killed the batch. Only `lifecycle get` had been wrapped. A node wedged badly
enough that its change_state service never answers is precisely the condition
being recovered from, so it must not raise -- the state is re-read afterwards
and the node reported stuck if it did not come up.

NEITHER is exercised in a trial batch yet. The simulator's ROS /clock bridge
stopped publishing partway through this session -- gz itself stayed healthy,
real_time_factor 0.9985 with sim_time advancing and `gz model --list`
answering, but /clock and /tf went silent on the ROS side, which starves the
costmaps of TF and takes nav2 down with it. That is worth knowing on its own:
several "nav2 lifecycle stalls" chased today may have had this upstream cause
rather than being nav2 flakiness.

The preflight did its job through all of it -- it aborted with a clear message
instead of spending five trials measuring a dead stack.

Not committed: MPPI GoalCritic/PathFollowCritic threshold_to_consider 1.4 ->
0.5, reverted. The theory was that the robot never leaves goal-approach mode
because targets sit 1.0..2.0 m out and the old radius is 1.4 m. Measurement did
not support it: with vx_max already at 0.8, MPPI peaked at 0.156 m/s and
lowering GoalCritic alone moved that to 0.165 m/s. Lowering both together was
never measured, because the clock bridge died first. Reverting rather than
shipping an unmeasured change.

Also established while chasing that: raising vx_max was pointless. MPPI emitted
0.156 m/s and /cmd_vel_smoothed, /cmd_vel and the actual odometry all read the
same value, so nothing downstream was ever clipping and the ceiling was never
the constraint.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 9225fe2  2026-08-22  Recover the grasp axis from depth when the silhouette is too round

The wrist roll aligns the claw across the shuttlecock's axis, and that axis
came only from a PCA of the 2D silhouette, gated on sqrt(l2/l1) < 0.75 so a
near-circular outline could not contribute noise. A shuttlecock lying down
does not reliably clear that gate. Measured at four headings, its eccentricity
ran 0.53..0.78 -- straddling the threshold -- so the axis resolved on some
headings and not others.

The depth image does not have that problem, because the object is a cone.
Lying down, its visible top rises from the cork end (~26 mm) to the skirt end
(~65 mm) across about 85 mm, so the surface carries a pronounced tilt whose
uphill direction IS the cork-to-skirt axis. Fitting z = ax + by + c over the
blob's deprojected points and taking atan2(b, a) recovers it, and the gradient
gives the sense for free -- uphill is the fat end -- which the PCA had to infer
separately by comparing silhouette widths.

Validated against Gazebo ground truth before touching the detector:

    truth  +89.7   depth +105.0   slope 0.53   err 15.3 deg
    truth +124.4   depth +125.9   slope 0.42   err  1.5 deg
    truth -179.3   depth +172.2   slope 0.46   err  8.5 deg
    truth   -5.7   depth   +7.1   slope 0.46   err 12.9 deg   <- PCA gate failed here

It resolves at every heading, to within about 15 deg, and the measured slope
0.42..0.53 matches the 0.46 the geometry predicts. 15 deg on an 85 mm object
displaces its ends by ~11 mm, inside what the claw's wrap tolerates.

Wired as a FALLBACK, not a replacement. Where the silhouette really is
elongated the existing path is better and is left untouched; this only fills in
headings that previously produced no axis at all. MIN_SLOPE rejects a flat fit,
so a shuttlecock standing on its cork -- level top, meaningless gradient -- still
correctly yields no axis.

Verified end to end at the heading where the PCA gate fails: "long axis 83 deg"
then "wrist roll 80 deg", gripper closed at -0.0180, object lifted to z=0.060.
Previously that heading gave "upright (no usable axis)" and no alignment.

Frame counters are logged per detection ("[pca N/depth M frames]") so it is
visible which path resolved the axis rather than having to infer it.

Honest limit: across a 5-trial batch every trial reported "pca 10/depth 0",
meaning the silhouette succeeded on all ten frames each time and the fallback
was never needed. It is validated in isolation, not yet exercised in a batch.

A related misreading is worth recording. The earlier "aligns on about half of
trials" was wrong: those logs DID resolve an axis. "upright (no usable axis)"
appears for mid-sweep bearings before the re-aim step, and I read it as the
final verdict instead of an intermediate one.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## edee191  2026-08-22  Speed up the pick loop, and instrument it so "faster" is measurable

Added per-stage timing first, because there was none: every previous run
reported success rates with no idea where the time went. The first
instrumented run answered the question immediately, and it was not where the
tuning had been aimed.

    nav 35s   fine 7s   vis 9s   grasp 19s   =  85s
     41%       8%        11%       22%

Navigation alone outweighed fine approach, vision and grasp combined, and
varied 14..75 s over similar 1.5..2.0 m distances. At 0.8 m/s the driving is
about 3 s of that.

What changed and why:

  FINE_V   0.12 -> 0.25 m/s   the fine approach only ever covers nav2's
  MOVE_V   0.10 -> 0.22 m/s   residual, a near-constant ~0.25 m, so the
  FINE_W   0.5  -> 1.0 rad/s  speed was pure overhead
  MOVE_W   0.5  -> 1.0 rad/s

  VISION_SETTLE 4.0 -> 2.0 s, with the arm trajectory 3 -> 2 s. This was the
  largest single cost: every detection attempt pays one arm move plus one
  settle, for up to 5 search bearings and up to 3 rounds. The settle was
  already measured against accuracy -- 1.5 s gave 9 mm detection error, 4.0 s
  gave 5 mm -- against a VISION_TOL of 25 mm, so 4 s bought 4 mm nobody needed.
  The settle must stay above the trajectory duration or a mid-swing frame gets
  paired with a settled TF, which is why both moved together.

  nav2 vx_max 0.5 -> 0.8 and wz_max 1.9 -> 2.5, with velocity_smoother's
  max_velocity and max_accel raised to match. MPPI is clamped by the smoother
  downstream, so raising one alone does nothing.

Result: 5/5 picked up, mean 85 s. Load peaked at 27 with the GUI running, so
these wall-clock figures are an upper bound rather than a clean measurement.

A negative result is recorded in amcl.yaml rather than discarded: loosening
xy_goal_tolerance to 0.50 saved 4 s of a 85 s trial and cost a pick, because
nav2 then used the whole budget every time and handed the fine approach double
the residual. Reverted to 0.25, with the numbers in a comment so it is not
retried.

Not addressed, and not caused by any of this: the wrist roll aligns on roughly
half of trials. The axis gate needs sqrt(l2/l1) < 0.75 and a lying shuttlecock
measures 0.93, so it sits on the boundary and resolves or not depending on
viewing angle. grasp_ball.py owns that detection with its own settle and has
not been touched since 12f9e62; VISION_SETTLE affects only the harness's
servo stage. Picks still succeed because the CAD claw cages the object rather
than needing to pinch across its axis.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 1514e99  2026-08-22  Recover stalled nav2 nodes automatically, and fix the check that never fired

Two things, and the second is the reason the first was needed.

1. The lifecycle check added in ef5d097 did not work. It tested
   `"active" not in state`, and `ros2 lifecycle get` prints "inactive [2]" for
   a stalled node -- "active" is a substring of "inactive", so every stalled
   node read as healthy. Verified against a genuinely stalled stack: the
   harness printed nothing, ran anyway, and both goals came back REJECTED.
   It never fired because the run that followed ef5d097 had been manually
   activated beforehand, so it was never exercised. Replaced with _is_active(),
   which compares the first word, with a docstring naming the trap.

2. Detection now leads to recovery instead of an abort. This stall is routine
   on this machine and always recoverable, so aborting only moved the manual
   step onto the user. _activate_nav2_nodes() drives each node to active,
   configuring first if it is unconfigured, and re-reads the state rather than
   trusting the exit code of `ros2 lifecycle set`.

Reproduced and fixed end to end. Bringing the stack up left:

    bt_navigator      inactive [2]      controller_server  active [3]
    planner_server    inactive [2]      amcl               active [3]
    behavior_server   inactive [2]
    velocity_smoother inactive [2]

Before the fix: no message, both trials nav REJECTED, run measured nothing.
After: "nav2 nodes not active: ... -- activating", then both trials
nav SUCCEEDED.

Root cause in nav2 is still not fixed and is not ours: a node's change_state
response fails to arrive in time under load ("failed to send response to
/planner_server/change_state (timeout)"), the manager treats the transition as
failed and stops there. The manager also blocks in its wait loop hard enough
that it will not answer a `ros2 param list` while stuck. This commit makes the
harness survive that rather than curing it.

Unrelated and unresolved: the two trials in the verification run both failed to
grasp (one "Arm move failed", one gripper stalling at -0.0034). Ruled out the
dead-code removal in 12dc004 as a cause -- rendering the URDF before and after
that commit gives byte-identical collision and visual geometry. n=2 against
10/10 earlier today, so it may be noise, but it wants a longer run.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## ef5d097  2026-08-22  Check nav2 is ACTIVE, not merely advertising, before starting a run

1546c82 added a pre-flight that waits for /navigate_to_pose. It is not enough.
bt_navigator advertises the action server while still inactive and then refuses
every goal:

    [bt_navigator] Action server is inactive. Rejecting the goal.

So wait_for_server returns true, the run starts, and all ten trials come back
REJECTED. That is the second full run lost to nav2 being half up, after the
NO_SERVER run that motivated the original check. The failure mode is different
each time and the symptom is the same: a batch that measures nothing.

The lifecycle manager stalls partway under load and leaves a split state.
Observed on a freshly rebooted machine at load ~17:

    bt_navigator      inactive [2]      controller_server  active [3]
    behavior_server   inactive [2]      smoother_server    active [3]
    planner_server    inactive [2]
    velocity_smoother inactive [2]

The pre-flight now queries `ros2 lifecycle get` for every nav2 node the harness
depends on and names the ones that are not active, with the command to fix
them. `ros2 lifecycle set /<node> activate` recovered all four here without a
relaunch.

Verified after activating: 10/10 picked up, 0 indeterminate, every trial
nav SUCCEEDED, on /scan_filtered end to end with the clearance monitor
reporting a real number on all ten (+0.055 .. +0.320 m).

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## b2e9acc  2026-08-21  Record the AMCL A/B honestly: filtering the scan does not measurably help it

1890d1a pointed AMCL at /scan_filtered and said the case rested on mechanism
rather than numbers, because the machine was too loaded to collect a fair
baseline. The machine is idle now, so here is the fair comparison. Three runs
per arm, seeding at ground truth and driving a three-leg square
(amcl_drift_test.py):

    /scan_filtered   114, 59, 66 mm      yaw 1.0, 1.3, 0.0 deg
    /scan (raw)       66, 112, 58 mm     yaw 1.2, 1.1, 2.4 deg

Position drift is indistinguishable. Means are 80 mm and 79 mm and the ranges
overlap almost entirely. Yaw is nominally better filtered, 0.8 deg against
1.6 deg, but that rests on one 2.4 deg outlier out of three samples and is not
a result either.

So the honest finding is that AMCL does not care. That makes sense in
hindsight: the 104 self-returns appear in every scan at the same bearings, so
they are a constant, and a particle filter weighting hypotheses against a
likelihood field is largely unmoved by a bias that applies equally to every
particle. The costmap is different -- it accumulates marks into a grid, which
is why removing them there took lethal cells within 0.8 m from 187 to 0.

The setting stays on /scan_filtered. Feeding a localiser input that is known
false is still wrong, it costs nothing, and leaving AMCL on raw while both
costmaps read filtered is an inconsistency someone would trip over. But it is
now recorded as principle rather than dressed up as a measured improvement.

Correcting myself: the earlier note claimed the filtered arm looked better on
the strength of four filtered runs against ONE raw run. That is not a
comparison, and I should not have written it down as though it leaned either
way. With three raw samples the difference disappears.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 310d9ed  2026-08-21  Keep the AMCL drift test instead of leaving it in /tmp

Written to answer whether pointing AMCL at /scan_filtered helped, and left in
/tmp where the next reboot would have deleted it -- after I had told the user
the test was saved. It is the only repeatable way to compare localisation
configurations here, so it belongs in the repo.

Seeds AMCL at Gazebo truth, drives a three-leg square, reports position and
heading error against truth. The docstring records what the numbers so far
actually are, that four samples against one is not a comparison, and that a
stalled lifecycle manager surfaces as a TypeError on a None pose rather than
anything readable.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 1890d1a  2026-08-21  Point AMCL at the filtered scan too

The costmaps moved to /scan_filtered in b547b8f; AMCL was left on raw /scan
pending evidence. This finishes the job, but the evidence is weaker than I
would like and the honest summary is "no measured harm", not "measured win".

What was measured. A repeatable drift test: seed AMCL at ground truth, drive a
fixed three-leg square with turns (turns are where scan matching earns its
keep), then compare the AMCL pose against Gazebo truth.

    /scan_filtered   67, 70, 57, 46 mm     yaw 2.1, 0.3, 1.7, 0.8 deg
    /scan (raw)      75 mm                 yaw 0.7 deg

Four samples against one. The filtered runs are all at or below the single raw
sample, which is suggestive and nothing more -- AMCL is a particle filter and
one raw sample is not a baseline. I tried to collect three raw runs for a fair
comparison and could not: nav2's lifecycle manager stalled at "Configuring
map_server" and left amcl and map_server unconfigured, which is the same
load-timeout cascade seen earlier today with bt_navigator, on a machine sitting
at load 13-17. That is a property of the machine, not of this change.

So the case for the change rests on the mechanism rather than the numbers.
AMCL's likelihood_field model scores each beam by its distance to the nearest
occupied cell in the map. The 104 self-returns land at ~0.1 m in a place the
map says is empty, so every one of them is a beam that can never match, and
they are present in every scan at the same bearings. Removing input that is
known-false cannot degrade the estimate, and leaving AMCL on raw while both
costmaps read filtered is an inconsistency someone would eventually trip over.

Verified functional: AMCL comes up subscribed to /scan_filtered and completed
four drift runs on it. If the drift question matters later, rerun the same test
on an idle machine with three or more samples per arm.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## b547b8f  2026-08-21  Filter the lidar returns that land on the robot's own arm

The lidar sees the arm, and the costmap marked it as an obstacle. Measured by
capturing /scan from four different robot poses and keeping only beams that saw
something under 0.45 m from EVERY pose, so anything surviving is mounted on the
robot rather than in the world:

    104 beams, 0.090..0.198 m, spanning +-174..180 deg in the laser frame

Those angles read as "behind the robot", and I chased that for a while. They
are not. base_link -> laser is yaw 3.142, so the laser frame is rotated 180 deg
and its +-180 deg is DEAD AHEAD in base_link. The returns land at x
-0.035..+0.073, which is the arm mount at x=+0.065. The robot was blanking a
~12 deg wedge directly in front of itself, in the direction it drives.

manipulation.yaml raises the lidar 0.07 to clear "the arm's base column, which
spans z 0.133..0.193", and at z=0.234 it does clear the column. The parked arm
folds back over itself and reaches lidar height anyway.

The costmap scan source runs obstacle_min_range 0.0, so these were marked
lethal a few centimetres ahead of the robot and travelled with it. Effect on
the local costmap, robot in open space:

    lethal cells within 0.5 m   14  ->  0
    lethal cells within 0.8 m  187  ->  0

A footprint test rather than a minimum range, deliberately. A blanket
obstacle_min_range blinds the costmap in every direction, including toward the
real obstacles it is about to hit. Every self-return endpoint falls inside the
chassis rectangle, so testing the endpoint against that rectangle removes
exactly the self-hits, at any range and bearing, and does not go stale if the
arm or lidar moves again. Measured in open space: /scan carried 104 returns
under 0.45 m, /scan_filtered carried 0, keeping 2896 of 3000 beams.

cmd_vel_guard now reads /scan_filtered and its SELF_FILTER radius is gone. That
radius was the same problem solved crudely, and its own comment said "a proper
fix is a footprint polygon filter on /scan rather than a radius". It discarded
everything inside 0.30 m, which against a 0.22 m footprint left about 8 cm of
real margin in exactly the region where stopping matters most. Verified after
the change: 2896 points received, open-space passthrough travelled 0.777 m.

/scan_filtered publishes RELIABLE to match the gz bridge's /scan. Publishing
BEST_EFFORT first silently starved the guard, which logged "offering
incompatible QoS. No messages will be received from it" and then held no scan
at all. A RELIABLE publisher still satisfies nav2's SensorDataQoS readers.

AMCL still consumes raw /scan. It localises to 14 mm today, so this is left
alone rather than changed speculatively, but it is a candidate: 104 phantom
returns at 0.1 m cannot be helping the scan match.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 1546c82  2026-08-21  Abort the run if /navigate_to_pose is missing instead of burning ten trials

A full 10-trial run was spent with nav2's action server down. Every goal
returned NO_SERVER, every trial scored INDETERMINATE, and about fifteen
minutes of simulator time measured nothing. The per-trial handling was
correct -- an unreachable action server is infrastructure, not a robot
failure, and excluding it from the rate is the right call -- but it should not
take ten trials to discover the stack is not up.

The failure is quiet in a specific way that makes a naive readiness check
useless. nav2's lifecycle manager can leave bt_navigator and behavior_server
"unconfigured" when transitions time out under load, while planner_server and
controller_server report active. Observed exactly that:

    bt_navigator     unconfigured [1]
    behavior_server  unconfigured [1]
    planner_server   inactive [2]
    controller_server inactive [2]

and /navigate_to_pose absent from `ros2 action list` entirely. Waiting on
planner_server, which is what I did, passes while the server this harness
actually calls does not exist. The check now waits on that server.

Relaunching navigation once the machine was quieter brought all four to
active [3] and /navigate_to_pose appeared; the following run was 9/10 with 0
indeterminate, so the stack itself is fine under lighter load.

main() now returns 1 on abort and __main__ propagates it, so a scripted run
fails loudly rather than printing a clean-looking empty summary.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 1e6b550  2026-08-20  Replace the contact-sensor collision check with continuous clearance tracking

The contact monitor never worked. It reported "collision check UNAVAILABLE"
on every trial of the last run, which was honest but useless.

Root cause, established rather than guessed: a contact sensor attached to a
model spawned at RUNTIME is never instantiated, so its topic never appears in
`gz topic -l`. Verified three ways, all negative -- static and non-static
bodies, and with an explicit <topic> override -- in a world that does load
gz-sim-contact-system (husarion_world.sdf line 10). The sensor SDF is valid;
nothing consumes it. It has been removed from obstacle_sdf() rather than left
in looking functional.

/world/<world>/dynamic_pose/info does work: it streams gz.msgs.Pose_V for every
moving entity, `rosbot` among them, for as long as the simulator runs. The
obstacles are static and absent from it, but their poses are known because this
harness placed them. ClearanceMonitor samples that stream for the whole trial
and reports the MINIMUM footprint-to-box gap.

That is strictly more informative than the contact boolean it replaces: it
separates a clean run from a near miss, and a negative value is a real overlap.
It also costs one subprocess instead of one per obstacle.

Demonstrated by driving straight through obstacle_0 with no navigation running:

    parsed pose samples          679      (the contact monitor produced 0)
    minimum clearance         -0.0700 m   overlap, correctly caught
    clearance at the final pose +0.6472 m looks perfectly clean

The geometry confirms the number exactly: the obstacle centre is 0.30 m off the
driven line, less 0.15 m of half-box and 0.22 m of robot radius, is -0.07 m. A
check that only samples the parked pose would have reported +0.647 and called
that run spotless, which is the whole point of sampling continuously.

Three-state reporting is kept deliberately. If the stream yields no parsable
poses, result() returns None and the trial prints "n/a", never a comfortable
number. This harness has now had two collision checks that could not fail --
comparing static obstacle poses before and after, then a sensor that never
existed -- and both read as clean runs. Silence must not look like success.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 12f9e62  2026-08-20  Correct the record on UPRIGHT_TOP_Z: it was never broken

Commit 0592a52 recorded this as a known bug, on the reasoning that
UPRIGHT_TOP_Z (0.075) is compared against ball[2], and that ball[2] is a
whole-blob centroid rather than the top-surface height the threshold was
calibrated for -- so a centroid could never reach 0.075 and upright objects
would always misclassify as lying. That reasoning was wrong.

Measured at the search pose, 8+ frames per sample, three target distances each:

    upright   0.0851 .. 0.0852     (standing height is 0.085)
    lying     0.0412 .. 0.0436

0.075 clears upright by 10 mm and lying by 32 mm, in the right direction both
times. The classification works.

The error was in what "centroid" means here. The blob is the object's VISIBLE
surface, and from the bird's-eye search pose the visible surface is its top, so
the deprojected centroid lands on the top face rather than the volumetric
centre. Measurement and threshold were in the same units all along, and the
top_z name was accurate.

The observation that started the false alarm was a lying shuttlecock reading
z=0.024. That was taken while the skirt mesh was silently failing to load, so
only the cork rendered and 0.024 is the cork top. It was evidence of the
missing mesh, which 0.592a52 itself fixed, not of a threshold fault.

The old comment also claimed lying reads 0.052..0.065. Measured, it is
0.0412..0.0436, so the real separation is wider than advertised rather than
narrower. Comment now carries the measured numbers, the reason the centroid is
a top-surface measure, and a note that a ~0.024 reading means a missing skirt
mesh -- so the next person does not re-derive the same wrong conclusion.

No behaviour change: the threshold and the comparison are untouched.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 414edd1  2026-08-20  Stop fine_approach commanding a strafe the base cannot perform

fine_approach corrected x and y at once by commanding linear.y, on the
reasoning that a mecanum base is holonomic. Measured against Gazebo ground
truth with /cmd_vel confirmed silent at idle, that lateral term is nearly
inert:

    commanded +x 0.15 m/s for 6 s  ->  travelled 0.839 m   (93%)
    commanded +y 0.15 m/s for 6 s  ->  travelled 0.024 m   (3%)

Nothing is misconfigured, which is why this took a while to pin down. The
controller emits textbook strafe wheel velocities (FL -3.061, FR +3.061,
RL +3.061, RR -3.061 rad/s). The URDF takes the mecanum branch and the
converted SDF keeps all four fdir1 roller diagonals with mu 0.8 / mu2 0.2.
But fdir1 and mu2 are ODE surface parameters and gz-sim runs DART by default,
whose contact model does not apply them, so friction comes out isotropic, the
four wheels' lateral components cancel, and the base slips instead of
strafing. A real ROSbot XL has physical rollers and would strafe; this is a
simulation limit, not a robot defect.

So the lateral command was never correcting anything and convergence came from
the forward and rotational terms anyway. fine_approach now turns to face the
goal, drives to it, and settles the final heading, using only the two channels
that work -- the same reason move_polar next door already turns and drives.
It backs up rather than spinning 180 degrees when the goal is behind.

Verified on the hardest case for a non-strafing controller, a residual 0.245 m
away and 78 deg off the nose, which is the lateral geometry the old code
claimed to handle: converged to dist_err 0.013 m and yaw_err -1.8 deg in 7 s,
against FINE_TOL 0.030 m and FINE_YAW_TOL 3 deg.

Two earlier claims of mine were wrong and are corrected here. The lateral
channel is NOT inverted -- +y moves left and -y moves right, correctly; only
the magnitude is wrong. And the mecanum friction is NOT lost in URDF-to-SDF
conversion; I had grepped for friction_direction1 when the element is fdir1.

vy_max in the MPPI block is documented as inert rather than removed:
motion_model is DiffDrive, so only (vx, wz) are ever sampled. Verified by
driving to a goal 0.3 m ahead and 1.2 m to the side, which produced
max |vy| = 0.000 over 250 /cmd_vel_nav samples. Switching motion_model to Omni
would make it live -- and would work on hardware while failing in simulation,
which is worth knowing before anyone tries it.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## a51b50d  2026-08-20  Give the depth cloud a clearing-only twin so elevated marks can be cleared

Fixes the permanent-phantom bug from 5b5a722. min_obstacle_height filters the
observation BUFFER, not merely what gets marked, so once an elevated obstacle
was removed the only thing the camera returned in its direction was floor at
z=0.000, the 0.10 m cut discarded it, and there were no rays left to raytrace
along. The mark could never be cleared. This adds a second source on the same
topic with marking: False and the floor cut opened to -0.05, purely so there is
something to clear along; the floor itself can never become an obstacle.

The reason this was reverted once before is that a single confounded test
suggested it stopped obstacles marking at all, which would be a collision risk
and strictly worse than a phantom. That test is now known to be worthless: the
depth cloud was running at 1.2 Hz at the time. With the cloud restored to
15.4 Hz by 7f63fe8, the measurements are repeatable and the fear is disproved.

Measured, headless, RTF 0.96, cloud 15.4 Hz:
  * Box floating at z 0.30..0.60 at 1.2 m, lidar confirmed blind to it (4.05 m
    straight through). MARKED 100 / 22 lethal within 15 s and STAYED marked
    across 45 s. So a floor return passing under it does not erase a mark for
    something physically present, which was the specific 2D-layer worry.
  * Removed: CLEARED within 15 s, stayed clear across 60 s.
  * Ground pillar z 0..0.60 same spot: marked correctly. World walls keep
    re-marking. The costmap is not blinded.

KNOWN LIMIT, measured not assumed. That ground pillar did NOT clear by raytrace
after removal: still 100 / 22 at t+90 s, with 2244 clearing-band returns passing
beyond the cell, zero returns marking it, and the lidar reading 4.05 m through
it. An explicit clear_entirely_global_costmap removed it and it stayed removed,
and the walls re-marked afterwards, so nothing was re-marking it. Raytrace
clearing is therefore not reliable for every geometry here; the elevated case
this fix targets does work. Why the two cases differ is NOT explained, and I am
not going to guess. In practice nav2's behaviour tree calls the clear-costmap
recovery, so a stale mark is not permanent.

Twice during this session I called a false regression by probing the global
costmap too early -- it publishes at 0.5 Hz and a mark can take past 45 s to
appear. Anything testing this should wait minutes, not seconds.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## fcfb8da  2026-08-20  Stop the harnesses scoring infrastructure faults as robot failures

Yesterday's 10-trial navigate-then-grasp run reported 8/10 for a robot that
had gone 10/10: `gz model -p` timed out twice under load, model_pose() returned
None for that, and None read as "the object is not there", which scored as a
failed grasp. One trial printed "[shuttlecock missing]" and another a final z
of 0.000, both while the object was in the gripper at z=0.060 with a clean
detection in its own grasp log.

Every gz call now goes through one choke point that treats a reply as
worthless and reads the world back instead:

  * model_pose retries with backoff and raises GzQueryFailed when Gazebo never
    answered. Absence is only ever concluded from Gazebo's own "No model named
    <x> was found"; silence is never absence. gz also exits 255 with its error
    text on stdout when unsourced, so the return code is checked too.
  * a third verdict, INDETERMINATE, covers failed queries, a launch that never
    exited, and a grasp log with none of the node's own markers in it (package
    not built, controllers not up). These leave the numerator and the
    denominator, so an infrastructure fault cannot move the success rate in
    either direction. The scoreboard always prints the excluded count.
  * remove and create are verified against `gz model --list` and raise on a
    mismatch. Both traps are documented at the call site: `type: 1` is LIGHT,
    not MODEL, and no-ops while looking like a success; a create naming a
    missing sdf_filename or mesh reports success and spawns nothing, which is
    how ten minutes of readings were once taken of an empty scene. The
    absolute-mesh-URI rewrite in ensure_shuttlecock is unchanged, and the
    files it points at are now checked for existence before spawning.
  * set_pose verifies by reading the pose back, loosely enough for the
    slide-and-settle but tight enough to catch a request that did nothing.
  * run_grasp kills the whole launch process group on timeout instead of
    raising TimeoutExpired out of the trial loop and discarding the run.

nav_grasp_trials and tipped_trials re-verify the world every trial rather than
once per run: losing the object mid-run used to turn every remaining trial
into a scored failure.

The obstacle collision check is replaced rather than kept. The obstacles are
<static>true</static>, so "their poses did not change, therefore no collision"
could not ever have failed -- a static body cannot be pushed. They now carry
contact sensors (husarion_world.sdf already loads gz-sim-contact-system) whose
topics are streamed for the duration of each trial, plus a ground-truth
plan-view clearance at the parked pose as a cross-check. If the contact topics
are not advertised the check reports UNAVAILABLE, never a clean run.

Also: jaw_geometry checks `ros2 pkg prefix` and exits non-zero on FAIL instead
of printing it and exiting 0; mesh_to_collision validates the binary-STL
triangle count against the file length and refuses to emit an empty collision
set. cmd_vel_guard.py and cmd_vel_relay.py are untouched.

Tested without a simulator: 58 isolated tests over the gz parsers, the
retry/absence/timeout split, spawn and remove verification, classify(),
run_grasp status detection, the contact monitor's three-state result, the
clearance geometry, and full dry runs of all three harness main() loops
against a fake world. Nothing here has been run against Gazebo yet.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 5b5a722  2026-08-19  Feed the ZED depth cloud into the global costmap as well

Previously the cloud went only to the local costmap, which left a real hole:
an obstacle floating at z 0.30..0.60 read 99 / 5 lethal cells locally but was
effectively absent from the global costmap, so the global planner routed
straight through it and only reactive local avoidance caught it. With the
cloud on the global obstacle_layer that same overhang reads 100 / 17.

Ranges mirror the local source, including obstacle_min_range 0.55 as the
self-filter -- the ZED rides on link5 and the claw otherwise marks itself a
permanent obstacle 0.3 m ahead, which inflation_radius 0.70 would smear into
a ~1 m bubble. That is worse here than locally, because the global costmap
never rolls away from a bad mark.

Two caveats found while verifying, both worth knowing before relying on this.

1. Coverage depends entirely on where the arm is parked. The camera is
   eye-in-hand, and after a grasp the arm left it pitched 1.371 rad -- 78.6
   deg, nearly straight down at the floor -- and the overhang registered
   nothing at all. Parked forward (pitch 0.200) it marked immediately. Two
   earlier explanations for the same observation, the 0.55 m self-filter and
   plain distance, were both wrong; it was arm pose. Anything relying on the
   global depth layer must park the arm forward first, which the trial
   harness does.

2. Marks were still present 65 s after the obstacle was deleted, with the
   camera streaming 6653 points through that corridor out to 3.43 m. A
   lidar-marked pillar behaved the same way, so this is not specific to the
   depth source -- but the lidar control was confounded by a return at 0.76 m
   occluding the cell under test, so the cause is NOT established. Treat
   global costmap clearing as unverified. A phantom obstacle here is more
   damaging than the blind spot this fixes, since it can make a goal
   permanently unplannable.

Trials after the change: 8/10. Both failures were ground-truth read flakes
under load, not robot failures -- gz model -p returned nothing, so one trial
scored "shuttlecock missing" and another read the final z as 0.000, while the
object was in fact present and lifted to z=0.060 and the grasp log showed a
clean detection. The harness should re-check the object each trial and retry
model_pose rather than scoring a timeout as a miss.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## f9cc351  2026-08-19  Add obstacles to the navigate-then-grasp trials

The 10/10 result was on an empty world, so the depth costmap never had to
avoid anything and nav2 never had to plan around anything. Five obstacles now
stand in the annulus targets are drawn from. They are deliberately absent
from husarion_world.yaml, so nav2 discovers them from live sensors rather
than from known map geometry.

Four are pillars spanning z 0..0.60, which the lidar sees. The fifth floats
at z 0.30..0.60 and the lidar plane at z=0.07 cannot see it at all; it exists
to exercise the ZED depth layer.

Verified where each obstacle actually lands, and the split is exactly what
the costmap configuration implies:
  global costmap (scan only)     pillars max_cost 100, 23..29 lethal cells
                                 overhang  effectively absent (1 cell)
  local costmap (scan + depth)   overhang max_cost 99, 5 lethal cells
So the global planner will route straight through the overhang and only the
local costmap catches it, reactively. That is a real limitation of putting
the cloud on the local costmap alone -- a deliberate choice, since the camera
swings with the arm and stale marks would otherwise persist in the map.

Targets keep OBST_CLEAR = 0.85 m from any obstacle. inflation_radius is
0.70 m, so a target any closer would have its standoff pose swallowed by
inflated cost and the goal would be unplannable -- that would measure nav2's
handling of impossible goals, not obstacle avoidance. 29% of uniformly drawn
spots survive the filter; the sampler resamples up to 200 times.

Result: 10/10 with obstacles, reach 0.077..0.169 m, vision converging to
0.005..0.021 m. nav2 did have to work for it -- the navigation log shows
"Failed to make progress" and spin recoveries -- though that log spans the
empty-world run too, so the counts are not cleanly attributable.

Note the obstacles are <static>true</static>, so their poses being unchanged
afterwards is NOT evidence of no contact; a static body cannot be pushed.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## b1eb0ec  2026-08-19  Add navigate-then-grasp, with a vision correction that makes it 10/10

The two halves of this robot had only ever been tested apart: every grasp
harness teleported the shuttlecock into the arm's reach band and never moved
the base. This drives to a target 1-2 m away, then picks it up.

Three stages, and the third is the one that matters. nav2's
general_goal_checker runs xy_goal_tolerance 0.25 m while the arm's whole
reach band is 0.050..0.218 m, so nav2 reports SUCCEEDED with the object out
of reach -- measured, it stopped 0.245..0.250 m from the goal on 8 of 10
trials. An AMCL-based fine approach closes that to ~0.026 m, but on its own
still gave only 6/10: it reported converging every time while four runs left
the object 0.24..0.41 m from the arm, the error lateral, target at ~72 deg
bearing. Vision and Gazebo ground truth agreed to within 7 mm while the fine
approach disagreed, so the map-frame pose it converged to was itself wrong.

The vision stage never consults map. It sweeps joint1 (a single forward look
misses a target at 72 deg), detects the object with the same HSV + median
depth pipeline as grasp_ball, and moves the base by an odom-tracked relative
displacement.

It turns and drives rather than strafing, which is not a stylistic choice.
Measured on this base with /cmd_vel otherwise silent:
  commanded +x 0.25 m  ->  actual +0.259 m        (good)
  rotation +/-0.4 rad/s ->  correct both ways     (good)
  commanded +y 0.25 m  ->  actual -0.109 m        (wrong way, wrong size)
The mecanum lateral channel cannot be trusted for open-loop corrections, so
move_polar uses only the two that behave. Rotation is about base_link's
origin, so the heading is atan2(py, px), not measured from the arm mount --
the arm sits on that same axis at (ARM_X, 0).

VISION_SETTLE is 4.0 s because the arm trajectory is 3 s and the camera rides
on link5: a shorter wait pairs a mid-swing image with a settled TF lookup.
Measured 1.5 s -> 9 mm detection error, 4.0 s -> 5 mm.

Detection constants are regex-read from grasp_ball.py rather than copied,
following repeatability_test._arm_x -- a duplicate went stale once already
and every trial then scored the robot down for a harness bug. Importing the
module directly would drag MoveItPy into the harness process.

Same seed, so identical placements:
  before  6/10, reach 0.14..0.41 m, four outside the band
  after  10/10, reach 0.116..0.171 m, all inside,
                vision converging to 0.008..0.022 m in 2-3 iterations

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 0592a52  2026-08-18  Fix the pick: spawn the shuttlecock with its skirt mesh actually loaded

Grasps were failing 1/3 and worse, with symptoms that looked like four
unrelated perception bugs. They had one cause: model.sdf refers to the
skirt as model://shuttlecock/meshes/skirt.stl, which Gazebo resolves by
searching GZ_SIM_RESOURCE_PATH. Nothing in this workspace puts
grab_sequence/models on that path, so spawning the model by absolute
sdf_filename loaded the cork cylinder and silently dropped the skirt --
no warning, no error, just a shuttlecock with no shuttlecock on it.

Measured with the skirt missing vs present:
  * yellow blob area            169 px  ->  1018 px
  * PCA eigenvalues       [56.3, 56.3]  ->  [175.5, 150.6]
  * long axis            "no usable axis" -> resolves, wrist roll runs
  * detection vs skirt_centre  41.9 mm  ->  4.9 mm

A bare cork is a circle, so its PCA eigenvalues come out exactly equal and
the axis -- and therefore the whole wrist-roll alignment -- never resolves.
Its centroid sits on the cork, ~42 mm from the skirt centre the harness
scores against, which is precisely the det_err that was being reported. It
also loses the skirt collision, so it rolls like a bare cylinder and leaves
the reachable band while settling, which is why some trials reported the
object "seen but not reachable" at radii it was never placed at.

ensure_shuttlecock() rewrites the mesh URI to an absolute file:// path and
respawns before each run, so the harness no longer depends on how Gazebo
happened to be launched. models/ is now installed to share/ as well, so
model:// resolves for anyone who does put it on the path.

An env hook was tried first and removed: ament_python has no automatic
environment-hook registration (that is ament_cmake's
ament_environment_hooks), so the .dsv installed but local_setup.dsv stayed
empty and the variable was never set.

Verified: 4/4 tipped picks, det_err 6/8/8 mm on three of four, wrist roll
firing at 34/-75/-106 deg, on a single clean sim at RTF 0.86.

Known and unfixed: UPRIGHT_TOP_Z = 0.075 is compared against ball[2], which
is the blob centroid (xs.mean/ys.mean with median depth), not the top
surface it was calibrated for -- the code even says "previously used only
pixels within 15 mm of the nearest depth". A centroid never reaches 0.075,
so upright objects classify as lying. Trial 2 here hit exactly that.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## cca76e5  2026-08-18  Feed the ZED depth cloud into the local costmap as a 3D obstacle source

The 2D lidar sits at z=0.07 and sees a single plane, so it misses table
edges, overhangs, and anything not crossing that height. Measured: with a
box floating at z 0.30..0.60 directly ahead, the lidar's nearest forward
return was 4.06 m -- it looks straight past it. The depth camera now marks
that box at cost 100 / 88 lethal cells.

The cloud was unusable as delivered: 1280x720 produced 921600 points at
0.33 Hz against a requested 30, i.e. ~1% of the requested rate, because
two full-res cameras are more than the renderer can sustain. Halving both
ZED cameras to 640x360 (stereolabs_zed.urdf.xacro, a vendored repo) took
it to 230400 points at 24.4 Hz -- a 4x pixel cut for a 70x rate gain,
confirming the renderer was the bottleneck rather than transport.

Both cameras had to move together: _image_cb finds the contour in the RGB
image, indexes the depth image at those same (u,v), and deprojects with
RGB intrinsics, so unequal resolutions would break that correspondence.
Two pixel-dependent constants rescale with it -- MIN_CONTOUR_AREA is an
area (80 -> 20) and the half_px gate is linear (2.0 -> 1.0).

obstacle_min_range 0.55 is a self-filter. The ZED is on link5 and the claw
hangs in its own field of view: 9470 points at 0.2..0.3 m from base_link
inside the marking band, with clean air out to the walls at 2.1 m. Left at
0.30 the claw marked itself as a permanent obstacle 0.3 m ahead, which
inflation_radius 0.70 expanded into a ~1 m bubble no plan could cross.

Local costmap only, deliberately. The camera swings with the arm, so marks
made at one arm pose are wrong at the next; the 3 m rolling window bounds
how long a stale mark survives. Stale marks are real: clearing only happens
along rays that cross a voxel, and with low walls the rays sweep only the
bottom of a column while mark_threshold 0 keeps the column lethal from one
surviving voxel. Verified the window does clear them -- drove to x=-1.77
and back, after which the region read max_cost=0.

Resolution was checked against the grasp pipeline and is not implicated:
re-running the trials at 1280x720 reproduced trial 1 exactly (same
placement, det_err 43mm, top_z=0.024, no usable axis, same failure). The
low top_z and missing PCA axis predate this change.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## a620a93  2026-08-18  Add cmd_vel_guard, a collision-checking replacement for collision_monitor

nav2's collision_monitor silently forwards nothing in this setup: it reports
action_type 0 (DO_NOTHING) and QoS matches end to end, yet /cmd_vel stays
empty while /cmd_vel_smoothed carries real commands. cmd_vel_relay.py proved
the chain was otherwise sound by bypassing it, but that removed all collision
protection. This restores the protection without the bug.

Two differences from collision_monitor target its observed failure modes:
TF is read at latest-available time rather than the message stamp, which
avoids the 4 ms extrapolation race that forced a hard stop every cycle; and
blocking publishes an explicit zero Twist instead of going silent, so the
base stops immediately rather than coasting to cmd_vel_timeout.

SELF_FILTER drops returns inside 0.30 m. Measured against a clear floor there
are zero returns inside 0.22 m but 102 between 0.22 and 0.30 m -- the chassis
corners (hypot(0.167,0.135)=0.215) and the arm base at x=+0.065. Without it
the forward simulation put those self-hits inside the footprint and refused
every command.

Verified in sim:
  * open space, 0.2 m/s commanded, robot traveled 0.72 m (passthrough)
  * 0.2 m/s for 20 s straight at a box face at x=0.90 -- enough to drive 4 m
    and through it -- stopped with the leading edge 30 mm short of contact
  * /navigate_to_pose returned SUCCEEDED, the first completed nav2 goal here

Known limits: collision_monitor is still ACTIVE and publishing to /cmd_vel,
harmless only because it forwards nothing; and the guard settles at a nonzero
scale at standoff, so whether it holds or creeps over a long run is untested.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 0a37cc6  2026-08-18  Add cmd_vel relay bypassing broken collision_monitor

collision_monitor forwards zero messages to /cmd_vel regardless of input.
Traced live during a nav2 goal: /cmd_vel_nav and /cmd_vel_smoothed both carry
real ~0.18 m/s commands with matching QoS end to end, and
/collision_monitor_state reports action_type: 0 (DO_NOTHING, i.e. it believes
nothing is wrong) throughout, yet /cmd_vel stays silent the whole time.

With collision_monitor deactivated and this relay running in its place
(cmd_vel_smoothed -> cmd_vel directly), the same goal drove the robot from
(0,0) to (1.80,-0.06) against a (2.0,0.0) target in one continuous run.

This is a workaround, not a fix -- the safety-stop layer is off while it runs.
Not yet root-caused inside collision_monitor itself.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## ba0e8b6  2026-08-18  Fix velocity_smoother zero-accel bug and collision_monitor transform timing

velocity_smoother's Y-axis max_accel/max_velocity were 0.0, a leftover from
the stock differential-drive config. Confirmed this was zeroing the entire
output vector including X: before the fix, cmd_vel_smoothed was 0.0 on every
message despite real ~0.17 m/s input from MPPI; after, it showed a proper
accel ramp.

collision_monitor's transform_tolerance (0.2) and source_timeout (1.0) were
tight enough to trip on a ~4ms timing race, forcing a hard stop every cycle
(Lookup would require extrapolation into the future). Raised both to 2.0.
Not yet verified end-to-end -- DDS shared-memory exhaustion in the environment
blocked further testing before this could be confirmed.

---
## da906b8  2026-08-17  Add SLAM and AMCL configs for husarion_world

Both start from the upstream defaults, changed only where this robot differs.

slam.yaml sets base_frame to base_link. slam_toolbox defaults to
base_footprint, which the ROSbot XL does not publish, so it warns "Failed to
compute odom pose" on every scan and never builds a map.

amcl.yaml is a copy of nav2_params.yaml with three changes:
  * base_frame_id to base_link, same reason as above. AMCL otherwise reports
    "Couldn't transform from laser to base_footprint" and never publishes
    map->odom, so the map frame simply does not exist.
  * robot_model_type to OmniMotionModel, since the base is mecanum and can
    strafe. The differential default has no model for sideways motion and
    degrades whenever the robot moves that way.
  * enable_stamped_cmd_vel on all five velocity publishers. The drive
    controllers run use_stamped_vel, so /cmd_vel is TwistStamped while nav2
    publishes plain Twist by default. The mismatch is silent: every node comes
    up active, plans compute, and the base never moves. It has to be set on all
    of them or bringup aborts instead with a cmd_vel_nav type collision.

Not yet addressed: bringing up the navigation stack needs each node activated
by hand on this machine, because lifecycle_manager's service calls time out and
it treats that as failure. Every node transitions fine when asked directly.
Raising bond_timeout and service_timeout on lifecycle_manager_navigation is the
likely fix, untested.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 9b0f951  2026-08-16  Aim the claw's pocket correctly, and fix a startup race that lost half the runs

Lying-shuttlecock pickups go from 0 to 5 of 6.

The grip reference was wrong. GRASP_OFFSET pointed at the midpoint of all the
claw's collision geometry, which is not a grip surface at all. Measuring the
mesh shows the pocket runs the claw's whole length, void spanning 2..78 mm below
the finger mount with no dead material under it, so aiming the midpoint left
38 mm of claw beneath the grip point and drove the tip through the floor on
anything lower than 38 mm. A lying shuttlecock needs about 21 mm. Aiming the
lower part of the pocket instead leaves 10 mm and reaches it. MIN_GRASP_Z now
refuses such grasps outright rather than silently jamming into the ground,
which previously just looked like the jaws closing on air.

Separately, cutting the fixed 13 s startup sleep earlier introduced a race:
waiting for move_group to appear says nothing about whether MoveIt's controller
action clients have connected, so roughly half of all runs fired their first
move into a dead client, lost every search pose and died in 6 s. Now waits for
the action server, then lets MoveIt's own client catch up. Startup is still
about 4 s rather than the original 13.

Supporting tools:
  jaw_geometry.py    reports the real jaw opening from the rendered URDF in
                     0.4 s, so claw geometry can be checked without launching
                     anything. Rotated collision pieces do not have the extents
                     their nominal thickness suggests, and eyeballing them in
                     RViz shows the visual mesh rather than what physics uses.
  mesh_to_collision.py  voxelises a claw STL into axis-aligned collision boxes,
                     preserving the concave pocket that a convex hull would fill.
  tipped_trials.py   runs picks with the shuttlecock lying on its side, the
                     harder case: only ~43 mm tall, rolls when touched, and
                     needs the wrist roll aligned across its axis.

repeatability_test.py now reads ARM_X from grasp_ball.py rather than keeping its
own copy, which had gone stale when the arm moved to the chassis front and was
placing every target behind the robot, scoring the robot down for a harness bug.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 470223d  2026-08-14  Improve detection accuracy and cut pick time from 43s to 17s

Detection changes, all driven by measurement against Gazebo ground truth:

* Orientation now comes from the measured top-surface height, not blob
  eccentricity. Standing the shuttlecock reads 0.085 and on its side 0.052 to
  0.065, which never overlap; eccentricity disagreed with the height on 5 of 10
  detections, in both directions, and since that branch picks the grasp height
  it was putting the jaws tens of mm off.
* The target estimate uses the whole blob with the median depth across it. The
  previous nearest-depth patch drops under 20 pixels at ordinary poses, and
  every frame was then discarded, so the object was simply never seen.
* joint1 is re-aimed at the object and the measurement repeated before
  committing. The sweep accepts whichever bearing first sees the target, which
  is rarely the one pointing at it: measured 17.8 mm error at bearing 170 for an
  object at 135, and 5.6 mm once turned to face it. Mean error over 12 randomised
  trials fell from 16.8 mm to 9.5 mm.

Speed, from 43 s to 17 s for a full pick. Most of it was slack that earlier
fixes had already made unnecessary: wait for move_group instead of sleeping a
fixed 13 s; settles cut from 3.0/2.5 s to 1.0 s, since they existed for a joint2
droop that the 4.1 Nm effort change removed; 45 depth samples down to 10,
because the measured frame-to-frame spread is 0.0 mm and averaging bought
nothing; per-move delays trimmed across the nine moves in a pick; and the
gripper now returns as soon as the jaws stall instead of always waiting 3 s.

ARM_X and the search bearings follow the arm to the chassis front. The bearings
face forward and stop at +-70 deg, beyond which the grasp point falls under the
chassis.

scripts/repeatability_test.py runs randomised trials and reports a success rate
with failure modes broken out. Every "it works" here had rested on a single run,
and several of those turned out to be confounded.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## cd5380a  2026-08-12  Vision-guided shuttlecock pick with the ROSbot XL arm

grasp_ball detects a yellow shuttlecock with OpenCV and picks it up off the
floor with MoveIt, on the simulated ROSbot XL.

The ZED is mounted on link5, so it is an eye-in-hand camera and the arm's own
pose decides what it can see. The node uses that: it sweeps joint1 through a
set of bird's-eye search poses where the camera looks down at the floor, and
the patch it sees is the same patch the arm can reach, so no base motion is
needed.

Notes on the parts that are easy to get wrong:

* Arm position uses a closed-form 4-DOF solution rather than a MoveIt pose
  goal. With only 4 joints a full 6-DOF pose is over-constrained and OMPL's
  goal sampler just fails with "Unable to sample any valid states".

* GRASP_OFFSET has two terms. The finger joints mount 0.0817 m down the tool
  from link5, and the pads sit further along the finger. Counting only the
  second term aims the pads through the floor.

* Detection transforms with the latest TF, valid only because it runs with the
  arm parked, and frames captured before the arm settled are dropped. Asking
  for the capture-time transform instead deadlocks the single-threaded
  executor, since TF trails the image stream and blocking inside the callback
  stops the spin that would deliver it.

* The gripper and wrist roll are commanded straight at their
  JointTrajectoryControllers. Brushing the target pushes the fingers onto their
  hard stop, and MoveIt's CheckStartStateBounds then refuses to plan at all,
  failing exactly when contact means it matters. Each trajectory is published
  once, since republishing restarts the motion and it never arrives.

* Wrist roll is aligned to the target before descending. PCA gives the object's
  axis only mod 180, so the detector resolves the ends by width (the skirt half
  of the silhouette is fatter) because the tapered jaw is not symmetric end to
  end and pointing its taper the wrong way grips worse than a flat jaw.

models/shuttlecock is a BWF-dimensioned shuttlecock with a generated conical
skirt mesh; sdformat 14 has no cone primitive.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---

## SHA mapping (original -> rewritten)

| before | after | subject |
|---|---|---|
| `ea0a24c17` | `2bf495ecb` | Give the carry stages reach margin so the hopper deposit actually runs |
| `8564a76fa` | `fd1142bd5` | Launch four shuttlecocks on the robot's own half of the net |
| `8e3189230` | `5bd024617` | Find grasp_ball and the shuttlecock model via the package, not a relative source path |
| `7ed269727` | `2bc3d9131` | Document the reasoning behind the code |
| `9763c1cfe` | `7924f46c5` | One launch file for the whole badminton-court grasp trial |
| `bf9e6e283` | `22f3c0750` | Change project name to Birdie Bot |
| `c55cd67f6` | `a594ec23c` | Park further out from the chassis; make the shuttle count settable |
| `2573edf77` | `b0bb2d609` | Carry the shuttlecock to the hopper and release it |
| `fba5421f3` | `b3a98842d` | Plan against the robot that is actually running |
| `71ac6ae8c` | `4bada8f9b` | Give the local costmap enough reach for MPPI's horizon |
| `64063c4ec` | `7b6e0ae2e` | Approach forwards from a metre out, and rehome the arm on the move |
| `59a131cec` | `1d23a9a45` | Pick the blob nearest the target, not the biggest in frame |
| `8dd124ce9` | `7fe7100b3` | Fix three faults that made collection runs unrunnable |
| `af593190c` | `7ba28499b` | Drop set -u from bringup.sh; ROS setup files trip it |
| `cda0ec302` | `05a2c92b0` | Drop MPPI temperature to 0.05; trials get 35% faster |
| `e03da204f` | `cd5934d6e` | Add a runbook and the operational scripts it describes |
| `3a8d4365b` | `fd0682998` | Add a court-clearing harness: one corner start, sixteen shuttles |
| `8d51c9f95` | `e37519b45` | Stop the guard braking on its own chassis, and unthrottle MPPI |
| `a4b7d61b7` | `790da5e07` | Spread the obstacles across the court instead of around the spawn |
| `6797c8389` | `806584fc1` | Vectorise the scan filter and the guard's collision test |
| `28d451c8d` | `59b8ee400` | Stop marking the badminton net as an obstacle |
| `83270ace5` | `36597a415` | Scatter shuttlecocks across the court instead of a 1-2 m ring |
| `a88b28e2a` | `37e1a1edf` | Add a badminton court world, and stop hardcoding the world name |
| `cd0c87811` | `0e1201570` | Stop the lifecycle managers tearing down nav2 mid-run over a missed heartbeat |
| `569981602` | `4e2585028` | Find the cause of the nav2 half-bringup: AMCL must be seeded first |
| `cc27cce24` | `98c39bbc2` | Drop the fixed 3 s arm wait, and stop the nav2 recovery from crashing the run |
| `9225fe2ae` | `d173376ce` | Recover the grasp axis from depth when the silhouette is too round |
| `edee19193` | `406d72fd1` | Speed up the pick loop, and instrument it so "faster" is measurable |
| `1514e99c9` | `79b828380` | Recover stalled nav2 nodes automatically, and fix the check that never fired |
| `ef5d09706` | `a8ca3e945` | Check nav2 is ACTIVE, not merely advertising, before starting a run |
| `b2e9acc0b` | `7685cf341` | Record the AMCL A/B honestly: filtering the scan does not measurably help it |
| `310d9ed4f` | `365dcbe13` | Keep the AMCL drift test instead of leaving it in /tmp |
| `1890d1afd` | `ebb350cdc` | Point AMCL at the filtered scan too |
| `b547b8fa4` | `1d86245f5` | Filter the lidar returns that land on the robot's own arm |
| `1546c8267` | `19ecd8eeb` | Abort the run if /navigate_to_pose is missing instead of burning ten trials |
| `1e6b55004` | `bd43adeb1` | Replace the contact-sensor collision check with continuous clearance tracking |
| `12f9e62cd` | `029546d0c` | Correct the record on UPRIGHT_TOP_Z: it was never broken |
| `414edd152` | `787598118` | Stop fine_approach commanding a strafe the base cannot perform |
| `a51b50d73` | `e8ec9b34e` | Give the depth cloud a clearing-only twin so elevated marks can be cleared |
| `fcfb8da1f` | `38c097557` | Stop the harnesses scoring infrastructure faults as robot failures |
| `5b5a7229c` | `592ec56d3` | Feed the ZED depth cloud into the global costmap as well |
| `f9cc351fb` | `7a5f09c18` | Add obstacles to the navigate-then-grasp trials |
| `b1eb0ec29` | `0a86b62c6` | Add navigate-then-grasp, with a vision correction that makes it 10/10 |
| `0592a52c4` | `b1d83f8c8` | Fix the pick: spawn the shuttlecock with its skirt mesh actually loaded |
| `cca76e5fa` | `1eaeaf8ce` | Feed the ZED depth cloud into the local costmap as a 3D obstacle source |
| `a620a9338` | `86102aa46` | Add cmd_vel_guard, a collision-checking replacement for collision_monitor |
| `0a37cc68b` | `bd661d56b` | Add cmd_vel relay bypassing broken collision_monitor |
| `ba0e8b6af` | `1b2da58d2` | Fix velocity_smoother zero-accel bug and collision_monitor transform timing |
| `da906b8e6` | `01d5416ea` | Add SLAM and AMCL configs for husarion_world |
| `9b0f951ff` | `7c7dc92ca` | Aim the claw's pocket correctly, and fix a startup race that lost half the runs |
| `470223dbc` | `d035d3367` | Improve detection accuracy and cut pick time from 43s to 17s |
| `cd5380a67` | `718f1fb13` | Vision-guided shuttlecock pick with the ROSbot XL arm |
