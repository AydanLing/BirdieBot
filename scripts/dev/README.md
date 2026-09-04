# Development scaffolding

One-off probes and experiments kept because they document how a number was
arrived at, not because anything depends on them. None are installed and none
are referenced by the launch file or the harnesses.

| script | what it answered |
|---|---|
| `amcl_drift_test.py` | how far AMCL wanders while the base sits still |
| `cmd_vel_relay.py` | whether the guard or the drivetrain was dropping commands |
| `jaw_geometry.py` | the claw's actual pocket geometry, for the wrist-roll alignment |
| `mesh_to_collision.py` | turning a visual mesh into something DART will accept |
| `tipped_trials.py` | grasp success against how far the shuttlecock is tipped over |
| `graph_dump.py` | dumps the live ROS graph as json, for the diagram in DESIGN.md |

Two of these were the starting points rather than probes:

| script | superseded by |
|---|---|
| `early_moveit_sequence.py` | `grab_sequence/grasp_ball.py` |
| `early_ball_detector.py` | the detector inside `grasp_ball.py`, and `NavGrasp._deproject_all` |

They were the first working versions — a bare MoveItPy move and an HSV blob
detector — and they are kept because the path from them to the current pick is
the more honest picture of how this was built.
