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
