# Planner sandbox

Runs the Intrinsic Core motion planner in-process on scene files, natively on
macOS (or Linux), and visualizes the result in [viser](https://viser.studio).
There's no cluster, behavior tree, ICON or world service. The planner talks
to an in-memory `World` loaded from SDF or `.gzf`.

```
scenario.textproto ──► plan_scenario (C++) ──► out/result.json + out/meshes/*.glb ──► viz/view.py (viser)
   SDF / .gzf scene        MotionPlanner::PlanTrajectory / PlanPath
   WorldUpdates            (the real planner sources from this checkout)
   MotionSpecifications
```

## Quick start

```bash
cd planner_sandbox
tools/fetch_lfs.sh        # robot meshes are in Git LFS (fetched into .git/lfs only)
bazel run //planner:plan_scenario -- --scenario=scenarios/ur5e_pillar.textproto --out=/tmp/ur5e
uv run viz/view.py /tmp/ur5e   # http://localhost:8080
```

The first build fetches and compiles about 30 external libraries, which takes a
few minutes. The viewer reloads whenever `result.json` changes, so you can keep
it open and re-run `plan_scenario` after editing a scenario or the planner.

Useful flags for `plan_scenario`:

* `--stderrthreshold=0`: show the planner's INFO logs (planning stages, timing).
* `--v=1`: also log the duration of each `ScopedSpan`
  (`MotionPlanner/PlanPathImpl`, `RrtConnectPathPlanner/Plan`, ...).
* `--timeout_s`: planning timeout for motions that don't set one.
* `--path_refinement_validation`, `--fallback_trajectory` and
  `--collision_threads`: planner flags (defaults match the production service).
* `-c dbg` / `--config=dbg` on the `bazel run` line builds a debuggable binary.

## Scenarios

A scenario (`planner/scenario.proto`) is one scene, one robot, and a list of
motions planned in order. Each motion starts where the previous one ended.

```textproto
scene {
  sdf: "my_cell.sdf"                       # or gzf: "my_cell.gzf"
  generate_group_ids: true
  world_updates_files: "my_cell_updates.pbtxt"   # intrinsic_proto.world.WorldUpdates
}
robot: "my_robot"
start_joint_positions: [...]
motions {
  name: "pick_approach"
  motion_specification { ... }         # intrinsic_proto.motion_planning.v1.MotionSpecification
  motion_planner_config { ... }        # optional MotionPlannerConfiguration
  path_only: false                     # true: PlanPath only (no trajectory)
}
motions { name: "grasp"   attach { object: "box" } }   # box now moves with the robot
motions { name: "release" detach { object: "box" } }   # box stays where it is
```

* Relative paths are relative to the scenario file.
* `model://` URIs in SDF/URDF are looked up in the scenario directory,
  `--asset_root` dirs, the checkout root, and each `intrinsic_*/` directory.
  So `model://intrinsic/robot_definitions/ur/ur5e` and the assets under
  `intrinsic/models` work as-is.
* The world updates set things the planner needs: the robot tip, IK solver key,
  self-collision exclusions, and object names. See
  `intrinsic/skills/apps/test_data/ur5e_world_updates.pbtxt`. SDF models only
  become objects (usable in attach/detach and collision rules) once they're
  named with `set_object_names`.
* `--print_objects` lists the loaded objects and their frames: the names to
  use in motion targets and collision rules.
* `attach` reparents an object onto the robot's last link (or the final entity
  of `parent`), keeping its world pose. From then on it's collision checked as
  part of the moving robot. `detach` reparents it to `root`.
* Path constraints: `motion_segments.path_constraints` holds along the whole
  segment, e.g. `rotation_cone` (tool axis within a cone, "pointing down
  within N degrees"), `position_bounding_box`, `point_at`, or an intersection
  of these. The planner then uses numerical IK and constrained RRT, which is
  much slower and can search indefinitely for infeasible problems. That's why
  `--timeout_s` (default 30 s) applies to motions without
  `motion_planner_config.timeout_sec`.
* Intended contacts, like the flange touching a grasped part or a part sitting
  on a table, are excluded per segment with `collision_settings.collision_rules`
  (`collision_action { is_excluded: true }`).

Examples:

* `scenarios/ur5e_basic.textproto`: the planner unit-test world. A joint
  move, then a linear Cartesian move.
* `scenarios/ur5e_pillar.textproto`: a pillar blocks the direct path, so
  `motion_type: ANY` has to plan around it. With `JOINT` it fails as expected.
* `scenarios/ur5e_pick_place.textproto`: pick and place over a wall in 9 steps.
  Move to pre-grasp, linear approach, attach, linear retract, transfer over the
  wall with the box attached and kept within 30° of upright (a `rotation_cone`
  path constraint), linear approach, detach, linear retract, home.

## How the build works

The main workspace can't build on macOS (CUDA, ROS 2, Linux-only toolchain
setup), and most of what it builds isn't needed for planning. This directory is
a separate Bazel module with only the planner's real dependencies: abseil,
protobuf, eigen, coal, assimp, CGAL, boost, nlopt, osqp, sdformat, and a few
small ones. It uses the host Xcode toolchain.

* `tools/gen_manifest.py` runs `bazel query` on the planner targets **in the
  main workspace** and turns the real dependency graph into `gen/manifest.json`.
  The `CUTS` and `STUBS` lists there say where the graph is cut.
* `bazel/intrinsic_src.bzl` is a repository rule. It builds `@intrinsic_src`
  from the manifest: a single `intrinsic/...` tree of **symlinks** into this
  checkout, plus generated BUILD files. Editing a planner source in
  `intrinsic_motion_planning/` is picked up by the next build.
* `stubs/` replaces a few targets:
  * `ScopedSpan` (OpenTelemetry/gRPC) becomes a timer.
  * Robotics Library dynamics return Unimplemented, so torque-limited TOPP isn't
    available.
  * `tf2::Stamped` is a plain template.
* `patches/` (generated by `tools/make_patches.py`) are applied to copies of
  the touched files only. Your checkout is never modified.
  * `no_ros_tf2`: drops the time-indexed tf2 buffer from `World`.
  * `no_object_world_client`: drops the gRPC world-client overload from
    `compute_ik_util`.
  * `no_grpc_status`: drops the `grpc::Status` overload of `GetExtendedStatus`.
  * `macos_portability`: futex via `os_sync_wait_on_address`,
    `program_invocation_name`, `ENOKEY`, `<bits/time.h>`, thread naming.

After upstream changes to the planner's dependencies, regenerate:

```bash
planner_sandbox/tools/gen_manifest.py    # needs bazel; runs the query in the main workspace
planner_sandbox/tools/make_patches.py    # if a patched file changed
```

## Not included

Things the sandbox doesn't cover:

* `ObjectWorldUpdates` scene files. These need the world service; only
  `WorldUpdates` are supported.
* Motion planner service features: caching and swept volumes.
* Torque limits, which need Robotics Library dynamics.
* Time-indexed TF lookups.
