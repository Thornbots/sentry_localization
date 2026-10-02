# sentry_localization: agent notes

Localization backends (SLAM / AMCL / EKF) for the Sentry. Consumes `/odom` +
`/scan`, produces `/localization/odom` and the `map->odom` owner. **Reference
docs live in `README.md`**; its `## Notes` section holds the per-parameter
tuning rationale and the dated measurement history that justifies the current
values. Read that before changing any YAML.

`README.md` commands are written for a human in a container terminal. You run
them from the host through `../isaac_ros_common/scripts/dexec.sh` (load the
`isaac-ros-docker` skill first), e.g.
`../isaac_ros_common/scripts/dexec.sh -- colcon build --symlink-install --packages-select sentry_localization`.

Launch through `thornbots_pkg`'s `auto.launch.py`, which includes this package's
`localization.launch.py`. Don't launch it standalone.

**Two ways an edit here silently does nothing:**

1. **`ros2_ws` shadowing.** `Dockerfile.thornbots` copies this package into
   `/workspaces/ros2_ws` at build time. Once it's built locally,
   `dexec.sh` picks up your `src/` edit but the user's terminal resolves to the
   image-baked snapshot. Confirm with
   `../isaac_ros_common/scripts/dexec.sh -- ros2 pkg prefix sentry_localization`
   (`/workspaces/isaac_ros-dev/…` = your edit is live).
2. **Config files are copied at build time**, not read from `src/`. After
   editing `config/*.yaml`, rebuild with `--symlink-install` (once done, later
   edits are live). If a result looks implausibly unaffected by a config change,
   diff `install/sentry_localization/share/…/config/X.yaml` against
   `config/X.yaml` before concluding the change didn't work.

## Testing

The drift/jerk suite is `ros2 launch sim localization_tests.launch.py`
(tests in `../sim/test/localization/`); see `../sim/AGENTS.md`. Run it after tuning `slam.yaml`, `amcl.yaml`, `ekf.yaml`, or
`sim`'s noise model.

## Scope

- Owns the backends and their tuning; `localization_mode` (`mapping` default
  in `auto.launch.py` / `slam` / `amcl` / `none`) picks the `map->odom` owner, `use_rf2o`
  independently picks the `odom->root` source.
- No direct hardware dependency; drivers, URDF, and the MCB relay belong to
  `../thornbots_pkg`.

## Open

- **`save_map` (the `.pgm`) fails about 1 save in 6** in sim: map_saver
  gives up waiting for `/map` ("Failed to spin map subscription"), with
  nothing else running (2026-10-02). The last good `.pgm` stays, and
  `serialize_map` never failed. `map_update_interval` is 5 s.
- **`/localization/map_odom` has no reader yet.** `mcb_relay`'s 0.02 m
  std gate rejects every pose it publishes (amcl 0.17-0.30 m, mapping
  0.05-0.08 m in sim), so wiring it in would stop all relocalizes. The
  gate, or how the backend covariance counts, is the user's call.
- **rf2o runs with `fixed_heading: true` and `odom_prior_topic: /odom`**
  (`localization.launch.py`). The first pins its yaw, since the chassis is
  assumed never to rotate; the second seeds each scan match with wheel
  odometry's motion in place of rf2o's constant-velocity guess, which read
  "stopped" at the start of every move and undershot it. Together they put
  the EKF under raw `/odom` in sim (`suite:=ekf`, real time). Unmeasured on
  hardware. The real robot may drift 1-5 deg in yaw, and `sentry_v2` in sim
  picks up ~1 deg in hard corners; noted, not acted on.
- **rf2o's grade thresholds are checked against the drift suite; its
  variances are guesses.** Both are in `config/rf2o.yaml`; README.md Notes
  has the distributions. rf2o grades every match from its own evidence and
  never against `/odom`, so a slip still corrects the EKF. After changing
  any, re-run `scan_degraded` and `drift_correction` (0.15 slip).
- **Jazzy: the drift suite and `suite:=ekf` give Humble's
  verdicts on the laptop.** No config key was renamed or removed. Defaults
  that changed or appeared, none of which we set: amcl
  `freespace_downsampling` (false), slam_toolbox `restamp_tf` and
  `check_min_dist_and_heading_precisely` (false), robot_localization
  `odomN_pose_use_child_frame` (false). All keep Humble's behaviour.
  robot_localization 3.8 logs "Failed to meet update rate!" at ERROR where
  Humble's printed it untagged; `sim`'s drift harness skips that line.

## Committing

This package is a submodule of `thornbots_workspace`, on branch `main`. Commit
and push here first, then bump this gitlink in `../` — one logical change, one
bump, never a gitlink pointing at an unpushed commit. Full rule in
`../CLAUDE.md` § Packages.
