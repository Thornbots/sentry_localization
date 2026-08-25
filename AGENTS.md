# sentry_localization — agent notes

Localization backends (SLAM / AMCL / EKF) for the Sentry. Consumes `/odom` +
`/scan`, produces `/localization/odom` and the `map->odom` owner. **Reference
docs live in `README.md`**; its `## Notes` section holds the per-parameter
tuning rationale and the dated measurement history that justifies the current
values. Read that before changing any YAML. This file is only the operating
contract for working here.

Parent conventions in `../CLAUDE.md` apply, notably: **in-code
comments/docstrings under 10 lines**; longer prose goes to a `## Notes`
subheading in `README.md`.

## Running anything

Never hand-roll `docker exec`. Use `../isaac_ros_common/scripts/dexec.sh`, the
only path with correct env parity (ROS_DOMAIN_ID, FastDDS profile, both
workspace installs, `-u admin` for GUI). Load the `isaac-ros-docker` skill
before your first container command.

```bash
# all paths below are relative to this package dir
../isaac_ros_common/scripts/dexec.sh -- colcon build --symlink-install --packages-select sentry_localization
../isaac_ros_common/scripts/dexec.sh -- colcon test --packages-select sentry_localization
```

Launch through `sentry_pkg`'s `auto.launch.py`, which includes this package's
`localization.launch.py`. Don't launch it standalone.

**Two ways an edit here silently does nothing:**

1. **`ros2_ws` shadowing.** `Dockerfile.thornbots` clones this package
   (`RECLONE_LOCALIZATION`) into `/workspaces/ros2_ws`. Once it's built
   locally, `dexec.sh` picks up your `src/` edit but the user's terminal
   resolves to the image-baked clone. Confirm with
   `../isaac_ros_common/scripts/dexec.sh -- ros2 pkg prefix sentry_localization`
   (`/workspaces/isaac_ros-dev/…` = your edit is live).
2. **Config files are copied at build time**, not read from `src/`. After
   editing `config/*.yaml`, rebuild with `--symlink-install` (once done, later
   edits are live). If a result looks implausibly unaffected by a config
   change, diff `install/sentry_localization/share/…/config/X.yaml` against
   `config/X.yaml` before concluding the change didn't work.

On 2026-07-25 this exact trap invalidated a full round of EKF measurements.

## Testing

The drift/jerk suite lives in `../sim/test/localization/run_localization_drift_tests.py`;
see `../sim/AGENTS.md`. Run it after tuning `slam.yaml`, `amcl.yaml`,
`ekf.yaml`, or `sim`'s noise model, and rebuild both packages first.

## Scope

- Owns the backends and their tuning; `localization_mode` (`slam` default /
  `mapping` / `amcl` / `none`) picks the `map->odom` owner, `use_ekf`
  independently picks the `odom->root` source.
- No direct hardware dependency; drivers, URDF, and the MCB relay belong to
  `../sentry_pkg`.
- Current state: localization is considered good enough; CV is the priority.
  Don't retune without being asked. See `../SESSION_NOTES.md`.
- Its own git repo (`Thornbots/sentry_localization`).
