# sentry_localization: agent notes

Localization backends (SLAM / AMCL / EKF) for the Sentry. Consumes `/odom` +
`/scan`, produces `/localization/odom` and the `map->odom` owner. **Reference
docs live in `README.md`**; its `## Notes` section holds the per-parameter
tuning rationale and the dated measurement history that justifies the current
values. Read that before changing any YAML.

Launch through `thornbots_pkg`'s `auto.launch.py`, which includes this package's
`localization.launch.py`. Don't launch it standalone.

**Two ways an edit here silently does nothing:**

1. **`ros2_ws` shadowing.** `Dockerfile.thornbots` clones this package
   (`RECLONE_LOCALIZATION`) into `/workspaces/ros2_ws`. Once it's built locally,
   `dexec.sh` picks up your `src/` edit but the user's terminal resolves to the
   image-baked clone. Confirm with
   `../isaac_ros_common/scripts/dexec.sh -- ros2 pkg prefix sentry_localization`
   (`/workspaces/isaac_ros-dev/…` = your edit is live).
2. **Config files are copied at build time**, not read from `src/`. After
   editing `config/*.yaml`, rebuild with `--symlink-install` (once done, later
   edits are live). If a result looks implausibly unaffected by a config change,
   diff `install/sentry_localization/share/…/config/X.yaml` against
   `config/X.yaml` before concluding the change didn't work.

## Testing

The drift/jerk suite lives in
`../sim/test/localization/run_localization_drift_tests.py`; see
`../sim/AGENTS.md`. Run it after tuning `slam.yaml`, `amcl.yaml`, `ekf.yaml`, or
`sim`'s noise model.

## Scope

- Owns the backends and their tuning; `localization_mode` (`amcl` default /
  `slam` / `mapping` / `none`) picks the `map->odom` owner, `use_ekf`
  independently picks the `odom->root` source.
- No direct hardware dependency; drivers, URDF, and the MCB relay belong to
  `../thornbots_pkg`.

## Open

- **rf2o publishes all-zero covariance**, which reads to the EKF as "infinitely
  certain." Needs a real commit to `Thornbots/rf2o_laser_odometry`. A
  container-local patch script (`rf2o_cov_patch.py`, x/y variance `0.02**2`, yaw
  `0.05**2`, unobserved axes `1e6`) once existed but is gone from disk — assume
  nothing is applied and redo it from scratch. On its own it moved measured
  accuracy by essentially nothing; the scan-convention bug was doing all the
  damage.
- **Sanity-gate rf2o's output against wheel odometry before the EKF, but
  asymmetrically.** Wheel odometry drifts slowly under normal conditions and
  slips on the ARCC "Bumpy Road" zone; lidar data quality is good. So
  rf2o/`odom` disagreement during a slip is exactly the signal the EKF should
  trust, not discard. The gate should catch rf2o's own failure modes only, not
  arbitrate normal disagreement.
