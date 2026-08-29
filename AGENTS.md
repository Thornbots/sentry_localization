# sentry_localization: agent notes

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

- Owns the backends and their tuning; `localization_mode` (`amcl` default /
  `slam` / `mapping` / `none`) picks the `map->odom` owner, `use_ekf`
  independently picks the `odom->root` source.
- No direct hardware dependency; drivers, URDF, and the MCB relay belong to
  `../sentry_pkg`.
- Current state: localization is considered good enough; CV is the priority.
  Don't retune without being asked.
- Its own git repo (`Thornbots/sentry_localization`).

## Open

- **rf2o publishes all-zero covariance**, which reads to the EKF as
  "infinitely certain." Needs a real commit to
  `Thornbots/rf2o_laser_odometry`. A container-local patch script
  (`rf2o_cov_patch.py`, x/y variance `0.02**2`, yaw `0.05**2`, unobserved axes
  `1e6`) once existed but is gone from disk — assume nothing is applied and
  redo it from scratch. On its own it moved measured accuracy by essentially
  nothing; the scan-convention bug was doing all the damage.
- **rf2o degrades at speed.** Sweep: tracks within ~1% up to 2.0 m/s, reports
  12% short at 4.0 m/s (`ratio 0.879`), which is what the drift suite's legs
  use. It is a range-flow method that linearizes on small inter-scan motion, so
  this is a suitability ceiling, not a bug. It also drifts ~2 cm over 20 s while
  completely stationary. Options: raise the sim lidar's 10 Hz rate, cap speed,
  or swap the scan matcher.
- **rf2o only reads its `lidar->base_frame` transform once**, on its first
  scan, and caches it for the node's lifetime — it structurally assumes a
  rigid sensor mount, which our head-mounted lidar isn't. Current mitigation is
  `sentry_pkg`'s `head_home_scan_gate.py`, which only lets scans through while
  `|head_yaw| <= home_yaw_tolerance`. Still open: verify end-to-end that the
  head returns home often enough to produce useful correction windows. If
  head-away-from-home time dominates, the correct fix is forking rf2o to
  re-query the transform every scan (considered, not implemented — it means
  maintaining a patched fork instead of vanilla upstream).
- **Sanity-gate rf2o's output against wheel odometry before the EKF, but
  asymmetrically.** Wheel odometry drifts slowly under normal conditions and
  slips on the ARCC "Bumpy Road" zone; lidar data quality is good. So
  rf2o/`odom` disagreement during a slip is exactly the signal the EKF should
  trust, not discard. The gate should catch rf2o's own failure modes only, not
  arbitrate normal disagreement.
- **Add a `laser_filters` chain upstream of the scan matcher** to strip
  physically-implausible points from mid-turn self-obstruction. Already a
  `package.xml` dependency, currently unused.
- **`amcl.yaml`'s particle-filter params are stock nav2 defaults** (`alpha1-5`,
  `z_hit`/`z_max`/`z_rand`/`z_short`, `min_particles`/`max_particles`), never
  tuned against this robot or map. Next step is comparing AMCL's quality and
  covariance against `slam` over a longer run — drift through the "Bumpy Road"
  zone, recovery from a bad initial pose — before tuning further.
- **`slam`/`amcl` + `use_ekf:=true` has never been exercised.** No automated
  test covers it. Worth knowing before it is: `mcb_relay`'s `relocalize` path
  compares `/localization/odom` against `/odom`, and today every map backend
  passes `/odom` through unchanged, so that check is structurally near-zero and
  the path is dormant. Turning on EKF under a map backend makes the two
  diverge for the first time and wakes that path up. No code change needed —
  just watch for it if testing this combination on hardware.
- **The drift suite has never run end-to-end against `--backend amcl` or
  `ekf`** — only a launch-level smoke test. Its thresholds (e.g.
  `CORRECTION_FRACTION`, `DRIFT_THRESHOLD`) are slam_toolbox-tuned and may need
  backend-specific constants against AMCL's different particle-filter noise.
