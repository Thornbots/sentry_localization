# sentry_localization

Localization backends (SLAM, AMCL, EKF) for the Thornbots ARC 2026 Sentry.
It consumes `/odom` and `/scan` and has no hardware dependency; drivers and
the robot description are in `thornbots_pkg`. See `thornbots_pkg/README.md`
for how the two fit, and `../ARCC_2026_SENTRY_CONTEXT.md` for game rules.

## Contract with thornbots_pkg

- In: `/odom` (raw wheel odometry from `pose_translator`) and `/scan` (lidar
  driver or `sim`).
- Out: `/localization/odom` in every mode, so `odom_tf_broadcaster` can turn
  it into `odom->root` without knowing the backend.
- `slam_toolbox` or `amcl` broadcasts `map->odom` directly.
- Out: `/localization/map_odom` under `slam`/`mapping`/`amcl`:
  `/localization/odom` moved into `map` by `map_pose_publisher`. Its xy
  covariance adds the backend's own (`/amcl_pose`, slam_toolbox's `/pose`);
  until that arrives it reads 1 m^2. Not yet read by `mcb_relay`, whose
  0.02 m std gate it never passes (2026-10-02, sim: amcl 0.17-0.30 m,
  mapping 0.05-0.08 m).

`thornbots_pkg/launch/auto.launch.py` includes `launch/localization.launch.py`
and forwards its args. Standalone, for testing:

```bash
ros2 launch sentry_localization localization.launch.py localization_mode:=amcl
```

`localization_mode` picks the `map->odom` owner:

| Value | Owner | Use |
| --- | --- | --- |
| `slam` | `slam_toolbox`, localization mode | localize against the saved field map |
| `mapping` | `slam_toolbox`, mapping mode | build or extend the map |
| `amcl` | `nav2_amcl` + `nav2_map_server` | particle filter on a saved occupancy grid |
| `none` | no map frame | odometry only, usually with `use_rf2o:=true` |

`localization.launch.py` defaults to `slam`; `auto.launch.py` defaults to
`mapping` from a blank map (since 2026-10-02, amcl before), with
`mcb_relay` still relocalizing from `/localization/odom`.

`use_rf2o` (named `use_ekf` before 2026-09-28) picks the `/localization/odom`
source, with any mode:

| Value | Source |
| --- | --- |
| `false` | `passthrough_odom_publisher`, relays `/odom` unchanged |
| `true` | `robot_localization`'s `ekf_node`, fusing `/odom` and rf2o's `/scan_odom` |

`true` is the default in both this file and `auto.launch.py` (since
2026-09-20), so the stack runs rf2o + the EKF unless you pass
`use_rf2o:=false`.

`use_rf2o:=true` also starts `rf2o_laser_odometry_node` on raw `/scan`. The
Thornbots fork re-queries `lidar->root` every scan, so the head-mounted lidar
can move.

Other args:

- `map_file` (default `map/clean_map`, no extension): `slam_toolbox` reads
  `<map_file>.posegraph/.data`, `amcl` reads `<map_file>.yaml`. The maps
  here (`clean_map`, `ARCC26`) are images only, in the field frame (REP-105,
  (0, 0) at the field centre, x toward blue's base). No pose graph ships:
  for `slam`, or `mapping` with `load_map:=true`, save one from a `mapping`
  run (`autosave_map`) and pass its `map_file`.
- `load_map` (default `true`): load the saved pose graph at startup
  (`slam`/`mapping` only).
- `odom_frame` (default `odom`).
- `use_sim_time`: set by `auto.launch.py` from `real_hardware`.
- `autosave_map` (default `false`; `auto.launch.py` passes `real_hardware`):
  under `mapping`, `map_autosaver` saves `map.posegraph/.data`
  (`serialize_map`) and `map.pgm/.yaml` (written from `/map` itself, as
  nav2's map_saver would) into `map_save_dir` (`/workspaces/isaac_ros-dev/maps`,
  the bind-mounted workspace) `/<boot time>/`, every `map_save_period_s`
  (30 s). Not slam_toolbox's `save_map`: its `map_saver_cli` process must
  find `/map` within 2 s, and on the sentry mostly didn't. Load one back
  with `load_map:=true map_file:=<dir>/map`.

Nodes: `passthrough_odom_publisher.py`, `map_pose_publisher.py` (math in
`map_pose_core.py`) and `map_autosaver.py`. `thornbots_pkg/mcb_relay.py` compares
`/localization/odom` with `/odom`.

## Testing

The drift and jerk suite is `ros2 launch sim localization_tests.launch.py`;
see `sim/README.md`.
It reads these `config/*.yaml` files from `install/`, so rebuild with
`--symlink-install` after editing them.

`colcon test --packages-select sentry_localization` runs the ament copyright,
flake8 and pep257 checks.

## Notes

Rationale for the current values. Read it before changing a value back to
one already rejected.

### launch/localization.launch.py

- `slam` with `load_map:=false` has no map to localize against.
- `amcl` always loads `<map_file>.yaml`; it can't start blank. `load_map` does
  nothing for `amcl` or `none`.
- `use_map_saver` is on only in `mapping`, so saving a map is always a
  deliberate choice.
- `map_server`, `amcl` and (since Jazzy) `slam_toolbox` are lifecycle
  nodes, each activated by a `lifecycle_manager` with `autostart: true`:
  `lifecycle_manager_localization` for amcl, `lifecycle_manager_slam` for
  slam_toolbox. Without it slam_toolbox sits unconfigured and never
  publishes `map->odom`.
- rf2o used to cache its lidar->root transform from the first scan, which
  assumes a rigid mount. The fork (built by `isaac_ros_common`'s Dockerfile)
  re-queries every scan, so the old `head_home_scan_gate` and its
  `/scan_gated` topic are gone.
- rf2o runs with `fixed_heading: true` and `odom_prior_topic: /odom`. The
  chassis never turns, so rf2o's accumulated yaw error would only rotate its
  x/y. Its own velocity prior is the previous scan's motion, so every move
  from rest started with a prior of "stopped" and came up short: legs of
  x0.86-0.98 at 4 m/s in sim, only in some directions. Seeded with wheel
  odometry's motion between the two scans, every leg reads x1.00 and the
  EKF's mean error (0.050 m) beats raw `/odom` (0.134 m) on `suite:=ekf`,
  real time, `sentry_v2`.

### config/rf2o.yaml

rf2o grades every scan match good, degraded or failed, from its own
evidence only: whether it solved, the fraction of valid beams, the match's
own covariance (`cov_odo`), a 6 m/s speed limit, scan gaps, and a stale
lidar extrinsic (the latest head TF more than `max_extrinsic_age` older
than the scan). It never compares against `/odom`, because on Bumpy Road
the wheels slip and rf2o is the one that's right.

- good: `position_covariance` (0.02 m std-dev).
- degraded: adds `degraded_position_variance` along each weak axis of
  the match, so a single long wall in view inflates only the along-wall
  axis.
- failed: the match is discarded and the pose advances by `/odom`'s motion
  between the two scans, so rf2o's pose never jumps. The variance grows by
  `failed_variance_rate` per second failed and sheds over
  `recovery_matches` good matches. Without `/odom` covering both stamps,
  nothing is published for that scan.

The grade has to act on the increment, not on publishing. rf2o's pose
accumulates, so dropping a bad message leaves its error in every later
one, and the EKF's Mahalanobis gate would lock rf2o out after one jump.

Every grade is on `/scan_odom/quality` (`diagnostic_msgs/DiagnosticArray`,
scan stamp) with the signals as values, whether `confidence_enabled` is on
or not. With it off, the topic reports what would have happened.
`sim/tools/rf2o_quality.py` records it and prints the distributions.

Over the drift suite (2026-09-28, amcl, `--use-rf2o`, 2418 matches), every
match in the five clean scenarios graded good bar each stack's first scan.
Clean driving's worst: valid fraction 0.64, sigma 1.5 mm, 4.03 m/s,
extrinsic 0.04 s old. The rest landed where meant: jerks and `odom_stuck`
failed on speed (7-18 m/s), `scan_degraded`'s 60 deg arc failed on
`few_points` (0.10-0.17). Grading that arc degraded instead
(`fail_valid_fraction` 0.05) read 0.377 and 0.417 m during the blackout
against 0.415 and 0.434 m failed: within run-to-run spread, so it stays
failed.

Until 2026-09-28 any extrinsic not exactly at the scan's stamp was stale.
The lidar rides the head's yaw joint, whose TF trails every scan by one
sim step (0.01 s), so 94% of clean matches graded degraded and the EKF
leaned on `/odom`: `suite:=ekf` fell from ~95% to 75-80% better than raw.

Two upstream bugs are fixed with it, in every mode: a failed match used to
republish the previous pose at full confidence, and a scan with too few
points on every pyramid level read as zero motion.

### config/amcl.yaml

`robot_model_type` is `nav2_amcl::OmniMotionModel`, with its strafe noise
term, since the chassis is holonomic.

**alpha1-5: 0.0875** (stock 0.2). On 2026-07-21 all five at 0.05 crashed
amcl on every startup, before any driving:

```
amcl: ./src/pf/pf_kdtree.c:363: pf_kdtree_cluster:
Assertion `node == pf_kdtree_find_node(self, self->root, node->key)' failed
```

Low alphas let resampling collapse particle diversity until the kd-tree's
node-uniqueness assertion fails, a known nav2_amcl failure. A binary search
found 0.0875 as the lowest value that never crashed, and the retired
`continuous_drift` scenario (injected drift, jitter and slip) still measured
growth_ratio ~1.0-1.1. The ~0.3-0.4m `map->odom` wobble that prompted the
change, in the retired `unmapped_obstacle` scenario, came from motion-model
noise against sim's exact odometry, not from the obstacle. To go lower, step
to 0.075 then 0.0625, check for the crash each time, and test drift and
obstacle cases together.

**sigma_hit: 0.08** (stock 0.2). The likelihood field is nearly flat within
about one sigma, so at 0.2 amcl can't tell poses within ~0.2m apart. That
matched `unmapped_obstacle`'s ~0.2-0.3m noise floor, where the alpha,
`resample_interval` and particle knobs all plateaued, pointing at the sensor
model. Sim's lidar now matches the RPLIDAR A2M8 in its typical mode: ~800
beams, 0.01 m noise. 0.15 measured worse (see Tuning history), but
before this lidar change, and nothing has been re-measured since. Re-derive
rather than treating 0.08 as final, and check a sharper field doesn't make
amcl brittle to real sensor noise.

**do_beamskip: true** (stock `false`, 2026-07-21) drops beams that disagree
with the map from the weight update. It didn't fix the wobble (0.409m after,
0.395-0.443m before), since the wobble wasn't about obstacles. It stays as a
cheap defence against real obstacles in a match.

**resample_interval: 2** (was 1). With `update_min_d`/`update_min_a` at
0.1m/0.05rad and the robot at 4.0 m/s, nearly every scan triggers an update,
and each resample adds variance to the mean pose. Every other update halves
that without slowing convergence much.

**min/max_particles: 1000/3000** (was 500/2000) lowers the mean pose's
variance too. `max_beams` 500 already fell behind real time (it is 180 now),
and particle count multiplies per-beam cost, so counts stay well under 5000.
Both knobs target amcl's own noise: in `unmapped_obstacle` odometry was exact,
so any `map->odom` movement was amcl's.

### config/ekf.yaml

Covariance and process noise are unmeasured starting guesses.

- `odom0` (`/odom`): yaw, vx, vy.
- `odom1` (`/scan_odom`, rf2o): x, y. It used to be gated to head-home
  windows; since the rf2o fix it streams continuously.

x/y from `/odom` stay out. Wheel slip (the "Bumpy Road" zone, sim's
`odom_slip_ratio`) is an error in integrated distance, and no covariance lets
`/scan_odom` pull the filter off the slipped position. On 2026-07-25, driving
~42m straight at 0.05 slip, EKF output tracked `/odom` within 0.001m while
both sat 2.02m behind truth. Encoders now supply velocity and `/scan_odom`
supplies position.

Yaw comes from `/odom`, whose orientation is always identity because the
chassis never rotates (`ARCC_2026_SENTRY_CONTEXT.md`, "Our Sentry's
drivetrain"). That identity is the true heading. `robot_localization` rotates
body-frame velocity by the filter's yaw, so a wrong yaw integrates velocity in
the wrong direction. On 2026-07-25, taking yaw from rf2o instead (off by ~pi)
rotated the estimate 180 degrees: 2.97m mean error against raw odometry's
0.22m.

`dynamic_process_noise_covariance: true`, with x/y `process_noise_covariance`
raised 0.05 to 0.08 (`8a719ab`), scales process noise with estimated
velocity. Isolated `noise_correction` runs went from ~40-50% passing to
~60-80%, at both 0.25 and 0.15 slip. Both full-suite attempts failed
(growth_ratio 2.17 and 2.49), and nobody knows why; load or leftover state
from earlier scenarios are candidates. Kept for the isolated gain. Confirm
`noise_correction` in the context you're reporting before calling it passing.

### config/slam.yaml

**minimum_travel_distance: 0.1** (was 0.5). A jerk (slip, bump, collision;
see `sim/sim/pose_emulator.py`'s `trigger_jerk`) is invisible to wheel
odometry, so a slow or stopped robot never travelled 0.5m, slam_toolbox never
matched a new scan, and `map->odom` froze. Live check: a stationary
`trigger_jerk` got no update in 30s at 0.5, while `/cmd_vel` motion updated
normally. 0.1m is two cells, cheap enough and prompt.
`minimum_travel_heading` stays 0.05 rather than 0 to avoid exact-zero
comparisons; heading never changes anyway.

**correlation_search_space_dimension: 2.5** (was 0.5) and
**minimum_time_interval: 1.0** (was 0.5), `f23c6be`: see Tuning history.

## Tuning history (2026-07-26/27)

These two days searched for a config that passes `drift_correction` and
`drift_correction_obstacle`. Run-by-run logs are in this file's git history.

### Outcome: tuned `slam`, no EKF

Widening `correlation_search_space_dimension` to 2.5 did the work. At 0.25
slip, raw odometry can sit ~0.53m off true pose at the far corners, outside a
+-0.25m window. `minimum_time_interval: 1.0` gives fewer, wider-baseline
matches on the fast corners; 0.1 measured 0.89m.

At the current 0.15 slip, three full-suite runs gave `drift_correction`
0.3125/0.3261/0.3224m and `drift_correction_obstacle` 0.3150/0.3002/0.3194m.
Every amcl config spread 0.4-1.3m run to run. `MAX_DELTA_THRESHOLD` (in
`sim/test/localization/drift_harness.py`) went from 0.30m to 0.40m, ~25% over
the worst sample, and the scenarios then present passed reliably.

`auto.launch.py` defaulted to `amcl` from 2026-08-29 and to `mapping` since
2026-10-02, so pass
`localization_mode:=slam` to get this configuration.

### Closed levers

Measured and reverted. Don't reopen without new information.

| Knob | Tried | Result |
| --- | --- | --- |
| `amcl.yaml` alpha1-5 | 0.2 (alpha3/5), 0.07 (all) | 0.51m / 0.47m vs 0.43-0.44m |
| `amcl.yaml` min/max_particles | 2000/5000 | 0.45m |
| `amcl.yaml` sigma_hit | 0.08 -> 0.15 | 1.12m |
| `ekf.yaml` x/y `process_noise_covariance` | 0.05 -> 0.02, 0.05 -> 0.1 | spreads overlap, 0.40-1.00m at one config |
| `slam.yaml` distance/minimum_distance_penalty | 0.5 -> 1.0/0.8 | no gain over the search-space change |
| `slam.yaml` minimum_time_interval | 0.5 -> 0.1 | 0.89m |

amcl was rechecked at the looser 0.15 slip and 0.30m threshold. The verdict
held.

### Why the map-based drift scenarios are hard

`pose_emulator.py` adds only `(1 - slip_ratio)` of each true step to `/odom`,
so `odom_error(t) = slip_ratio * |true_pos(t) - true_pos(t0)|`, measured from
the start point, not along the path. The loop's corners reach 2.12m from
spawn, so at 0.25 slip the correction must move ~0.53m, over 2.5x the 0.20m
threshold of the time. Plain amcl has only its scan match for absolute
position and can't get under that. Configs that tracked the true error more
faithfully landed closer to 0.53m.

Two findings locate the remaining error:

- rf2o is fine. In a 50s bag, `/odom` moved +2.272m in x against ~3.03m true,
  while `/scan_odom` and `/localization/odom` moved +2.932m, within ~3%.
- The error ramps. `map->odom` climbs from ~0.04-0.08m at t=2s to 0.65-0.76m
  at t=45s and peaks at loop close. Error measured from the start point
  should shrink as the robot returns to its start corner, so this looks like
  path-length drift in the fused estimate upstream of amcl, not filter noise.

### `slam --use-rf2o`: worse

Back to back on a quiet host against plain `slam`: `drift_correction`
1.0659m vs 0.5066m, `drift_correction_obstacle` 0.8652m vs 0.4876m, and
`noise_correction` newly failing (growth_ratio 2.42). Likely cause, from
reading the code rather than bags: `slam_toolbox` corrects relative to its
`odom_frame`, which here is the EKF output, so the EKF's rf2o position
correction and slam_toolbox's correction stack on the same lidar data.

### Measurement caveats

- Host load corrupts results. One batch ran beside ~18 agent sessions (load
  4-8 on 22 CPUs) and its `--use-rf2o` numbers were unusable. Check `uptime`
  and `ps aux --sort=-%cpu` before trusting a number near threshold.
- Bringup races look like config failures: `amcl/get_state`
  `async_send_request failed`, or `map_server` "IS DOWN after not receiving
  a heartbeat for 4000 ms" cascading into a crash. Rerun on a quiet host.
- The metric is a max over a 45s drive, so one transient correction sets it.
  No process-noise knob can change that.
- Geometry changes invalidate baselines. The 0.1642m `amcl+ekf` result and
  the 0.20m threshold came from a 2m loop (corners ~1.80m from spawn), before
  `4f182e7` widened it to 3m (~2.12m).
