# sentry_localization

Localization backends (SLAM, AMCL, EKF) for the Thornbots ARC 2026 Sentry.
It consumes `/odom` and `/scan` and has no hardware dependency; drivers and
the robot description are in `thornbots_pkg`. See `thornbots_pkg/README.md`
for how the two fit, and `../docs/ARCC_2026_SENTRY_CONTEXT.md` for game rules.

## Contract with thornbots_pkg

- In: `/odom` (raw wheel odometry from `pose_translator`) and `/scan` (lidar
  driver or `sim`).
- Out: `/localization/odom` in `nav_odom` in every mode. The broadcaster
  compares it with same-stamp raw `/odom` to publish translation-only
  `nav_odom->odom`; raw `odom->root` remains independent.
- `slam_toolbox` or `amcl` broadcasts `map->nav_odom` directly. Frame roles
  and deviations are owned by [thornbots_pkg](../thornbots_pkg/README.md#pose_translator).
- Out: `/localization/map_odom` under `slam`/`mapping`/`amcl`:
  `/localization/odom` moved into `map` by `map_pose_publisher`. Its xy
  covariance adds the backend's own (`/amcl_pose`, slam_toolbox's `/pose`);
  until that arrives it reads 1 m^2. Not yet read by `mcb_relay`, whose
  0.02 m std gate it never passes (2026-10-02, sim: amcl 0.17-0.30 m,
  mapping 0.05-0.08 m).

`thornbots_pkg/launch/auto.launch.py` includes `launch/localization.launch.py`.
Use that robot entrypoint; do not run a second localization launch alongside it.
The [operator workflow](../thornbots_pkg/README.md#field-day-and-room-workflow) wraps the same entrypoint.

`localization_mode` picks the `map->nav_odom` owner:

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
| `false` | `passthrough_odom_publisher`, raw pose in identical navigation axes, labeled `nav_odom` |
| `true` | `ekf_input_publisher` supplies root velocity/fixed heading; `ekf_node` fuses these with rf2o's `/scan_odom` position |

`true` is the default in both this file and `auto.launch.py` (since
2026-09-20), so the stack runs rf2o + the EKF unless you pass
`use_rf2o:=false`.

`use_rf2o:=true` also starts `rf2o_laser_odometry_node` on raw `/scan`. The
Thornbots fork re-queries `lidar->root` every scan, so the head-mounted lidar
can move.

Other args:

- `map_file` (default `map/clean_map`, no extension): `slam_toolbox` reads
  `<map_file>.posegraph/.data`, `amcl` reads `<map_file>.yaml`. The
  shipped `clean_map` is the [CAD-derived occupancy map](#saved-map). No pose graph ships:
  for `slam`, or `mapping` with `load_map:=true`, save one from a `mapping`
  run (`autosave_map`) and pass its `map_file`.
- `load_map` (default `true`): load the saved pose graph at startup
  (`slam`/`mapping` only).
- `odom_frame` (default `nav_odom`).
- `use_sim_time`: set by `auto.launch.py` from `real_hardware`.
- `autosave_map` (default `false`; `auto.launch.py` passes `real_hardware`):
  under `mapping`, `map_autosaver` saves `map.posegraph/.data`
  (`serialize_map`) and `map.pgm/.yaml` (written from `/map` itself, as
  nav2's map_saver would) into `map_save_dir` (`/workspaces/isaac_ros-dev/maps`,
  the bind-mounted workspace) `/<boot time>/`, every `map_save_period_s`
  (30 s). Not slam_toolbox's `save_map`: its `map_saver_cli` process must
  find `/map` within 2 s, and on the sentry mostly didn't. Load one back
  with `load_map:=true map_file:=<dir>/map`. Saved-map startup currently
  hard-codes `map_start_pose` to `[0, 0, 0]`; `initial_x`/`initial_y` seed
  EKF and, by default, AMCL only. Loading at a nonzero field spawn still needs
  [track H's carried-map work](../docs/ROADMAP.md#h-slam-at-amcls-level).

Nodes (C++, `src/`): `ekf_input_publisher`, `passthrough_odom_publisher`, `map_pose_publisher` (math in
`map_pose_core`) and `map_autosaver` (file writers in `map_image`). `thornbots_pkg/src/mcb_relay.cpp` compares
`/localization/odom` with raw `/odom` plus its last sent navigation offset.

## Testing

The drift and jerk suite is `ros2 launch sim localization_tests.launch.py`;
see `sim/README.md`.
It reads these `config/*.yaml` files from `install/`, so rebuild with
`--symlink-install` after editing them.

`colcon test --packages-select sentry_localization` runs the ament copyright,
flake8 and pep257 checks on `launch/`, plus gtest unit tests for
`map_pose_core` and `map_image`, and the standalone [field-map tests](#saved-map).

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
  publishes `map->nav_odom`.
- Each backend and its lifecycle manager share the C++ `localization_backend`
  process, constructed locally without a `load_node` service call. Local service endpoints share a DDS participant, so a configure
  response cannot race discovery of a remote client's response reader. Each
  component has its own executor; the manager can wait for a transition while
  the backend services it. This uses the existing transition and bond deadlines.
- rf2o used to cache its lidar->root transform from the first scan, which
  assumes a rigid mount. The fork (built by `isaac_ros_common`'s Dockerfile)
  re-queries every scan, so the old `head_home_scan_gate` and its
  `/scan_gated` topic are gone.
- rf2o runs with `fixed_heading: true` and `odom_prior_topic: /odom`. The
  `root` is heading-fixed (the chassis yaw is a joint below it), so rf2o's
  accumulated yaw error would only rotate its x/y. Its own velocity prior is the previous scan's motion, so every move
  from rest started with a prior of "stopped" and came up short: legs of
  x0.86-0.98 at 4 m/s in sim, only in some directions. Seeded with wheel
  odometry's motion between the two scans, every leg reads x1.00 and the
  EKF's mean error (0.050 m) beats raw `/odom` (0.134 m) on `suite:=ekf`,
  real time, `sentry_v2`.

### config/rf2o.yaml

rf2o grades every scan match good, degraded or failed, from its own
evidence only: whether it solved, the fraction of valid beams, the match's
own covariance (`cov_odo`), a 6 m/s speed limit and scan gaps. It never
compares against `/odom`, because on Bumpy Road the wheels slip and rf2o is
the one that's right. A scan whose lidar extrinsic is stale (the latest head
TF more than `max_extrinsic_age` older than the scan) is skipped like a
dropped scan: the head's swing since the last match is unknown, and
matching across it reads that swing as translation.

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
`ros2 run sim rf2o_quality record OUT` records it and
`ros2 run sim rf2o_quality summary FILE...` prints the distributions; see
[quality recording](../sim/README.md#more-on-the-tests).

Over the drift suite (2026-09-28, amcl, `--use-rf2o`, 2418 matches), every
match in the five clean scenarios graded good bar each stack's first scan.
Clean driving's worst: valid fraction 0.64, sigma 1.5 mm, 4.03 m/s,
extrinsic 0.04 s old. The rest landed where meant: jerks and `odom_stuck`
failed on speed (7-18 m/s), `scan_degraded`'s 60 deg arc failed on
`few_points` (0.10-0.17). Grading that arc degraded instead
(`fail_valid_fraction` 0.05) read 0.377 and 0.417 m during the blackout
against 0.415 and 0.434 m failed: within run-to-run spread, so it stays
failed.

Until 2026-09-28 any extrinsic not exactly at the scan's stamp was stale,
so 94% of clean matches graded degraded and the EKF leaned on `/odom`:
`suite:=ekf` fell from ~95% to 75-80% better than raw. The lidar rides the
head's yaw joint 0.22 m off its axis, and its TF comes from
`robot_state_publisher` at the 90 Hz POSE rate
([auto.launch.py](../thornbots_pkg/README.md#pose_translator)). Each match
is seeded with the head's swing between the two scans' extrinsics. Until
2026-10-22 the head TF was throttled to 20 Hz and the seed held the head
still, so a parked robot gained 0.1-0.4 m whenever the head slewed onto a
target; nothing corrects that while parked, since AMCL waits for
`update_min_d` of motion.

Two upstream bugs are fixed with it, in every mode: a failed match used to
republish the previous pose at full confidence, and a scan with too few
points on every pyramid level read as zero motion.

### config/amcl.yaml

**Tracked lidar mount.** nav2_amcl (Jazzy) caches each scan frame's
`root->lidar` x/y at its first scan and re-reads only the mount yaw per scan.
The head-mounted lidar sits about 0.22 m off root's yaw axis, so `localization_backend`
runs `TrackedMountAmcl`, which refreshes that x/y from TF at each scan stamp
before AMCL's own callback. `root` stays AMCL's base, so seeds and `/amcl_pose`
keep their meaning. Parked checks cannot see this bug, since parked AMCL adds no
motion noise; a head-sweeping drive can. Measurements are in the
[full-field investigation](../docs/investigations/fieldmap-divergence-2026-10-22.md#amcl-moving-mount).
It relies on AmclNode's protected members; recheck it after a nav2 upgrade.

`robot_model_type` is `nav2_amcl::OmniMotionModel`, with its strafe noise
term, since the chassis is holonomic (it translates without turning).

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
model. Sim's lidar matches the RPLIDAR A2M8's 720-bin grid with 0.01 m
noise ([sim README](../sim/README.md#lidar-format-and-model-limits)) and runs
on the real field mesh. 0.15 measured worse (see Tuning history), but on the
old noise model, and nothing has been re-measured since. Re-derive
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

- `odom0` (`/localization/ekf_input`): fixed root heading, vx, vy. The adapter
  preserves `/odom` capture stamps and root-frame twist, discards raw x/y,
  and defines identity heading directly in navigation axes. It requires no TF.
- `odom1` (`/scan_odom`, rf2o): x, y. It used to be gated to head-home
  windows; since the rf2o fix it streams continuously.

x/y from `/odom` stay out. Wheel slip (the "Bumpy Road" zone, sim's
`odom_slip_ratio`) is an error in integrated distance, and no covariance lets
`/scan_odom` pull the filter off the slipped position. On 2026-07-25, driving
~42m straight at 0.05 slip, EKF output tracked `/odom` within 0.001m while
both sat 2.02m behind truth. Encoders now supply velocity and `/scan_odom`
supplies position.

The heading measurement is identity because `root`
is heading-fixed: the physical chassis spins while driving
(`ARCC_2026_SENTRY_CONTEXT.md`, "Our Sentry's drivetrain"), but that yaw is
a joint below `root`. That identity is the true `root` heading. `robot_localization` rotates
body-frame velocity by the filter's yaw, so a wrong yaw integrates velocity in
the wrong direction. On 2026-07-25, taking yaw from rf2o instead (off by ~pi)
rotated the estimate 180 degrees: 2.97m mean error against raw odometry's
0.22m.

Directly fusing `/odom` yaw after separating `nav_odom` would transform that
measurement through `nav_odom->odom`, computed from this filter's output.
The adapter removes that circular dependency: only fixed navigation heading
and root-frame twist enter `odom0`. `nav_odom` keeps raw odometry's axes;
the correction TF changes translation only, matching the MCB navigation offset.
Historical measurements below use the frame names present during those runs.

`dynamic_process_noise_covariance: true`, with x/y `process_noise_covariance`
raised 0.05 to 0.08 (`8a719ab`), scales process noise with estimated
velocity. Isolated `noise_correction` runs went from ~40-50% passing to
~60-80%, at both 0.25 and 0.15 slip. Both full-suite attempts failed
(growth_ratio 2.17 and 2.49), and nobody knows why; load or leftover state
from earlier scenarios are candidates. Kept for the isolated gain. Confirm
`noise_correction` in the context you're reporting before calling it passing.

### config/slam.yaml

**base_frame: lidar** (was `root`). slam_toolbox 2.8.5 caches each scan frame's
mounting transform when it first sees that frame. Our lidar moves with the head
relative to heading-fixed `root`; a fixed cached mount interprets head motion as
robot motion. Using the sensor itself as SLAM's tracking base makes that cached
transform identity. The stamped `nav_odom->lidar` lookup carries the head's yaw
and offset on every scan; `map->nav_odom` still corrects the public root pose.
AMCL and rf2o keep their `root` base and refresh the moving mount each scan
([AMCL](#configamclyaml)). This does not compensate physical scan
tilt or height changes missing from the planar TF tree. See the
[full-field investigation](../docs/investigations/fieldmap-divergence-2026-10-22.md).
Start a new mapping session after this change; do not resume a serialized graph
created with the old root tracking base.

**minimum_travel_distance: 0.1** (was 0.5). A jerk (slip, bump, collision;
see `sim/src/pose_emulator.cpp`'s `trigger_jerk`) is invisible to wheel
odometry, so a slow or stopped robot never travelled 0.5m, slam_toolbox never
matched a new scan, and `map->odom` froze. Live check: a stationary
`trigger_jerk` got no update in 30s at 0.5, while `/cmd_vel` motion updated
normally. 0.1m is two cells, cheap enough and prompt.
`minimum_travel_heading: 0.05` also admits scans as the lidar/head rotates,
including a parked coverage sweep.

**min_laser_range: 0.15** (slam.yaml and amcl.yaml `laser_min_range`). The
RPLIDAR A2M8's minimum is 0.05 m (real scans report `range_min` 0.05), so 0.15
is a margin, not the hardware limit. It is kept because nothing proves the
self-filter covers the body: [`lidar_self_filter`](../thornbots_pkg/README.md#lidar_self_filter)
blanks only a CAD-derived 5-81 deg sector whose real zero direction is
unmeasured, the head-fixed parts start 0.045 m from the lidar, and the
2026-10-18 robot capture logged per-scan statistics, not per-beam ranges
(0.05-0.15 m returns were never inspected). Sim cannot test it: its lidar never
sees the robot. Lower it only after a real `/scan_raw` shows nothing of the
robot between 0.05 and 0.15 m.

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
Every amcl config spread 0.4-1.3m run to run. `kMaxDelta` (in
`sim/test/localization/localization_suite.cpp`) went from 0.30m to 0.40m, ~25% over
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

`sim/src/pose_emulator.cpp` adds only `(1 - slip_ratio)` of each true step to `/odom`,
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
- Bringup failures are distinct from tuning results. Check lifecycle activation
  and each link of `map -> nav_odom -> odom -> root` before interpreting an
  absent `map->root` as missing raw odometry. A discarded startup is not an
  accuracy sample.
- The metric is a max over a 45s drive, so one transient correction sets it.
  No process-noise knob can change that.
- Geometry changes invalidate baselines. The 0.1642m `amcl+ekf` result and
  the 0.20m threshold came from a 2m loop (corners ~1.80m from spawn), before
  `4f182e7` widened it to 3m (~2.12m).


### Initial field position

For direct launches, `initial_x/y` seed the navigation EKF in firmware odometry coordinates. AMCL
uses independent `amcl_initial_x/y/yaw`; x/y default to `initial_x/y`, yaw to
zero for compatibility. A measured field start belongs only in AMCL. Feeding
it to EKF can make `mcb_relay` reset firmware navigation to the wrong position.
Raw odometry and its constants remain unchanged. `root` has identity raw yaw;
chassis heading is the separate `chassis_yaw` joint. Field root yaw is measured
chassis heading minus the captured boot chassis heading, not chassis heading
itself. Team never mirrors coordinates implicitly.

On a different field, the measured start may differ from firmware `START_X` /
`START_Y`. The [operator workflow](../thornbots_pkg/README.md#field-day-and-room-workflow) enables
`seed_from_raw_odom:=true`, deriving the navigation seed from fresh raw odometry
before EKF starts; it does not accept a hand-entered seed. Register the map with
the measured physical pose. This changes only
map localization. It does not relocate raw CV aim, firmware routes or waypoints;
the team must match those assumptions separately before autonomous use.

## Broad-field coverage

Map overlapping views of both side lanes, the centre, and the far ends; return
through previously observed geometry and close the loop at the captured start.
A small central loop can look accurate while leaving much of the field unseen.
Inspect coverage and wall placement, including the backs of obstacles, before
deploying the map. Keep a rollback snapshot if a transfer distorts established
walls. The [full-field investigation](../docs/investigations/fieldmap-divergence-2026-10-22.md)
records the validated sim route, coverage, input choices and remaining limits.

Use level, parked observation stations around rough ground. Pause scan insertion
before a rough transfer, drive using the existing controls, stop on level ground,
and let the chassis settle before resuming. On the real robot, keep the head
centred and still while mapping: the Oct 25 rehearsal measured head-turn-dependent
map shifts and the lidar mount remains under investigation. Old workflow prompts suggesting a parked head
sweep are not hardware acceptance guidance. Both workflow modes launch the full
`auto.launch.py` graph (without YOLO), including CV aim/patrol by default. For
manual mapping/localization append `enable_cv_target_bridge:=false
 enable_target_selector:=false enable_target_tracker:=false patrol_enabled:=false`
(on one command line). These flags suppress Jetson head commands; they do not
lock the firmware gimbal. At pinned MCBV3 `8d46da5`, select controller left MID
(out of automatic aim/fire) and right MID (manual no-spin), centre with the
right stick and leave it neutral; verify the head stays still. Keep the controller
on and avoid keyboard/autonomous modes during acceptance. `patrol_enabled:=false`
alone still permits target aim. Draft workspace #100 removes patrol; recheck the
flags and firmware controls when adopting it. Fill coverage gaps by moving between
level stations with overlapping views, checking that scans stay on mapped walls.
Long unobserved transfers leave wheel slip for the next match to recover.
Reduced speed alone did not make the simulated rough crossings reliable.

`pause "$SESSION"` and `resume "$SESSION"` set slam_toolbox's scan insertion.
Its service only toggles, so each command reads `paused_new_measurements`, calls
the toggle only when needed and fails unless the parameter then matches; the
live mapping session must be this session. Pause before each rough transfer;
resume parked and settled, before observing or saving. This pauses mapping
updates, not the robot. It does not repair invalid scans or an already
distorted map. If the chassis cannot settle or scan/wall overlap cannot be
recovered, stop and remap from level stations.
Hardware still needs lidar geometry, extrinsics, wheel-slip and surveyed-heading
validation; the sim's ground-truth-derived raw input establishes no wheel accuracy.

## Saved map

`clean_map` is the canonical CAD-derived occupancy image for the corrected
2026-10-20 field export, not a SLAM map or serialized pose graph. Geometry
and frame placement come from [sim's world](../sim/README.md#the-world-frame).
Its 2 cm canvas, YAML origin and thresholds are unchanged. The field frame
is REP-105, centred at (0, 0), with x toward blue's base. Pixels
are 0 occupied, 254 free and 128 unknown, including unobserved exterior and
enclosed regions. The generator requires trinary mode (also the default when
omitted), negate=0 and thresholds that decode all three intensities with a
strict margin: 128 corresponds to occupancy 127/255, between this map's
free and occupied thresholds. YAML metadata stays unchanged.

`tools/field_map.py` continues the CAD-slice workflow from the held map draft,
reading collision poses/scales from the SDF instead of duplicating placement.
It slices at nominal lidar z = 0.3543931 m (estimated root height
approximately 0.0093 m plus URDF mount 0.3450931 m), flood fills the field
interior from (0, 0), keeps surfaces bordering that interior,
and grows occupied cells by one cell to cover raster discretization.
Plane intersections use a 1e-7 m tolerance to classify on-plane vertices and
deduplicate endpoints; noncoplanar intersections use the farthest distinct
endpoints. A single tangent vertex contributes no segment. A coplanar edge
contributes that edge; a fully coplanar face contributes its three edges,
not a filled horizontal surface. Segments are sampled at resolution/4,
which is approximate rasterization rather than an exact supercover. This
is a horizontal ideal-geometry slice; it does not model robot tilt, sensor
noise, dynamic obstacles or material returns. Confirm the export against
the physical field before relying on this map on hardware.

NumPy, SciPy, Pillow, PyYAML and pytest are declared test dependencies;
`rosdep install` supplies them for the existing colcon/CI test workflow.
No extra runtime dependency is needed to load the shipped PGM assets.
With those dependencies available, regenerate from this package:

```bash
python3 tools/field_map.py map/clean_map.yaml ../sim/world/ARCC_Field_2026.sdf map/clean_map.pgm --z 0.3543931 --grow 1
python3 -m pytest test/test_field_map.py
```

The registered `test_field_map` test always checks synthetic plane/mesh
fixtures, metadata constraints and independent YAML-based decoding of the
saved image; it runs in a standalone localization checkout without sim.
Optional actual-field bounds and image reproduction use an explicitly
supplied corrected-world fixture (including its package-relative meshes):

```bash
FIELD_MAP_WORLD=/path/to/sim/world/ARCC_Field_2026.sdf python3 -m pytest test/test_field_map_fixture.py
```

To register that additional test during a colcon build, pass
`--cmake-args -DSENTRY_FIELD_MAP_WORLD=/path/to/sim/world/ARCC_Field_2026.sdf`.
Without that option only the standalone suite is registered; direct pytest
collection reports the actual-field tests skipped when no fixture is given.
An explicitly provided missing or incompatible fixture fails the test/build.

The restored bar at y approximately +3 m now intersects the lidar slice.
This static CAD map has no measured runtime localization accuracy. It still
needs AMCL/scan agreement validation on the corrected world; prior-export
localization measurements do not validate it.
