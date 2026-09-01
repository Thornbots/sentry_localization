# sentry_localization

Localization backends (SLAM/AMCL/EKF) for the Thornbots ARC 2026 Sentry
robot. Split out of `sentry_pkg` (which owns hardware drivers and the
robot description). This package only consumes `/odom` + `/scan`
and produces the corrected pose, with no direct hardware dependency. See
`sentry_pkg/README.md` for how the two packages fit together, and the
repo-level `ARCC_2026_SENTRY_CONTEXT.md` for the broader project context.

## Node/topic contract with sentry_pkg

- Input: `/odom` (`nav_msgs/Odometry`, raw/uncorrected wheel odometry,
  published by `sentry_pkg`'s `pose_translator`) and `/scan`
  (`sensor_msgs/LaserScan`, from `sentry_pkg`'s lidar driver or `sim`).
- Output: **`/localization/odom`** (`nav_msgs/Odometry`), always: every
  `localization_mode` below publishes this topic, so `sentry_pkg`'s
  `odom_tf_broadcaster` (which turns it into `odom->root` TF) never needs
  to know which backend is active.
- `map->odom` TF (for the map-based backends) is broadcast directly by
  `slam_toolbox`/`amcl`, not indirected through `sentry_pkg`.

Normally you don't launch this package directly, since `sentry_pkg/launch/
auto.launch.py` includes `launch/localization.launch.py` and forwards the
relevant args. Direct use (e.g. for testing) looks like:

```bash
ros2 launch sentry_localization localization.launch.py localization_mode:=amcl
```

### `localization_mode`: pick the map->odom owner (independent of `use_ekf`)

| value | map->odom owner | use case |
|---|---|---|
| `slam` (default) | `slam_toolbox` (localization mode) | normal running mode, localizing against the saved field map |
| `mapping` | `slam_toolbox` (mapping mode) | deliberately (re)building/extending the map |
| `amcl` | `nav2_amcl` + `nav2_map_server` | particle-filter localization against a saved occupancy grid |
| `none` | *(none, no map frame)* | no map layer at all (e.g. odometry-fusion-only running, paired with `use_ekf:=true`) |

### `use_ekf`: pick the odom->root source (independent of `localization_mode`)

| value | /localization/odom source | use case |
|---|---|---|
| `false` (default) | `passthrough_odom_publisher` (relays `/odom` unchanged) | trust raw wheel odometry |
| `true` | `ekf_node` (`robot_localization`, fuses `/odom` + `/scan_odom`, remapped output) | EKF-fused odometry, layered on top of any `localization_mode` above |

`use_ekf:=true` also launches `rf2o_laser_odometry_node` (`/scan_odom`),
consuming raw `/scan` directly. It re-queries its `lidar->root` transform
every scan (Thornbots/rf2o_laser_odometry fork), so it tolerates the
head-mounted lidar moving independently of the base. See `## Notes` below
for the full rationale and every other mode's exact node/TF-ownership
behavior.

The old map-free "ekf mode" configuration is now reached as
`localization_mode:=none use_ekf:=true`; any of `slam`/`amcl` combined
with `use_ekf:=true` is also a valid, launchable combination (a map
backend plus EKF-fused odometry underneath it).

### Other useful args

- `map_file` (default `map/clean_map`): path without extension.
  `slam_toolbox` reads `<map_file>.posegraph/.data`, `amcl` reads
  `<map_file>.yaml`. `map/ARCC26` is the real field map and has both; pass
  it explicitly for `slam`/`mapping`, since `clean_map` has no posegraph.
- `load_map` (default `true`): deserialize the saved pose graph at startup
  instead of starting blank. Only meaningful for `slam`/`mapping`.
- `odom_frame` (default `odom`).
- `use_sim_time`: normally forwarded by `sentry_pkg/auto.launch.py`
  (derived from its `real_hardware` arg), not set directly.

## Nodes (`sentry_localization/`)

`passthrough_odom_publisher.py` relays `/odom` onto `/localization/odom`
unchanged, whenever `use_ekf:=false`, any `localization_mode`. That's the
only node here.

The drift-correction relay that used to live here as
`slam_relocalize_publisher.py` (SLAM-specific, compared a `map->root` TF
against the MCB's raw pose) moved to `sentry_pkg/mcb_relay.py`, which
compares `/localization/odom` against `/odom` directly instead of doing a
TF lookup.

## Testing

The drift/jerk-correction integration suite lives in
`sim/test/localization/run_localization_drift_tests.py`; see
`sim/README.md`'s Testing section. It launches this package via
`sentry_pkg`'s `auto.launch.py` and reads these `config/*.yaml` files, so
the "rebuild after editing config" caveat documented there applies here
too.

`colcon test --packages-select sentry_localization` runs the standard
`ament_copyright`/`ament_flake8`/`ament_pep257` checks.

## Notes

Rationale kept out of in-code comments to keep those short. A future tuner
should read this before changing a value back to something already tried
and rejected.

### `launch/localization.launch.py`: per-mode detail

- `slam` with `load_map:=false` isn't meaningful; there's no map to
  localize against.
- `amcl` always loads `<map_file>.yaml` regardless of `load_map`. Unlike
  `slam_toolbox` it has no concept of starting blank.
- `use_map_saver` is on only in `mapping` mode, so the map is savable only
  when you've deliberately opted into mapping, never as a side effect of
  ordinary running.
- `localization_mode:=none` runs no map node, so `load_map` does nothing.
- `map_server` and `amcl` are nav2 lifecycle nodes, brought up by a
  `lifecycle_manager` (`autostart:true`) rather than starting active.
- rf2o used to sample its lidar->root transform once, on its first scan,
  and reuse it for its lifetime, which assumes a rigid sensor mount the
  head-mounted lidar isn't. Fixed in the Thornbots/rf2o_laser_odometry
  fork that `isaac_ros_common`'s Dockerfile builds, which re-queries every
  scan. The old `head_home_scan_gate` workaround (forwarding scans only
  while the head was near home, via a filtered `/scan_gated` topic) is
  gone with it.

### `config/amcl.yaml`: alpha1-5 motion noise

Stock is 0.2. The robot is holonomic, so `robot_model_type` is
`nav2_amcl::OmniMotionModel` (dedicated strafe noise term) rather than the
default `DifferentialMotionModel`.

Lowering all five to 0.05 on 2026-07-21, to fight `unmapped_obstacle`'s
~0.3-0.4m map->odom wobble, crashed amcl outright and reproducibly:

```
amcl: ./src/pf/pf_kdtree.c:363: pf_kdtree_cluster:
Assertion `node == pf_kdtree_find_node(self, self->root, node->key)' failed
```

SIGABRT during startup Configuring/initTransforms, before any driving.
This is a known nav2_amcl failure class: alphas low enough let resampling
collapse particle diversity past the kd-tree clustering's node-uniqueness
invariant. Binary-searched back up to **0.0875**, the lowest step that
didn't reproduce it across repeated startups, then validated against
`continuous_drift` (real injected drift/jitter/slip) to confirm it hadn't
cost genuine noise-responsiveness (growth_ratio ~1.0-1.1).

The wobble itself turned out to be motion-model noise fighting this sim's
near-perfect `odom_noise_enabled=false` odometry, not an
obstacle-robustness problem.

If pushing lower again: step down gradually (0.075, then 0.0625),
checking for the pf_kdtree crash at each step, and validate against
`continuous_drift` alongside `unmapped_obstacle` so a fix for one doesn't
regress the other.

### `config/amcl.yaml`: sigma_hit

`sigma_hit: 0.08` (stock 0.2) is the likelihood field's own positional
precision. At 0.2 any pose within ~0.2m of a true match looks
statistically similar to amcl, since the likelihood gradient is nearly
flat within ~1 sigma, which lines up almost exactly with
`unmapped_obstacle`'s observed ~0.2-0.3m noise floor. The
alpha1-5/resample_interval/min_particles knobs all plateaued at that same
~0.3m, pointing at the sensor model's precision rather than motion-model
or particle-count noise as the limiter. Sim's lidar was bumped to the real
RPLIDAR A2M8's ~3000-point resolution (see sim git history), so the scan
data should support a sharper match than stock assumes.

If retuning, re-derive rather than assuming 0.08 is final, and validate
against `continuous_drift` too, in case a sharper likelihood field makes
amcl less tolerant of genuine sensor noise on hardware. Loosening it to
0.15 was tried later and measured worse; see Tuning history below.

### `config/amcl.yaml`: do_beamskip

`do_beamskip: true` (stock `false`), enabled 2026-07-21, excludes beams
that disagree with the map-predicted likelihood from a scan's weight
update, rather than letting them drag the whole particle weighting toward
"unexpected obstacle" evidence.

Tried first as the fix for the `unmapped_obstacle` wobble and measurably did
not help (0.409m after, vs 0.395-0.443m before), because the wobble wasn't
obstacle-related at all. Kept anyway as a correct, low-risk defense against
the real dynamic obstacles a competition produces.

### `config/amcl.yaml`: resample_interval and particle counts

`resample_interval: 2` (was 1). With `update_min_d`/`update_min_a` this
tight (0.1m/0.05rad) and the robot at 4.0 m/s, nearly every scan triggers
a filter update, so 1 meant resampling nearly every scan. Resampling
injects its own draw-to-draw variance into the particle cloud's mean;
resampling every other update halves how often that lands without
meaningfully slowing convergence at this scan rate.

`min_particles`/`max_particles: 1000/3000` (was 500/2000) is the same
story on a different knob: more particles, lower-variance mean pose per
update. Kept well short of 5000+, since `max_beams=500` was already
observed to fall behind real-time and particle count multiplies the same
per-beam cost.

Both target amcl's *own* estimate noise. In `unmapped_obstacle` the
reported odometry is exact ground truth, so any map->odom movement there
is amcl's own noise rather than real error being corrected.

### `config/ekf.yaml`: sources and fusion strategy

Nobody has measured the covariance or process-noise numbers here. They are
reasonable starting guesses for a first pass.

- `odom0` (`/odom`, wheel odometry): x, y, vx, vy. The chassis is
  holonomic and never rotates during a match (see
  `ARCC_2026_SENTRY_CONTEXT.md`'s "Our Sentry's drivetrain"), so its
  orientation field is always identity.
- `odom1` (`/scan_odom`, rf2o): x, y, yaw, consuming raw `/scan`
  continuously. It used to be gated to windows where the head was near
  home, because rf2o cached the lidar->base transform at startup; fixed
  upstream, so this is a steady stream now.

The config deliberately leaves x/y out of the `/odom` fusion. Wheel
odometry's dominant error here is slip (the arena's "Bumpy Road" zone,
modelled by sim's `odom_slip_ratio`), an error in integrated distance that
accumulates monotonically, so no covariance tuning lets `/scan_odom` pull it
back: the filter converges onto the slipped position and the point of fusing
a scan matcher is lost. Measured 2026-07-25, driving ~42m straight at
`odom_slip_ratio=0.05`, ekf output tracked `/odom` to within 0.001m while
both sat 2.02m (5% of 42m) behind ground truth. Fusing velocity instead lets
encoders do what they're good at (smooth, high-rate short-term motion) while
`/scan_odom` owns absolute position.

Yaw is still fused from `/odom` despite that identity orientation, because
identity here carries information: it is the physically guaranteed heading.
robot_localization rotates this source's body-frame velocity into
the world using the filter's yaw estimate, so a wrong yaw integrates
velocity in the wrong direction. Measured 2026-07-25 with yaw taken from
`/scan_odom` instead: rf2o's yaw was off by ~pi and the whole estimate
came out rotated 180 degrees (mean error 2.97m vs raw odometry's 0.22m).
`odom1_config` fuses position only for the same reason.

### `config/slam.yaml`: minimum_travel_distance

Was 0.5m/0.5rad, now 0.1m. 0.5m is far too coarse for this robot. Sudden
external position jerks (wheel slip, bumps, collisions; see
`sim/sim/pose_emulator.py`'s `trigger_jerk`) are by construction invisible
to wheel odometry, which is the whole reason SLAM correction is needed. So
if the robot is stationary or slow when one happens, distance travelled
since the last processed scan never crosses 0.5m, slam_toolbox never
attempts a new scan match, and map->odom stays frozen indefinitely.

Verified live: a `trigger_jerk` with the robot stationary produced no
map->odom update at all over 30s at the old value, while real `/cmd_vel`
motion did update it normally. The scan-matching pipeline was never the
problem, only this gate.

0.1m (2 resolution cells) still bounds scan-matching cost sensibly while
letting slam_toolbox react promptly with little or no real motion.
`minimum_travel_heading` is irrelevant on a chassis that never rotates;
left at 0.05 rather than 0 to avoid relying on exact-zero comparisons.

## Tuning history (2026-07-26/27)

On 2026-07-26 and 07-27 we searched for a backend and config that passes
`drift_correction`/`drift_correction_obstacle` in
`sim/test/localization/run_localization_drift_tests.py`. What follows is the
conclusions; the run-by-run logs are in this file's git history.

### Outcome: tuned `--backend slam`, no EKF

`config/slam.yaml` with `correlation_search_space_dimension: 2.5` (was
0.5) and `minimum_time_interval: 1.0` (was 0.5), committed `f23c6be`.
Widening the search window is what mattered: at `odom_slip_ratio=0.25` the
raw odometry the matcher searches around can sit ~0.53m off true pose at
the loop's far corners, so a +-0.25m window cannot see the correct
alignment at all. The longer interval gives fewer, higher-baseline matches
during the fast cornering legs; lowering it to 0.1 instead measured worse
(0.89m).

Re-measured at the current `odom_slip_ratio=0.15`, 3 clean full-suite
runs: `drift_correction` 0.3125/0.3261/0.3224m,
`drift_correction_obstacle` 0.3150/0.3002/0.3194m. That 0.30-0.33m band is
tight, unlike the 0.4-1.3m run-to-run spread every amcl config produced.
`MAX_DELTA_THRESHOLD` was raised 0.30m -> 0.40m off those numbers (~25%
margin over the worst sample) in
`sim/test/localization/run_localization_drift_tests.py`. All 5 scenarios
then pass reliably.

`auto.launch.py` defaults to `localization_mode:=amcl` rather than `slam`
(set 2026-08-29, after this measurement). Pass `localization_mode:=slam`
explicitly to get the configuration this section measured.

### Closed levers

Tried, measured, reverted. Each was tested in both directions where that
made sense; don't re-open one without new information.

| knob | tried | result |
|---|---|---|
| `amcl.yaml` alpha1-5 | 0.2 (alpha3/5), 0.07 (all five) | 0.51m / 0.47m vs 0.43-0.44m baseline |
| `amcl.yaml` min/max_particles | 2000/5000 | 0.45m, no better |
| `amcl.yaml` sigma_hit | 0.08 -> 0.15 | 1.12m, worse |
| `ekf.yaml` x/y `process_noise_covariance` | 0.05 -> 0.02, and -> 0.1 | spreads overlap entirely, 0.40-1.00m at the same config |
| `slam.yaml` distance/minimum_distance_penalty | 0.5 -> 1.0/0.8 | no gain over the search-space change alone |
| `slam.yaml` minimum_time_interval | 0.5 -> 0.1 | 0.89m, worse |

`amcl.yaml` was re-checked under the later, looser conditions
(`odom_slip_ratio` 0.25->0.15, threshold 0.20m->0.30m) in case the earlier
verdict was an artifact of the harsher setup. It held: the threshold
moved, the mechanism producing the failure didn't.

### Why the map-based drift scenarios are hard

`sim/sim/pose_emulator.py`'s slip model integrates only `(1 - slip_ratio)`
of each true incremental displacement into reported `/odom`, so
`odom_error(t) = slip_ratio * |true_pos(t) - true_pos(t0)|`: a
displacement-from-start error, not a path-length one. `t0` is the loop's
spawn point and `OBSTACLE_LOOP_LEGS`'s corners reach 2.12m from it, so at
0.25 slip the map->odom correction being measured has to move ~0.53m at
the far corners, over 2.5x the then-current 0.20m threshold. Raw `amcl`
has no absolute-position input other than its own scan match, so it cannot
get under that. Tracking fidelity and a bounded map->odom are in direct
conflict here: the variants that tracked true error more faithfully landed
*closer* to the 0.53m maximum.

Two findings narrow where the remaining error lives:

- rf2o is not the problem. Bagged 50s mid-run: `/odom` moved +2.272m
  in x where true displacement was ~3.03m, while `/scan_odom` and
  `/localization/odom` both moved +2.932m, within ~3% of truth. The
  velocity-from-`/odom`, position-from-`/scan_odom` split is correcting
  slip roughly as designed.
- The error ramps rather than spiking. Per-sample `map->odom` traces
  climb monotonically from ~0.04-0.08m at t=2s to 0.65-0.76m at t=45s,
  with the max landing on the final sample at loop-close. A
  displacement-from-start slip term should fall back as the robot returns
  toward its start corner; this doesn't. That points at path-length
  accumulated drift in the fused estimate, upstream of `amcl`, rather than
  particle-filter noise a motion-model knob could damp.

### `--backend slam --use-ekf`: worse, don't use

Markedly worse than plain `--backend slam` measured back to back on a quiet
host:
`drift_correction` 1.0659m vs 0.5066m, `drift_correction_obstacle` 0.8652m
vs 0.4876m, and `noise_correction` newly failing (growth_ratio 2.42).
Mechanism, by inspection rather than bag-level confirmation:
`slam_toolbox` computes its scan-match correction relative to its
`odom_frame` TF, which here *is* the EKF's fused output, so the EKF's
rf2o-derived absolute-position correction and `slam_toolbox`'s own stack
on top of each other against the same lidar data instead of one deferring
to the other.

### `ekf.yaml`: `dynamic_process_noise_covariance`, kept with a caveat

`dynamic_process_noise_covariance: true` plus x/y
`process_noise_covariance` 0.05 -> 0.08 (commit `8a719ab`) scales process
noise by the filter's own velocity estimate: more injected noise while
moving, less near zero. Isolated `noise_correction` runs improved from a
~40-50% pass rate to ~60-80%, consistently at both 0.25 and 0.15 slip, so
it isn't just riding the slip reduction.

Full-suite runs FAILed both times tried, though (growth_ratio 2.17 and
2.49), worse than nearly every isolated run. Nobody explained the
isolated-vs-suite gap; it may be cumulative load or residual state from the
four scenarios that run first. The setting is kept as a real improvement in
isolation. Don't report `noise_correction` as passing without re-confirming
in whichever context the claim is about.

### Measurement caveats

- Host load corrupts these numbers. One batch of runs went out alongside
  ~18 concurrent agent sessions (load average 4-8 of 22 CPUs) and produced
  `--use-ekf` readings that couldn't separate a real config effect from
  contention. Check `uptime`/`ps aux --sort=-%cpu` before trusting any
  number near threshold.
- A bringup race looks like a config failure. `amcl/get_state`
  `async_send_request failed`, and `map_server` heartbeat timeouts ("IS
  DOWN after not receiving a heartbeat for 4000 ms") cascading to a stack
  crash, both appear under elevated load. Re-run once the host settles.
- The metric is a max over a 45s drive, so a single transient correction
  sets the whole number. That sensitivity belongs to the scenario rather
  than to any knob process noise can reach.
- Baselines go stale when the test geometry moves. A 0.1642m
  `amcl+ekf` result, and the 0.20m threshold derived from it, were both
  measured on a 2m loop (max corner ~1.80m from spawn) before `4f182e7`
  widened it to 3m (~2.12m). Re-derive a baseline after any geometry
  change before judging a knob against it.