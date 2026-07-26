# sentry_localization

Localization backends (SLAM/AMCL/EKF) for the Thornbots ARC 2026 Sentry
robot. Split out of `sentry_pkg` (which owns hardware drivers and the
robot description) — this package only ever consumes `/odom` + `/scan`
and produces the corrected pose, no direct hardware dependency. See
`sentry_pkg/README.md` for how the two packages fit together, and the
repo-level `SESSION_NOTES.md` / `ARCC_2026_SENTRY_CONTEXT.md` for the
broader project context.

## Node/topic contract with sentry_pkg

- Input: `/odom` (`nav_msgs/Odometry`, raw/uncorrected wheel odometry,
  published by `sentry_pkg`'s `pose_translator`) and `/scan`
  (`sensor_msgs/LaserScan`, from `sentry_pkg`'s lidar driver or `sim`).
- Output: **`/localization/odom`** (`nav_msgs/Odometry`), always — every
  `localization_mode` below publishes this topic, so `sentry_pkg`'s
  `odom_tf_broadcaster` (which turns it into `odom->root` TF) never needs
  to know which backend is active.
- `map->odom` TF (for the map-based backends) is broadcast directly by
  `slam_toolbox`/`amcl`, not indirected through `sentry_pkg`.

Normally you don't launch this package directly — `sentry_pkg/launch/
auto.launch.py` includes `launch/localization.launch.py` and forwards the
relevant args. Direct use (e.g. for testing) looks like:

```bash
ros2 launch sentry_localization localization.launch.py localization_mode:=amcl
```

### `localization_mode` — pick the map->odom owner (independent of `use_ekf`)

| value | map->odom owner | use case |
|---|---|---|
| `slam` (default) | `slam_toolbox` (localization mode) | normal running mode, localizing against the saved field map |
| `mapping` | `slam_toolbox` (mapping mode) | deliberately (re)building/extending the map |
| `amcl` | `nav2_amcl` + `nav2_map_server` | particle-filter localization against a saved occupancy grid |
| `none` | *(none — no map frame)* | no map layer at all (e.g. odometry-fusion-only running, paired with `use_ekf:=true`) |

### `use_ekf` — pick the odom->root source (independent of `localization_mode`)

| value | /localization/odom source | use case |
|---|---|---|
| `false` (default) | `passthrough_odom_publisher` (relays `/odom` unchanged) | trust raw wheel odometry |
| `true` | `ekf_node` (`robot_localization`, fuses `/odom` + `/scan_odom`, remapped output) | EKF-fused odometry, layered on top of any `localization_mode` above |

`use_ekf:=true` also launches `rf2o_laser_odometry_node` (`/scan_odom`),
consuming raw `/scan` directly — it re-queries its `lidar->root` transform
every scan (Thornbots/rf2o_laser_odometry fork), so it tolerates the
head-mounted lidar moving independently of the base. See `## Notes` below
for the full rationale and every other mode's exact node/TF-ownership
behavior.

The old map-free "ekf mode" configuration is now reached as
`localization_mode:=none use_ekf:=true`; any of `slam`/`amcl` combined
with `use_ekf:=true` is also a valid, launchable combination (a map
backend plus EKF-fused odometry underneath it).

### Other useful args

- `map_file` (default `map/clean_map`) — path (no extension) to the map:
  `slam_toolbox` reads `<map_file>.posegraph/.data`, `amcl` reads
  `<map_file>.yaml`. `map/ARCC26` is the real field map (has both); pass
  `map_file:=<pkg_share>/map/ARCC26` explicitly for `slam`/`mapping` since
  `clean_map` only has a `.yaml/.pgm` (no posegraph yet).
- `load_map` (default `true`) — deserialize `map_file`'s saved pose graph
  at startup instead of starting blank. Only meaningful for
  `slam`/`mapping`.
- `odom_frame` (default `odom`).
- `use_sim_time` — normally forwarded by `sentry_pkg/auto.launch.py`
  (derived from its `real_hardware` arg), not set directly.

## Nodes (`sentry_localization/`)

- `passthrough_odom_publisher.py` — relays `/odom` onto
  `/localization/odom` unchanged. Used whenever `use_ekf:=false`, any
  `localization_mode`.
- `simple_relocalize_publisher.py` — relocalization helper (see the file's
  docstring for specifics). The backend-agnostic drift-correction relay
  that used to live here as `slam_relocalize_publisher.py` (SLAM-specific,
  compared a `map->root` TF against the MCB's raw pose) has moved to
  `sentry_pkg/mcb_relay.py`, which compares `/localization/odom` (this
  package's one guaranteed output, any backend) against `/odom` directly
  instead of a TF lookup — see `sentry_pkg/README.md`.

## Testing

The localization drift/jerk-correction integration suite moved to
`sim/test/localization/run_localization_drift_tests.py` — see
`sim/README.md`'s Testing section for usage. It still launches this
package (via `sentry_pkg`'s `auto.launch.py`) and reads its
`config/*.yaml` files, so the same "rebuild after editing config" caveat
documented there applies to `sentry_localization` specifically.

Standard `colcon test`-style checks (`ament_copyright`/`ament_flake8`/
`ament_pep257`) also apply via the normal `colcon test --packages-select
sentry_localization`.

## Notes

Tuning history, rationale, and postmortems trimmed out of in-code comments
to keep those short. Kept here so a future tuner knows this history exists
before changing a value back to something already tried and rejected.

### `launch/localization.launch.py` — per-mode map/load_map/lifecycle detail

Beyond the `localization_mode`/`use_ekf` tables and rf2o summary above:

- `slam` with `load_map:=false` is not a meaningful combination — there's
  no map to localize against in that mode.
- `amcl` always loads `<map_file>.yaml` regardless of `load_map` — it has
  no concept of starting blank, unlike `slam_toolbox`.
- `use_map_saver` is only turned on in `mapping` mode — the map is only
  ever savable/updatable when you've deliberately opted into mapping,
  never as a side effect of ordinary localization/amcl/none running.
- `localization_mode:=none` runs no map node at all, so `load_map` has no
  effect there.
- `map_server` and `amcl` are nav2 lifecycle nodes, brought up by a
  `lifecycle_manager` node (`autostart:true`) rather than starting active
  on their own.
- rf2o used to only sample the lidar->root transform once, on its first
  received scan, and reuse that cached transform for its lifetime — it
  assumed a rigidly-fixed sensor mount, which the head-mounted lidar
  isn't. That's fixed in Thornbots/rf2o_laser_odometry (the fork
  `isaac_ros_common`'s Dockerfile builds), which re-queries the transform
  every scan instead, so the `head_home_scan_gate` workaround (only
  forwarding scans while the head was near home, via a filtered
  `/scan_gated` topic) is no longer needed and has been removed.

### `config/amcl.yaml` — alpha1-5 motion-noise tuning history

Stock value is 0.2 for alpha1-5. Robot is holonomic (translates in x/y
without rotating to move), so `robot_model_type` is set to
`nav2_amcl::OmniMotionModel` (adds a dedicated strafe noise term) rather
than the default `DifferentialMotionModel`.

Tried lowering all five to 0.05 on 2026-07-21 to fight `unmapped_obstacle`'s
~0.3-0.4m map->odom wobble (later diagnosed as motion-model noise fighting
this sim's near-perfect, `odom_noise_enabled=false` odometry, not an
obstacle-robustness issue — see git history). 0.05 crashed amcl outright,
reproducibly:

```
amcl: ./src/pf/pf_kdtree.c:363: pf_kdtree_cluster:
Assertion `node == pf_kdtree_find_node(self, self->root, node->key)' failed
```

The process dies with SIGABRT during startup Configuring/initTransforms,
before any driving even starts. This is a known class of nav2_amcl bug
where alpha values too low let resampling collapse particle diversity
enough to break the kd-tree clustering's node-uniqueness invariant.

Binary-searched back up from there to 0.0875 (the current value) — the
lowest step that didn't reproduce the crash across repeated startups. Also
validated against `continuous_drift` (real injected drift/jitter/slip) to
confirm this didn't cost genuine noise-responsiveness — still passes
cleanly (growth_ratio ~1.0-1.1).

If pushing lower again: step down gradually (e.g. 0.075, then 0.0625) and
check for the pf_kdtree crash at each step, not straight to an aggressive
value, AND validate against `continuous_drift` alongside
`unmapped_obstacle` so a fix for one doesn't regress the other.

### `config/amcl.yaml` — sigma_hit precision rationale

`sigma_hit: 0.08` (was stock 0.2) — this is the likelihood field's own
positional precision: at 0.2, any pose within ~0.2m of a true match looks
statistically similar to amcl (the likelihood gradient is close to flat
within ~1 sigma), which lines up almost exactly with `unmapped_obstacle`'s
observed ~0.2-0.3m map->odom noise floor. The alpha1-5/resample_interval/
min_particles knobs above all plateaued at that same ~0.3m regardless,
pointing at the sensor model's own precision rather than motion-model or
particle-count noise as the actual limiter. Sim's lidar was bumped to the
real RPLIDAR A2M8's ~3000-point resolution (see sim git history), so the
scan data itself should support a sharper match than stock's 0.2m assumes.

If retuning, re-derive rather than assuming 0.08 is final — validate
against `continuous_drift` (real injected noise) too, in case a sharper
likelihood field makes amcl less tolerant of genuine sensor noise on real
hardware.

### `config/amcl.yaml` — do_beamskip rationale

`do_beamskip: true` (was `false`, stock default) — enabled 2026-07-21,
nav2_amcl's built-in mechanism for excluding beams that disagree with the
map-predicted likelihood (e.g. an unmapped obstacle) from a scan's weight
update, rather than letting them drag the whole particle weighting toward
"unexpected obstacle" evidence.

Tried FIRST as the fix for `unmapped_obstacle`'s ~0.3-0.4m map->odom
wobble, and measurably did NOT help (0.409m after enabling vs.
0.395-0.443m before) — turned out the wobble wasn't obstacle-related at
all (the actual diagnosis and fix was the alpha1-5 motion-model noise
above). Left enabled anyway: it's still correct, low-risk defense for the
real dynamic obstacles the competition will actually produce (other
robots, thrown game pieces), just not what fixed this particular symptom.

### `config/amcl.yaml` — resample_interval rationale

`resample_interval: 2` (was 1) — with `update_min_d`/`update_min_a` this
tight (0.1m/0.05rad) and the robot at 4.0 m/s, nearly every incoming scan
triggers a filter update, so `resample_interval: 1` meant resampling on
nearly every scan too. Resampling introduces its own draw-to-draw variance
in the particle cloud's mean; at this update rate that variance itself was
a plausible contributor to `unmapped_obstacle`'s map->odom wobble during
sharp cornering (reported odometry is exact ground truth in that scenario,
so any map->odom movement there is purely amcl's own estimate noise, not
real error being corrected — see alpha1-5 notes above for the full
diagnosis). Resampling every other update halves how often that variance
gets injected without meaningfully slowing convergence at this scan rate.

### `config/amcl.yaml` — min_particles/max_particles rationale

`min_particles`/`max_particles: 1000/3000` (was 500/2000) is part of the
same wobble-diagnosis story as alpha1-5 and resample_interval above: more
particles means a lower-variance mean-pose estimate per update, which
should damp some of `unmapped_obstacle`'s map->odom wobble — the same
"amcl's own estimate noise, not real error" story, just a different knob.
Kept well short of 5000+: `max_beams=500` was already observed to fall
behind real-time, and particle count multiplies the same per-beam
likelihood evaluation cost, so this is a moderate step, not a maximal one.

### `config/ekf.yaml` — odometry sources and fusion strategy

First-pass EKF config — not final tuning. Covariance/process-noise numbers
are reasonable starting guesses, not measured/validated.

Fuses two odometry-shaped sources into odom->root:

- `odom0` (`/odom`, from `pose_translator`/wheel odometry): x, y, vx, vy
  only. Chassis is holonomic and never rotates during a match (fixed
  heading — see `ARCC_2026_SENTRY_CONTEXT.md`'s "Our Sentry's drivetrain"),
  so `/odom`'s orientation field is always identity and carries no
  information on its own — see below for why yaw is still fused from it.
- `odom1` (`/scan_odom`, from `rf2o_laser_odometry`): x, y, yaw. rf2o
  consumes raw `/scan` continuously — it used to be gated to only the
  windows where the head was near home (`head_home_scan_gate`), because
  rf2o cached the lidar->base transform once at startup and never
  re-queried it, which broke the moment the head-mounted lidar moved.
  That's fixed upstream now (see the launch.py notes above), so
  `/scan_odom` is a steady stream again, not bursty.

Yaw is fused from `/odom` despite the chassis's heading being physically
fixed, per user direction — the low yaw process noise still keeps yaw
tightly trusted overall, this just lets rf2o's yaw estimate contribute
continuously rather than being ignored.

### `config/ekf.yaml` — odom0_config velocity-only fusion rationale

`odom0_config` fuses velocity only from `/odom` — x/y are deliberately NOT
fused from this source.

Wheel odometry's dominant error mode here is slip (the arena's "Bumpy
Road" zone; modelled by sim's `odom_slip_ratio`), which is an error in
*integrated distance*. Fusing this source's absolute x/y makes that slip
part of the filter's absolute position estimate directly, and since it
accumulates monotonically no amount of covariance tuning lets `/scan_odom`
pull it back — the filter converges onto the slipped position and the
whole point of fusing a scan matcher is lost. Measured 2026-07-25: driving
~42m in a straight line with `odom_slip_ratio=0.05`, ekf output tracked
`/odom` to within 0.001m while both sat 2.02m (= 5% of 42m) behind ground
truth.

Fusing velocity instead lets wheel encoders do what they are good at
(smooth, high-rate short-term motion) while `/scan_odom` owns absolute
position, which is what actually corrects slip.

Yaw IS fused from this source, despite `/odom`'s orientation always being
identity. That identity is not "no information" — it is the true,
physically-guaranteed heading: the chassis is holonomic and never rotates
during a match. Pinning yaw here matters because robot_localization
rotates this source's body-frame velocity into the world using the
filter's yaw estimate, so a wrong yaw integrates velocity in the wrong
direction. Measured 2026-07-25 with yaw instead taken from `/scan_odom`:
rf2o's yaw was off by ~pi and the entire estimate came out as the
trajectory rotated 180 degrees (mean error 2.97m vs raw odometry's 0.22m).

`odom1_config` (from `/scan_odom`) fuses position only — yaw deliberately
NOT fused from rf2o, for the same reason: its yaw was measured off by
~pi, and the chassis heading is pinned by `odom0` anyway, so there is
nothing for rf2o's yaw to contribute except error.

### `config/slam.yaml` — minimum_travel_distance rationale

Was 0.5m/0.5rad. 0.5m is far too coarse for this robot: sudden external
position jerks (wheel slip on the arena's "Bumpy Road" zone,
bumps/collisions — see `sim/sim/pose_emulator.py`'s `trigger_jerk`, added
to model exactly this) are, by construction, invisible to wheel odometry
(that's the whole point of needing SLAM correction) — so if the robot is
stationary or moving slowly when one happens, "distance traveled since the
last processed scan" (as measured off reported `/odom`, which the jerk
deliberately leaves unchanged) never crosses a 0.5m threshold, and
slam_toolbox never even attempts a new scan match to notice the
discrepancy: map->odom stays frozen indefinitely.

Verified live in sim: a `trigger_jerk` call with the robot stationary
produced no map->odom update at all over a 30s window at the old 0.5
value; driving the robot for real (actual `/cmd_vel` motion, not a jerk)
did update map->odom normally, confirming the scan-matching pipeline
itself was never the problem, just this gate.

0.1m (2 resolution cells) still limits scan-matching frequency/cost
sensibly but lets slam_toolbox react promptly to a jerk even with little
or no real motion since the last scan. `minimum_travel_heading` is
irrelevant either way — this chassis is holonomic and never rotates its
heading — left small (0.05) rather than 0 to avoid relying on exact-zero
comparisons.
