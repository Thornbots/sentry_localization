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

### 2026-07-26 — `drift_correction`/`drift_correction_obstacle` under
`odom_slip_ratio=0.25`: structural bound for `--backend amcl`, and a
host-load confound that blocked confirming `--backend amcl --use-ekf`

Attempted to tune `amcl.yaml`/`ekf.yaml` to pass both
`drift_correction`/`drift_correction_obstacle` (`MAX_DELTA_THRESHOLD=0.20m`)
under the new `odom_slip_ratio=0.25` default, for both `--backend amcl`
and `--backend amcl --use-ekf`. Net result: no YAML change (both files
are unchanged from before this session) — every tuning direction tried
either made things worse or made no measurable difference, and a
structural argument (below) says raw `amcl` likely cannot pass this pair
at all under 25% slip, regardless of `amcl.yaml` tuning.

**The structural bound (`--backend amcl`, no EKF).** `sim/sim/
pose_emulator.py`'s slip model integrates only `(1 - slip_ratio)` of each
true incremental displacement into reported `/odom`, which solves to
`odom_error(t) = slip_ratio * |true_pos(t) - true_pos(t0)|` — a
displacement-from-start error, not a path-length one. `t0` is effectively
the loop's own spawn point (`(0,0)`, same as `OBSTACLE_XY`), and
`OBSTACLE_LOOP_LEGS`'s corners sit up to `sqrt(1.5^2+1.5^2)=2.12m` from
there, so the raw odom-frame divergence that `amcl`'s own map->odom
correction must track to stay accurate reaches `0.25*2.12 ≈ 0.53m` at the
far corners — over 2.5x `MAX_DELTA_THRESHOLD`. Since raw `amcl` (no EKF)
has no other absolute-position input than its own lidar scan match, its
map->odom TF necessarily has to move by close to that much to remain
correct; the test measures exactly that movement. Confirmed empirically:
every `drift_correction`(`_obstacle`) measurement taken this session
across every `amcl.yaml` variant tried, on `--backend amcl`, landed at
0.42–0.99m — never once near 0.20m, and instructively, the variants that
made `amcl` track the true error *more* faithfully (raising `alpha3`/
`alpha5` to 0.2) produced deltas *closer to* the 0.53m theoretical max
(0.51m), while the variants that made it lag/undercorrect (`alpha`
lowered to 0.07, or more particles) gave smaller-but-still-failing deltas
(0.45–0.47m). Tracking fidelity and a bounded map->odom are in direct
conflict here — this isn't a knob `amcl.yaml` has.

Tried (all single runs — see host-load caveat below before trusting exact
numbers, though the qualitative pattern held up over 8 varied runs):
alpha1-5 raised to 0.2 (`alpha3`/`alpha5` only) → 0.5121m; alpha1-5
lowered to 0.07 (all five) → 0.4692m; `min_particles`/`max_particles`
raised to 2000/5000 (alphas at documented 0.0875) → 0.4502m. None beat
the documented-baseline 0.0875/1000/3000 (0.4259–0.4447m across repeats).
All reverted; `amcl.yaml` is unchanged from before this session.

**`--backend amcl --use-ekf`: could not get a trustworthy read this
session.** The session's stated prior baseline was `drift_correction`
0.1991m (PASS) and `drift_correction_obstacle` 0.2660m (FAIL, just over).
Tried lowering `ekf.yaml`'s x/y `process_noise_covariance` (0.05→0.02,
smoother/more-trusted prediction) and raising it (0.05→0.1, faster
response) — both made `drift_correction_obstacle` much worse (0.5986m
and 0.5953m respectively), not better in either direction. Reverted to
confirm the baseline itself: three consecutive **unmodified**-config runs
of `drift_correction_obstacle` gave a lifecycle-manager bringup race
(`amcl/get_state` `async_send_request failed`, not a config issue —
confirmed via `sentry_pkg.log`, teardown-time SIGINT only, `rf2o`/`ekf`
alive and streaming the whole window), then 0.5188m, then 0.6798m — none
close to the documented 0.2660m, all with `ekf_filter_node`/`rf2o`
confirmed healthy throughout (continuous `Laser odom` lines to the very
end of each run, no crash before teardown). A final full-suite confirmation
run made it worse still: `noise_correction` (previously a trivial PASS)
FAILed (growth_ratio 2.14, then a full TF timeout on the run before that),
`drift_correction` hit 0.9908m.

Root cause: this host was running **~18 concurrent Claude Code sessions**
during this work (`ps aux --sort=-%cpu`, `uptime` load average 4–8 out of
22 CPUs, rising mid-run every time it was checked) — exactly the
"heavy unrelated CPU load ... can slow scan-matching enough to make
timing-sensitive scenarios look like false failures" case the test
script's own `check_no_orphans` warning describes, just far more
severe/persistent than that warning anticipates. This makes every
number collected this session for `--use-ekf` unreliable as a signal for
telling a real config effect from load-induced degradation — a change
that looks like it helped or hurt could just as easily be the difference
between running alongside 4 other agents vs. 12. No YAML change from this
session was kept for `ekf.yaml` either; it is unchanged from before.

**For the next person tuning this**: re-run the suite when this host is
otherwise idle (`uptime`/`ps aux --sort=-%cpu` first) before trusting any
`--use-ekf` number close to the 0.20m threshold, and before concluding a
knob helped or hurt on `drift_correction`(`_obstacle`) specifically — the
signal is currently smaller than the noise this host can inject. The
`--backend amcl` (no EKF) structural bound above doesn't depend on host
load and should hold regardless: getting under 0.20m there would need
either a smaller `odom_slip_ratio` (not this test's call to make) or an
absolute-position correction upstream of `amcl`'s own map->odom (i.e.
what `--use-ekf` already tries to be) — not a further `amcl.yaml` knob.

### 2026-07-26 (later same day) — `--backend amcl --use-ekf` full 5-scenario
attempt: `process_noise_covariance` x/y closed as a lever; the loop
geometry moved out from under the old 0.1642m/0.20m calibration

Focused entirely on `--backend amcl --use-ekf`, tuning only `ekf.yaml`
(no `amcl.yaml`/launch/threshold changes, per this session's scope). Net
result: **no config change kept** — `ekf.yaml` is byte-identical to
before this session (`git diff` empty). Final confirmed state:

| scenario | result |
|---|---|
| `baseline` | PASS (0.0000m) |
| `noise_correction` | FAIL (growth_ratio 2.23, threshold 2.0) |
| `drift_correction` | FAIL (0.8038m, threshold 0.20m) |
| `drift_correction_obstacle` | FAIL (0.5917m, threshold 0.20m) |
| `jerk_with_motion` | PASS (8/8 trials) |

**First finding: the 0.1642m EKF baseline this morning's session cited
was measured against a loop geometry that no longer exists.** Per git
history on `sim`: `8c35cc6` (measured `amcl+ekf` at 0.1642m,
`MAX_DELTA_THRESHOLD=0.30m`) and `1e63c66` (tightened the threshold to
0.20m, citing that same 0.1642m) both predate `4f182e7`
("Recenter drift test's cornering loop on origin, widen to 3m"), which
widened `OBSTACLE_LOOP_LEGS` from a 2m square (max corner distance from
spawn ~1.80m) to the current 3m square (max corner distance ~2.12m — the
figure the earlier session's `0.25*2.12≈0.53m` structural-bound math
already uses for `--backend amcl`). So the 0.1642m PASS and the 0.20m
threshold were both calibrated on the smaller, since-abandoned loop;
today's 3m loop is a genuinely harder test that EKF's 0.1642m result was
never actually measured against. This isn't captured by "host load noise"
and isn't a `ekf.yaml` knob — it's a test-geometry/threshold staleness gap,
out of scope for this session to fix (`run_localization_drift_tests.py`/
thresholds were explicitly off-limits).

**Second finding: rf2o (`/scan_odom`) is not the problem — it's the one
input tracking ground truth.** Recorded a 50s `ros2 bag` of `/odom`,
`/scan_odom`, `/localization/odom` mid-`drift_correction`-run to check
whether rf2o's absolute x/y (fused directly into the EKF per
`odom1_config`, see the design-rationale entry above) might be reversed
or drifting independently, per a hypothesis that its position-only fusion
could be inheriting scan-matching error wholesale. It isn't: over that
window `/odom` moved +2.272m in x while true displacement under
`odom_slip_ratio=0.25` is `2.272/0.75 ≈ 3.03m`; `/scan_odom` (and
`/localization/odom`, which tracks it almost exactly since x/y comes
from rf2o) moved +2.932m — within ~3% of the true 3.03m. rf2o is doing
its job; the fusion design (velocity from `/odom`, position from
`/scan_odom`) is correcting slip roughly as intended. The elevated
`drift_correction`(`_obstacle`) numbers aren't caused by a bad rf2o
input.

**Third finding: `process_noise_covariance` x/y (indices [0][0]/[1][1])
is now closed as a tuning lever**, having been tested in both directions
across two sessions with no reliable signal against this scenario's own
run-to-run noise floor:

| x/y process noise | `drift_correction` runs (this session) |
|---|---|
| 0.05 (documented default) | 0.9972, 0.9135, 0.3999 m (3 runs) |
| 0.02 (this session) | 0.9980, 0.6108 m (2 runs) |
| 0.02 (prior session, heavy load) | 0.5986 m (1 run) |
| 0.1 (prior session, heavy load) | 0.5953 m (1 run) |

The 0.02 and 0.05 ranges overlap entirely (0.40–1.00m spread at the
*same* config), and pre-run host load did not correlate with the result
across these five same-session measurements (e.g. lowest load 0.58
preceded the worst result, 0.9972m) — this run-to-run spread looks
load-independent this time, not the contention story from the prior
session. Most likely driver: the test reports a **max** over a 45s drive,
so a single amcl correction update dominates the whole number; that's a
scenario/threshold sensitivity issue, not something `process_noise_covariance`
can smooth out. Reverted; `ekf.yaml` unchanged.

**Net assessment**: `--backend amcl --use-ekf` does not currently pass
`noise_correction`/`drift_correction`/`drift_correction_obstacle`
reliably. Unlike the raw-`amcl` case, there's no clean structural
argument that it *can't* — rf2o measurably supplies a real, ~97%-accurate
absolute-position correction — but two closed tuning directions
(`process_noise_covariance` x/y, both signs) and a stale
threshold/geometry calibration leave no remaining `ekf.yaml`-only lever
found this session. Next steps needing scope outside `ekf.yaml`
(explicitly not done here): re-derive `MAX_DELTA_THRESHOLD`/re-measure a
clean baseline against the current 3m loop before any further `ekf.yaml`
tuning judges itself against a number that may not be achievable on this
geometry; and/or reduce the test's max-over-window metric's sensitivity
to a single transient correction.

### 2026-07-27 — `--backend slam` tuned to a meaningfully better (but still
failing) `drift_correction`/`drift_correction_obstacle`; `--backend slam
--use-ekf` measured worse across the board

Followed up on the two amcl/amcl+ekf sessions above by trying
`slam_toolbox` itself (`--backend slam`, `localization_mode:=slam` against
`map/ARCC26`) as the map->odom correction source, tuning only
`config/slam.yaml`, under the same `odom_slip_ratio=0.25`/3m-loop
conditions.

**Untuned `--backend slam` baseline** (before this session's edits):
`baseline`/`noise_correction`/`jerk_with_motion` PASS,
`drift_correction` 0.8558m FAIL, `drift_correction_obstacle` 0.5485m FAIL
(threshold 0.20m both).

**What worked: widening `correlation_search_space_dimension` (0.5 -> 2.5)
and raising `minimum_time_interval` (0.5 -> 1.0)**. Rationale: at
`odom_slip_ratio=0.25` the raw odometry the matcher searches around can
be off from true pose by up to ~0.53m at the loop's far corners (same
structural-bound math as the amcl session above) — a 0.5m-wide (+-0.25m)
correlation search window can't even see the true alignment to consider
it as a candidate. Widening to 2.5 (+-1.25m) fixed that. Iterating on
`drift_correction` alone: dimension alone (1.5) got 0.55-0.58m (two runs);
dimension 2.5 got 0.50-0.59m (three runs, no further gain past 1.5, so
2.5 mostly just adds margin, not signal); lowering `minimum_time_interval`
to 0.1 (more frequent, smaller-baseline matches) made it measurably
*worse* (0.89m) — reverted; raising it to 1.0 instead (fewer,
higher-baseline matches during the fast cornering legs) gave the best
single measurement, 0.50m. Also tried loosening
`distance_variance_penalty`/`minimum_distance_penalty` (0.5->1.0/0.8,
meant to stop the match score from penalizing the correct-but-far-from-
odom alignment) — no improvement over the search-space+interval change
alone (0.59m vs 0.50m, within this scenario's own run-to-run spread) —
reverted, kept only the two changes above.

**Confirmed full-suite result, `--backend slam` (tuned, clean/idle
container run)**:

| scenario | result |
|---|---|
| `baseline` | PASS (0.0075m) |
| `noise_correction` | PASS (growth_ratio 0.74) |
| `drift_correction` | FAIL (0.5066m, threshold 0.20m) |
| `drift_correction_obstacle` | FAIL (0.4876m, threshold 0.20m) |
| `jerk_with_motion` | PASS (8/8 trials) |

A meaningful, repeatable improvement over the untuned baseline
(0.86m/0.55m -> ~0.49-0.51m, roughly halved on `drift_correction`), and
comparable to or slightly better than the best numbers either `amcl`
session above ever got (raw `amcl` 0.42-0.99m; `amcl+ekf` best single run
0.40m, typically 0.6-1.0m) — but still ~2.5x over `MAX_DELTA_THRESHOLD`.
This looks like the same structural floor as the amcl case: the test
samples the correction only once, at the end of each leg's 1.0s
stationary dwell, and `slam_toolbox`'s distance/heading travel-gating
means no new scan gets matched at all once the robot is truly
stationary — so the sampled value is frozen at whatever the single scan
match right before arrival produced, and that match still has to pull in
a true-pose offset that can reach ~0.53m. Widening the search space and
raising the match interval improved how reliably that one match finds
the *right* alignment, but doesn't change the fact that a single
scan-match correction is being asked to close most of a 0.5m gap in one
shot.

**`--backend slam --use-ekf` (EKF-fused odometry underneath slam_toolbox
localization) measured markedly worse, not better**, in the same
clean-container full-suite run:

| scenario | result |
|---|---|
| `baseline` | PASS (0.0000m) |
| `noise_correction` | FAIL (growth_ratio 2.42, threshold 2.0) |
| `drift_correction` | FAIL (1.0659m, threshold 0.20m) |
| `drift_correction_obstacle` | FAIL (0.8652m, threshold 0.20m) |
| `jerk_with_motion` | PASS (8/8 trials) |

Every failing number roughly doubled versus plain `--backend slam`
(1.07m/0.87m vs 0.51m/0.49m), and `noise_correction` newly failed too.
Root-caused (by inspection, not fixed, per this session's scope) to
architecture rather than a knob: `slam_toolbox`'s `odom_frame` TF, which
its own scan-match correction is computed relative to, *is* the EKF's
fused output here (not raw `/odom`) — so the EKF's own rf2o-derived
absolute-position correction and `slam_toolbox`'s own scan-match
absolute-position correction are two independent corrections stacked on
top of each other against the same underlying lidar data, rather than
one deferring to the other. That's a plausible mechanism for the
"correction" overshooting/compounding instead of averaging out, though
not confirmed by a bag-level investigation this session (out of scope —
see the top-level task's instruction to prioritize `slam.yaml` and not
re-litigate `ekf.yaml` beyond a quick look). This combination was already
flagged in `sim/README.md` as untested before this session; it is now
tested, and measurably not an improvement over `--backend slam` alone.

**Net assessment**: `--backend slam` (no EKF), tuned as above, is the
best `drift_correction`/`drift_correction_obstacle` result found across
all backends tried this session and the two prior amcl sessions, but
still fails both under the current `odom_slip_ratio=0.25`/3m-loop/0.20m
threshold combination — same likely floor as `amcl`: a single-shot,
post-dwell scan-match correction is being measured against a slip-driven
raw-odom error that can be significantly larger than the threshold at
the loop's far corners. `--backend slam --use-ekf` should not be used
in its current form; it performs worse than `--backend slam` alone.
**config/ekf.yaml change note**: another concurrent session was actively
editing `config/ekf.yaml` in this same checkout while this session ran
(a `dynamic_process_noise_covariance` addition, uncommitted at the time
of writing) — that file was left untouched by this session; only
`config/slam.yaml` was tuned/committed here.

### 2026-07-27 — `--backend amcl --use-ekf` re-examined under the new
0.15 slip / 0.30m threshold / 30s `noise_correction` window: still no
`amcl.yaml` lever found; `drift_correction`(`_obstacle`) trace is a real
ramp, not motion-model noise

Prompted by `run_localization_drift_tests.py`'s thresholds changing since
the two 2026-07-26 sessions above (`odom_slip_ratio` 0.25→0.15,
`MAX_DELTA_THRESHOLD` 0.20m→0.30m, `noise_correction` window 60s→30s) —
the earlier "amcl.yaml is a closed lever" conclusion was reached under
strictly harsher conditions, so it was worth re-checking rather than
assuming it still holds. Scope: `config/amcl.yaml` only (no `ekf.yaml`,
`slam.yaml`, test script, or threshold edits).

**Clean confirmed baseline (unmodified `amcl.yaml`, idle-ish container,
`sim_errors=0`/`sentry_errors=0` throughout)**:

| scenario | result |
|---|---|
| `baseline` | PASS (0.0000m) |
| `noise_correction` | PASS (growth_ratio 1.50, threshold 2.0) |
| `drift_correction` | FAIL (0.8504m, threshold 0.30m) |
| `drift_correction_obstacle` | FAIL (0.6161m, threshold 0.30m) |
| `jerk_with_motion` | PASS (8/8 trials) |

Same 3/5 pattern as both 2026-07-26 sessions — the two map-based-drift
scenarios still fail, just against a looser 0.30m bar now. Two isolated
clean single-scenario runs taken while tuning (0.6480m/0.7642m) plus this
run's 0.8504m/0.6161m span roughly the same 0.6–1.3m noise band the prior
sessions documented — this scenario pair's run-to-run spread is still
comparable to or larger than the gap to threshold, same caveat as before.

**Per-sample trace shows a ramp, not a spike.** Read the raw `t=Ns
|map->odom - pre-loop|=...` lines from the two clean isolated runs (not
just the summary max): both climb roughly monotonically from ~0.04–0.08m
at t=2s to their final 0.65–0.76m at t=45s, tracking the loop's
progressively-more-offset corners rather than jumping once and settling.
This means amcl is faithfully correcting a real, growing map->odom
discrepancy over the course of the loop — not injecting its own transient
particle-filter noise on top of a small true error, which is the failure
mode `alpha1-5`/particle-count/`resample_interval` were tuned against
previously. A knob that damps amcl's own estimate noise has nothing to
act on here.

**Only new thing tried: `sigma_hit` loosened 0.08→0.15** (the one
untested-against-this-scenario knob flagged in this file's own tuning
notes — "re-derive rather than assuming 0.08 is final"). Single clean run,
`drift_correction` alone: **1.1203m — worse**, and nowhere near the 0.30m
target; within the same noise band as the closed alpha1-5/particle-count
directions from 2026-07-26, not a real improvement or a real regression
either. Reverted; `amcl.yaml` is byte-identical to before this session
(`git diff` empty).

**Net assessment**: the 2026-07-26 "amcl.yaml tuning is a closed lever for
`drift_correction`(`_obstacle`)" conclusion holds under the new, more
permissive conditions too — the threshold moved but the mechanism
producing the failure didn't. The EKF-fused odometry `amcl` consumes here
is far more accurate than raw slipped `/odom` (rf2o corrects most of the
slip per the 2026-07-26 EKF session's bag-level finding), yet
`drift_correction`(`_obstacle`) still measures 2–2.5x over threshold with
a monotonic-ramp signature — pointing at a residual EKF-vs-ground-truth
gap that grows over the loop (a fusion-design/tuning question, out of
this session's `amcl.yaml`-only scope) rather than anything in `amcl`'s
own motion/sensor model. No config change kept this session.

Collision note: one `drift_correction` launch this session briefly
overlapped with the other concurrently-running session's `noise_correction`
stack (both live for a few seconds before being caught and killed) —
caught immediately via `sim_errors`/`sentry_errors` and process-list
checks, my own run was killed right away and its result discarded, and
the other session's window was flagged to them as a result to
double-check rather than silently trust.

### 2026-07-27 — `noise_correction` boundedness: `dynamic_process_noise_covariance`
kept (partial win, not a reliable pass); `drift_correction`(`_obstacle`)
convergence-timing investigation (no change)

Two follow-up passes on `--backend amcl --use-ekf`, `ekf.yaml`-only, in
response to changed priorities mid-session (focus moved from
`drift_correction`(`_obstacle`) to `noise_correction` specifically, then
the harness's own `MAX_DELTA_THRESHOLD` 0.20m->0.30m and default
`odom_slip_ratio` 0.25->0.15 changed under this session, both changes made
by the coordinating session, not this one).

**Convergence-timing check (`drift_correction`, no change kept).** Investigated
whether the correction TF is actually converged by the 1.0s post-leg dwell
mark or still settling when sampled, since `drift_correction`/
`drift_correction_obstacle` only ever sample once per leg, right at the end
of the dwell. Recorded `/tf`'s `map->odom` at full rate through a
`drift_correction` run: early legs show ~1.3s genuine settling gaps within
the 1.75s leg cycle (no change until the next leg), but by t≈34s the same
signal never stops moving between legs at all, coinciding with the
run's largest sampled deltas (0.5-0.78m). This looks like real
convergence breakdown compounding lap over lap, not merely
"distance-from-spawn is larger at some corners" — but no `ekf.yaml` knob
found to address it specifically (`process_noise_covariance` was already
closed off per the entry above; robot_localization has no separate
"time-to-converge" parameter beyond process noise). Not pursued further
after the coordinator deprioritized these two scenarios in favor of
`noise_correction`.

**`noise_correction` (`dynamic_process_noise_covariance: true` + x/y
`process_noise_covariance` 0.05->0.08, kept).** `dynamic_process_noise_covariance`
(a `robot_localization` param not previously tried, distinct from static
process noise) scales `process_noise_covariance` by the filter's own
current velocity estimate — more injected noise while actually moving,
less near zero velocity — on the theory that `noise_correction`'s
growth_ratio check (is the correction keeping pace with 60s of continuous
injected drift, not just converging after one cornering event) might
benefit from faster measurement trust while driving. Measured (isolated
`--scenario noise_correction` runs, discarding any run where another
concurrent session's stack was found running in parallel — see
`sim/README.md`'s multi-agent coordination note for why that corrupts
results):

| config | slip=0.25 (stale) | slip=0.15 (current) |
|---|---|---|
| static 0.05, no dynamic (baseline) | 2.02 FAIL, 1.81 PASS, 2.23 FAIL | 2.09 FAIL, 1.40 PASS |
| 0.08 + dynamic=true (kept) | 1.51, 1.92, 1.41 PASS; 2.17, 2.03 FAIL | 1.80, 1.58, 1.92, 1.55 PASS; 2.36 FAIL |

Pass rate improved at both slip values (baseline ~40-50% vs. this config
~60-80%), consistently across two independent slip settings, so this
isn't just riding the slip reduction. **However, full-suite (`--headless`,
no `--scenario` filter) confirmation runs gave FAIL both times tried
(growth_ratio 2.17 and 2.49)** — worse than every isolated-run number above
except the single worst outlier. The mechanism for this isolated-vs-suite
gap wasn't identified (possibly cumulative load/residual state from the
four scenarios run before `noise_correction` in suite order) — flagging
it rather than papering over it. **Net: kept as a real, measured
improvement in isolation, but not a reliable pass — don't report this as
"noise_correction passes" without re-confirming in whatever context
matters for the actual claim.**

Also hit once during this session's testing, at this config: a
`map_server` lifecycle heartbeat timeout ("CRITICAL FAILURE: SERVER
map_server IS DOWN after not receiving a heartbeat for 4000 ms") under
elevated host load, cascading to a full stack crash mid-scenario. Same
class of bringup race documented earlier in this file's
`drift_correction_obstacle` entry (host-load-driven, not config-driven);
not re-tested against baseline to confirm it's config-independent, so
noted rather than asserted.
