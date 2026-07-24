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

### `localization_mode` — pick the whole localization scheme

| value | map->odom owner | /localization/odom source | use case |
|---|---|---|---|
| `slam` (default) | `slam_toolbox` (localization mode) | `passthrough_odom_publisher` (relays `/odom`) | normal running mode, localizing against the saved field map |
| `mapping` | `slam_toolbox` (mapping mode) | `passthrough_odom_publisher` | deliberately (re)building/extending the map |
| `amcl` | `nav2_amcl` + `nav2_map_server` | `passthrough_odom_publisher` | particle-filter localization against a saved occupancy grid |
| `ekf` | *(none — no map frame)* | `ekf_node` (`robot_localization`, fuses `/odom` + `/scan_odom`, remapped output) | odometry fusion only, no map |

`ekf` mode also launches `rf2o_laser_odometry_node` (`/scan_odom`) and
`head_home_scan_gate` (gates lidar input to `rf2o` to only when the head
is near its home/yaw≈0 position — `rf2o` caches its `lidar->root`
transform once on the first scan, so it can't tolerate an arbitrarily
moving head-mounted lidar). See the `home_yaw_tolerance` arg and the
module docstring in `launch/localization.launch.py` for the full
rationale and every other mode's exact node/TF-ownership behavior.

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
  `/localization/odom` unchanged. Used in `slam`/`mapping`/`amcl` modes.
- `head_home_scan_gate.py` — republishes raw `/scan` onto `/scan_gated`
  only while the head is near home (yaw), for feeding `rf2o` safely.
  Only used in `localization_mode:=ekf`.
- `slam_relocalize_publisher.py` / `simple_relocalize_publisher.py` —
  relocalization helpers (see each file's docstring for specifics).

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
