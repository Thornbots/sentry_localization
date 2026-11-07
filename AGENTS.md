# sentry_localization

Follow [workspace rules](../AGENTS.md) and [CI](../docs/CI.md).
Read [tuning rationale](README.md#notes) before changing YAML.
Launch through `thornbots_pkg`'s `auto.launch.py`, never alongside it separately.
Rebuild copied configs with `--symlink-install`; verify installed YAML when
an edit appears ineffective. Container overlay checks live in the Docker skill.

## Scope

Own SLAM/AMCL/EKF and their tuning; hardware, URDF and relay belong to
`thornbots_pkg`. See [interfaces](README.md#contract-with-thornbots_pkg).

## Testing

Rerun [drift/EKF suites](../sim/README.md#more-on-the-tests) after backend/noise
changes; rf2o changes also need `scan_degraded` and `drift_correction` at 0.15 slip.
Sim/bench runs require [approval](../AGENTS.md#containers-and-runs).

## Open

Software commitments: [ROADMAP](../docs/ROADMAP.md#additional-mini-work).
Physical validation: [ROBOT_TESTS](../docs/ROBOT_TESTS.md#consolidated-hardware-checks).
