# Controller notes — mav_controllers_ros quirks that shape this package

Findings about the
[mav_controllers_ros](https://github.com/mzahana/mav_controllers_ros)
geometric attitude controller that a tuner (or anyone editing
`geometric_controller.yaml` by hand) needs to know.

## Gains

- **`gains.ki.*` integrates per callback without `dt`.** Its effect
  therefore scales with your setpoint rate rather than with time. Leave it
  at 0 — the conductor's bounded acceleration trim covers the same need
  during tuning. (Fixed on the `production-hardening` branch.)
- **`gains.kib.*` is declared but unused** in `GeometricAttitudeControl`.
  Don't tune it.
- **`max_thrust` scales every loop gain implicitly.** The commanded
  acceleration is converted to a normalised throttle through it, so a wrong
  thrust map multiplies `kx` and `kv` alike. Measure it from a hover ulog
  (`geo-tuner-hover`), never guess it. See TUNING_GUIDE.md A.6.

## What makes hot tuning possible

The node accepts live `ros2 param set` for `gains.*`, `attctrl_tau`,
`max_accel` and `max_tilt_angle`. That is the whole mechanism behind the
conductor's in-flight gain updates and the RViz Gain panel — no landing, no
restart.

## Setpoint contract

Setpoints go to `geometric_controller/multi_dof_setpoint`
(`trajectory_msgs/MultiDOFJointTrajectory`). **The controller only publishes
a command when a setpoint arrives**, so a stream, not a one-shot, is
required: the conductor streams at 50 Hz. Odometry on
`geometric_controller/odom` should likewise be ≥ 50 Hz.

## Design caps the tuner enforces

Two hard caps bound the achievable position-loop bandwidth, and the
designer reports which one binds:

```
wn ≤ (2/attctrl_tau)/4      time-scale separation from the attitude loop
wn ≤ 0.35/latency           transport delay
```

In flight, a third and stronger cap applies — the Routh–Hurwitz margin on
the *identified* lag, `wn ≤ 2ζ/(stability_margin·τ̂)` (see
[ARCHITECTURE.md](ARCHITECTURE.md) and TUNING_GUIDE.md A.5a).

## production-hardening branch

`mzahana/mav_controllers_ros` `main` = `ros2_humble` =
`production-hardening` carries: dt-correct integrator, anti-windup,
altitude-priority saturation, rate feedforward, stream watchdogs, hold
failsafe, an online thrust-scale estimator, and the `trajectory_test_node`
plus `panel_support.launch.py` (`gain_saver`) that the RViz panels drive.
geo_tuner is validated against both this and stock upstream.
