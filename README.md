# geo_tuner

Systematic gain design and **safe in-flight auto-tuning** for the
[mav_controllers_ros](https://github.com/mzahana/mav_controllers_ros)
geometric attitude controller (PX4 offboard, body-rates + thrust via mavros).

## Why this works

The controller's position loop outputs *acceleration*:
`a_fb = kx·e_pos + kv·e_vel`, so the ideal closed loop per axis is a double
integrator under PD control — meaning the gains are physical quantities:

```
kx = wn²           [1/s²]     (position stiffness = natural frequency²)
kv = 2·ζ·wn        [1/s]      (damping)
```

Everything here designs, verifies, and corrects `(wn, ζ)` instead of blindly
twiddling `kx/kv`. The only vehicle-specific unknowns are:

1. **the thrust map** (`max_thrust` in geometric_mavros.yaml) — measured from
   one hover flight ulog;
2. **the inner-loop bandwidth** — handled by **PX4 Autotune** (run it first);
3. **the real plant-gain/lag deviation** — identified *in flight* by the
   tuning conductor and folded into the gains automatically.

## Workflow

### Step 0 — one-time setup

```bash
# offline tools live in an isolated venv (never touches system python)
cd geo_tuner && python3 -m venv .venv
.venv/bin/pip install pyulog numpy scipy pyyaml
# ROS package: drop geo_tuner + mav_controllers_ros in your ws and colcon build
```

Run **PX4 Autotune** (rate + attitude loops) in Position mode. Make sure
mavros streams odometry ≥ 50 Hz to `geometric_controller/odom`.

### Step 1 — thrust map from a hover ulog

Fly a 1–2 min steady hover in **Position mode** (full flight battery), pull
the `.ulg`, then:

```bash
.venv/bin/python -m geo_tuner.cli.analyze_hover flight.ulg --mass 2.5 --design
```

This prints the measured hover throttle and `max_thrust`, and (with
`--design`) writes ready-to-use `geometric_controller.yaml` +
`geometric_mavros.yaml` with principled starting gains. To control the design
knobs directly:

```bash
.venv/bin/python -m geo_tuner.cli.design_gains \
    --mass 2.5 --hover-throttle 0.45 \
    --attctrl-tau 0.3 --zeta 0.95 --latency 0.08 --out-dir cfg/
```

The design honors two hard caps and tells you which one binds:
`wn ≤ (2/attctrl_tau)/4` (time-scale separation) and `wn ≤ 0.35/latency`.

### Step 2 — verify in simulation (no Gazebo needed)

```bash
ros2 launch geo_tuner sim_tune.launch.py                       # perfect model
ros2 launch geo_tuner sim_tune.launch.py thrust_scale_error:=0.9   # robustness
```

This runs the **real compiled controller node** against a lightweight
quadrotor simulator (rate-loop lag, thrust-map error, odom delay + noise) and
the tuning conductor, end to end. Expect `status: complete` in
`/tmp/geo_tuner_report.yaml` with `wn_effective` at the ladder target.

### Step 3 — in-flight auto-tune

1. Take off, hover in OFFBOARD under the geometric controller near the
   configured `hover_position` (edit `config/tuner_field.yaml`).
2. Pilot thumb on the RC mode switch — **switching out of OFFBOARD always
   overrides everything**. The conductor never arms/disarms or changes modes.
3. `ros2 launch geo_tuner field_tune.launch.py`

The conductor then, fully automatically:

- reads the controller's current gains as the safe baseline;
- injects small alternating steps (z first, then x, y), fits a
  second-order-plus-delay model to each response;
- computes the plant-gain factor α (lumping thrust-map error and inner-loop
  lag) and re-places the closed-loop poles at the target `(wn, ζ)` via a
  **live parameter update** — no landing, no restart;
- walks `wn` up the configured ladder, re-identifying at each rung;
- trims steady-state offsets through the setpoint acceleration feedforward
  (bounded, ±3 m/s²) and converts any persistent z-trim into a
  **max_thrust correction suggestion**;
- writes a full session report + tuned yaml snippet to `report_path`.

**Safety monitor** (independent, always on): tilt, position error, speed,
altitude floor/ceiling, odometry staleness, and roll/pitch-rate oscillation
energy. Any violation → gains restored to the last known-safe set, hover hold,
session aborted with a diagnosis in the report. Fit-quality gates reject bad
identifications; per-episode gain changes are rate-limited
(`max_gain_change_factor`); the ladder refuses to push bandwidth into the
measured delay margin (`wn·delay ≤ 0.45`).

### Step 4 — after the session

Copy `final_gains` from the report into `geometric_controller.yaml`. If the
report suggests a `max_thrust` correction, apply it to
`geometric_mavros.yaml` — that fix benefits everything, not just tuning.
For the agility pass, re-run with `attctrl_tau: 0.2` and a higher ladder
(the separation cap then allows `wn ≤ 2.5`).

## Full PX4 SITL validation (d2dtracker docker)

The same conductor works unchanged against PX4 SITL because it only touches
the controller's ROS interface. In the container
(`mzahana/px4-simulation-cuda12.2.0-ubuntu22`):

```bash
# inside the container, in the shared-volume ros2_ws:
cd ~/shared_volume/ros2_ws/src && ln -s /path/to/geo_tuner .
cd .. && colcon build --packages-select geo_tuner && source install/setup.bash
# bring up PX4 SITL + mavros + geometric controller as usual (d2dtracker_sim),
# put the vehicle in OFFBOARD hover, then:
ros2 launch geo_tuner field_tune.launch.py    # same launch as the field
```

Note: if the image defaults to the zenoh RMW without a router, run with
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.

## Tests

```bash
env PYTHONPATH= .venv/bin/python -m pytest test/ -q     # 24 unit tests
ros2 launch geo_tuner sim_tune.launch.py                # closed-loop e2e
```

Validated on ROS 2 Jazzy (host) and Humble (d2dtracker docker image),
against both upstream `mav_controllers_ros` and its `production-hardening`
branch (dt-correct integrator, anti-windup, altitude-priority saturation,
rate feedforward, watchdogs, thrust-scale estimator):
- perfect model → converges, all axes `wn_effective = target`, ζ = 0.95
- 10 % thrust-map error → converges + reports "multiply max_thrust by 0.92"
- 30 % thrust-map error → safe abort, gains restored, actionable diagnosis

## Implementation notes / gotchas found in the controller

- `gains.ki.*` integrates **per callback without dt** — its effect scales
  with your setpoint rate. Leave at 0 (the conductor's accel-trim covers the
  same need during tuning).
- `gains.kib.*` is declared but **unused** in `GeometricAttitudeControl` —
  don't tune it.
- `max_thrust` scales *every* loop gain implicitly: measure it (Step 1),
  never guess it.
- The node accepts live `ros2 param set` for `gains.*`, `attctrl_tau`,
  `max_accel`, `max_tilt_angle` — this is what makes hot tuning possible.
- Setpoints via `geometric_controller/multi_dof_setpoint`
  (MultiDOFJointTrajectory): the controller only publishes commands when a
  setpoint arrives, so the conductor streams at 50 Hz.
