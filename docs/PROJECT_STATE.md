# Project state — interceptor drone controller & tuning stack

Handoff summary for a fresh session. Last updated: 2026-08-21.

## The system

Interceptor drone framework: ROS 2 Humble (Jetson Orin NX 16GB on the
vehicle; docker SITL on the dev machine), PX4 + mavros, geometric SE3
controller from `mzahana/mav_controllers_ros`
(`geometric_attitude_control_node` → `geometric_mavros_node` → PX4 in
body-rates+thrust offboard). Airframe: X500v2, ~2.5 kg, SIYI A8 mini
gimbal. SITL vehicle: x500_d435 (mass 2.0) in `mzahana/d2dtracker_sim`
under namespace `/interceptor`.

## Repositories and branches (all pushed to GitHub)

| Repo | Branch | State |
|---|---|---|
| `mzahana/geo_tuner` | `main` (90db467) | tuning toolkit + field checklist (this repo) |
| `mzahana/mav_controllers_ros` | `main` = `ros2_humble` = `production-hardening` (139f3ad) | hardened controller + trajectory test node + hold failsafe |
| `mzahana/d2dtracker_sim` | `main` | SITL bringup launch + tuned configs |

Local checkouts: `~/src/ihunter_fixes/{geo_tuner, mav_controllers_ros}`;
sim package at `~/d2dtracker_cuda_shared_volume/ros2_ws/src/d2dtracker_sim`.

## What was built

### 1. Controller hardening (mav_controllers_ros, see CHANGES.md)
dt-correct position integrator (+ conditional-integration anti-windup),
altitude-priority accel saturation, body-rate feedforward from
differential flatness (jerk → rates, `enable_rate_feedforward`), dead
code removed (kib, alternate attitude laws), odom/setpoint watchdogs,
completed live-parameter callback with validation, cmd-timeout failsafe
fixed (hold 1 s then release to PX4 failsafe instead of feeding stale
setpoints forever), IMU-based online thrust-scale estimator
(`enable_thrust_estimator`, publishes `geometric_mavros/thrust_scale_estimate`),
**yawctrl_tau** (decoupled yaw bandwidth; ≤0 → follows attctrl_tau),
yaw reference extraction fixed (atan2, not eulerAngles),
**hold-on-setpoint-loss failsafe** (`hold_on_setpoint_timeout: true`,
50 Hz watchdog in geometric_controller_node: if the setpoint stream dies
mid-flight with fresh odom + motors enabled, latch a closed-loop position
hold at the current pose; new setpoints release it, disarm/odom-stale
clears it — planner death is now field-safe by itself).

### 1b. Trajectory test node (mav_controllers_ros, 2026-08-21)
`trajectory_test_node` (test/trajectory_test_node.cpp) — safety-gated
reference generator to evaluate the controller in SITL and the field:
setpoint / circle / lemniscate (Gerono), full pos/vel/acc/jerk + yaw/
yaw_dot references. Key design (user requirement — the controller goes
unstable on far/step setpoints): idle = hold-setpoint stream glued to the
current pose (safe OFFBOARD engage); start = min-jerk transition to the
*nearest* point on the curve, then C2 smoothstep speed ramp; stop = ramp
down + hold. Closed-form feasibility derating (speed/accel/jerk from max
|p'|,|p''|,|p'''|), geofence pre-check + runtime abort, OFFBOARD/armed/
motors/odom gating, `trajectory_test/start|stop` Trigger services,
`auto_start` for SITL only. RViz: planned/reference/actual paths +
setpoint pose, layout in rviz/trajectory_test.rviz, launch arg
`rviz:=true` (namespace auto-filled). Files: config/trajectory_test.yaml,
launch/trajectory_test.launch.py, docs/TRAJECTORY_TESTING.md (simple
field/SITL guide), test/scripts/ python harnesses (fake vehicle; verify
limits/continuity/derating/dropout-abort/hold-failsafe — all passing in
the Humble container). Container gotcha: process comm truncates to 15
chars, so kill with `pkill -f 'lib/mav_controllers_ros/[t]raj'`, never
`pkill -x trajectory_test_node`.

### 2. geo_tuner package (this repo)
- `core/gain_design.py` — pole-placement gain design from vehicle
  params: kx=ωn², kv=2ζωn, caps ωn ≤ (2/attctrl_tau)/4 and ωn·Td ≤ 0.35;
  max_thrust = m·g/hover_throttle; identification correction
  kx=ωn*²/α, kv=2ζωn*/α.
- `core/step_fit.py` — 2nd-order+delay step fit; multi-start; wn
  constrained to √kx·[√0.4, √2.5] (fixes the higher-order-response
  ambiguity that produced absurd α); quality gates.
- `core/first_order_fit.py` — yaw first-order+delay fit.
- `core/aggregate.py` — median-of-N aggregation + consistency gate.
- `core/safety.py` — independent monitor (tilt, pos error, velocity,
  altitude, odom staleness, roll/pitch and yaw oscillation RMS).
- `core/thrust_model.py` + `geo-tuner-hover` — max_thrust from a PX4
  hover ulog.
- `tuning_conductor.py` — in-flight auto-tuner: hovers in OFFBOARD,
  steps each axis (z, x, y, yaw), identifies, corrects gains live via
  parameter services, walks a wn ladder. **Median-of-N**: flies
  `episodes_per_rung` (default 3) steps per (axis, rung), updates from
  the median α (yaw: median T), refuses updates when estimates spread
  > `estimate_consistency` (1.35×). OFFBOARD supervision (pause/resume
  on mode change), abort → restore last-safe gains + hover. Bounded
  accel-feedforward trim stands in for integral action and diagnoses
  thrust-map error. Writes a full YAML report incl.
  `controller_yaml_snippet` and `final_yawctrl_tau`.
- `quad_sim.py` — lightweight plant for closed-loop testing without
  Gazebo (real compiled controller + this sim + conductor:
  `ros2 launch geo_tuner sim_tune.launch.py`).
- Docs: `TUNING_GUIDE.md` (phases + full math appendix A.1–A.11),
  `SITL_RECIPE.md` (step-by-step SITL cycle incl. PX4 autotune + ulog),
  `FIELD_CHECKLIST.md` (printable field-day checklist: bench prep →
  PX4 autotune/hover ulog → controller sanity flight → conductor
  session → circle/lemniscate agility ladder + emergency card).
- 39 unit tests (`pytest test/test_core.py`, use the repo `.venv`).

### 3. SITL environment
Container `d2dtracker_cuda` (image mzahana/px4-simulation-cuda12.2.0-
ubuntu22), shared volume `~/d2dtracker_cuda_shared_volume`, user `user`
(exec with `-u user`, source /opt/ros/humble/setup.bash, use
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`). PX4 v1.14 SITL built. Workspace
`shared_volume/ros2_ws` has mavros, hardened mav_controllers_ros,
geo_tuner, d2dtracker_sim built. Minimal bringup:
`ros2 launch d2dtracker_sim sitl_bringup.launch.py` (gz default world +
PX4 + mavros ns `/interceptor`, auto-sets 100 Hz streams for msgs
31/32/105; `with_controller:=true` adds the geometric controller).

## Current tuned state (SITL, d2dtracker_sim configs)

- `max_thrust: 27.5` (34.0 × 0.81 identified thrust-map scale).
- Gains (second session): pos [1.125, 1.764, 2.757], vel [1.781, 1.95,
  2.619]; `ki.z: 1.0` + online thrust estimator enabled (kills the
  ~0.34 m static z offset); `yawctrl_tau: 1.103`; `attctrl_tau: 0.3`
  (raises the separation cap to 1.67 rad/s for the next ladder).
- Known: pre-median-of-N sessions showed run-to-run gain scatter —
  expected (gains embed 1/α; compare `wn_effective`, not kx). The
  median-of-N tuner (pushed, rebuilt in container) fixes this; the
  next session should show spreads ≤ ~1.15× and reproducible gains.

## Verification status

- 39/39 unit tests pass.
- Closed-loop regression (real controller + quad_sim, 20% deliberate
  thrust error): complete; spreads 1.01–1.07×; z α 1.07 → 0.93; yaw
  τ 0.30 → 0.48 → 0.77 toward T_target 0.35 s; ki verified dt-invariant
  (0.36 m offset → 0.000).
- Real PX4 SITL sessions flown by the user; thrust map, stream rates,
  and fitter constraints were corrected from the first session's report.

## Not done / next

- Re-run SITL session with median-of-N (expect α≈1, consistent gains);
  then ladder to 2.0 rad/s with attctrl_tau 0.3.
- Fly the trajectory_test_node in full PX4 SITL (so far verified against
  a faked vehicle, not the closed sim loop): circle then lemniscate,
  ramp `speed`, watch reference vs actual paths in RViz.
- Phase 4/5: field tuning on the real X500v2 — follow
  `docs/FIELD_CHECKLIST.md` (config/tuner_field.yaml,
  launch/field_tune.launch.py, hover ≥10 m), then the agility pass with
  trajectory_test (mav_controllers_ros docs/TRAJECTORY_TESTING.md).
- Real vehicle: set mass 2.5 and re-derive max_thrust before flight.

## Working constraints (respect these)

- Never install into system Python — use the repo `.venv`.
- Edit *source* configs in `shared_volume/ros2_ws/src/...`, then
  `colcon build --symlink-install` — never the install space.
- Don't kill container processes while the user is working.
- Tuned gains applied live by the conductor are RAM-only: persist them
  to the yaml (report's `controller_yaml_snippet`) explicitly.
