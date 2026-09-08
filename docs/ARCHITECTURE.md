# Architecture — what is in the package and how the pieces fit

Reference for developers and for anyone debugging a session. For the
procedure, see [SITL_RECIPE.md](SITL_RECIPE.md) (simulation) and
[FIELD_CHECKLIST.md](FIELD_CHECKLIST.md) (real vehicle); for the math, see
[TUNING_GUIDE.md](TUNING_GUIDE.md).

## Why the gains are computable in the first place

The controller's position loop outputs *acceleration*:
`a_fb = kx·e_pos + kv·e_vel`, so the ideal closed loop per axis is a double
integrator under PD control — which makes the gains physical quantities:

```
kx = wn²           [1/s²]     (position stiffness = natural frequency²)
kv = 2·ζ·wn        [1/s]      (damping)
```

Everything in this package designs, verifies and corrects `(wn, ζ)` rather
than twiddling `kx`/`kv`. That leaves only three vehicle-specific unknowns:

1. **the thrust map** (`max_thrust` in `geometric_mavros.yaml`) — measured
   from one hover flight ulog (`geo-tuner-hover`);
2. **the inner-loop bandwidth** — owned by **PX4 Autotune**, run first;
3. **the real plant-gain / lag deviation** — identified *in flight* by the
   tuning conductor and folded into the gains automatically.

Full derivations: TUNING_GUIDE.md Appendix A.

## One package, three products

| Path | What |
| --- | --- |
| `include/geo_tuner/core/`, `src/core/` | ROS-free identification and design math: closed-loop and step fits (bounded Levenberg–Marquardt on Eigen), the safety monitor, robust aggregation, pole-placement gain design, gain YAML emission |
| `src/tuning_conductor.cpp`, `src/quad_sim.cpp`, `src/nodes/` | the `tuning_conductor` and `quad_sim` nodes, and the `geo-tuner-design` CLI |
| `include/geo_tuner/rviz/`, `src/rviz/` | the five RViz panels (`geo_tuner_panels` plugin library) |

`scripts/geo-tuner-hover` is the one remaining Python tool (it parses ulog
with pyulog). It is offline-only — nothing in the flight path imports it.

### Optional dependencies

The panels build **only where RViz is installed** —
`find_package(rviz_common QUIET)`. A vehicle-side build (Jetson, headless
docker) silently skips them and pulls in no Qt or OGRE. `mavros_msgs` is
likewise optional: without it the panels lose the PX4 flight-mode readout
and the conductor cannot supervise OFFBOARD.

### Panels

Registered under the `geo_tuner/` pluginlib prefix:

| Class | Purpose |
| --- | --- |
| `geo_tuner/GeoFieldPanel` | all four panels as tabs in one dock — use this one unless you want them docked separately |
| `geo_tuner/TunerPanel` | run a session: start, watch the ladder, accept, abort; sets the step envelope in flight |
| `geo_tuner/ControllerHealthPanel` | stream rates and staleness, failsafes, tracking error, throttle/thrust-map health, active gains as wn/ζ (read-only) |
| `geo_tuner/GainPanel` | read and change gains as wn/ζ behind an interlock, with design caps, bounded steps, revert, and Save-to-vehicle via `gain_saver` |
| `geo_tuner/TrajectoryTestPanel` | remote for `trajectory_test_node`: setpoint / circle / lemniscate, speed, start, stop |

The panels used to live in a separate `geo_tuner_rviz_plugins` package.
Folding them in here renamed the classes from
`geo_tuner_rviz_plugins/XPanel` to **`geo_tuner/XPanel`**. A hand-saved
`.rviz` config from before the merge needs that prefix updated; the shipped
`rviz/geo_field.rviz` already has it.

## What the conductor does, step by step

Fully automatic once started:

- reads the controller's current gains as the **safe baseline**;
- injects small alternating steps (z first, then x, y, yaw) and identifies
  each response **against the closed loop it actually commanded** — `kx`
  and `kv` are known, so the only unknowns are the plant-gain factor α
  (thrust-map error, inner-loop droop) and the in-loop lag τ;
- re-places the closed-loop poles at the target `(wn, ζ)` through a **live
  parameter update** — no landing, no restart;
- walks `wn` up the configured ladder, re-identifying at each rung;
- trims steady-state offsets through the setpoint acceleration feedforward
  (bounded, ±3 m/s²) and converts a persistent z-trim into a **`max_thrust`
  correction suggestion**;
- writes a session report plus a tuned YAML snippet.

Each `(axis, rung)` is repeated `episodes_per_rung` times and the gains
update from the **median** identified α; a consistency gate blocks the
update when the accepted estimates disagree by more than
`estimate_consistency`.

## Safety architecture

An independent monitor runs at all times: tilt, position error, speed,
altitude floor and ceiling, odometry staleness, and roll/pitch-rate
oscillation energy. Any violation → gains restored to the last known-safe
set, hover hold, session aborted with a diagnosis in the report.

On top of that:

- fit-quality gates reject bad identifications;
- per-episode gain changes are rate-limited (`max_gain_change_factor`);
- the ladder refuses to climb past the Routh–Hurwitz stability margin
  implied by the *measured* in-loop lag — the closed loop is
  `tau·s³ + s² + α·kv·s + α·kx`, unstable at `kv = tau·kx`, so the cap is
  `wn ≤ 2ζ/(stability_margin·τ̂)`. The bandwidth limit is therefore derived
  from what the vehicle actually did, not assumed.

The conductor **never arms, disarms or changes flight mode**. Switching out
of OFFBOARD on the RC overrides everything.

Details and quantities: TUNING_GUIDE.md §3 and A.10.

## ROS interface (tuning_conductor)

| Kind | Name | Type |
| --- | --- | --- |
| pub | `geometric_controller/multi_dof_setpoint` (`setpoint_topic`) | `trajectory_msgs/MultiDOFJointTrajectory`, streamed at 50 Hz |
| pub | `geo_tuner/status` | `std_msgs/String` |
| pub | `geo_tuner/health` | `diagnostic_msgs/DiagnosticStatus` |
| sub | `geometric_controller/odom` (`odom_topic`) | `nav_msgs/Odometry`, ≥ 50 Hz |
| sub | `mavros/state` | `mavros_msgs/State` (optional) |
| srv | `~/start`, `~/abort`, `~/accept`, `~/restore`, `~/reset` | `std_srvs/Trigger` |

Gains are read and written on the controller node named by
`controller_node` via ROS parameter services.

## Executables, launch files, configs

| Executable | Role |
| --- | --- |
| `tuning_conductor` | the auto-tune node (field and SITL alike) |
| `quad_sim` | lightweight quadrotor plant for the no-Gazebo closed-loop test |
| `geo-tuner-design` | offline pole-placement gain designer (C++) |
| `geo-tuner-hover` | offline ulog → hover throttle → `max_thrust` (Python; run it as `.venv/bin/python scripts/geo-tuner-hover`, it needs pyulog) |
| `tracking_viz.py` | commanded/measured poses → Paths + error, for RViz |

| Launch file | Brings up |
| --- | --- |
| `sim_tune.launch.py` | real controller + `quad_sim` + conductor. Args: `thrust_scale_error`, `report_path`, `step_size`, `step_size_z`, `yaw_step` |
| `field_tune.launch.py` | conductor only, for a real or SITL vehicle. Args: `params`, `require_enable`, `ns`, `output_dir` |
| `field_monitor.launch.py` | RViz with the field panels (+ `tracking_viz`). Args: `rviz_config`, `tracking_viz` |
| `sitl_test.launch.py` | d2dtracker SITL bringup with panels. Args: `ns` (default `interceptor`), `open_rviz`, `tuner`, `trajectory_type` |

`config/tuner_field.yaml` and `config/tuner_sitl.yaml` are the conductor's
parameters; both use a `/**:` node path so they apply in whatever namespace
the node is launched into. The file is commented parameter by parameter —
read it before a field session. The knobs that matter most:

- `hover_mode` — `capture` (default) tunes about the point the pilot hands
  the vehicle over at, captured when OFFBOARD engages: the setpoint never
  jumps and the vehicle is never commanded to fly anywhere to start.
  `fixed` (simulators) flies to `hover_position`, ramped at
  `hover_approach_speed` — still never a step;
- `min_tuning_altitude` — the session refuses to start below this height
  **above ground** and holds position instead; it never climbs to reach it.
  The effective gate is this or `safety.min_altitude` plus the room a
  downward z step needs (`step_size_z × (1 + z_step_margin)`), whichever is
  higher. Live-settable from the Tuner panel;
- `agl_topic` — where height above ground comes from. Odometry z is *not*
  AGL: PX4's local origin sat 6.6 m below ground on the first field
  session, which put every altitude limit 6.6 m out;
- `step_size`, `step_size_z`, `max_yaw_step` — the manoeuvre envelope: the
  vehicle stays within ± these of the hover point, so set them to the space
  you actually have (live-settable from the Tuner panel);
- `wn_ladder`, `zeta_target` — the bandwidth targets;
- `output_dir` — the one knob for session data;
- `safety.*` — the monitor's limits.

## Session outputs

`output_dir` is the single knob: every session writes a timestamped
`geo_tuner_report_<stamp>.yaml` plus an `episodes_<stamp>/` folder of raw
per-episode CSVs (`t`, `y` per odometry sample) under it, so a day of
repeated sessions never overwrites anything. Leave it unset and the node
falls back to the YAML's `report_path` — a single, overwritten file.

The report carries every episode's fit (α, τ, NRMSE, overshoot, action),
every bucket's verdict, the `final_gains`, and any `max_thrust` correction
suggestion. The CSVs are the raw responses behind those numbers, for
offline analysis of a session whose estimates disagreed.

## Validation status

Validated on ROS 2 Humble (d2dtracker docker image) against both upstream
`mav_controllers_ros` and its `production-hardening` branch (dt-correct
integrator, anti-windup, altitude-priority saturation, rate feedforward,
watchdogs, thrust-scale estimator):

- perfect model → converges, all axes `wn_effective` = target, ζ = 0.95
- 10 % thrust-map error → converges, reports "multiply max_thrust by 0.92"
- 30 % thrust-map error → safe abort, gains restored, actionable diagnosis

Honest, itemised status: TUNING_GUIDE.md §4 and
[PROJECT_STATE.md](PROJECT_STATE.md).
