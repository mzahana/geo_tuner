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

Fully automatic once started. The session is flown in **rounds**:

1. **Preconditions.** Reads the controller's gains and `mass` as the safe
   baseline, and refuses to start until `/<ns>/geometric_mavros_node` reports
   `enable_thrust_estimator: false` — that estimator adapts the very plant
   gain being measured. (It is checked on the live node because a launch-file
   pin silently lost to the persisted override on every tuning flight
   2026-09-09..13: an exact node-name key beats a `/**` key in ROS 2
   parameter files.)
2. **Excite and record.** Each axis flies `episodes_per_round` steps
   (0 → +d → 0 → −d → 0). The whole time, at 50 Hz, the conductor records
   setpoint, measured velocity, commanded acceleration (`SE3Command.force /
   mass`) and the gains in force (`geo_tuner_session_<stamp>.csv`).
3. **Identify** (`core/accel_loop_id`). Per position axis, from everything
   recorded so far: `a = α · e^(−d s)/(τ s + 1) · a_cmd + w`. Estimated with
   the **setpoint as instrument** (a command under feedback is correlated
   with the wind; the setpoint is not): the nominal-loop command at several
   time shifts spans the candidate responses, and both α and the lag are
   chosen by **two-stage least squares** -- the residual projected onto the
   instruments, which the gust cannot bias. (Choosing the lag on the plain
   residual read it short in simulated gusts and long on the field flights.)
   **90 % intervals from a block jackknife**. The work runs on a worker
   thread in state IDENTIFY while the vehicle holds the hover point. Yaw
   keeps its first-order fit of the heading step.
4. **Decide** (`core/loop_design`). The design is lag-aware: `kx = wn²/α`,
   `kv = 2ζwn/α` with `wn` the largest value ≤ `wn_target` keeping
   `pm_nominal_deg` at the estimate and `pm_worst_deg` on the worst plant in
   the interval. The gains in force are
   - **updated** when they lack the margins (less `pm_tolerance_deg`) or lie
     outside the range the α interval supports (±`gain_tolerance`) -- both
     significant at the interval's level however wide it is -- by the
     smallest move the evidence supports: to the nearest edge of that range
     (the design at the upper α for soft gains, the lower α for stiff ones;
     the soft edge for missing margin), never to the point estimate, and at
     most `max_gain_change_factor` from the session's entry gains;
   - **confirmed** when inside the range with margins AND the α interval is
     at most `max_alpha_ci_ratio` wide. 1.19 is derived, not tuned: with the
     10 % tolerance it bounds a confirmed gain's error to ~20 %;
   - **inconclusive** otherwise: another round is flown and pooled.
5. **Validate.** Applied gains fly the next round and are judged on **that
   round's data alone**, as a safety test: the nominal phase margin at the
   fresh estimate must stay above the `pm_worst_deg` floor, the α intervals
   of design and validation must overlap (the plant did not change), and the
   measured overshoot must stay under `validation_max_overshoot`. A failure
   restores the entry gains. It deliberately does not re-demand the design
   target: on a one-round estimate that rejects correct updates about half
   the time. Whether validated gains are the best is decided again, on the
   pooled data, by step 4. A change is never applied without a round left
   to validate it — `max_rounds: 1` is therefore a check-only session.

The accel-trim learner still absorbs steady offsets through the setpoint
feedforward (bounded) and turns a persistent z trim into a `max_thrust`
suggestion.

### Why this design (and not step-response fitting)

Up to 2026-09-13 the conductor fitted `(α, τ)` to each step's position
response and gated per-bucket spreads. Three field flights showed per-episode
α scattering ±12 % and τ 5–268 ms on one airframe; a Monte Carlo of that
fitter with the measured disturbance (0.13 m/s² rms, ~2 s correlation) and the
true α fixed at 1.0 reproduced both the scatter and the "τ on the solver
floor" fits, and put the chance of a two-rung session failing its spread gate
at 61 % by noise alone. One step cannot separate gain from lag against a gust.
The session recording of those flights gives, with the method above, the
same α and lag on all three (x/y lag 110–130 ms, z 60–70 ms, α intervals
overlapping) — pinned as tests in `test/test_identification.cpp`. An
output-error check (closed loop simulated from the setpoint and gains alone,
scored on measured velocity) independently puts the best lag at x 90–110,
y 110–130, z 50–90 ms on the same flights.

## Safety architecture

An independent monitor runs at all times: tilt, position error, speed,
altitude floor and ceiling, odometry staleness, and roll/pitch-rate
oscillation energy. Any violation → gains restored to the last known-safe
set, hover hold, session aborted with a diagnosis in the report.

After an abort the **full** safe set (all axes and yawctrl_tau) is resent and
the controller's answer awaited, with retries; health key `restore` reads
`pending`, `confirmed` or `UNCONFIRMED`, and `~/reset` is refused until it is
confirmed. In ABORT and DONE, while PX4 is not in OFFBOARD, the held setpoint
follows the vehicle (and the trim is dropped), so re-engaging OFFBOARD later
never flies back to where the session ended. Losing the height-above-ground
topic mid-session aborts instead of re-basing the floor on odometry z.

On top of that:

- a session cannot start unless the controller node answers
  `enable_thrust_estimator` with an explicit boolean false (the node reads it
  only at startup, so the fix is a relaunch, never `ros2 param set`);
- gain changes are rate-limited (`max_gain_change_factor`) and designed to
  keep phase margin on the worst plant the evidence allows;
- no change survives without an out-of-sample validation; a failed one
  restores the entry gains, so a session never ends worse than it began.

The conductor **never arms, disarms or changes flight mode**. Switching out
of OFFBOARD on the RC overrides everything.

## ROS interface (tuning_conductor)

| Kind | Name | Type |
| --- | --- | --- |
| pub | `geometric_controller/multi_dof_setpoint` (`setpoint_topic`) | `trajectory_msgs/MultiDOFJointTrajectory`, streamed at 50 Hz |
| pub | `geo_tuner/status` | `std_msgs/String` |
| pub | `geo_tuner/health` | `diagnostic_msgs/DiagnosticStatus` |
| sub | `geometric_controller/odom` (`odom_topic`) | `nav_msgs/Odometry`, ≥ 50 Hz |
| sub | `geometric_controller/cmd` (`cmd_topic`) | `mav_controllers_ros/SE3Command` (identification input) |
| sub | `mavros/state` | `mavros_msgs/State` (optional) |
| srv | `~/start`, `~/abort`, `~/accept`, `~/restore`, `~/reset` | `std_srvs/Trigger` |

Gains are read and written on the controller node named by
`controller_node` via ROS parameter services; `enable_thrust_estimator` is
read from `estimator_node` (a bare name takes the controller's namespace).

## Executables, launch files, configs

| Executable | Role |
| --- | --- |
| `tuning_conductor` | the auto-tune node (field and SITL alike) |
| `quad_sim` | lightweight quadrotor plant for the no-Gazebo closed-loop test |
| `geo-tuner-design` | offline pole-placement gain designer (C++) |
| `geo-tuner-identify` | the conductor's identification + verdict, offline, on a session recording CSV |
| `geo-tuner-bag-export` | older tuning bag → session recording CSV (needs a sourced ROS env) |
| `geo-tuner-hover` | offline ulog → hover throttle → `max_thrust` (Python; run it as `.venv/bin/python scripts/geo-tuner-hover`, it needs pyulog) |
| `tracking_viz.py` | commanded/measured poses → Paths + error, for RViz |

| Launch file | Brings up |
| --- | --- |
| `sim_tune.launch.py` | real controller + `quad_sim` + conductor. Args: `thrust_scale_error`, `gust_sigma`, `kx_xy`/`kv_xy`/`kx_z`/`kv_z` (start gains), `report_path`, `step_size`, `step_size_z`, `yaw_step` |
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
- `wn_target`, `zeta_target`, `pm_nominal_deg`/`pm_worst_deg` — what the
  design aims for, and the margins it will not trade away for bandwidth;
- `max_rounds` — 3 to tune, 1 to check without changing anything;
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
