# geo_tuner

Gain design and **safe in-flight auto-tuning** for the
[mav_controllers_ros](https://github.com/mzahana/mav_controllers_ros)
geometric attitude controller (PX4 offboard, body-rates + thrust via
mavros) — plus the **RViz field panels** that drive and monitor it.

Instead of twiddling `kx`/`kv` by hand, you pick a bandwidth and damping
`(wn, ζ)`; the tools compute the gains, and a **tuning conductor** flies
small test steps, measures what the vehicle actually did, and corrects the
gains live — with an independent safety monitor that restores the previous
gains and aborts on any violation.

---

## New here? Read this section only

**What you need:** ROS 2 Humble or Jazzy, a PX4 vehicle (or SITL) running
`mav_controllers_ros`, mavros streaming odometry at ≥ 50 Hz.

### 1. Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/mzahana/geo_tuner.git
cd .. && colcon build --packages-select geo_tuner && source install/setup.bash
```

One offline tool (`geo-tuner-hover`, which parses PX4 ulogs) needs Python.
Keep it in a venv — never install into system python:

```bash
cd src/geo_tuner && python3 -m venv .venv && .venv/bin/pip install pyulog numpy
```

Everything else — conductor, identification math, gain designer, simulator,
RViz panels — is C++ and needs no Python.

### 2. Try it in simulation first (30 seconds, no Gazebo)

```bash
ros2 launch geo_tuner sim_tune.launch.py
```

This runs the **real compiled controller** against a lightweight quadrotor
plant plus the conductor, end to end. When it finishes,
`/tmp/geo_tuner_report.yaml` should say `status: complete` with
`wn_effective` at the ladder target. Try
`thrust_scale_error:=0.9` to watch it detect and correct a bad thrust map.

If that works, your build is good and you understand what a session looks
like.

### 3. Then pick your path

| I want to… | Go to |
| --- | --- |
| Run the full cycle in PX4 SITL | **[docs/SITL_RECIPE.md](docs/SITL_RECIPE.md)** — command by command |
| Tune the real vehicle in the field | **[docs/FIELD_CHECKLIST.md](docs/FIELD_CHECKLIST.md)** — printable step-by-step checklist |
| Understand the theory and the plan | **[docs/TUNING_GUIDE.md](docs/TUNING_GUIDE.md)** — phases 0–5 + full math |

---

## The four steps of a tuning campaign

A short map of the whole process. Each step has its own detailed page.

### Step 0 — PX4 inner loops (once per airframe)

Run **PX4 Autotune** (rate + attitude loops) in Position mode — QGroundControl
→ Vehicle Setup → PID Tuning → Autotune. geo_tuner tunes the *outer*
position loop and assumes the inner loops are already good.

### Step 1 — thrust map from a hover ulog

Fly 1–2 min of steady hover in Position mode on a full battery, pull the
`.ulg`, then:

```bash
# from the package directory, using the venv you made above
.venv/bin/python scripts/geo-tuner-hover flight.ulg --mass 2.5 --design
```

This prints the measured hover throttle and `max_thrust`, and with
`--design` writes ready-to-use `geometric_controller.yaml` +
`geometric_mavros.yaml` with principled starting gains.

To drive the design knobs directly:

```bash
ros2 run geo_tuner geo-tuner-design \
    --mass 2.5 --hover-throttle 0.45 \
    --attctrl-tau 0.3 --zeta 0.95 --latency 0.08 --out-dir cfg/
```

### Step 2 — verify in simulation

`sim_tune.launch.py` above, then the full PX4 SITL cycle in
[docs/SITL_RECIPE.md](docs/SITL_RECIPE.md). Rehearse here before flying.

### Step 3 — in-flight auto-tune

1. Edit `config/tuner_field.yaml` — at minimum `min_tuning_altitude` (the
   height above ground the session refuses to start below) and the step
   sizes (the manoeuvre envelope: the vehicle stays within ± those of the
   point it is handed, so set them to the space you have).
2. Take off, hover where you want the session to run — at or above
   `min_tuning_altitude` — and hand over in OFFBOARD. The tuner tunes about
   that point: it never commands a climb, and if you hand it over too low
   it holds position and says so instead.
3. Keep a thumb on the RC mode switch — **switching out of OFFBOARD always
   overrides everything**. The conductor never arms, disarms, or changes
   modes.
4. Start the session:

```bash
ros2 launch geo_tuner field_tune.launch.py output_dir:=~/tuning_logs
```

Or start it from the RViz **Tuner** panel (recommended with a ground
laptop): launch with `require_enable:=true` and press START.

`output_dir` is the one knob for session data: each session writes a
timestamped report YAML plus a folder of raw per-episode CSVs under it, so
a day of repeated sessions never overwrites anything.

### Step 4 — after the session

- Copy `final_gains` from the report into `geometric_controller.yaml`.
- If the report suggests a `max_thrust` correction, apply it to
  `geometric_mavros.yaml` — that fix benefits everything, not just tuning.
- Keep the whole `output_dir` folder: the reports carry every episode's fit
  (α, τ, NRMSE, overshoot, action) and the CSVs are the raw responses
  behind them.
- For an agility pass, re-run with `attctrl_tau: 0.2` and a higher ladder.

---

## Ground station (RViz panels)

```bash
ros2 launch geo_tuner field_monitor.launch.py
```

Add the **GeoField** panel in RViz — it carries all four field panels as
tabs: Tuner (run a session), Health (controller state at a glance), Gains
(edit wn/ζ behind an interlock, save to the vehicle), Fly (drive
`trajectory_test_node`).

The panels are pure consumers and are built only where RViz is installed,
so a headless vehicle-side build skips them entirely.

## Tests

```bash
colcon test --packages-select geo_tuner && colcon test-result --all   # gtest units
ros2 launch geo_tuner sim_tune.launch.py                              # closed-loop e2e
```

## Documentation

| Document | Contents |
| --- | --- |
| [docs/SITL_RECIPE.md](docs/SITL_RECIPE.md) | Full tuning cycle in the d2dtracker PX4 SITL, command by command |
| [docs/FIELD_CHECKLIST.md](docs/FIELD_CHECKLIST.md) | Printable field-day checklist: bench prep → PX4 autotune → controller sanity → auto-tune → agility |
| [docs/TUNING_GUIDE.md](docs/TUNING_GUIDE.md) | The complete plan (phases 0–5), safety architecture, validation status, and Appendix A with every derivation |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Package layout, nodes, launch files, parameters, ROS interface, what the conductor does internally, session outputs |
| [docs/CONTROLLER_NOTES.md](docs/CONTROLLER_NOTES.md) | mav_controllers_ros quirks that shape this package (`ki` without dt, unused `kib`, `max_thrust` scaling, the setpoint contract) |
| [docs/PROJECT_STATE.md](docs/PROJECT_STATE.md) | Cross-repo state and history — where each branch stands |

Validated on ROS 2 Humble (d2dtracker docker image) against both upstream
`mav_controllers_ros` and its `production-hardening` branch. Licensed under
the terms in [LICENSE](LICENSE).
