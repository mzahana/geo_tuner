# SITL tuning recipe — full cycle, step by step

Everything runs inside the `d2dtracker_cuda` container. Prereqs (already
done on this machine): ros2_ws built with `mav_controllers_ros`
(production-hardening), `geo_tuner`, `d2dtracker_sim`, mavros; PX4 v1.14
SITL built.

Open three container terminals (each: `cd ~/src/d2dtracker_sim_docker &&
./docker_run_with_cuda.sh`). If your setup uses the zenoh RMW, keep your
zenoh router terminal as usual.

## 0. (Optional) PX4 autotune + hover-ulog analysis — SITL rehearsal

Both Phase-0/Phase-1 steps of the tuning guide can be rehearsed in SITL.
The numbers you get apply to the *simulated* vehicle only — their value in
SITL is validating the procedure and the tooling end-to-end before doing
the same two flights on the real X500v2.

**Autotune (rate + attitude loops).** With the sim flying in Position
mode (after step 2 below):

```bash
$PX4BIN/px4-param set MC_AT_EN 1        # enable the autotune module (once, then restart PX4)
# ... hover in Position/Altitude mode, then:
$PX4BIN/px4-param set MC_AT_START 1     # start the identification sequence
```

The vehicle injects small excitation for ~40 s, computes rate/attitude
gains, and applies them (per MC_AT_APPLY). Easier alternative: QGroundControl
→ Vehicle Setup → PID Tuning → Autotune, which drives the same module with
progress UI.

**Hover ulog → max_thrust.** SITL logs ulog automatically while armed.
Fly 1–2 min of steady hover (Position mode), disarm, then find the log —
it's inside the shared volume, so it's visible on the host too:

```bash
ls -t ~/shared_volume/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/  | head
```

Analyze it on the host with the geo_tuner venv (pyulog lives there):

```bash
cd ~/src/ihunter_fixes/geo_tuner
.venv/bin/python -m geo_tuner.cli.analyze_hover \
    ~/d2dtracker_cuda_shared_volume/PX4-Autopilot/build/px4_sitl_default/rootfs/log/<date>/<file>.ulg \
    --mass 2.0 --design
```

(2.0 kg ≈ the sim x500's mass.) It prefers PX4's `hover_thrust_estimate`
topic — which SITL logs — and prints the hover throttle, the implied
`max_thrust`, and ready-to-use yaml. Cross-check: the in-flight tuner's
identified α and the ulog-derived `max_thrust` should agree.

## 1. Bring up sim + PX4 + mavros + controller (terminal A)

```bash
cd ~/shared_volume/ros2_ws && source install/setup.bash
ros2 launch d2dtracker_sim sitl_bringup.launch.py          # add headless:=1 if no GUI needed
```

Wait for PX4's `Ready for takeoff!` in the console. Gazebo shows the
x500 at the origin of the empty world.

## 2. Take off in a PX4-native mode (terminal B)

```bash
PX4BIN=~/shared_volume/PX4-Autopilot/build/px4_sitl_default/bin
$PX4BIN/px4-commander takeoff        # arms and climbs to ~2.5 m, then holds
```

Confirm it hovers (Gazebo, or `ros2 topic echo /interceptor/mavros/local_position/pose --once`).

## 3. Start the tuner (terminal C)

```bash
cd ~/shared_volume/ros2_ws && source install/setup.bash
ros2 launch geo_tuner field_tune.launch.py ns:=interceptor \
    params:=$(ros2 pkg prefix geo_tuner)/share/geo_tuner/config/tuner_sitl.yaml
```

The conductor reads the controller's current gains (its safe baseline)
and starts streaming setpoints — this stream is what makes PX4 willing
to enter OFFBOARD. It explicitly waits for `mavros/state` to report
OFFBOARD before any episode runs (status: "Waiting for PX4 mode
OFFBOARD"); until then it tracks the current position so the mode switch
is bumpless. If you flip out of OFFBOARD mid-session, tuning pauses
(episode discarded, gains kept) and resumes when you switch back.

## 4. Hand control to the geometric controller (terminal B)

```bash
$PX4BIN/px4-commander mode offboard
```

From this moment the tuning session is fully automatic:

- steps of ±0.4 m (z) / ±0.5 m (x, y), one axis at a time;
- each response fitted, gains corrected live (`wn` ladder 1.2 → 1.6 → 2.0
  rad/s, ζ = 0.95);
- watch progress:  `ros2 topic echo /interceptor/geo_tuner/status`
- any safety violation → gains restored, hover hold, session aborted with
  a diagnosis. To take over manually at ANY time:
  `$PX4BIN/px4-commander mode position` (this overrides offboard instantly).

The session ends with `Tuning complete` in terminal C; the vehicle keeps
hovering under the tuned controller.

## 5. Land and harvest the results (terminal B)

```bash
$PX4BIN/px4-commander mode auto:loiter   # leave offboard
$PX4BIN/px4-commander land
```

The report is at `~/shared_volume/geo_tuner_report.yaml` (also visible on
the host at `~/d2dtracker_cuda_shared_volume/geo_tuner_report.yaml`):

- `final_gains` → copy `kx`/`kv` into
  `d2dtracker_sim/config/geometric_controller/geometric_controller.yaml`
  (`gains.pos.*` / `gains.vel.*`), rebuild `d2dtracker_sim`;
- `wn_effective` / `zeta_effective` should sit at the ladder target
  (1.6–2.0 rad/s, ζ ≈ 0.95);
- if `diagnosis` suggests a `max_thrust` correction, apply it to
  `geometric_mavros.yaml` — that fixes the thrust map for everything.

## 6. Verify (optional but recommended)

Re-run steps 1–4 once with the new yaml: the first episodes should now
measure `wn` already at target (α ≈ 1), and the report shows small
corrections only. Then exercise a dynamic reference (e.g. the circular
trajectory node in `mav_controllers_ros`, or your MPC) and check tracking.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| OFFBOARD rejected | Conductor not running (no setpoint stream) — do step 3 before step 4 |
| Abort right after start: `position error bound exceeded` | Vehicle hovering far from (0,0,2.5) — take off at the origin, or edit `hover_position` |
| Abort: `below minimum altitude` | Take off first (step 2); the tuner never performs takeoff |
| `could not reach hover point` + thrust-map diagnosis | Sim `geometric_mavros.yaml` `max_thrust` off — apply the suggested factor |
| No odometry warning | mavros not up yet, or wrong namespace (`ns:=interceptor` missing) |
