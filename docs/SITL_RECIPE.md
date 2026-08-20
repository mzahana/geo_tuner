# SITL tuning recipe — full cycle, step by step

Everything runs inside the `d2dtracker_cuda` container. Prereqs (already
done on this machine): ros2_ws built with `mav_controllers_ros`
(production-hardening), `geo_tuner`, `d2dtracker_sim`, mavros; PX4 v1.14
SITL built.

Open three container terminals (each: `cd ~/src/d2dtracker_sim_docker &&
./docker_run_with_cuda.sh`). If your setup uses the zenoh RMW, keep your
zenoh router terminal as usual.

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
and starts streaming hover setpoints at (0, 0, 2.5) — this stream is what
makes PX4 willing to enter OFFBOARD. It waits in hover until you switch
modes; nothing moves yet.

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
