# Field Tuning Checklist — X500v2 + Geometric Controller

A step-by-step checklist for tuning the drone and the geometric controller
in the field. Print it or keep it open on the laptop. Work top to bottom;
don't skip boxes. Simple language on purpose.

Roles: **Pilot** (RC in hand at all times, ready to take over) and
**Operator** (laptop). One person can do both, but two is better.

The full math and reasoning is in [TUNING_GUIDE.md](TUNING_GUIDE.md).
This page is only the "do this, then this" version.

---

## 0. At the bench (before driving out)

Vehicle and software:

- [ ] Battery charged (plus spares). Props tight, correct orientation.
- [ ] Jetson boots, ROS 2 workspace builds, latest `mav_controllers_ros`
      (hardened) and `geo_tuner` pulled and built.
- [ ] mavros connects to PX4; `mavros/local_position/odom` streams at
      ~100 Hz (`ros2 topic hz /<ns>/mavros/local_position/odom`).
- [ ] Controller config has the **real vehicle** numbers, not SITL:
  - [ ] `mass: 2.5` (geometric_controller.yaml AND geometric_mavros.yaml)
  - [ ] `max_thrust`: from a real hover log (Flight 1 below). Until then
        use the design estimate: `max_thrust = m*g / hover_throttle`.
  - [ ] `hold_on_setpoint_timeout: true` (safety: controller holds
        position if a test node dies).
- [ ] Starting gains computed and written into the yaml:

      ```bash
      geo-tuner-design --mass 2.5 --hover-throttle <from a hover log> \
          --wn 1.2 --zeta 0.95 --attctrl-tau 0.3
      ```

- [ ] PX4 failsafes set: RC loss action, geofence around the test area,
      `COM_OBL_ACT` (offboard loss) = Hold or Return, battery failsafe on.
- [ ] Pilot has practiced the takeover: flip out of OFFBOARD → Position
      mode. This overrides everything, always.

Pack: laptop, RC, batteries, props, USB cables, this checklist.

---

## 1. Flight 1 — PX4 basics (no geometric controller yet)

Goal: good inner loops and a real thrust number. All in **Position mode**.

- [ ] Take off in Position mode. Check it flies normally: steady hover,
      clean stick response, no oscillation.
- [ ] Run **PX4 autotune** (QGroundControl → Vehicle Setup → PID Tuning →
      Autotune, or `MC_AT_START`). Keep hovering while it excites the
      vehicle (~40 s). Accept the new gains.
- [ ] Re-check feel after autotune: small, crisp stick inputs.
- [ ] Fly **1–2 minutes of steady hover**, then land and disarm
      (this writes the hover ulog we need).
- [ ] Pull the ulog and compute the real thrust scale:

      ```bash
      geo-tuner-hover /path/to/hover.ulg --mass 2.5
      ```

- [ ] Write the reported `max_thrust` into geometric_mavros.yaml.
      **This number matters more than any gain.**

Swap/charge battery.

---

## 2. Flight 2 — geometric controller sanity check

Goal: prove the controller holds and moves the drone safely **before**
any auto-tuning. Use the trajectory test node in `setpoint` mode
(see mav_controllers_ros `docs/TRAJECTORY_TESTING.md`).

- [ ] Launch controller + mavros interface + trajectory test node
      (with RViz if you have a screen: `rviz:=true`).
- [ ] Take off in Position mode, climb to **≥ 10 m**, hover.
- [ ] Switch to **OFFBOARD**. The trajectory node streams "hold here" —
      the drone should just sit still. Watch 20–30 s.
  - [ ] Altitude steady (no slow climb/sink → if it sinks or climbs,
        `max_thrust` is off; land and re-check step 1).
  - [ ] No wobble or oscillation. If it shakes: pilot takes over, land,
        reduce gains (lower `--wn`), try again.
- [ ] Small moves: set `setpoint.x: 2.0` (relative), call
      `.../trajectory_test/start`. The drone flies a gentle 2 m move.
      Repeat for y and z (±1–2 m). Each move should be smooth, with a
      small overshoot at most.
- [ ] Pilot takeover drill (once, on purpose): flip to Position mode
      mid-move. The controller must hand over instantly.
- [ ] Land (pilot, Position mode). 

If any box fails here, **do not continue** to auto-tuning. Fix it first.

---

## 3. Flight 3 — in-flight auto-tune (tuning conductor)

Goal: identified, corrected gains on the real vehicle.

Before arming:

- [ ] Open `config/tuner_field.yaml` and check:
  - [ ] `hover_position`: ≥ 10 m altitude, inside the geofence, clear air.
  - [ ] `wn_ladder`: start modest, e.g. `[1.2, 1.6]`. You can ladder
        higher on a later flight.
  - [ ] safety block: `min_altitude`, `max_altitude`, `max_pos_error`
        match your field, not someone else's.
- [ ] Fresh battery (a full session with `episodes_per_rung: 3` takes
      most of one).

Fly:

- [ ] Take off in Position mode, climb near the hover point, switch to
      OFFBOARD (controller holds).
- [ ] Start the tuner:

      ```bash
      ros2 launch geo_tuner field_tune.launch.py ns:=<ns> \
          params:=config/tuner_field.yaml
      ```

- [ ] The conductor hovers, then steps each axis (z first, then x, y,
      yaw), several times per rung. **This looks like small, repeated
      nudges — that is normal.** Watch the terminal: it prints each
      identification and any gain update.
- [ ] Pilot: hands on sticks the whole time. Abort = flip out of
      OFFBOARD. The conductor pauses on mode change and resumes when
      you come back to OFFBOARD; its own safety monitor also aborts and
      restores the last safe gains on tilt/error/oscillation limits.
- [ ] When it prints the session summary, land and disarm.

After landing:

- [ ] Open the report (`report_path`, default `/tmp/geo_tuner_report.yaml`).
- [ ] Check `estimate spreads` ≤ ~1.15× (consistent identification).
      If spreads are large or many episodes were rejected: fly the
      session again before trusting the gains.
- [ ] Copy the `controller_yaml_snippet` from the report into
      geometric_controller.yaml (and `final_yawctrl_tau` if present).
      **Gains applied in flight are RAM-only — if you skip this, they
      are gone at the next reboot.**
- [ ] Commit the yaml change (or at least back it up) before flying on.

---

## 4. Flight 4 — verify tracking, then push agility

Goal: measure how well the tuned controller tracks, then raise speed
step by step. Uses the trajectory test node (`circle`, then
`lemniscate`).

- [ ] Launch with RViz: `rviz:=true`. Start a bag recording:

      ```bash
      ros2 bag record /<ns>/geometric_controller/setpoint \
          /<ns>/mavros/local_position/odom \
          /<ns>/geometric_controller/control_errors
      ```

- [ ] Take off, hover ≥ 10 m, OFFBOARD.
- [ ] Circle, gentle: `radius: 5`, `speed: 2`. Call start. Watch RViz:
      red (actual) should sit on yellow (reference).
- [ ] Call stop between runs, then raise speed one step at a time:

      ```bash
      ros2 param set /<ns>/trajectory_test_node speed 3.0   # then 4.0 ...
      ```

  - [ ] After each run, look at the tracking error (RViz gap, or
        `control_errors`). Stop raising speed when the error grows fast
        or the vehicle looks stressed — that is the current agility
        limit; note it.
  - [ ] If the node says "derated by accel/jerk limits", enlarge the
        circle radius instead of pushing speed.
- [ ] Repeat with `trajectory_type: lemniscate` (the figure-8 has
      direction reversals — a much harder test; expect to find the
      limit at lower speed than the circle).
- [ ] Land, stop the bag. Keep the bags and the tuner reports together —
      they are the evidence for this tune.

---

## 5. Back home

- [ ] All yaml changes committed and pushed.
- [ ] Tuner reports + bags archived (date + battery + weather notes).
- [ ] Note the found agility limits (max clean circle speed, max clean
      figure-8 speed) in the flight log.
- [ ] Anything odd (aborts, oscillation, big z offset) → open an issue /
      note it in PROJECT_STATE.md so the next session starts informed.

---

## Quick emergency card

| Situation | Action |
|---|---|
| Anything looks wrong | Pilot: flip out of OFFBOARD (Position mode). Always works, always allowed. |
| Drone oscillates / shakes | Take over, land. Lower gains (smaller `--wn` or previous yaml). Don't re-try with the same gains. |
| Drone slowly sinks/climbs in hover under the controller | `max_thrust` wrong → redo the hover-ulog step. |
| Tuner aborted itself | It restored the last safe gains and holds. Read its message, land if unsure. |
| Trajectory node died | Controller holds position by itself (`hold_on_setpoint_timeout`). Take over when ready. |
| Odometry lost | Controller goes silent → PX4 offboard-loss failsafe acts. Pilot takes over immediately. |
