"""In-flight auto-tuner for the mav_controllers_ros geometric controller.

Episode-based identification tuner ("Tier 2"): while the vehicle hovers in
OFFBOARD under the geometric controller, this node

  1. publishes hover setpoints (MultiDOFJointTrajectory, the controller's
     standard setpoint input — no custom messages needed);
  2. injects a small position step on one axis and records the response;
  3. fits a second-order-plus-delay model to the response;
  4. corrects kx/kv by pole placement toward the target (wn, zeta) via a
     live parameter update on the controller node (no landing/restart);
  5. repeats per axis, walking wn up a conservative ladder, and finally
     writes a tuned geometric_controller.yaml + full session report.

A fully independent SafetyMonitor watches odometry the entire time. Any
violation aborts the session: gains are restored to the last known-safe
set and the node holds a hover setpoint. The pilot's RC mode switch out
of OFFBOARD always overrides everything — this node never arms, disarms
or changes flight modes.

The node is intentionally plant-agnostic: it works identically against
the lightweight simulator (geo_tuner quad_sim), PX4 SITL, and the real
vehicle, because it only talks to the controller's ROS interface.
"""

from __future__ import annotations

import math
import time
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Parameter services used directly (rclpy.parameter_client does not exist
# on ROS 2 Humble, the distro on the Jetson/docker image)
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

import yaml
from geometry_msgs.msg import Transform, Twist
from mavros_msgs.msg import State as MavrosState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from trajectory_msgs.msg import (MultiDOFJointTrajectory,
                                 MultiDOFJointTrajectoryPoint)

from geo_tuner.core.first_order_fit import fit_first_order
from geo_tuner.core.gain_design import (
    correct_gains_from_identification, wn_zeta_from_pd)
from geo_tuner.core.safety import OdomSample, SafetyLimits, SafetyMonitor
from geo_tuner.core.step_fit import fit_step_response


class State(Enum):
    WAIT_ODOM = auto()
    WAIT_ENABLE = auto()
    WAIT_OFFBOARD = auto()
    GOTO_HOVER = auto()
    SETTLE = auto()
    STEP = auto()
    ANALYZE = auto()
    UPDATE_GAINS = auto()
    DONE = auto()
    ABORT = auto()


AXES = {"x": 0, "y": 1, "z": 2}

# Identified plant-gain factors outside this range are physically
# implausible (thrust maps are not off by >2.5x on a flying vehicle) and
# indicate a corrupted episode — such identifications are discarded.
ALPHA_MIN, ALPHA_MAX = 0.4, 2.5

# States in which the conductor is actively flying the vehicle (safety
# monitoring + offboard supervision apply).
ACTIVE_STATES = frozenset({State.GOTO_HOVER, State.SETTLE, State.STEP,
                           State.ANALYZE, State.UPDATE_GAINS})


class TuningConductor(Node):

    def __init__(self):
        super().__init__("tuning_conductor")

        # ---- parameters ----
        self.declare_parameter("controller_node", "geometric_controller_node")
        self.declare_parameter("setpoint_topic",
                               "geometric_controller/multi_dof_setpoint")
        self.declare_parameter("odom_topic", "geometric_controller/odom")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("hover_position", [0.0, 0.0, 3.0])
        self.declare_parameter("step_size", 0.5)        # m
        self.declare_parameter("step_size_z", 0.4)      # m
        self.declare_parameter("settle_time", 4.0)      # s before each step
        self.declare_parameter("hover_timeout", 20.0)   # s to reach hover point
        self.declare_parameter("episode_time", 6.0)     # s of recording
        # comma-separated to dodge YAML 1.1 parsing of bare "y" as a bool;
        # may include "yaw" for heading-loop identification
        self.declare_parameter("axes", "z,x,y,yaw")  # z first
        self.declare_parameter("yaw_step", 0.5)          # rad
        self.declare_parameter("yaw_time_constant", 0.35)  # s, target T
        self.declare_parameter("yawctrl_tau_min", 0.15)
        self.declare_parameter("yawctrl_tau_max", 1.2)
        # wn ladder: identify at each rung before pushing bandwidth up
        self.declare_parameter("wn_ladder", [1.2, 1.6, 2.0])
        self.declare_parameter("zeta_target", 0.95)
        self.declare_parameter("max_gain_change_factor", 1.6)
        self.declare_parameter("require_enable", False)  # gate on a Bool topic
        # OFFBOARD supervision: episodes only run while PX4 is in OFFBOARD
        # (otherwise the vehicle ignores our setpoints and every fit is
        # garbage). Disable only for plant simulators without mavros.
        self.declare_parameter("require_offboard", True)
        self.declare_parameter("mavros_state_topic", "mavros/state")
        self.declare_parameter("offboard_mode", "OFFBOARD")
        self.declare_parameter("report_path", "/tmp/geo_tuner_report.yaml")
        # safety limits
        self.declare_parameter("safety.max_tilt", 0.6)
        self.declare_parameter("safety.max_pos_error", 2.0)
        self.declare_parameter("safety.max_velocity", 4.0)
        self.declare_parameter("safety.min_altitude", 1.0)
        self.declare_parameter("safety.max_altitude", 40.0)
        self.declare_parameter("safety.odom_timeout", 0.4)
        self.declare_parameter("safety.osc_rate_rms", 1.2)

        gp = lambda n: self.get_parameter(n).value
        self.rate_hz = float(gp("rate_hz"))
        self.hover = list(gp("hover_position"))
        self.step_size = float(gp("step_size"))
        self.step_size_z = float(gp("step_size_z"))
        self.settle_time = float(gp("settle_time"))
        self.hover_timeout = float(gp("hover_timeout"))
        self.episode_time = float(gp("episode_time"))
        self.axes = [a.strip() for a in str(gp("axes")).split(",")
                     if a.strip() in AXES or a.strip() == "yaw"]
        self.yaw_step = float(gp("yaw_step"))
        self.yaw_T_target = float(gp("yaw_time_constant"))
        self.yaw_tau_min = float(gp("yawctrl_tau_min"))
        self.yaw_tau_max = float(gp("yawctrl_tau_max"))
        self.wn_ladder = [float(w) for w in gp("wn_ladder")]
        self.zeta_target = float(gp("zeta_target"))
        self.max_change = float(gp("max_gain_change_factor"))
        self.report_path = str(gp("report_path"))

        self.safety = SafetyMonitor(SafetyLimits(
            max_tilt=float(gp("safety.max_tilt")),
            max_pos_error=float(gp("safety.max_pos_error")),
            max_velocity=float(gp("safety.max_velocity")),
            min_altitude=float(gp("safety.min_altitude")),
            max_altitude=float(gp("safety.max_altitude")),
            odom_timeout=float(gp("safety.odom_timeout")),
            osc_rate_rms=float(gp("safety.osc_rate_rms")),
        ))

        # ---- interfaces ----
        self.sp_pub = self.create_publisher(
            MultiDOFJointTrajectory, gp("setpoint_topic"), 10)
        self.status_pub = self.create_publisher(String, "geo_tuner/status", 10)
        self.odom_sub = self.create_subscription(
            Odometry, gp("odom_topic"), self._odom_cb,
            qos_profile_sensor_data)
        self.require_offboard = bool(gp("require_offboard"))
        self.offboard_mode = str(gp("offboard_mode"))
        self.px4_mode: str | None = None
        if self.require_offboard:
            self.state_sub = self.create_subscription(
                MavrosState, gp("mavros_state_topic"), self._state_cb, 10)
        ctrl = str(gp("controller_node")).strip("/")
        self.get_param_cli = self.create_client(
            GetParameters, f"/{ctrl}/get_parameters")
        self.set_param_cli = self.create_client(
            SetParameters, f"/{ctrl}/set_parameters")

        # ---- state ----
        self.state = State.WAIT_ODOM
        self.state_t0 = self._now()
        self.odom: OdomSample | None = None
        self.setpoint = list(self.hover)
        self.setpoint_yaw = 0.0
        self.yaw_tau: float | None = None   # effective yaw time-constant param
        self.pre_step_yaw = 0.0
        self.rung = 0                  # index into wn_ladder
        self.axis_idx = 0
        self.step_sign = 1.0
        self.recording: list[tuple[float, float]] = []  # (t, pos[axis])
        self.step_t0 = 0.0
        self.pre_step_pos = 0.0
        self.gains: dict[str, tuple[float, float]] | None = None  # axis->(kx,kv)
        self.safe_gains: dict[str, tuple[float, float]] | None = None
        self.results: list[dict] = []
        self.abort_reason = ""
        self.diagnosis = ""
        # Acceleration feedforward trim [m/s^2]: cancels steady-state
        # offsets (thrust-map error on z, wind on x/y) the way an integral
        # term would, but transparently and bounded. Sent as the setpoint's
        # acceleration, which the controller uses as feedforward a_ref.
        self.a_trim = [0.0, 0.0, 0.0]
        self.trim_updates = 0
        self.max_trim = 3.0            # m/s^2 per axis
        self.max_trim_updates = 8
        self._pending_future = None
        self._analysis: dict | None = None

        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f"Tuning conductor up. axes={self.axes} wn_ladder={self.wn_ladder} "
            f"zeta={self.zeta_target} hover={self.hover}")

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _state_cb(self, msg: MavrosState):
        self.px4_mode = msg.mode

    @property
    def _in_offboard(self) -> bool:
        return (not self.require_offboard) or self.px4_mode == self.offboard_mode

    def _odom_cb(self, msg: Odometry):
        self.odom = OdomSample(
            t=self._now(),
            pos=(msg.pose.pose.position.x, msg.pose.pose.position.y,
                 msg.pose.pose.position.z),
            vel=(msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                 msg.twist.twist.linear.z),
            quat=(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                  msg.pose.pose.orientation.y, msg.pose.pose.orientation.z),
            body_rates=(msg.twist.twist.angular.x, msg.twist.twist.angular.y,
                        msg.twist.twist.angular.z))

    def _publish_setpoint(self):
        msg = MultiDOFJointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        pt = MultiDOFJointTrajectoryPoint()
        tr = Transform()
        tr.translation.x, tr.translation.y, tr.translation.z = self.setpoint
        tr.rotation.w = math.cos(0.5 * self.setpoint_yaw)
        tr.rotation.z = math.sin(0.5 * self.setpoint_yaw)
        pt.transforms.append(tr)
        pt.velocities.append(Twist())
        acc = Twist()
        acc.linear.x, acc.linear.y, acc.linear.z = self.a_trim
        pt.accelerations.append(acc)
        msg.points.append(pt)
        self.sp_pub.publish(msg)

    def _status(self, text: str, log: bool = True):
        if log:
            self.get_logger().info(text)
        self.status_pub.publish(String(data=f"[{self.state.name}] {text}"))

    def _goto(self, state: State):
        self.state = state
        self.state_t0 = self._now()

    @staticmethod
    def _yaw_of(q: tuple) -> float:
        w, x, y, z = q
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    # ------------------------------------------------------------------
    # gain get/set through the controller's parameter interface
    def _request_gains(self):
        req = GetParameters.Request()
        req.names = ["gains.pos.x", "gains.pos.y", "gains.pos.z",
                     "gains.vel.x", "gains.vel.y", "gains.vel.z",
                     "yawctrl_tau", "attctrl_tau"]
        self._pending_future = self.get_param_cli.call_async(req)

    def _apply_yaw_tau(self, tau: float):
        req = SetParameters.Request()
        req.parameters.append(ParameterMsg(
            name="yawctrl_tau",
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=float(tau))))
        self._pending_future = self.set_param_cli.call_async(req)

    def _apply_gains(self, gains: dict[str, tuple[float, float]]):
        req = SetParameters.Request()
        for ax, (kx, kv) in gains.items():
            for name, val in ((f"gains.pos.{ax}", kx), (f"gains.vel.{ax}", kv)):
                req.parameters.append(ParameterMsg(
                    name=name,
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE,
                        double_value=float(val))))
        self._pending_future = self.set_param_cli.call_async(req)

    # ------------------------------------------------------------------
    def _tick(self):
        now = self._now()

        # Safety runs while the conductor is actively flying the vehicle.
        # It deliberately does NOT run in WAIT_OFFBOARD: there the pilot /
        # another mode is in command and e.g. tilt limits don't apply.
        if self.odom is not None and self.state in ACTIVE_STATES:
            active_sp = tuple(self.setpoint)
            violations = self.safety.check(self.odom, active_sp)
            violations += self.safety.check_stale(now)
            if violations:
                self._abort(", ".join(v.value for v in violations))

        # Pilot/mode supervision: leaving OFFBOARD mid-session pauses the
        # tuner (episode discarded, gains kept) until OFFBOARD returns.
        if self.state in ACTIVE_STATES and not self._in_offboard:
            self._status(f"PX4 left {self.offboard_mode} "
                         f"(now: {self.px4_mode}); pausing tuning")
            self.recording = []
            self.safety.reset()
            self._goto(State.WAIT_OFFBOARD)

        # Setpoint stream must never stop while OFFBOARD is active (and it
        # must already flow in WAIT_OFFBOARD, or PX4 refuses the switch).
        if self.state not in (State.WAIT_ODOM, State.WAIT_ENABLE):
            self._publish_setpoint()

        handler = getattr(self, f"_st_{self.state.name.lower()}")
        handler(now)

    # ---- states ----
    def _st_wait_odom(self, now):
        if self.odom is not None:
            self._status("Odometry received; requesting current gains")
            self._request_gains()
            self._goto(State.WAIT_ENABLE)

    def _st_wait_enable(self, now):
        # Capture the controller's current gains as the known-safe baseline
        if self._pending_future is not None and self._pending_future.done():
            res = self._pending_future.result()
            self._pending_future = None
            try:
                vals = [p.double_value for p in res.values[:6]]
            except (AttributeError, IndexError):
                self._abort("could not read controller gains")
                return
            self.gains = {"x": (vals[0], vals[3]),
                          "y": (vals[1], vals[4]),
                          "z": (vals[2], vals[5])}
            self.safe_gains = dict(self.gains)
            for ax, (kx, kv) in self.gains.items():
                wn, zeta = wn_zeta_from_pd(kx, kv)
                self._status(f"baseline {ax}: kx={kx:.2f} kv={kv:.2f} "
                             f"(wn={wn:.2f}, zeta={zeta:.2f})")
            # Yaw time constant: yawctrl_tau if the controller has it and
            # it is > 0, else it follows attctrl_tau. A controller without
            # the parameter (upstream build) can't be yaw-tuned.
            yaw_tau = att_tau = 0.0
            if len(res.values) >= 8:
                if res.values[6].type == ParameterType.PARAMETER_DOUBLE:
                    yaw_tau = res.values[6].double_value
                if res.values[7].type == ParameterType.PARAMETER_DOUBLE:
                    att_tau = res.values[7].double_value
            if yaw_tau > 0.0:
                self.yaw_tau = yaw_tau
            elif att_tau > 0.0:
                self.yaw_tau = att_tau
            if "yaw" in self.axes:
                if self.yaw_tau is None:
                    self._status("controller has no yawctrl_tau/attctrl_tau "
                                 "params; skipping yaw axis")
                    self.axes = [a for a in self.axes if a != "yaw"]
                else:
                    self._status(f"baseline yaw: tau={self.yaw_tau:.3f} "
                                 f"(target T={self.yaw_T_target:.2f}s)")
            self.setpoint = list(self.hover)
            if not self._in_offboard:
                self._status(f"Waiting for PX4 mode {self.offboard_mode} "
                             "(setpoint stream active; switch modes to start)")
                self._goto(State.WAIT_OFFBOARD)
            else:
                self._goto(State.GOTO_HOVER)

    def _st_wait_offboard(self, now):
        # Track the current position so the eventual OFFBOARD engage is
        # bumpless; the session then proceeds via GOTO_HOVER.
        if self.odom is not None:
            self.setpoint = list(self.odom.pos)
        if self._in_offboard:
            self._status(f"{self.offboard_mode} engaged; resuming "
                         f"(rung {self.rung + 1}/{len(self.wn_ladder)}, "
                         f"axis {self.axes[self.axis_idx]})")
            self.setpoint = list(self.hover)
            self._goto(State.GOTO_HOVER)

    def _st_goto_hover(self, now):
        if self.odom is None:
            return
        err = math.dist(self.odom.pos, self.setpoint)
        if err < 0.3 and math.sqrt(sum(v * v for v in self.odom.vel)) < 0.3:
            self._status(f"At hover point; settling {self.settle_time}s "
                         f"(rung {self.rung + 1}/{len(self.wn_ladder)}, "
                         f"axis {self.axes[self.axis_idx]})")
            self._goto(State.SETTLE)
            return
        # Steady offset (velocity small, error persistent): absorb it into
        # the acceleration feedforward trim, the bounded stand-in for
        # integral action. The residual force each axis is missing equals
        # kx * offset (that's what the feedback is currently supplying).
        speed = math.sqrt(sum(v * v for v in self.odom.vel))
        if now - self.state_t0 > 5.0 and speed < 0.3 and self.gains is not None:
            updated = False
            for ax, i in AXES.items():
                off = self.setpoint[i] - self.odom.pos[i]
                if abs(off) > 0.15:
                    kx = self.gains[ax][0]
                    new = self.a_trim[i] + 0.8 * kx * off
                    self.a_trim[i] = max(-self.max_trim,
                                         min(self.max_trim, new))
                    updated = True
            if updated:
                self.trim_updates += 1
                self._status(
                    f"steady offset -> accel trim "
                    f"[{self.a_trim[0]:+.2f}, {self.a_trim[1]:+.2f}, "
                    f"{self.a_trim[2]:+.2f}] m/s^2 "
                    f"({self.trim_updates}/{self.max_trim_updates})")
                if (self.trim_updates > self.max_trim_updates
                        or any(abs(a) >= self.max_trim for a in self.a_trim)):
                    self._set_trim_diagnosis()
                    self._abort("steady-state offset exceeds trim authority"
                                + (f"; {self.diagnosis}" if self.diagnosis else ""))
                    return
                self.state_t0 = now  # give the trim time to act
                return
        if now - self.state_t0 > self.hover_timeout:
            self._set_trim_diagnosis()
            self._abort(f"could not reach hover point in "
                        f"{self.hover_timeout:.0f}s"
                        + (f"; {self.diagnosis}" if self.diagnosis else ""))

    def _set_trim_diagnosis(self):
        """A persistent z trim maps 1:1 to a thrust-map (max_thrust) error."""
        az = self.a_trim[2]
        if abs(az) > 0.15:
            alpha = 9.81 / (9.81 + az)
            self.diagnosis = (
                f"z accel trim {az:+.2f} m/s^2 implies thrust-map scale "
                f"~{alpha:.2f}: multiply max_thrust in geometric_mavros.yaml "
                f"by {alpha:.2f}")

    def _st_settle(self, now):
        if now - self.state_t0 < self.settle_time:
            return
        ax = self.axes[self.axis_idx]
        self.recording = []
        self.step_t0 = now
        if ax == "yaw":
            step = self.yaw_step * self.step_sign
            self.pre_step_yaw = self._yaw_of(self.odom.quat)
            self.setpoint = list(self.hover)
            self.setpoint_yaw = self.pre_step_yaw + step
            self._status(f"Yaw step {step:+.2f} rad")
        else:
            i = AXES[ax]
            step = self.step_size_z if ax == "z" else self.step_size
            step *= self.step_sign
            self.pre_step_pos = self.odom.pos[i]
            self.setpoint = list(self.hover)
            self.setpoint[i] += step
            self._status(f"Step {step:+.2f} m on {ax}")
        self._goto(State.STEP)

    def _st_step(self, now):
        ax = self.axes[self.axis_idx]
        if self.odom is not None:
            if ax == "yaw":
                dyaw = self._yaw_of(self.odom.quat) - self.pre_step_yaw
                dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))  # unwrap
                self.recording.append((now - self.step_t0, dyaw))
            else:
                i = AXES[ax]
                self.recording.append((now - self.step_t0,
                                       self.odom.pos[i] - self.pre_step_pos))
        if now - self.state_t0 >= self.episode_time:
            self._goto(State.ANALYZE)

    def _analyze_yaw(self, now):
        t = np.array([r[0] for r in self.recording])
        y = np.array([r[1] for r in self.recording])
        step = self.yaw_step * self.step_sign
        try:
            fit = fit_first_order(t, y, step=step,
                                  T_guess=max(self.yaw_tau / 2.0, 0.05))
        except (ValueError, RuntimeError) as e:
            self._abort(f"yaw fit failed: {e}")
            return
        rec = {"axis": "yaw", "rung": self.rung,
               "T_target": self.yaw_T_target, "step": round(step, 3),
               "T_meas": round(fit.T, 3), "delay": round(fit.delay, 3),
               "nrmse": round(fit.nrmse, 3),
               "yaw_tau_applied": round(self.yaw_tau, 3)}
        self._status(f"yaw: T={fit.T:.2f}s delay={fit.delay * 1e3:.0f}ms "
                     f"nrmse={fit.nrmse:.2f}")
        self.setpoint_yaw = 0.0  # heading back to nominal after episode
        if not fit.ok:
            rec["action"] = "fit rejected; keeping yawctrl_tau"
            self.results.append(rec)
            self._status("Yaw fit quality gate failed; not updating")
            self._next_episode()
            return
        # T scales with the applied tau through the same (unknown)
        # efficiency factor, which cancels in the ratio update.
        tau_new = self.yaw_tau * self.yaw_T_target / fit.T
        tau_new = float(np.clip(tau_new, self.yaw_tau / self.max_change,
                                self.yaw_tau * self.max_change))
        tau_new = float(np.clip(tau_new, self.yaw_tau_min, self.yaw_tau_max))
        rec.update({"yaw_tau_new": round(tau_new, 3), "action": "tau updated"})
        self.results.append(rec)
        self._status(f"yaw: tau {self.yaw_tau:.3f} -> {tau_new:.3f}")
        self.yaw_tau = tau_new
        self._apply_yaw_tau(tau_new)
        self._goto(State.UPDATE_GAINS)

    def _st_analyze(self, now):
        ax = self.axes[self.axis_idx]
        if ax == "yaw":
            self._analyze_yaw(now)
            return
        i = AXES[ax]
        step = (self.step_size_z if ax == "z" else self.step_size) * self.step_sign
        t = np.array([r[0] for r in self.recording])
        y = np.array([r[1] for r in self.recording])
        kx_now, kv_now = self.gains[ax]
        wn_pred = math.sqrt(kx_now)
        try:
            fit = fit_step_response(
                t, y, step=step, wn_guess=wn_pred,
                zeta_guess=self.zeta_target,
                wn_bounds=(wn_pred * math.sqrt(ALPHA_MIN),
                           wn_pred * math.sqrt(ALPHA_MAX)))
        except (ValueError, RuntimeError) as e:
            self._abort(f"step fit failed on {ax}: {e}")
            return

        wn_target = self.wn_ladder[self.rung]
        rec = {"axis": ax, "rung": self.rung, "wn_target": wn_target,
               "step": step, "wn_meas": round(fit.wn, 3),
               "zeta_meas": round(fit.zeta, 3), "delay": round(fit.delay, 3),
               "nrmse": round(fit.nrmse, 3),
               "overshoot": round(fit.overshoot, 3),
               "kx_applied": round(kx_now, 3), "kv_applied": round(kv_now, 3)}
        self._status(f"{ax}: wn={fit.wn:.2f} zeta={fit.zeta:.2f} "
                     f"delay={fit.delay * 1e3:.0f}ms nrmse={fit.nrmse:.2f} "
                     f"os={fit.overshoot * 100:.0f}%")

        if not fit.ok:
            rec["action"] = "fit rejected; keeping gains"
            self.results.append(rec)
            self._status(f"Fit quality gate failed on {ax}; not updating gains")
            self._next_episode()
            return

        # Latency sanity: don't push bandwidth into the delay margin
        if fit.delay > 0 and wn_target * fit.delay > 0.45:
            rec["action"] = (f"wn_target {wn_target:.2f} unsafe with measured "
                             f"delay {fit.delay * 1e3:.0f} ms; ladder stopped")
            self.results.append(rec)
            self._status(rec["action"])
            self._finish()
            return

        kx_new, kv_new, alpha = correct_gains_from_identification(
            kx_now, fit.wn, wn_target, self.zeta_target)
        # Plausibility gate: a real vehicle's thrust map is not off by
        # more than ~2.5x. An alpha outside the box means the episode was
        # corrupted (bias transient, mode change, fit ambiguity) — discard.
        if not (ALPHA_MIN <= alpha <= ALPHA_MAX):
            rec.update({"alpha": round(alpha, 3),
                        "action": "alpha implausible; keeping gains"})
            self.results.append(rec)
            self._status(f"{ax}: alpha={alpha:.2f} outside "
                         f"[{ALPHA_MIN}, {ALPHA_MAX}]; discarding episode")
            self._next_episode()
            return
        # Rate-limit gain changes per episode
        kx_new = float(np.clip(kx_new, kx_now / self.max_change,
                               kx_now * self.max_change))
        kv_new = float(np.clip(kv_new, kv_now / self.max_change,
                               kv_now * self.max_change))
        rec.update({"alpha": round(alpha, 3), "kx_new": round(kx_new, 3),
                    "kv_new": round(kv_new, 3), "action": "gains updated"})
        self.results.append(rec)

        self.safe_gains = dict(self.gains)  # current set flew safely
        self.gains[ax] = (kx_new, kv_new)
        self._analysis = {"axis": ax}
        self._apply_gains({ax: self.gains[ax]})
        self._status(f"{ax}: alpha={alpha:.2f} -> kx {kx_now:.2f}->{kx_new:.2f}, "
                     f"kv {kv_now:.2f}->{kv_new:.2f}")
        self._goto(State.UPDATE_GAINS)

    def _st_update_gains(self, now):
        if self._pending_future is None or not self._pending_future.done():
            if now - self.state_t0 > 3.0:
                self._abort("parameter update timed out")
            return
        res = self._pending_future.result()
        self._pending_future = None
        try:
            ok = all(r.successful for r in res.results)
        except AttributeError:
            ok = False
        if not ok:
            self._abort("controller rejected parameter update")
            return
        self._next_episode()

    def _next_episode(self):
        # Alternate step direction to stay centered on the hover point
        self.step_sign *= -1.0
        self.axis_idx += 1
        if self.axis_idx >= len(self.axes):
            self.axis_idx = 0
            self.rung += 1
            if self.rung >= len(self.wn_ladder):
                self._finish()
                return
        self.setpoint = list(self.hover)
        self._goto(State.GOTO_HOVER)

    def _finish(self):
        self.setpoint = list(self.hover)
        self._set_trim_diagnosis()
        self._write_report(status="complete")
        self._status(f"Tuning complete. Report: {self.report_path}")
        self._goto(State.DONE)

    def _st_done(self, now):
        pass  # keep publishing hover setpoint until pilot takes over

    def _abort(self, reason: str):
        if self.state == State.ABORT:
            return
        self.abort_reason = reason
        self.get_logger().error(f"ABORT: {reason}")
        if (not self.diagnosis and self.odom is not None
                and abs(self.setpoint[2] - self.odom.pos[2]) > 0.5):
            self.diagnosis = (
                "large altitude error at abort: likely thrust-map error. "
                "Verify max_thrust in geometric_mavros.yaml — fly a "
                "Position-mode hover and run geo-tuner-hover on the ulog.")
        self.setpoint = list(self.hover)
        if self.safe_gains is not None and self.gains != self.safe_gains:
            self.gains = dict(self.safe_gains)
            self._apply_gains(self.gains)
            self.get_logger().warn("Restored last known-safe gains")
        self._write_report(status=f"aborted: {reason}")
        self._goto(State.ABORT)

    def _st_abort(self, now):
        pass  # hold hover; pilot takes over via RC

    # ------------------------------------------------------------------
    def _write_report(self, status: str):
        # last identified plant-gain factor per axis (lumps thrust-map
        # error and inner-loop lag); effective wn = sqrt(alpha * kx)
        alpha_by_axis = {}
        for r in self.results:
            if "alpha" in r:
                alpha_by_axis[r["axis"]] = r["alpha"]
        final = {}
        if self.gains:
            for ax, (kx, kv) in self.gains.items():
                wn, zeta = wn_zeta_from_pd(kx, kv)
                final[ax] = {"kx": round(kx, 3), "kv": round(kv, 3),
                             "wn_nominal": round(wn, 3),
                             "zeta_nominal": round(zeta, 3)}
                if ax in alpha_by_axis:
                    a = alpha_by_axis[ax]
                    final[ax]["alpha"] = a
                    final[ax]["wn_effective"] = round(math.sqrt(a * kx), 3)
                    final[ax]["zeta_effective"] = round(
                        a * kv / (2.0 * math.sqrt(a * kx)), 3)
        report = {
            "status": status,
            "diagnosis": self.diagnosis,
            "accel_trim": [round(a, 3) for a in self.a_trim],
            "final_yawctrl_tau": (round(self.yaw_tau, 3)
                                  if self.yaw_tau is not None else None),
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "zeta_target": self.zeta_target,
            "wn_ladder": self.wn_ladder,
            "episodes": self.results,
            "final_gains": final,
            "controller_yaml_snippet": {
                "gains": {
                    "pos": {ax: final[ax]["kx"] for ax in final},
                    "vel": {ax: final[ax]["kv"] for ax in final},
                } if final else {},
            },
        }
        try:
            with open(self.report_path, "w") as f:
                yaml.safe_dump(report, f, sort_keys=False)
        except OSError as e:
            self.get_logger().error(f"could not write report: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TuningConductor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
