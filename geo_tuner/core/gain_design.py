"""Principled gain design for mav_controllers_ros GeometricAttitudeControl.

The outer (position) loop of the geometric controller is feedback-linearized:

    a_fb = kx * e_pos + kv * e_vel   (+ integral, + feedforward)

so the ideal closed loop per axis is a double integrator under PD control:

    e_ddot + kv * e_dot + kx * e = 0

which maps directly to second-order dynamics:

    kx = wn^2          [1/s^2]
    kv = 2 * zeta * wn [1/s]

Gains are therefore designed by choosing (wn, zeta) subject to physical
constraints (inner-loop bandwidth, latency, thrust headroom) instead of
hand-twiddling kx/kv.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


G = 9.81


@dataclass
class VehicleParams:
    """Physical vehicle description (the only field-measured inputs)."""

    mass: float                     # kg, with battery
    hover_throttle: float           # normalized [0..1], from PX4 log at flight voltage
    max_tilt_angle: float = 0.52    # rad (30 deg default while tuning)

    @property
    def max_thrust(self) -> float:
        """Total max collective thrust [N] implied by the hover point.

        This is the value the geometric_mavros node must use so that
        commanded force in N maps to the correct normalized PX4 thrust.
        """
        return self.mass * G / self.hover_throttle

    @property
    def vertical_accel_headroom(self) -> float:
        """Max upward acceleration [m/s^2] beyond gravity compensation."""
        return G * (1.0 / self.hover_throttle - 1.0)

    @property
    def lateral_accel_max(self) -> float:
        """Max horizontal acceleration [m/s^2] at the tilt limit."""
        return G * math.tan(self.max_tilt_angle)


@dataclass
class LoopShape:
    """Design targets and constraints for the cascaded loops."""

    attctrl_tau: float = 0.3        # controller param; attitude BW ~ 2/tau [rad/s]
    timescale_separation: float = 4.0  # attitude BW / position BW, >= 3
    latency: float = 0.08           # s, EKF + mavros + offboard round trip
    latency_margin: float = 0.35    # require wn * latency <= this
    zeta: float = 0.95              # target damping (near-critical for chase)
    z_gain_factor: float = 1.6      # z loop can be stiffer (direct thrust authority)

    @property
    def attitude_bandwidth(self) -> float:
        return 2.0 / self.attctrl_tau

    @property
    def wn_max_separation(self) -> float:
        return self.attitude_bandwidth / self.timescale_separation

    @property
    def wn_max_latency(self) -> float:
        return self.latency_margin / self.latency


@dataclass
class GainSet:
    """A complete, ready-to-apply geometric controller gain set."""

    kx: tuple  # (x, y, z)
    kv: tuple
    wn_xy: float
    wn_z: float
    zeta: float
    attctrl_tau: float
    max_thrust: float
    notes: list = field(default_factory=list)

    def as_param_dict(self) -> dict:
        return {
            "gains.pos.x": self.kx[0],
            "gains.pos.y": self.kx[1],
            "gains.pos.z": self.kx[2],
            "gains.vel.x": self.kv[0],
            "gains.vel.y": self.kv[1],
            "gains.vel.z": self.kv[2],
            "attctrl_tau": self.attctrl_tau,
        }


def pd_from_wn_zeta(wn: float, zeta: float) -> tuple:
    """Map second-order targets to controller gains: (kx, kv)."""
    return wn * wn, 2.0 * zeta * wn


def wn_zeta_from_pd(kx: float, kv: float) -> tuple:
    """Inverse map: (wn, zeta) implied by a gain pair."""
    if kx <= 0:
        raise ValueError(f"kx must be > 0, got {kx}")
    wn = math.sqrt(kx)
    return wn, kv / (2.0 * wn)


def design_gains(vehicle: VehicleParams, shape: LoopShape,
                 wn_request: float | None = None) -> GainSet:
    """Compute a gain set honoring all constraints.

    wn_request: desired position-loop natural frequency [rad/s]. If None,
    the maximum allowed by the constraints is used. If the request exceeds
    a constraint, it is clipped and a note is recorded.
    """
    notes = []
    wn_cap = min(shape.wn_max_separation, shape.wn_max_latency)
    if shape.wn_max_latency < shape.wn_max_separation:
        notes.append(
            f"Position bandwidth limited by latency ({shape.latency*1e3:.0f} ms): "
            f"wn <= {shape.wn_max_latency:.2f} rad/s")
    else:
        notes.append(
            f"Position bandwidth limited by attitude loop (tau={shape.attctrl_tau}): "
            f"wn <= {shape.wn_max_separation:.2f} rad/s")

    wn_xy = wn_cap if wn_request is None else min(wn_request, wn_cap)
    if wn_request is not None and wn_request > wn_cap:
        notes.append(f"Requested wn={wn_request:.2f} clipped to {wn_cap:.2f} rad/s")

    wn_z = min(wn_xy * shape.z_gain_factor ** 0.5, wn_cap * shape.z_gain_factor ** 0.5)

    kx_xy, kv_xy = pd_from_wn_zeta(wn_xy, shape.zeta)
    kx_z, kv_z = pd_from_wn_zeta(wn_z, shape.zeta)

    # Sanity: acceleration a 1 m step would command vs. physical headroom.
    a_step = kx_xy * 1.0
    if a_step > vehicle.lateral_accel_max:
        notes.append(
            f"A 1 m lateral step commands {a_step:.1f} m/s^2 > tilt-limited "
            f"{vehicle.lateral_accel_max:.1f} m/s^2; the max_tilt/max_accel "
            "clamps will engage on large steps (acceptable, but keep field "
            "test steps small).")
    if vehicle.hover_throttle > 0.6:
        notes.append(
            f"Hover throttle {vehicle.hover_throttle:.2f} > 0.6: low thrust "
            "headroom; vehicle is heavy for its powertrain.")

    return GainSet(
        kx=(round(kx_xy, 3), round(kx_xy, 3), round(kx_z, 3)),
        kv=(round(kv_xy, 3), round(kv_xy, 3), round(kv_z, 3)),
        wn_xy=wn_xy, wn_z=wn_z, zeta=shape.zeta,
        attctrl_tau=shape.attctrl_tau,
        max_thrust=round(vehicle.max_thrust, 2),
        notes=notes,
    )


def correct_gains_from_identification(kx_applied: float, wn_measured: float,
                                      wn_target: float, zeta_target: float) -> tuple:
    """One iteration of identification-based gain correction.

    If the measured closed-loop natural frequency differs from the one
    predicted by the applied gains, the discrepancy is a multiplicative
    plant-gain error alpha (thrust-map error, inner-loop droop, ...):

        effective dynamics: e_ddot = -alpha*(kx e + kv e_dot)
        =>  wn_measured^2 = alpha * kx_applied

    Solving for the gains that place the *effective* poles at the target:

        kx_new = wn_target^2 / alpha
        kv_new = 2 * zeta_target * wn_target / alpha
    """
    if wn_measured <= 0 or kx_applied <= 0:
        raise ValueError("wn_measured and kx_applied must be > 0")
    alpha = (wn_measured ** 2) / kx_applied
    kx_new = wn_target ** 2 / alpha
    kv_new = 2.0 * zeta_target * wn_target / alpha
    return kx_new, kv_new, alpha


def geometric_controller_yaml(g: GainSet, mass: float,
                              max_tilt_angle: float = 0.52,
                              max_accel: float = 5.0) -> dict:
    """Full ros__parameters dict for geometric_controller.yaml."""
    return {
        "geometric_controller_node": {
            "ros__parameters": {
                "mass": float(mass),
                "use_external_yaw": True,
                "gains": {
                    "pos": {"x": g.kx[0], "y": g.kx[1], "z": g.kx[2]},
                    "vel": {"x": g.kv[0], "y": g.kv[1], "z": g.kv[2]},
                    # Integral off during tuning; the implementation
                    # accumulates per-callback (no dt) so keep tiny if used.
                    "ki": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "kib": {"x": 0.0, "y": 0.0, "z": 0.0},
                },
                "drag": {"kd": {"x": 0.0, "y": 0.0, "z": 0.0}},
                "attctrl_tau": float(g.attctrl_tau),
                "max_pos_int": 0.5,
                "mas_pos_int_b": 0.5,  # (sic) param name in the node
                "max_tilt_angle": float(max_tilt_angle),
                "max_accel": float(max_accel),
                "yaw_gain": 0.4,
            }
        }
    }


def geometric_mavros_yaml(g: GainSet) -> dict:
    return {
        "geometric_mavros_node": {
            "ros__parameters": {
                "num_props": 4,
                "kf": 1.0,
                "max_thrust": float(g.max_thrust),
                "lin_cof_a": 1.0,
                "lin_int_b": 0.0,
                "se3_cmd_timeout": 0.25,
            }
        }
    }
