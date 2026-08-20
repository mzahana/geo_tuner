"""Extract the hover thrust operating point from a PX4 ulog.

The geometric_mavros node maps commanded force [N] to normalized PX4
thrust as  throttle = force / max_thrust.  For that map to be correct at
hover (where accuracy matters most):

    max_thrust = mass * g / hover_throttle

hover_throttle is taken, in order of preference, from:
  1. PX4's hover thrust estimator topic (hover_thrust_estimate) — best;
  2. the mean commanded thrust (vehicle_thrust_setpoint / actuator_controls)
     over automatically detected hover segments (armed, |v| small).

Requires pyulog (offline tool only; the ROS nodes never import this).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

G = 9.81


@dataclass
class HoverAnalysis:
    hover_throttle: float
    hover_throttle_std: float
    source: str
    n_hover_seconds: float
    battery_voltage: float | None
    mass: float
    notes: list = field(default_factory=list)

    @property
    def max_thrust(self) -> float:
        return self.mass * G / self.hover_throttle

    def summary(self) -> str:
        lines = [
            f"hover_throttle : {self.hover_throttle:.4f} +/- {self.hover_throttle_std:.4f}  ({self.source})",
            f"hover data     : {self.n_hover_seconds:.1f} s",
            f"mass           : {self.mass:.3f} kg",
            f"max_thrust     : {self.max_thrust:.2f} N   <-- set in geometric_mavros.yaml",
        ]
        if self.battery_voltage is not None:
            lines.append(f"battery voltage: {self.battery_voltage:.2f} V (analysis conditions)")
        lines += [f"note: {n}" for n in self.notes]
        return "\n".join(lines)


def _get_dataset(ulog, name):
    try:
        return ulog.get_dataset(name)
    except (KeyError, IndexError, ValueError):
        return None


def analyze_hover_ulog(ulog_path: str, mass: float,
                       hover_speed_max: float = 0.4,
                       min_segment_s: float = 3.0) -> HoverAnalysis:
    from pyulog import ULog  # deferred: offline tool dependency

    ulog = ULog(ulog_path)
    notes = []

    # Battery voltage during flight (for repeatability notes)
    batt = _get_dataset(ulog, "battery_status")
    battery_voltage = None
    if batt is not None:
        v = batt.data.get("voltage_filtered_v", batt.data.get("voltage_v"))
        if v is not None and len(v):
            battery_voltage = float(np.median(v))

    # Preferred: PX4 hover thrust estimator
    hte = _get_dataset(ulog, "hover_thrust_estimate")
    if hte is not None and "hover_thrust" in hte.data and len(hte.data["hover_thrust"]) > 10:
        ht = np.asarray(hte.data["hover_thrust"], dtype=float)
        valid = ht[np.isfinite(ht) & (ht > 0.05) & (ht < 0.95)]
        if valid.size > 10:
            # Use the last third: the estimator converges over the flight
            tail = valid[-valid.size // 3:]
            return HoverAnalysis(
                hover_throttle=float(np.median(tail)),
                hover_throttle_std=float(np.std(tail)),
                source="hover_thrust_estimate (PX4 estimator)",
                n_hover_seconds=float(
                    (hte.data["timestamp"][-1] - hte.data["timestamp"][0]) * 1e-6),
                battery_voltage=battery_voltage, mass=mass, notes=notes)
        notes.append("hover_thrust_estimate present but unusable; "
                     "falling back to thrust-setpoint averaging")

    # Fallback: average commanded thrust over detected hover segments
    lpos = _get_dataset(ulog, "vehicle_local_position")
    if lpos is None:
        raise RuntimeError("ulog has neither usable hover_thrust_estimate "
                           "nor vehicle_local_position")
    t_pos = np.asarray(lpos.data["timestamp"], dtype=float) * 1e-6
    speed = np.sqrt(np.asarray(lpos.data["vx"]) ** 2 +
                    np.asarray(lpos.data["vy"]) ** 2 +
                    np.asarray(lpos.data["vz"]) ** 2)

    thr_t = thr_u = None
    ts = _get_dataset(ulog, "vehicle_thrust_setpoint")
    if ts is not None:
        thr_t = np.asarray(ts.data["timestamp"], dtype=float) * 1e-6
        # PX4 body z is down: hover thrust is -xyz[2]
        thr_u = -np.asarray(ts.data["xyz[2]"], dtype=float)
        source = "vehicle_thrust_setpoint over hover segments"
    else:
        ac = _get_dataset(ulog, "actuator_controls_0")
        if ac is not None and "control[3]" in ac.data:
            thr_t = np.asarray(ac.data["timestamp"], dtype=float) * 1e-6
            thr_u = np.asarray(ac.data["control[3]"], dtype=float)
            source = "actuator_controls_0.control[3] over hover segments"
    if thr_u is None:
        raise RuntimeError("No thrust setpoint topic found in ulog")

    hover_mask_pos = speed < hover_speed_max
    hover_mask = np.interp(thr_t, t_pos, hover_mask_pos.astype(float)) > 0.5
    hover_mask &= (thr_u > 0.1) & (thr_u < 0.95)

    # Keep only contiguous segments longer than min_segment_s
    keep = np.zeros_like(hover_mask)
    if hover_mask.any():
        idx = np.flatnonzero(np.diff(np.concatenate(([0], hover_mask.view(np.int8), [0]))))
        for a, b in zip(idx[::2], idx[1::2]):
            if thr_t[min(b - 1, len(thr_t) - 1)] - thr_t[a] >= min_segment_s:
                keep[a:b] = True
    if not keep.any():
        raise RuntimeError(
            f"No hover segments >= {min_segment_s}s with speed < "
            f"{hover_speed_max} m/s found. Fly a longer steady hover.")

    u = thr_u[keep]
    dur = float(np.sum(keep) * np.median(np.diff(thr_t)))
    return HoverAnalysis(
        hover_throttle=float(np.median(u)),
        hover_throttle_std=float(np.std(u)),
        source=source, n_hover_seconds=dur,
        battery_voltage=battery_voltage, mass=mass, notes=notes)
