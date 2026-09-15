"""Signals derived from the bag and log for the checks.

The methods here are lifted from the prototype scripts that did the 09-14
analysis (docs/tuner_doctor/prototypes: steps.py hover_noise, zalpha.py
hover thrust) so the doctor reproduces the numbers that session established:
hover wander [7.5, 7.9, 7.4] cm, hover thrust 0.595 vs 0.597 predicted.
"""
from __future__ import annotations

import dataclasses

import numpy as np

from .bag import BagData
from .launch_log import LaunchLog

G = 9.81


@dataclasses.dataclass
class Window:
    """The tuning span in epoch seconds: OFFBOARD handover to Tuning
    complete. Falls back to first-settle/last-event when a log lacks the
    OFFBOARD gate (quad_sim sessions engage immediately)."""
    t0: float
    t1: float
    source: str

    @property
    def span_s(self) -> float:
        return self.t1 - self.t0


def session_window(log: LaunchLog) -> Window | None:
    start = log.first("offboard_engaged") or log.first("settling") or log.first("step")
    end = log.last("tuning_complete") or log.last("accepted")
    if start is None or start.epoch is None:
        return None
    t1 = end.epoch if end and end.epoch else log.events[-1].epoch
    if t1 is None:
        return None
    return Window(start.epoch, t1,
                  f"{start.type} -> {end.type if end else 'last event'}")


def sp_jumps(bag: BagData, thresh: float = 0.05) -> np.ndarray:
    """Times of setpoint changes (position or yaw), from the recorded
    multi_dof_setpoint stream."""
    sp = bag.arrays.get("sp")
    if sp is None or len(sp) < 2:
        return np.empty(0)
    d = np.abs(np.diff(sp[:, 1:5], axis=0))
    # yaw wraps; a pi jump in the yaw column that is only wrap is rare in a
    # tuning session (steps are 0.5 rad) and merely widens the exclusion.
    return sp[1:, 0][np.any(d > thresh, axis=1)]


def hover_noise(bag: BagData, exclude_s: float = 5.0,
                t_range: tuple[float, float] | None = None):
    """Hover position wander std per axis (m), the C2 method of
    prototypes/steps.py: odom minus the held setpoint, OFFBOARD only,
    excluding `exclude_s` after every setpoint jump, demeaned. `t_range`
    restricts to an epoch window (D3 compares before/after a gain change);
    velocity std rides along for the same windows."""
    od, sp, state = (bag.arrays.get(k) for k in ("odom", "sp", "state"))
    if od is None or sp is None or state is None:
        return None
    t = od[:, 0]
    spi = np.searchsorted(sp[:, 0], t).clip(1, len(sp) - 1)
    ref = sp[spi - 1, 1:4]
    err = od[:, 1:4] - ref
    ok = np.ones(len(t), bool)
    for j in sp_jumps(bag):
        ok &= ~((t >= j) & (t < j + exclude_s))
    offb = np.interp(t, state[:, 0], state[:, 1]) > 0.5
    ok &= offb
    if t_range is not None:
        ok &= (t >= t_range[0]) & (t <= t_range[1])
    if ok.sum() < 50:
        return None
    e = err[ok] - err[ok].mean(axis=0)
    vel = od[ok, 4:7]
    return {"std_m": e.std(axis=0), "n_samples": int(ok.sum()),
            "vel_std": (vel - vel.mean(axis=0)).std(axis=0)}


def hover_thrust(bag: BagData, window: Window | None,
                 exclude_s: float = 4.0) -> dict | None:
    """Median normalized thrust while parked (B3, prototypes/zalpha.py):
    excludes `exclude_s` after each setpoint jump > 0.3 m; the median is
    robust to the step transients that remain."""
    att = bag.arrays.get("att_thrust")
    sp = bag.arrays.get("sp")
    if att is None or sp is None or len(att) < 50:
        return None
    t = att[:, 0]
    ok = np.ones(len(t), bool)
    if window is not None:
        ok &= (t >= window.t0) & (t <= window.t1)
    jumps = sp[1:, 0][np.any(np.abs(np.diff(sp[:, 1:4], axis=0)) > 0.3, axis=1)]
    for j in jumps:
        ok &= ~((t >= j) & (t < j + exclude_s))
    if ok.sum() < 50:
        return None
    return {"median": float(np.median(att[ok, 1])), "n_samples": int(ok.sum())}


def odom_health(bag: BagData, window: Window | None) -> dict | None:
    """A3: rate, worst gap and stamp monotonicity of the controller's
    odometry inside the tuning window."""
    od = bag.arrays.get("odom")
    if od is None or len(od) < 10:
        return None
    t = od[:, 0]
    if window is not None:
        m = (t >= window.t0) & (t <= window.t1)
        if m.sum() > 10:
            t = t[m]
    dt = np.diff(t)
    return {
        "rate_hz": float(1.0 / np.median(dt)),
        "max_gap_s": float(dt.max()),
        "backwards_stamps": int((dt < 0).sum()),
        "n_samples": len(t),
    }


def altitude_at(bag: BagData, epoch: float) -> float | None:
    ra = bag.arrays.get("rel_alt")
    if ra is None or len(ra) < 2:
        return None
    return float(np.interp(epoch, ra[:, 0], ra[:, 1]))
