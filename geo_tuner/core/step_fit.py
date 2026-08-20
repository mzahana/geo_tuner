"""Fit a second-order-plus-delay model to a recorded step response.

Used by the in-flight tuner: after a small setpoint step on one axis, the
position response is fit to

    y(t) = A * step2(t - td; wn, zeta)

where step2 is the unit step response of wn^2 / (s^2 + 2 zeta wn s + wn^2).
The identified (wn, zeta, td) tell us where the closed-loop poles actually
are — absorbing thrust-map error, inner-loop lag and transport delay — and
gain_design.correct_gains_from_identification turns that into a gain update.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares


@dataclass
class StepFitResult:
    wn: float          # rad/s
    zeta: float
    delay: float       # s
    amplitude: float   # fitted steady-state (should be ~ step size)
    rmse: float        # residual RMS, same units as y
    nrmse: float       # rmse / |step|
    overshoot: float   # fraction of step, from the data
    converged: bool

    @property
    def ok(self) -> bool:
        """Fit quality gate used before trusting a gain update."""
        return self.converged and self.nrmse < 0.15 and 0.05 < self.zeta < 2.5


def second_order_step(t: np.ndarray, wn: float, zeta: float) -> np.ndarray:
    """Unit step response of a standard second-order system (t >= 0)."""
    t = np.maximum(t, 0.0)
    if zeta < 1.0 - 1e-9:
        wd = wn * math.sqrt(1.0 - zeta * zeta)
        phi = math.acos(zeta)
        y = 1.0 - np.exp(-zeta * wn * t) / math.sqrt(1.0 - zeta * zeta) \
            * np.sin(wd * t + phi)
    elif zeta > 1.0 + 1e-9:
        s1 = -wn * (zeta - math.sqrt(zeta * zeta - 1.0))
        s2 = -wn * (zeta + math.sqrt(zeta * zeta - 1.0))
        y = 1.0 + (s2 * np.exp(s1 * t) - s1 * np.exp(s2 * t)) / (s1 - s2)
    else:  # critically damped
        y = 1.0 - np.exp(-wn * t) * (1.0 + wn * t)
    return np.where(t > 0, y, 0.0)


def fit_step_response(t: np.ndarray, y: np.ndarray, step: float,
                      wn_guess: float = 2.0, zeta_guess: float = 0.9,
                      ) -> StepFitResult:
    """Fit (wn, zeta, delay, amplitude) to a measured step response.

    t: seconds, 0 at step command time. y: position relative to the
    pre-step position (same sign convention as `step`).
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    if t.size < 10:
        raise ValueError("Need at least 10 samples to fit a step response")
    if abs(step) < 1e-6:
        raise ValueError("step must be nonzero")

    yn = y / step  # normalize to unit step

    def residuals(p):
        wn, zeta, td, amp = p
        return amp * second_order_step(t - td, wn, zeta) - yn

    p0 = np.array([wn_guess, zeta_guess, 0.05, 1.0])
    lb = np.array([0.1, 0.05, 0.0, 0.3])
    ub = np.array([30.0, 2.5, 0.6, 1.7])
    p0 = np.clip(p0, lb, ub)

    sol = least_squares(residuals, p0, bounds=(lb, ub), method="trf")
    wn, zeta, td, amp = sol.x
    res = residuals(sol.x)
    rmse = float(np.sqrt(np.mean(res ** 2))) * abs(step)
    nrmse = rmse / abs(step)

    # Overshoot straight from the data (robust to model mismatch)
    ss = float(np.mean(yn[t > (t[-1] - 0.2 * (t[-1] - t[0]))])) if t.size else 1.0
    peak = float(np.max(yn * np.sign(ss))) if ss != 0 else float(np.max(yn))
    overshoot = max(0.0, (peak - abs(ss)) / max(abs(ss), 1e-6))

    return StepFitResult(
        wn=float(wn), zeta=float(zeta), delay=float(td),
        amplitude=float(amp * step), rmse=rmse, nrmse=nrmse,
        overshoot=overshoot, converged=bool(sol.success),
    )
