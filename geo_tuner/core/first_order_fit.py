"""Fit a first-order-plus-delay model to a yaw step response.

The yaw closed loop under the geometric controller is (small angles)

    psi_dot = (1/T) * (psi_ref - psi),   T ~ yawctrl_tau / 2  (+ rate lag)

so a reference step gives  psi(t) = A * (1 - exp(-(t - td)/T)).
Identifying T tells us the *effective* yaw time constant, and

    yawctrl_tau_new = yawctrl_tau_applied * T_target / T_measured

places it at the target (the applied tau and measured T are proportional
through the same unknown efficiency factor, which cancels).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares


@dataclass
class FirstOrderFitResult:
    T: float           # time constant [s]
    delay: float       # s
    amplitude: float   # fitted steady state (~ step)
    nrmse: float
    converged: bool
    at_bounds: bool = False

    @property
    def ok(self) -> bool:
        return self.converged and self.nrmse < 0.15 and not self.at_bounds


def first_order_step(t: np.ndarray, T: float) -> np.ndarray:
    t = np.maximum(t, 0.0)
    return np.where(t > 0, 1.0 - np.exp(-t / max(T, 1e-6)), 0.0)


def fit_first_order(t: np.ndarray, y: np.ndarray, step: float,
                    T_guess: float = 0.3,
                    T_bounds: tuple = (0.02, 2.0)) -> FirstOrderFitResult:
    """Fit (T, delay, amplitude); y is the response relative to the
    pre-step value, same sign convention as `step`."""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    if t.size < 10:
        raise ValueError("Need at least 10 samples")
    if abs(step) < 1e-6:
        raise ValueError("step must be nonzero")

    yn = y / step

    def residuals(p):
        T, td, amp = p
        return amp * first_order_step(t - td, T) - yn

    lb = np.array([T_bounds[0], 0.0, 0.3])
    ub = np.array([T_bounds[1], 0.4, 1.7])
    p0 = np.clip(np.array([T_guess, 0.05, 1.0]), lb, ub)
    sol = least_squares(residuals, p0, bounds=(lb, ub), method="trf")
    T, td, amp = sol.x

    def _pinned(v, lo, hi, skip_lo=False):
        span = hi - lo
        return ((not skip_lo and v - lo < 0.01 * span)
                or hi - v < 0.01 * span)

    at_bounds = (_pinned(T, lb[0], ub[0])
                 or _pinned(td, lb[1], ub[1], skip_lo=True)
                 or _pinned(amp, lb[2], ub[2]))

    nrmse = float(np.sqrt(np.mean(sol.fun ** 2)))
    return FirstOrderFitResult(
        T=float(T), delay=float(td), amplitude=float(amp * step),
        nrmse=nrmse, converged=bool(sol.success), at_bounds=bool(at_bounds))
