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
    at_bounds: bool = False  # a parameter pinned at the optimizer bounds

    @property
    def ok(self) -> bool:
        """Fit quality gate used before trusting a gain update.

        A parameter pinned at its optimizer bound means the model could
        not explain the data inside the physically plausible box — the
        classic wn/zeta/delay ambiguity of overdamped-looking responses.
        Such fits are rejected rather than acted on.
        """
        return (self.converged and self.nrmse < 0.15
                and 0.05 < self.zeta < 2.5 and not self.at_bounds)


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
                      wn_bounds: tuple | None = None,
                      ) -> StepFitResult:
    """Fit (wn, zeta, delay, amplitude) to a measured step response.

    t: seconds, 0 at step command time. y: position relative to the
    pre-step position (same sign convention as `step`).

    wn_bounds: optional (lo, hi) prior on wn. In closed-loop
    identification the applied gains predict wn up to the plant-gain
    factor alpha, so wn is *known* to lie in
    wn_pred * [sqrt(alpha_min), sqrt(alpha_hi)]. Constraining the fit to
    that box resolves the (wn, zeta, delay) ambiguity of higher-order
    responses: the optimizer explains the data with a physically possible
    wn and lets zeta/delay absorb the inner-loop lag. wn landing on these
    deliberate prior bounds does NOT mark the fit at_bounds.
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

    lb = np.array([0.1, 0.05, 0.0, 0.3])
    ub = np.array([30.0, 2.5, 0.6, 1.7])
    wn_prior_bounds = wn_bounds is not None
    if wn_prior_bounds:
        lb[0], ub[0] = wn_bounds
        wn_guess = float(np.clip(wn_guess, lb[0], ub[0]))

    # The (wn, zeta, delay) triple is weakly identifiable for
    # near-critically-damped responses: "high wn + overdamped + large
    # delay" mimics "wn near truth + small delay" almost exactly (the
    # response is really higher-order: inner-loop lag stacks on the
    # position loop). Resolve the ambiguity by multi-start optimization
    # and, among solutions of near-equal residual, prefer the one whose
    # wn is closest to the prior wn_guess (i.e. plant gain alpha ~ 1).
    starts = [
        (wn_guess, zeta_guess, 0.03),
        (wn_guess, zeta_guess, 0.15),
        (0.6 * wn_guess, 0.7, 0.05),
        (1.5 * wn_guess, 1.3, 0.05),
        (wn_guess, 0.5, 0.30),
    ]
    sols = []
    for wn0, z0, td0 in starts:
        p0 = np.clip(np.array([wn0, z0, td0, 1.0]), lb, ub)
        s = least_squares(residuals, p0, bounds=(lb, ub), method="trf")
        if s.success:
            sols.append(s)
    if not sols:
        sols = [least_squares(residuals,
                              np.clip(np.array([wn_guess, zeta_guess, 0.05, 1.0]), lb, ub),
                              bounds=(lb, ub), method="trf")]

    def _nrmse_of(s):
        return float(np.sqrt(np.mean(s.fun ** 2)))

    best = min(_nrmse_of(s) for s in sols)
    near_best = [s for s in sols if _nrmse_of(s) <= max(1.25 * best, best + 0.005)]
    sol = min(near_best, key=lambda s: abs(math.log(max(s.x[0], 1e-6) / wn_guess)))
    wn, zeta, td, amp = sol.x

    # Bound-pinning check (delay's lower bound of 0 is a legitimate value
    # and is excluded; everything else pinned means an untrustworthy fit).
    def _pinned(v, lo, hi, skip_lo=False):
        span = hi - lo
        return ((not skip_lo and v - lo < 0.01 * span)
                or hi - v < 0.01 * span)

    at_bounds = ((not wn_prior_bounds and _pinned(wn, lb[0], ub[0]))
                 or _pinned(zeta, lb[1], ub[1])
                 or _pinned(td, lb[2], ub[2], skip_lo=True)
                 or _pinned(amp, lb[3], ub[3]))
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
        at_bounds=bool(at_bounds),
    )
