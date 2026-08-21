"""Robust aggregation of repeated per-episode identifications.

A single 6-second step fit is a noisy estimator of the plant-gain factor
alpha (position axes) or the closed-loop time constant T (yaw): process
noise, the structural 2nd-order approximation of a truly higher-order
lateral response, and delay/wn ambiguity all inject episode-to-episode
variance that maps 1:1 (inverted) into the applied gains.

The fix is statistical, not structural: repeat the step N times per
(axis, rung), aggregate with the *median* (robust to one corrupted
episode), and refuse to update at all when the surviving estimates
disagree by more than a consistency factor — inconsistent estimates mean
the identification, not the plant, is the problem, and applying any of
them would just bake noise into the gains.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class RobustEstimate:
    """Outcome of aggregating N repeated positive-ratio estimates."""
    value: float          # median of the accepted estimates (nan if none)
    n_used: int           # how many estimates went into the median
    spread: float         # max/min ratio of the used estimates (>= 1)
    ok: bool              # enough estimates and spread within the gate
    reason: str = ""      # human-readable rejection reason if not ok


def robust_ratio_estimate(values: list[float], *, min_count: int = 2,
                          max_spread: float = 1.35) -> RobustEstimate:
    """Aggregate repeated estimates of a positive ratio-type quantity
    (alpha, T) into one robust value.

    values      accepted per-episode estimates (already gated for fit
                quality and physical plausibility); must be > 0
    min_count   minimum number of estimates required to act at all
    max_spread  max/min ratio allowed among the estimates. 1.35 means
                the worst pair disagrees by <= 35% — beyond that the
                session's identification is inconsistent and no gain
                update should be made from it.
    """
    vals = [float(v) for v in values if v > 0.0 and math.isfinite(v)]
    n = len(vals)
    if n == 0:
        return RobustEstimate(float("nan"), 0, float("inf"), False,
                              "no accepted estimates")
    med = _median(vals)
    spread = max(vals) / min(vals)
    if n < min_count:
        return RobustEstimate(med, n, spread, False,
                              f"only {n} accepted estimate(s), need "
                              f">= {min_count}")
    if spread > max_spread:
        return RobustEstimate(med, n, spread, False,
                              f"estimates inconsistent: spread "
                              f"{spread:.2f}x > {max_spread:.2f}x")
    return RobustEstimate(med, n, spread, True)


def _median(vals: list[float]) -> float:
    s = sorted(vals)
    n = len(s)
    m = n // 2
    return s[m] if n % 2 else 0.5 * (s[m - 1] + s[m])


@dataclass
class EpisodeBucket:
    """Accumulates per-episode estimates for one (axis, rung) pair."""
    alphas: list[float] = field(default_factory=list)
    delays: list[float] = field(default_factory=list)

    def add(self, alpha: float, delay: float):
        self.alphas.append(float(alpha))
        self.delays.append(float(delay))

    @property
    def count(self) -> int:
        return len(self.alphas)

    def median_delay(self) -> float:
        return _median(self.delays) if self.delays else 0.0
