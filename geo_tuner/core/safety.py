"""Independent safety monitor for in-flight tuning episodes.

Pure logic (no ROS) so it is unit-testable. The conductor node feeds it
odometry samples; any violation aborts the tuning session into a hover
hold with the last known-safe gains.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from enum import Enum


class Violation(Enum):
    TILT = "tilt limit exceeded"
    POS_ERROR = "position error bound exceeded"
    VELOCITY = "velocity bound exceeded"
    ALTITUDE_LOW = "below minimum altitude"
    ALTITUDE_HIGH = "above maximum altitude"
    OSCILLATION = "oscillation detected (body rate energy)"
    ODOM_STALE = "odometry stale"


@dataclass
class SafetyLimits:
    max_tilt: float = 0.6          # rad (~34 deg), > controller max_tilt_angle
    max_pos_error: float = 2.0     # m, distance from active setpoint
    max_velocity: float = 4.0      # m/s
    min_altitude: float = 1.0      # m
    max_altitude: float = 40.0     # m
    odom_timeout: float = 0.3      # s
    # Oscillation detector: RMS of mean-removed body rates over the window
    osc_window: float = 2.0        # s
    osc_rate_rms: float = 1.2      # rad/s, roll+pitch combined
    osc_yaw_rate_rms: float = 1.5  # rad/s, yaw alone (drag-torque axis)
    osc_min_samples: int = 20


@dataclass
class OdomSample:
    t: float                       # s
    pos: tuple                     # (x, y, z) m
    vel: tuple                     # (vx, vy, vz) m/s
    quat: tuple                    # (w, x, y, z)
    body_rates: tuple = (0.0, 0.0, 0.0)  # rad/s


def tilt_from_quat(q: tuple) -> float:
    """Angle between body z and world z, from (w,x,y,z) quaternion."""
    w, x, y, z = q
    # R[2][2] of the rotation matrix
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(max(-1.0, min(1.0, r22)))


@dataclass
class SafetyMonitor:
    limits: SafetyLimits = field(default_factory=SafetyLimits)
    _rates: deque = field(default_factory=deque)   # (t, wx, wy, wz)
    _last_t: float | None = None

    def check(self, s: OdomSample, setpoint: tuple | None) -> list[Violation]:
        """Feed one odometry sample; returns violations (empty = safe)."""
        v: list[Violation] = []
        self._last_t = s.t

        if tilt_from_quat(s.quat) > self.limits.max_tilt:
            v.append(Violation.TILT)

        speed = math.sqrt(sum(c * c for c in s.vel))
        if speed > self.limits.max_velocity:
            v.append(Violation.VELOCITY)

        if s.pos[2] < self.limits.min_altitude:
            v.append(Violation.ALTITUDE_LOW)
        if s.pos[2] > self.limits.max_altitude:
            v.append(Violation.ALTITUDE_HIGH)

        if setpoint is not None:
            err = math.sqrt(sum((a - b) ** 2 for a, b in zip(s.pos, setpoint)))
            if err > self.limits.max_pos_error:
                v.append(Violation.POS_ERROR)

        # Rolling body-rate oscillation energy
        self._rates.append((s.t, *s.body_rates))
        t0 = s.t - self.limits.osc_window
        while self._rates and self._rates[0][0] < t0:
            self._rates.popleft()
        if len(self._rates) >= self.limits.osc_min_samples:
            n = len(self._rates)

            def _var(axis):
                vals = [r[axis] for r in self._rates]
                mean = sum(vals) / n
                return sum((x - mean) ** 2 for x in vals) / n

            if math.sqrt(_var(1) + _var(2)) > self.limits.osc_rate_rms:
                v.append(Violation.OSCILLATION)
            if math.sqrt(_var(3)) > self.limits.osc_yaw_rate_rms:
                v.append(Violation.OSCILLATION)

        return v

    def check_stale(self, now: float) -> list[Violation]:
        if self._last_t is None:
            return []
        if now - self._last_t > self.limits.odom_timeout:
            return [Violation.ODOM_STALE]
        return []

    def reset(self):
        self._rates.clear()
        self._last_t = None
