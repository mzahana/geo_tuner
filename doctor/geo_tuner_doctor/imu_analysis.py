"""E1: is the vertical plant softness real? Port of prototypes/zalpha.py.

Cross-checks the commanded z acceleration against two independent measured
accelerations -- IMU (rotated to world, gravity removed) and d/dt of odom
velocity -- and measures the incremental thrust slope at hover. On 09-14:
identify said z alpha 0.87 (2SLS), IMU OLS 0.77, velocity OLS 0.82, thrust
slope 12.6 vs the linear map's 16.4 m/s^2: the softness is the vehicle
(thrust curve flatter than the linear map at hover), not EKF lag.

Plain OLS under feedback is biased low -- these numbers are cross-checks
labelled as such, never quoted as the plant.
"""
from __future__ import annotations

import numpy as np

from .bag import BagData
from .signals import G, Window

FS = 50.0


def _lp(fs: float = FS, hz: float = 3.0):
    from scipy.signal import butter, filtfilt
    b, a = butter(2, hz / (fs / 2))
    return lambda x: filtfilt(b, a, x)


def _ols_lagged(y: np.ndarray, x: np.ndarray, mask: np.ndarray,
                max_lag_steps: int = 16):
    """Best (lag_ms, slope, unexplained fraction) over a 0..300 ms lag grid."""
    best = None
    for lag in range(0, max_lag_steps):
        xs = np.roll(x, lag)
        xs[:lag] = x[0]
        m = mask.copy()
        m[:lag] = False
        yy = y[m] - y[m].mean()
        xx = xs[m] - xs[m].mean()
        denom = xx @ xx
        if denom <= 0:
            continue
        al = (xx @ yy) / denom
        res = float(np.sum((yy - al * xx) ** 2) / np.sum(yy ** 2))
        if best is None or res < best[2]:
            best = (lag * int(1000 / FS), float(al), res)
    return best


def z_plant_cross_check(bag: BagData, window: Window, mass: float,
                        max_thrust: float) -> dict | None:
    """OLS alpha of IMU/velocity accel against the z command, and the
    incremental thrust slope, all on a 50 Hz grid over the tuning window."""
    need = ("cmd", "imu", "odom", "sp", "att_thrust")
    if any(bag.arrays.get(k) is None for k in need):
        return None
    from scipy.spatial.transform import Rotation
    lp = _lp()
    tg = np.arange(window.t0, window.t1, 1.0 / FS)
    if len(tg) < 500:
        return None

    cmd = bag.arrays["cmd"]
    u_z = np.interp(tg, cmd[:, 0], cmd[:, 3] / mass - G)

    imu = bag.arrays["imu"]  # t, qw, qx, qy, qz, ax, ay, az
    rot = Rotation.from_quat(imu[:, [2, 3, 4, 1]])  # x, y, z, w order
    aw = rot.apply(imu[:, 5:8])
    aw[:, 2] -= G
    ai_z = np.interp(tg, imu[:, 0], aw[:, 2])

    od = bag.arrays["odom"]
    v_z = np.interp(tg, od[:, 0], od[:, 6])
    av_z = np.gradient(lp(v_z), 1.0 / FS)

    sp = bag.arrays["sp"]
    r_z = np.interp(tg, sp[:, 0], sp[:, 3])
    jumps = tg[1:][np.abs(np.diff(r_z)) > 0.3]
    excited = np.zeros(len(tg), bool)
    for j in jumps:
        excited |= (tg >= j) & (tg < j + 3.0)
    if excited.sum() < 100:
        return None

    uf = lp(u_z)
    imu_fit = _ols_lagged(lp(ai_z), uf, excited)
    vel_fit = _ols_lagged(av_z, uf, excited)

    att = bag.arrays["att_thrust"]
    hv = np.interp(tg, att[:, 0], att[:, 1])
    thrust_fit = _ols_lagged(lp(ai_z), lp(hv), excited)

    out = {"linear_map_accel_per_thrust": max_thrust / mass}
    if imu_fit:
        out["imu_ols"] = {"alpha": round(imu_fit[1], 3),
                          "lag_ms": imu_fit[0],
                          "unexplained": round(imu_fit[2], 2)}
    if vel_fit:
        out["velocity_ols"] = {"alpha": round(vel_fit[1], 3),
                               "lag_ms": vel_fit[0],
                               "unexplained": round(vel_fit[2], 2)}
    if thrust_fit:
        out["imu_accel_per_thrust"] = round(thrust_fit[1], 1)
    return out
