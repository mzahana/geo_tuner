"""Load the conductor's 50 Hz session recording.

Columns: t,seg,rx,ry,rz,vx,vy,vz,ux,uy,uz,kxx,kxy,kxz,kvx,kvy,kvz
r = setpoint position, v = measured velocity (ENU world), u = commanded
acceleration, kx/kv = gains in force. `seg` increments on every recording
gap, which in practice means one seg per round -- geo-tuner-identify
consumes exactly this file, so the doctor re-identifies from the same bytes
the flight did.
"""
from __future__ import annotations

import dataclasses
from pathlib import Path

import numpy as np

COLUMNS = (
    "t,seg,rx,ry,rz,vx,vy,vz,ux,uy,uz,kxx,kxy,kxz,kvx,kvy,kvz"
).split(",")


@dataclasses.dataclass
class SessionCsv:
    path: Path
    t: np.ndarray
    seg: np.ndarray
    r: np.ndarray  # (n, 3) setpoint position
    v: np.ndarray  # (n, 3) measured velocity
    u: np.ndarray  # (n, 3) commanded acceleration
    kx: np.ndarray  # (n, 3)
    kv: np.ndarray  # (n, 3)

    @property
    def n(self) -> int:
        return len(self.t)

    @property
    def duration_s(self) -> float:
        return float(self.t[-1] - self.t[0]) if self.n else 0.0

    @property
    def segments(self) -> list[int]:
        return [int(s) for s in np.unique(self.seg)]

    @property
    def rate_hz(self) -> float:
        if self.n < 2:
            return 0.0
        dt = np.diff(self.t)
        # Median, not mean: the inter-segment gaps would drag the mean down.
        return float(1.0 / np.median(dt))


def load_session_csv(path: Path) -> SessionCsv:
    path = Path(path)
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.ndim == 0:  # single row
        data = data.reshape(1)
    missing = [c for c in COLUMNS if c not in (data.dtype.names or ())]
    if missing:
        raise ValueError(f"{path}: missing columns {missing}")

    def cols(names):
        return np.column_stack([data[c] for c in names])

    return SessionCsv(
        path=path,
        t=np.asarray(data["t"], dtype=float),
        seg=np.asarray(data["seg"], dtype=int),
        r=cols(["rx", "ry", "rz"]),
        v=cols(["vx", "vy", "vz"]),
        u=cols(["ux", "uy", "uz"]),
        kx=cols(["kxx", "kxy", "kxz"]),
        kv=cols(["kvx", "kvy", "kvz"]),
    )
