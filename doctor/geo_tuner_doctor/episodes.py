"""Load the per-episode step dumps.

One CSV per flown step: header `# axis=z step=1 round=0 rep=0` (legacy dirs
say `rung=` -- 09-10 and 09-13 in the field logs), then `t,y` with y the
position/yaw minus the pre-step value, truncated at episode end.
"""
from __future__ import annotations

import dataclasses
import re
from pathlib import Path

import numpy as np

_HEADER_RE = re.compile(r"(\w+)=([^\s]+)")


@dataclasses.dataclass
class EpisodeDump:
    path: Path
    axis: str
    step: float
    round_: int  # `round=` or legacy `rung=`
    rep: int
    t: np.ndarray
    y: np.ndarray

    @property
    def duration_s(self) -> float:
        return float(self.t[-1] - self.t[0]) if len(self.t) else 0.0


def load_episode(path: Path) -> EpisodeDump:
    path = Path(path)
    with open(path) as fh:
        header = fh.readline()
    if not header.startswith("#"):
        raise ValueError(f"{path}: expected '# axis=... step=...' header")
    fields = dict(_HEADER_RE.findall(header))
    data = np.genfromtxt(path, delimiter=",", skip_header=2)
    if data.ndim == 1:
        data = data.reshape(-1, 2) if data.size else np.empty((0, 2))
    return EpisodeDump(
        path=path,
        axis=fields.get("axis", "?"),
        step=float(fields.get("step", "nan")),
        round_=int(fields.get("round", fields.get("rung", 0))),
        rep=int(fields.get("rep", 0)),
        t=data[:, 0],
        y=data[:, 1],
    )


def load_episode_dir(dirpath: Path) -> list[EpisodeDump]:
    return [load_episode(p) for p in sorted(Path(dirpath).glob("ep*.csv"))]
