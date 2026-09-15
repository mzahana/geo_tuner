"""Load a geo_tuner report, either generation, into one typed shape.

Two generations exist in the field logs:
- principled (geo_tuner >= e0c1237, 2026-09-13 17:00): episodes carry `round`,
  plus per-round `identification`/`verdict` entries and yaw-round entries;
  `final_gains` per axis. 9a7b429 added `accepted_at`, `yaw_result`,
  per-episode `u_peak`/`u_peak_frac`/`near_saturation` — but a report written
  by an *older* conductor at accept time lacks them (the 09-14 field report).
- legacy (<= 8e59550): episodes carry `rung` and per-episode alpha/tau_lag;
  `wn_ladder` instead of `wn_target`.

The loader classifies entries instead of assuming order, and keeps the raw
dict so a check can reach fields this schema view does not model.
"""
from __future__ import annotations

import dataclasses
import datetime as _dt
from pathlib import Path
from typing import Any

import yaml

from . import timeutil

GEN_LEGACY = "legacy"
GEN_PRINCIPLED = "principled"
GEN_PRINCIPLED_V2 = "principled-9a7b429"  # report written at/after accept by >= 9a7b429


@dataclasses.dataclass
class StepEpisode:
    axis: str
    round_: int  # `round` (principled) or `rung` (legacy)
    rep: int
    step: float
    kx_applied: float | None
    kv_applied: float | None
    overshoot: float | None
    settle_s: float | None
    record_s: float | None
    action: str
    # legacy per-episode fit
    alpha: float | None = None
    tau_lag: float | None = None
    # 9a7b429 saturation guard
    u_peak: float | None = None
    u_peak_frac: float | None = None
    near_saturation: bool | None = None
    raw: dict = dataclasses.field(default_factory=dict, repr=False)


@dataclasses.dataclass
class RoundIdentification:
    axis: str
    round_: int
    kx_in_force: float | None
    kv_in_force: float | None
    ok: bool
    alpha: float | None
    alpha_ci: tuple[float, float] | None
    lag_ms: float | None
    lag_ci_ms: tuple[float, float] | None
    delay_ms: float | None
    tau_ms: float | None
    r2: float | None
    excited_s: float | None
    verdict: str | None
    why: str | None
    action: str | None
    design: dict = dataclasses.field(default_factory=dict)
    raw: dict = dataclasses.field(default_factory=dict, repr=False)

    @property
    def ci_ratio(self) -> float | None:
        if self.alpha_ci and self.alpha_ci[0] > 0:
            return self.alpha_ci[1] / self.alpha_ci[0]
        return None


@dataclasses.dataclass
class YawRound:
    round_: int
    n_used: int | None
    tau_in_force: float | None
    spread: float | None
    action: str | None
    raw: dict = dataclasses.field(default_factory=dict, repr=False)


@dataclasses.dataclass
class Report:
    path: Path
    raw: dict = dataclasses.field(repr=False, default_factory=dict)
    generation: str = GEN_PRINCIPLED
    status: str = ""
    diagnosis: str = ""
    accel_trim: tuple[float, float, float] | None = None
    final_yawctrl_tau: float | None = None
    yaw_result: str | None = None
    time: _dt.datetime | None = None
    accepted_at: _dt.datetime | None = None
    session_duration_s: float | None = None
    rounds_flown: int | None = None
    session_recording: str | None = None
    step_episodes: list[StepEpisode] = dataclasses.field(default_factory=list)
    identifications: list[RoundIdentification] = dataclasses.field(default_factory=list)
    yaw_rounds: list[YawRound] = dataclasses.field(default_factory=list)
    final_gains: dict[str, dict[str, Any]] = dataclasses.field(default_factory=dict)
    other_entries: list[dict] = dataclasses.field(default_factory=list)

    def identifications_for(self, axis: str) -> list[RoundIdentification]:
        return [i for i in self.identifications if i.axis == axis]

    @property
    def last_identification(self) -> dict[str, RoundIdentification]:
        out: dict[str, RoundIdentification] = {}
        for i in self.identifications:
            out[i.axis] = i
        return out


def _f(d: dict, key: str) -> float | None:
    v = d.get(key)
    return None if v is None else float(v)


def _ci(v) -> tuple[float, float] | None:
    if isinstance(v, (list, tuple)) and len(v) == 2:
        return (float(v[0]), float(v[1]))
    return None


def detect_generation(doc: dict) -> str:
    episodes = doc.get("episodes") or []
    if "wn_ladder" in doc or any("rung" in e for e in episodes):
        return GEN_LEGACY
    if (
        "accepted_at" in doc
        or "yaw_result" in doc
        or any("u_peak" in e for e in episodes)
    ):
        return GEN_PRINCIPLED_V2
    return GEN_PRINCIPLED


def load_report(path: Path, tz=None) -> Report:
    path = Path(path)
    doc = yaml.safe_load(path.read_text())
    if not isinstance(doc, dict):
        raise ValueError(f"{path}: not a mapping")
    tz = tz or timeutil.local_tz()
    r = Report(path=path, raw=doc, generation=detect_generation(doc))
    r.status = str(doc.get("status", ""))
    r.diagnosis = str(doc.get("diagnosis", "") or "")
    trim = doc.get("accel_trim")
    if isinstance(trim, list) and len(trim) == 3:
        r.accel_trim = tuple(float(x) for x in trim)
    r.final_yawctrl_tau = _f(doc, "final_yawctrl_tau")
    r.yaw_result = doc.get("yaw_result")
    r.time = timeutil.parse_report_time(doc.get("time"), tz)
    r.accepted_at = timeutil.parse_report_time(doc.get("accepted_at"), tz)
    r.session_duration_s = _f(doc, "session_duration_s")
    r.rounds_flown = doc.get("rounds_flown")
    r.session_recording = doc.get("session_recording")
    r.final_gains = doc.get("final_gains") or {}

    round_key = "rung" if r.generation == GEN_LEGACY else "round"
    for e in doc.get("episodes") or []:
        if not isinstance(e, dict):
            r.other_entries.append({"value": e})
            continue
        if "identification" in e:
            ident = e.get("identification") or {}
            r.identifications.append(
                RoundIdentification(
                    axis=str(e.get("axis")),
                    round_=int(e.get(round_key, 0)),
                    kx_in_force=_f(e, "kx_in_force"),
                    kv_in_force=_f(e, "kv_in_force"),
                    ok=bool(ident.get("ok", False)),
                    alpha=_f(ident, "alpha"),
                    alpha_ci=_ci(ident.get("alpha_ci")),
                    lag_ms=_f(ident, "lag_ms"),
                    lag_ci_ms=_ci(ident.get("lag_ci_ms")),
                    delay_ms=_f(ident, "delay_ms"),
                    tau_ms=_f(ident, "tau_ms"),
                    r2=_f(ident, "r2"),
                    excited_s=_f(ident, "excited_s"),
                    verdict=e.get("verdict"),
                    why=e.get("why"),
                    action=e.get("action"),
                    design=e.get("design") or {},
                    raw=e,
                )
            )
        elif e.get("axis") == "yaw" and ("spread" in e or "n_used" in e) and "rep" not in e:
            r.yaw_rounds.append(
                YawRound(
                    round_=int(e.get(round_key, 0)),
                    n_used=e.get("n_used"),
                    tau_in_force=_f(e, "yaw_tau_in_force"),
                    spread=_f(e, "spread"),
                    action=e.get("action"),
                    raw=e,
                )
            )
        elif "rep" in e:
            r.step_episodes.append(
                StepEpisode(
                    axis=str(e.get("axis")),
                    round_=int(e.get(round_key, 0)),
                    rep=int(e.get("rep", 0)),
                    step=float(e.get("step", 0.0)),
                    kx_applied=_f(e, "kx_applied"),
                    kv_applied=_f(e, "kv_applied"),
                    overshoot=_f(e, "overshoot"),
                    settle_s=_f(e, "settle_s"),
                    record_s=_f(e, "record_s"),
                    action=str(e.get("action", "")),
                    alpha=_f(e, "alpha"),
                    tau_lag=_f(e, "tau_lag"),
                    u_peak=_f(e, "u_peak"),
                    u_peak_frac=_f(e, "u_peak_frac"),
                    near_saturation=e.get("near_saturation"),
                    raw=e,
                )
            )
        else:
            r.other_entries.append(e)
    return r
