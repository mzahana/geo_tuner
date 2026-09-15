"""Ensemble step responses vs the identified model (checks D2/D3).

Method lifted from prototypes/steps.py + ens.py, which established the 09-14
numbers this must reproduce (rms z 0.009, x 0.015, y 0.017). Score
ensembles, never single steps: with ~7 cm hover wander a single 1 m step's
overshoot is +/-7 % noise.

Steps are detected from setpoint jumps in the bag and matched, in order and
by sign, to the report's episodes to attach the gains in force; the response
is normalized against the setpoints before/after the jump (tracking, not
displacement) and averaged per (axis, kx, kv) group.
"""
from __future__ import annotations

import dataclasses

import numpy as np

from .bag import BagData
from .report import Report

AXIS_COL = {"x": 0, "y": 1, "z": 2}
TG = np.arange(0, 4.0, 0.02)  # common comparison grid [s]


def detect_steps(bag: BagData, thresh: float = 0.15):
    """[(t0, axis, amplitude)] of position-setpoint jumps, merged when a
    return-to-hover coincides within 0.3 s on the same axis."""
    sp = bag.arrays.get("sp")
    if sp is None or len(sp) < 2:
        return []
    out = []
    for ax, i in AXIS_COL.items():
        s = sp[:, 1 + i]
        ds = np.diff(s)
        for k in np.where(np.abs(ds) > thresh)[0]:
            out.append((float(sp[k + 1, 0]), ax, float(ds[k])))
    out.sort()
    merged = []
    for st in out:
        if merged and st[0] - merged[-1][0] < 0.3 and st[1] == merged[-1][1]:
            continue
        merged.append(st)
    return merged


def match_steps_to_episodes(bag: BagData, report: Report):
    """Attach kx/kv_applied to each detected step, consuming the report's
    per-axis episode list in order with a sign check (the tuner steps out
    and back; returns land where the previous step started)."""
    eps = {ax: [e for e in report.step_episodes if e.axis == ax]
           for ax in AXIS_COL}
    k = {ax: 0 for ax in AXIS_COL}
    matched = []
    for t0, ax, amp in detect_steps(bag):
        if ax not in eps or k[ax] >= len(eps[ax]):
            continue
        ep = eps[ax][k[ax]]
        if np.sign(ep.step) != np.sign(amp):
            continue
        k[ax] += 1
        matched.append({"t0": t0, "axis": ax, "amp": amp,
                        "kx": ep.kx_applied, "kv": ep.kv_applied,
                        "round": ep.round_})
    return matched


def normalized_response(bag: BagData, t0: float, axis: str) -> np.ndarray | None:
    """Measured response on TG, normalized to the setpoint jump at t0; NaN
    after a following step cuts the window."""
    od, sp = bag.arrays["odom"], bag.arrays["sp"]
    i = AXIS_COL[axis]
    j = np.searchsorted(sp[:, 0], t0)
    if j < 2 or j + 1 >= len(sp):
        return None
    s0, s1 = sp[j - 2, 1 + i], sp[j + 1, 1 + i]
    if abs(s1 - s0) < 1e-6:
        return None
    m = (od[:, 0] >= t0) & (od[:, 0] <= t0 + TG[-1] + 0.1)
    if not m.any() or od[m, 0][-1] - t0 < TG[-1]:
        return None
    y = (od[m, 1 + i] - s0) / (s1 - s0)
    yi = np.interp(TG, od[m, 0] - t0, y)
    nxt = sp[(sp[:, 0] > t0 + 0.3) & (sp[:, 0] < t0 + TG[-1])]
    if len(nxt) > 1:
        d = np.abs(np.diff(nxt[:, 1 + i]))
        if np.any(d > 0.05):
            cut = nxt[1:, 0][d > 0.05][0] - t0
            yi = yi.copy()
            yi[TG > cut] = np.nan
    return yi


def simulate_model(kx: float, kv: float, alpha: float, delay_s: float,
                   tau_s: float, t: np.ndarray = TG, dt: float = 0.001) -> np.ndarray:
    """Closed loop of the identified plant alpha e^{-ds}/(tau s + 1) under
    the position/velocity gains, unit step."""
    n = int(t[-1] / dt) + 2
    nd = int(round(delay_s / dt))
    buf = np.zeros(nd + 1)
    p = v = a = 0.0
    out = np.zeros(n)
    for k in range(n):
        u = kx * (1.0 - p) - kv * v
        buf = np.roll(buf, -1)
        buf[-1] = u
        ud = buf[0]
        a += dt / tau_s * (alpha * ud - a) if tau_s > 0 else 0.0
        v += a * dt
        p += v * dt
        out[k] = p
    return np.interp(t, np.arange(n) * dt, out)


def step_metrics(y: np.ndarray, t: np.ndarray = TG) -> dict:
    os_ = float(np.nanmax(y) - 1.0)
    t10 = float(t[np.argmax(y >= 0.1)])
    t90 = float(t[np.argmax(y >= 0.9)]) if (y >= 0.9).any() else float("nan")
    m3 = t <= 3.0
    trapz = getattr(np, "trapezoid", None) or np.trapz  # numpy 2 renamed it
    iae = float(trapz(np.abs(1.0 - y[m3]), t[m3]))
    return {"overshoot": os_, "rise_s": t90 - t10, "iae3": iae}


@dataclasses.dataclass
class EnsembleGroup:
    axis: str
    kx: float
    kv: float
    n: int
    mean: np.ndarray             # measured ensemble on TG (NaN-padded tail)
    model: np.ndarray | None
    rms: float | None            # rms(measured - model) over valid samples
    measured_metrics: dict
    model_metrics: dict | None
    t0s: list[float]


def build_ensembles(bag: BagData, report: Report,
                    models: dict[str, tuple[float, float, float]] | None
                    ) -> list[EnsembleGroup]:
    """Per (axis, kx, kv) group; `models` maps axis -> (alpha, delay_s,
    tau_s), typically from geo-tuner-identify on the full session."""
    groups: dict[tuple, list] = {}
    for st in match_steps_to_episodes(bag, report):
        if st["kx"] is None:
            continue
        key = (st["axis"], round(st["kx"], 2), round(st["kv"], 2))
        groups.setdefault(key, []).append(st)
    out = []
    for (ax, kx, kv), steps in sorted(groups.items()):
        ys = [normalized_response(bag, st["t0"], ax) for st in steps]
        ys = [y for y in ys if y is not None]
        if not ys:
            continue
        mean = np.nanmean(np.array(ys), axis=0)
        valid = ~np.isnan(mean)
        mean_filled = np.where(valid, mean, mean[valid][-1])
        model = rms = mm = None
        if models and ax in models:
            alpha, delay_s, tau_s = models[ax]
            model = simulate_model(kx, kv, alpha, delay_s, tau_s)
            rms = float(np.sqrt(np.nanmean((mean - model) ** 2)))
            mm = step_metrics(model)
        out.append(EnsembleGroup(
            axis=ax, kx=kx, kv=kv, n=len(ys), mean=mean_filled, model=model,
            rms=rms, measured_metrics=step_metrics(mean_filled),
            model_metrics=mm, t0s=[st["t0"] for st in steps]))
    return out
