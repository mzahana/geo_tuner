"""Plots for report.md (PLAN.md section 7, item 7). matplotlib Agg only --
the doctor runs headless in the container. Every plot is optional: a missing
input or a missing matplotlib skips that plot, never fails the run.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .checks import Context, SETTLE_TOL_POS_M
from .ensembles import TG
from .session import Session


def render_plots(s: Session, ctx: Context, out_dir: Path) -> list[str]:
    """Write plots/*.png under out_dir; returns relative paths written."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    out_dir.mkdir(parents=True, exist_ok=True)
    written: list[str] = []

    def save(fig, name: str):
        fig.tight_layout()
        fig.savefig(out_dir / name, dpi=110)
        plt.close(fig)
        written.append(name)

    # -- ensemble step vs model, one panel per (axis, gains) group
    groups = getattr(ctx, "_ensembles", None)
    if groups:
        n = len(groups)
        fig, axes = plt.subplots(1, n, figsize=(3.2 * n, 3.0), sharey=True)
        axes = np.atleast_1d(axes)
        for axp, g in zip(axes, groups):
            axp.plot(TG, g.mean, label=f"flown (n={g.n})")
            if g.model is not None:
                axp.plot(TG, g.model, "--",
                         label=f"model (rms {g.rms:.3f})")
            axp.axhline(1.0, color="gray", lw=0.5)
            axp.set_title(f"{g.axis}  kx {g.kx:.2f} kv {g.kv:.2f}", fontsize=9)
            axp.set_xlabel("s")
            axp.legend(fontsize=7)
        axes[0].set_ylabel("normalized step response")
        save(fig, "ensembles.png")

    # -- CI ratio per round per axis
    if s.report and s.report.identifications:
        by_axis: dict[str, list] = {}
        for ident in s.report.identifications:
            if ident.ci_ratio:
                by_axis.setdefault(ident.axis, []).append(
                    (ident.round_, ident.ci_ratio))
        if by_axis:
            fig, axp = plt.subplots(figsize=(4.5, 3.0))
            for ax_name, pts in sorted(by_axis.items()):
                pts.sort()
                axp.plot([p[0] for p in pts], [p[1] for p in pts],
                         "o-", label=ax_name)
            axp.axhline(1.19, color="red", lw=0.8, ls=":",
                        label="confirm gate 1.19")
            axp.set_xlabel("round")
            axp.set_ylabel("alpha CI ratio")
            axp.legend(fontsize=8)
            save(fig, "ci_ratio.png")

    # -- time budget
    if s.report and s.report.step_episodes:
        settle = sum(e.settle_s or 0.0 for e in s.report.step_episodes)
        record = sum(e.record_s or 0.0 for e in s.report.step_episodes)
        total = s.report.session_duration_s or (settle + record)
        other = max(total - settle - record, 0.0)
        fig, axp = plt.subplots(figsize=(4.5, 1.8))
        axp.barh([0], [settle], label=f"settle {settle:.0f} s")
        axp.barh([0], [record], left=[settle], label=f"record {record:.0f} s")
        axp.barh([0], [other], left=[settle + record],
                 label=f"other {other:.0f} s")
        axp.set_yticks([])
        axp.set_xlabel("s")
        axp.legend(fontsize=7, ncol=3)
        save(fig, "time_budget.png")

    # -- hover wander per axis
    from . import signals
    if s.bag is not None:
        hn = signals.hover_noise(s.bag)
        if hn is not None:
            fig, axp = plt.subplots(figsize=(3.5, 2.6))
            axp.bar(["x", "y", "z"], 100 * hn["std_m"])
            axp.axhline(100 * SETTLE_TOL_POS_M, color="red", lw=0.8, ls=":",
                        label=f"settle_tol {100 * SETTLE_TOL_POS_M:.0f} cm")
            axp.set_ylabel("hover wander std [cm]")
            axp.legend(fontsize=8)
            save(fig, "hover_wander.png")

    # -- commanded vs IMU z acceleration over the tuning window
    if (s.bag is not None and ctx.window is not None
            and s.log is not None and s.log.first("mass")):
        try:
            from scipy.signal import butter, filtfilt
            from scipy.spatial.transform import Rotation
            cmd, imu = s.bag.arrays.get("cmd"), s.bag.arrays.get("imu")
            if cmd is not None and imu is not None:
                mass = s.log.first("mass").fields["mass_kg"]
                fs = 50.0
                tg = np.arange(ctx.window.t0, ctx.window.t1, 1 / fs)
                b, a = butter(2, 3.0 / (fs / 2))
                u_z = filtfilt(b, a, np.interp(
                    tg, cmd[:, 0], cmd[:, 3] / mass - signals.G))
                rot = Rotation.from_quat(imu[:, [2, 3, 4, 1]])
                aw = rot.apply(imu[:, 5:8])[:, 2] - signals.G
                ai = filtfilt(b, a, np.interp(tg, imu[:, 0], aw))
                fig, axp = plt.subplots(figsize=(7.0, 2.6))
                tt = tg - tg[0]
                axp.plot(tt, u_z, lw=0.7, label="commanded z accel")
                axp.plot(tt, ai, lw=0.7, label="IMU z accel (world, 3 Hz LPF)")
                axp.set_xlabel("s from window start")
                axp.set_ylabel("m/s$^2$")
                axp.legend(fontsize=8)
                save(fig, "z_accel.png")
        except ImportError:
            pass

    return written
