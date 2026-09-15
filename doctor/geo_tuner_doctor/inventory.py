"""Render the session inventory: what was found, matched by which clock,
what is missing, and the first facts out of each artifact.

This is the phase-1 output; the check/recommend/report layers grow on top
of it (PLAN.md section 4).
"""
from __future__ import annotations

import datetime as _dt

from . import timeutil
from .session import Session


def _line(label: str, value: str) -> str:
    return f"  {label:<16} {value}"


def render_inventory(s: Session) -> str:
    art = s.artifacts
    out: list[str] = []
    out.append(f"session {art.name} ({art.layout})")
    out.append(_line("start", timeutil.fmt_both_clocks(art.start)))
    out.append(_line("end", timeutil.fmt_both_clocks(art.end)))

    out.append(_line("report", str(art.report)))
    if s.report:
        r = s.report
        dur = f"{r.session_duration_s:.1f} s" if r.session_duration_s else "?"
        out.append(_line("", f"generation={r.generation} status={r.status} "
                             f"duration={dur} rounds={r.rounds_flown}"))
        out.append(_line("", f"{len(r.step_episodes)} step episodes, "
                             f"{len(r.identifications)} axis identifications, "
                             f"{len(r.yaw_rounds)} yaw rounds, "
                             f"final gains for [{', '.join(r.final_gains)}]"))
    if s.report_complete:
        out.append(_line("report@complete", str(art.report_complete)))

    if art.csv:
        out.append(_line("session CSV", str(art.csv)))
        if s.csv:
            out.append(_line("", f"{s.csv.n} rows @ {s.csv.rate_hz:.0f} Hz, "
                                 f"{len(s.csv.segments)} segments, "
                                 f"{s.csv.duration_s:.1f} s span"))
    if art.episodes_dir:
        out.append(_line("episodes", str(art.episodes_dir)))
        if s.episode_dumps is not None:
            axes = {}
            for e in s.episode_dumps:
                axes[e.axis] = axes.get(e.axis, 0) + 1
            per = ", ".join(f"{a}:{n}" for a, n in sorted(axes.items()))
            out.append(_line("", f"{len(s.episode_dumps)} dumps ({per})"))

    if art.log:
        out.append(_line("launch log", str(art.log)))
        if s.log:
            lg = s.log
            start = lg.run_start.strftime("%H:%M:%S UTC") if lg.run_start else "?"
            steps = len(lg.all("step", "yaw_step"))
            out.append(_line("", f"run start {start}, {len(lg.events)} events, "
                                 f"{steps} steps, "
                                 f"{len(lg.all('identification_axis'))} identification lines, "
                                 f"{len(lg.all('trim_update'))} trim updates"))
            complete = lg.first("tuning_complete")
            accepted = lg.first("accepted")
            saved = lg.first("gains_saved")
            out.append(_line("", "complete={} accepted={} saved={}".format(
                "yes" if complete else "no",
                "yes" if accepted else "no",
                "yes" if saved else "no")))

    if art.bag:
        out.append(_line("bag", str(art.bag)))
        if s.bag_status:
            st = "closed" if s.bag_status.closed else f"NOT CLOSED ({s.bag_status.reason})"
            out.append(_line("", st))
        if s.bag:
            src = "cache" if s.bag.from_cache else "read"
            sizes = ", ".join(
                f"{k}:{len(v)}" for k, v in sorted(s.bag.arrays.items()))
            out.append(_line("", f"[{src}] {sizes}"))
            if s.bag.strings:
                out.append(_line("", "string streams: " + ", ".join(
                    f"{k}:{len(v)}" for k, v in sorted(s.bag.strings.items()))))

    if art.override_baks:
        out.append(_line("override saves", ", ".join(p.name for p in art.override_baks)))

    for label in art.missing:
        out.append(_line("MISSING", label))
    for label, err in s.load_errors.items():
        out.append(_line("LOAD ERROR", f"{label}: {err}"))
    return "\n".join(out)
