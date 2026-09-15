"""Cross-session trend (check group F, --history).

One row per session in the logs tree: date, generation, duration, rounds,
outcome, final gains, per-axis alpha/lag. Sessions without a session CSV
(legacy, pre-e0c1237) are re-identified from their bag through
geo-tuner-bag-export -> geo-tuner-identify, so the whole history speaks the
principled method's language; exported CSVs are cached next to the doctor
output. On the 09-10/09-13 bags this says what the 09-14 flight then
confirmed: z kv 2.50 -> UPDATE to ~3.0-3.2, z lag ~70 ms.
"""
from __future__ import annotations

import dataclasses
import os
import shutil
import subprocess
from pathlib import Path

from . import overrides as ovr
from . import timeutil
from .discover import SessionArtifacts, discover
from .findings import WARN, Finding
from .identify import IdentifyUnavailable, find_binary, run_identify
from .report import Report, load_report


def find_bag_export() -> str | None:
    env = os.environ.get("GEO_TUNER_BAG_EXPORT_BIN")
    if env and Path(env).is_file():
        return env
    path = shutil.which("geo-tuner-bag-export")
    if path:
        return path
    for p in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        cand = Path(p) / "lib" / "geo_tuner" / "geo-tuner-bag-export"
        if cand.is_file():
            return str(cand)
    return None


def export_bag(bag_dir: Path, report_path: Path, out_csv: Path) -> bool:
    """Bag -> session CSV via the installed exporter; cached on out_csv."""
    if out_csv.is_file() and out_csv.stat().st_size > 1000:
        return True
    binary = find_bag_export()
    if binary is None:
        return False
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    proc = subprocess.run(
        [binary, str(bag_dir), str(out_csv), "--report", str(report_path)],
        capture_output=True, text=True, timeout=300)
    return proc.returncode == 0 and out_csv.is_file()


@dataclasses.dataclass
class HistoryRow:
    name: str
    date: str
    generation: str
    status: str
    duration_s: float | None
    rounds: int | None
    final_gains: dict            # axis -> {kx, kv}
    axes: dict                   # axis -> {alpha, alpha_ci, lag_ms, lag_ci_ms,
    #                                        verdict, why} (identify or report)
    ident_source: str            # "csv" | "bag-export" | "report" | "none"


def _axes_from_identify(doc: dict) -> dict:
    out = {}
    for ax, r in doc["axes"].items():
        if r["ok"]:
            out[ax] = {"alpha": r["alpha"], "alpha_ci": r["alpha_ci"],
                       "lag_ms": r["lag_ms"], "lag_ci_ms": r["lag_ci_ms"],
                       "verdict": r["verdict"], "why": r["why"],
                       "design_kx": r["design"]["kx"],
                       "design_kv": r["design"]["kv"]}
    return out


def _axes_from_report(r: Report) -> dict:
    out = {}
    for ax, fg in (r.final_gains or {}).items():
        if fg.get("alpha") is not None:
            out[ax] = {"alpha": fg["alpha"], "alpha_ci": fg.get("alpha_ci"),
                       "lag_ms": fg.get("lag_ms"),
                       "lag_ci_ms": fg.get("lag_ci_ms"),
                       "verdict": "validated" if fg.get("validated") else "kept",
                       "why": fg.get("outcome", "")}
    return out


def build_row(art: SessionArtifacts, cache_dir: Path) -> HistoryRow:
    try:
        r = load_report(art.report)
    except Exception:
        return HistoryRow(art.name, "?", "unreadable", "?", None, None, {}, {}, "none")
    gains = {ax: {"kx": fg.get("kx"), "kv": fg.get("kv")}
             for ax, fg in (r.final_gains or {}).items()}
    axes: dict = {}
    source = "none"
    csv = art.csv
    if csv is None and art.bag is not None:
        # Legacy flight: rebuild the recording from the bag. The exporter
        # needs the report to reconstruct the gains in force.
        out_csv = cache_dir / f"{art.bag.name}.csv"
        if export_bag(art.bag, art.report, out_csv):
            csv, source = out_csv, "bag-export"
    elif csv is not None:
        source = "csv"
    if csv is not None and find_binary() is not None:
        try:
            axes = _axes_from_identify(run_identify(csv))
        except IdentifyUnavailable:
            axes, source = {}, "none"
    if not axes:
        axes = _axes_from_report(r)
        source = "report" if axes else "none"
    date = r.time.strftime("%Y-%m-%d %H:%M") if r.time else art.name
    return HistoryRow(art.name, date, r.generation, r.status,
                      r.session_duration_s, r.rounds_flown, gains, axes, source)


def gains_timeline(root: Path) -> list[dict]:
    """What flew when, from the override .bak history plus the live file."""
    cfg = root / "mav_controllers_config"
    rows = []
    for p in sorted(cfg.glob("geometric_controller.override.yaml.*.bak")):
        t = timeutil.parse_bak_stamp(p.name)
        g = ovr.controller_gains(p)
        if t and g:
            rows.append({"until": t.strftime("%Y-%m-%d %H:%M"), "gains": g,
                         "file": p.name})
    live = cfg / "geometric_controller.override.yaml"
    g = ovr.controller_gains(live) if live.is_file() else None
    if g:
        rows.append({"until": "now", "gains": g, "file": live.name})
    return rows


def _intervals_disjoint(a, b) -> bool:
    return bool(a and b and (a[1] < b[0] or b[1] < a[0]))


def drift_findings(rows: list[HistoryRow]) -> list[Finding]:
    """WARN when alpha or lag moves beyond the intervals between
    consecutive identified sessions: the plant changed."""
    out = []
    for prev, cur in zip(rows, rows[1:]):
        for ax in cur.axes:
            p, c = prev.axes.get(ax), cur.axes[ax]
            if not p:
                continue
            drift = {}
            if _intervals_disjoint(p.get("alpha_ci"), c.get("alpha_ci")):
                drift["alpha"] = {"from": p["alpha"], "to": c["alpha"]}
            if _intervals_disjoint(p.get("lag_ci_ms"), c.get("lag_ci_ms")):
                drift["lag_ms"] = {"from": p["lag_ms"], "to": c["lag_ms"]}
            if drift:
                out.append(Finding(
                    "PLANT_DRIFT", WARN,
                    f"{ax}: plant changed between {prev.name} and {cur.name}",
                    axis=ax, evidence=drift,
                    source=[f"history re-identification ({prev.ident_source} "
                            f"vs {cur.ident_source})"],
                    explanation="alpha/lag moved beyond both sessions' "
                                "intervals; props, ESC, payload or attitude "
                                "loop changed between flights",
                    recommendation_ids=["REC_CHECK_HARDWARE"]))
    return out


def build_history(root: Path, cache_dir: Path) -> tuple[list[HistoryRow], list[Finding]]:
    rows = [build_row(art, cache_dir) for art in discover(root)]
    return rows, drift_findings(rows)


def render_history_md(rows: list[HistoryRow], timeline: list[dict]) -> list[str]:
    out = ["## Session history", ""]
    out.append("| session | generation | status | dur s | rounds | axis | "
               "kx/kv | alpha [CI] | lag ms | verdict | ident |")
    out.append("|---|---|---|---|---|---|---|---|---|---|---|")
    for row in rows:
        first = True
        for ax in ("x", "y", "z"):
            a = row.axes.get(ax)
            g = row.final_gains.get(ax, {})
            lead = (f"| {row.date} | {row.generation} | {row.status} | "
                    f"{row.duration_s or '?'} | {row.rounds or '-'} "
                    if first else "| | | | | ")
            first = False
            if a:
                ci = a.get("alpha_ci")
                ci_s = (f"[{ci[0]:.2f}, {ci[1]:.2f}]"
                        if ci and isinstance(ci[0], float) else "")
                cell = (f"| {ax} | {g.get('kx', '?')}/{g.get('kv', '?')} | "
                        f"{a['alpha']:.2f} {ci_s} | "
                        f"{a.get('lag_ms') or '?'} | {a.get('verdict', '')} | "
                        f"{row.ident_source} |")
            else:
                cell = (f"| {ax} | {g.get('kx', '?')}/{g.get('kv', '?')} | "
                        "- | - | - | - |")
            out.append(lead + cell)
    if timeline:
        out += ["", "### Gains in force (override history)", ""]
        out.append("| until | x kx/kv | y kx/kv | z kx/kv | file |")
        out.append("|---|---|---|---|---|")
        for e in timeline:
            g = e["gains"]

            def cell(ax):
                v = g.get(ax, {})
                return f"{v.get('kx', '?')}/{v.get('kv', '?')}"
            out.append(f"| {e['until']} | {cell('x')} | {cell('y')} | "
                       f"{cell('z')} | {e['file']} |")
    out.append("")
    return out
