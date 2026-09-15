"""Find the artifacts of one tuning session in a fetched logs tree.

Layout after `ihunter fetch` (~/src/ihunter_logs):
    mav_controllers_config/geo_tuner_report_<LOCAL>.yaml
    mav_controllers_config/geo_tuner_session_<LOCAL>.csv
    mav_controllers_config/episodes_<LOCAL>/
    mav_controllers_config/geometric_{controller,mavros}.override.yaml[.<stamp>.bak]
    logs/tuner_<UTC>.log
    bags/tuner_<UTC>/

The quad_sim fixtures use a flat layout instead (report.yaml,
report_complete.yaml, launch.log, geo_tuner_session_*.csv in one directory,
no bag); both are supported so sim sessions exercise the same loaders.
"""
from __future__ import annotations

import dataclasses
import datetime as _dt
from pathlib import Path

import yaml

from . import timeutil

# A log/bag starts up to a few minutes before the conductor stamps the report
# name (09-14: log 17 s early). The generous early side absorbs a slow bringup;
# anything further off belongs to a different session.
MATCH_EARLY_S = 300.0
MATCH_LATE_S = 60.0


@dataclasses.dataclass
class SessionArtifacts:
    report: Path
    layout: str = "logs_tree"  # or "flat"
    stamp: str | None = None  # local stamp string from the report name
    start: _dt.datetime | None = None  # best-known session start (aware)
    end: _dt.datetime | None = None  # report `time:` (aware)
    csv: Path | None = None
    episodes_dir: Path | None = None
    log: Path | None = None
    bag: Path | None = None
    report_complete: Path | None = None  # flat layout: report at Tuning complete
    overrides: list[Path] = dataclasses.field(default_factory=list)
    override_baks: list[Path] = dataclasses.field(default_factory=list)
    missing: list[str] = dataclasses.field(default_factory=list)

    @property
    def name(self) -> str:
        if self.layout == "flat":
            return self.report.parent.name
        return self.stamp or self.report.stem


def _report_window(report: Path, tz) -> tuple[_dt.datetime | None, _dt.datetime | None]:
    """(start, end) of the session, best effort. The name stamp is the
    session start; `time:` inside is when the report was written (end).
    The 09-09 report has neither name time nor session_duration_s, so its
    start stays unknown and window matching degrades to date-only."""
    start = timeutil.parse_local_stamp(report.name, tz)
    end = None
    try:
        doc = yaml.safe_load(report.read_text()) or {}
        end = timeutil.parse_report_time(doc.get("time"), tz)
        if start is None and end is not None and doc.get("session_duration_s"):
            start = end - _dt.timedelta(seconds=float(doc["session_duration_s"]))
    except (OSError, yaml.YAMLError):
        pass
    return start, end


def _closest_in_window(candidates, start: _dt.datetime | None):
    """Pick the candidate whose UTC name stamp is closest to the session
    start, inside [-MATCH_EARLY_S, +MATCH_LATE_S]. Many restarts share a day
    (ten tuner logs on 09-10), so 'same date' is never enough."""
    if start is None:
        return None
    best, best_dt = None, None
    for path, t in candidates:
        if t is None:
            continue
        delta = (t - start).total_seconds()
        if -MATCH_EARLY_S <= delta <= MATCH_LATE_S:
            if best_dt is None or abs(delta) < best_dt:
                best, best_dt = path, abs(delta)
    return best


def discover_logs_tree(root: Path, report: Path, tz=None) -> SessionArtifacts:
    tz = tz or timeutil.local_tz()
    cfg = root / "mav_controllers_config"
    s = SessionArtifacts(report=report)
    m = timeutil.LOCAL_STAMP_RE.search(report.name)
    if m and m.group(2):
        s.stamp = m.group(0)
    s.start, s.end = _report_window(report, tz)

    # CSV and episodes share the report's local stamp verbatim: all three are
    # written by the conductor from the same time_point.
    if s.stamp:
        csv = cfg / f"geo_tuner_session_{s.stamp}.csv"
        eps = cfg / f"episodes_{s.stamp}"
        s.csv = csv if csv.is_file() else None
        s.episodes_dir = eps if eps.is_dir() else None

    logs = sorted((root / "logs").glob("tuner_*.log")) if (root / "logs").is_dir() else []
    s.log = _closest_in_window(
        [(p, timeutil.parse_utc_name(p.name)) for p in logs], s.start
    )
    bags = sorted(p for p in (root / "bags").glob("tuner_*") if p.is_dir()) if (root / "bags").is_dir() else []
    s.bag = _closest_in_window(
        [(p, timeutil.parse_utc_name(p.name)) for p in bags], s.start
    )

    s.overrides = sorted(cfg.glob("geometric_*.override.yaml"))
    # .baks written during or shortly after the session are its saves.
    if s.start is not None:
        late = (s.end or s.start) + _dt.timedelta(minutes=15)
        for p in sorted(cfg.glob("geometric_*.override.yaml.*.bak")):
            t = timeutil.parse_bak_stamp(p.name, tz)
            if t is not None and s.start <= t <= late:
                s.override_baks.append(p)

    for label, val in [
        ("session CSV", s.csv),
        ("episode dumps", s.episodes_dir),
        ("launch log", s.log),
        ("bag", s.bag),
    ]:
        if val is None:
            s.missing.append(label)
    return s


def discover_flat(session_dir: Path, tz=None) -> SessionArtifacts:
    tz = tz or timeutil.local_tz()
    report = session_dir / "report.yaml"
    s = SessionArtifacts(report=report, layout="flat")
    if not report.is_file():
        s.missing.append("report")
    else:
        s.start, s.end = _report_window(report, tz)
    rc = session_dir / "report_complete.yaml"
    s.report_complete = rc if rc.is_file() else None
    log = session_dir / "launch.log"
    s.log = log if log.is_file() else None
    csvs = sorted(session_dir.glob("geo_tuner_session_*.csv"))
    s.csv = csvs[-1] if csvs else None
    eps = sorted(p for p in session_dir.glob("episodes_*") if p.is_dir())
    s.episodes_dir = eps[-1] if eps else None
    if s.csv:
        m = timeutil.LOCAL_STAMP_RE.search(s.csv.name)
        if m and m.group(2):
            s.stamp = m.group(0)
    for label, val in [("session CSV", s.csv), ("launch log", s.log)]:
        if val is None:
            s.missing.append(label)
    # Sim sessions carry no bag by design; not listed as missing noise.
    return s


def find_reports(root: Path) -> list[Path]:
    cfg = root / "mav_controllers_config"
    if not cfg.is_dir():
        return []
    return sorted(cfg.glob("geo_tuner_report_*.yaml"))


def discover(target: Path, latest: bool = False, tz=None) -> list[SessionArtifacts]:
    """Resolve `target` (logs tree, report path, or flat session dir) to
    session artifact sets, oldest first."""
    target = target.expanduser()
    if target.is_file():
        root = target.parent.parent  # .../mav_controllers_config/report.yaml
        if target.parent.name == "mav_controllers_config":
            return [discover_logs_tree(root, target, tz)]
        return [discover_flat(target.parent, tz)]
    if (target / "mav_controllers_config").is_dir():
        reports = find_reports(target)
        if latest and reports:
            reports = reports[-1:]
        return [discover_logs_tree(target, r, tz) for r in reports]
    if (target / "report.yaml").is_file():
        return [discover_flat(target, tz)]
    raise FileNotFoundError(
        f"{target}: neither a logs tree (mav_controllers_config/), a report "
        "yaml, nor a session dir with report.yaml"
    )
