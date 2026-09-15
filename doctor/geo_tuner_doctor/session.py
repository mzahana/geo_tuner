"""One tuning session: discovered artifacts + lazily loaded contents.

Checks (phase 2+) take a Session and declare which pieces they need; a piece
that failed to load is recorded in `load_errors` and the check is skipped
with that reason instead of crashing the whole run.
"""
from __future__ import annotations

import dataclasses
from pathlib import Path

from . import bag as bag_mod
from .discover import SessionArtifacts
from .episodes import EpisodeDump, load_episode_dir
from .launch_log import LaunchLog, parse_launch_log
from .report import Report, load_report
from .session_csv import SessionCsv, load_session_csv


@dataclasses.dataclass
class Session:
    artifacts: SessionArtifacts
    report: Report | None = None
    report_complete: Report | None = None
    csv: SessionCsv | None = None
    episode_dumps: list[EpisodeDump] | None = None
    log: LaunchLog | None = None
    bag_status: bag_mod.BagStatus | None = None
    bag: bag_mod.BagData | None = None
    load_errors: dict[str, str] = dataclasses.field(default_factory=dict)

    @property
    def name(self) -> str:
        return self.artifacts.name


def load_session(art: SessionArtifacts, cache_dir: Path | None = None,
                 read_bag: bool = True) -> Session:
    s = Session(artifacts=art)

    def _try(label, fn):
        try:
            return fn()
        except bag_mod.BagUnavailable as e:
            s.load_errors[label] = f"unavailable: {e}"
        except Exception as e:  # a corrupt artifact must not sink the rest
            s.load_errors[label] = f"{type(e).__name__}: {e}"
        return None

    if art.report and art.report.is_file():
        s.report = _try("report", lambda: load_report(art.report))
    if art.report_complete:
        s.report_complete = _try(
            "report_complete", lambda: load_report(art.report_complete))
    if art.csv:
        s.csv = _try("csv", lambda: load_session_csv(art.csv))
    if art.episodes_dir:
        s.episode_dumps = _try(
            "episodes", lambda: load_episode_dir(art.episodes_dir))
    if art.log:
        s.log = _try("log", lambda: parse_launch_log(art.log))
    if art.bag:
        s.bag_status = _try("bag_status", lambda: bag_mod.check_closed(art.bag))
        if read_bag and s.bag_status and s.bag_status.closed:
            cache = cache_dir or (art.bag.parent / "doctor_cache")
            s.bag = _try("bag", lambda: bag_mod.load_bag(art.bag, cache))
    return s
