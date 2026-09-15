"""Stamp parsing for session artifacts.

Two clocks name the same session (09-14: report 08-58-27 local, bag/log
055811/055810 UTC). Reports, episode dirs, session CSVs and override .bak
files carry LOCAL stamps; launch logs and bags carry UTC stamps. Artifacts
are matched by time window, never by string equality of names.
"""
from __future__ import annotations

import datetime as _dt
import re

# geo_tuner_report_2026-09-14_08-58-27.yaml (local); the first field report,
# geo_tuner_report_2026-09-09.yaml, has only the date -- read `time:` inside.
LOCAL_STAMP_RE = re.compile(r"(\d{4}-\d{2}-\d{2})(?:_(\d{2})-(\d{2})-(\d{2}))?")
# tuner_20260914-055811 (UTC): launch log and bag directory names.
UTC_NAME_RE = re.compile(r"(\d{8})-(\d{6})")
# geometric_controller.override.yaml.20260914-090515.bak (local).
BAK_STAMP_RE = re.compile(r"\.(\d{8})-(\d{6})\.bak$")


def local_tz() -> _dt.tzinfo:
    """The machine's local zone. The field laptop and the dev PC sit in the
    vehicle's timezone (+03:00 on 09-14); --utc-offset exists for the day
    that stops being true."""
    return _dt.datetime.now().astimezone().tzinfo


def parse_local_stamp(text: str, tz: _dt.tzinfo | None = None) -> _dt.datetime | None:
    """Local stamp out of a report/episodes/CSV name; None when only a date
    is present (time unknown, not midnight)."""
    m = LOCAL_STAMP_RE.search(text)
    if not m or m.group(2) is None:
        return None
    date = _dt.date.fromisoformat(m.group(1))
    t = _dt.time(int(m.group(2)), int(m.group(3)), int(m.group(4)))
    return _dt.datetime.combine(date, t, tzinfo=tz or local_tz())


def parse_local_date(text: str) -> _dt.date | None:
    m = LOCAL_STAMP_RE.search(text)
    return _dt.date.fromisoformat(m.group(1)) if m else None


def parse_utc_name(text: str) -> _dt.datetime | None:
    """UTC stamp out of a log/bag name (tuner_20260914-055811)."""
    m = UTC_NAME_RE.search(text)
    if not m:
        return None
    return _dt.datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S").replace(
        tzinfo=_dt.timezone.utc
    )


def parse_bak_stamp(name: str, tz: _dt.tzinfo | None = None) -> _dt.datetime | None:
    """Local stamp of an override .bak (written by gain_saver on save)."""
    m = BAK_STAMP_RE.search(name)
    if not m:
        return None
    return _dt.datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S").replace(
        tzinfo=tz or local_tz()
    )


def parse_report_time(text: str, tz: _dt.tzinfo | None = None) -> _dt.datetime | None:
    """The report's `time:` field ("2026-09-14 09:05:14", local clock)."""
    try:
        naive = _dt.datetime.strptime(str(text).strip(), "%Y-%m-%d %H:%M:%S")
    except (ValueError, TypeError):
        return None
    return naive.replace(tzinfo=tz or local_tz())


def fmt_both_clocks(t: _dt.datetime | None) -> str:
    """Always show both clocks: the Jetson's RTC can be wrong at boot and
    the two naming conventions have already caused one mismatched analysis."""
    if t is None:
        return "unknown"
    return "{} local / {} UTC".format(
        t.strftime("%Y-%m-%d %H:%M:%S"),
        t.astimezone(_dt.timezone.utc).strftime("%H:%M:%S"),
    )
