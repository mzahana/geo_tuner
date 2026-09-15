"""Parse a tuner launch log into typed events with epoch time.

The log is the only artifact that carries ROS epoch stamps for every
conductor decision, and after the 09-14 flight it proved more trustworthy
than the report (the report was written at completion by a pre-9a7b429
conductor: trim [0,0,0] vs the log's [0.20,-0.10,-0.00]). Checks therefore
lean on these events; keep the regexes in sync with tuning_conductor.cpp.

Field logs prefix node names with the namespace (interceptor.tuning_conductor),
quad_sim logs do not (tuning_conductor); match on the suffix.
"""
from __future__ import annotations

import dataclasses
import datetime as _dt
import re
from pathlib import Path

# `[tuning_conductor-6] [INFO] [1789365507.236051480] [interceptor.tuning_conductor]: msg`
_ROS_LINE_RE = re.compile(
    r"^(?:\[[^\]]+\] )?\[(\w+)\] \[(\d+\.\d+)\] \[([\w./]+)\]: (.*)$"
)
# `### 2026-09-14T05:58:10Z ihunter-run start tuner` (written by bin/ihunter)
_RUN_START_RE = re.compile(r"^### (\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})Z ihunter-run start (\S+)")
_LAUNCH_USER_RE = re.compile(r"^\[(\w+)\] \[launch\.user\]: ?(.*)$")


@dataclasses.dataclass
class Event:
    epoch: float | None  # ROS epoch seconds; None for launch.user lines
    node: str  # suffix-normalized node name ("" for launch.user)
    type: str
    fields: dict
    raw: str

    @property
    def time(self) -> _dt.datetime | None:
        if self.epoch is None:
            return None
        return _dt.datetime.fromtimestamp(self.epoch, tz=_dt.timezone.utc)


@dataclasses.dataclass
class LaunchLog:
    path: Path
    run_start: _dt.datetime | None = None  # from the ihunter-run header (UTC)
    launch_target: str | None = None  # "tuner", "navigator", ...
    events: list[Event] = dataclasses.field(default_factory=list)

    def all(self, *types: str) -> list[Event]:
        return [e for e in self.events if e.type in types]

    def first(self, *types: str) -> Event | None:
        for e in self.events:
            if e.type in types:
                return e
        return None

    def last(self, *types: str) -> Event | None:
        for e in reversed(self.events):
            if e.type in types:
                return e
        return None


_AXIS = r"(x|y|z|yaw)"
_NUM = r"([-+]?\d+(?:\.\d+)?)"

# (regex on the message, event type, field names in group order).
# Conductor lines first; they are what the checks consume.
_CONDUCTOR_PATTERNS: list[tuple[re.Pattern, str, tuple[str, ...]]] = [
    (re.compile(
        rf"Step envelope: \+/-{_NUM} m lateral, \+/-{_NUM} m vertical, "
        rf"\+/-{_NUM} rad yaw .*max_pos_error {_NUM} m"),
     "step_envelope", ("lateral_m", "vertical_m", "yaw_rad", "max_pos_error_m")),
    (re.compile(
        rf"Tuning conductor up\. axes=\[([^\]]+)\] wn_target={_NUM} \(z {_NUM}\) "
        rf"zeta={_NUM} PM>={_NUM}/{_NUM} deg rounds<=(\d+), (\d+) steps/axis/round"),
     "conductor_up", ("axes", "wn_target", "wn_target_z", "zeta", "pm_nominal",
                      "pm_worst", "rounds_max", "steps_per_axis")),
    (re.compile(rf"Altitude gate: start at >= {_NUM} m AGL"),
     "altitude_gate", ("gate_m",)),
    (re.compile(rf"baseline {_AXIS}: kx={_NUM} kv={_NUM} \(wn={_NUM}, zeta={_NUM}\)"),
     "baseline_gains", ("axis", "kx", "kv", "wn", "zeta")),
    (re.compile(rf"baseline yaw: tau={_NUM} \(target T={_NUM}s\)"),
     "baseline_yaw", ("tau", "target_T")),
    (re.compile(r"thrust estimator is off \(verified\)"),
     "estimator_off_verified", ()),
    (re.compile(rf"(\w+) -> {_NUM}[^-]*\(applies from the next episode\)"),
     "param_change", ("name", "value")),
    (re.compile(rf"min_tuning_altitude -> {_NUM} m AGL"),
     "param_change_altitude", ("value",)),
    (re.compile(r"Start requested"),
     "start_requested", ()),
    # legacy logs say "rung R/N" where principled says "round R/N"
    (re.compile(rf"OFFBOARD engaged; resuming \((?:round|rung) (\d+)/(\d+), axis {_AXIS}\)"),
     "offboard_engaged", ("round", "rounds_max", "axis")),
    (re.compile(rf"At hover point; settling \(<= {_NUM}s\) \((?:round|rung) (\d+)/(\d+), axis {_AXIS}\)"),
     "settling", ("cap_s", "round", "rounds_max", "axis")),
    (re.compile(rf"Step {_NUM} m on {_AXIS}"),
     "step", ("size", "axis")),
    (re.compile(rf"Yaw step {_NUM} rad"),
     "yaw_step", ("size",)),
    (re.compile(rf"{_AXIS}: step {_NUM} flown, overshoot {_NUM}%"),
     "step_flown", ("axis", "size", "overshoot_pct")),
    (re.compile(rf"yaw: T={_NUM}s delay={_NUM}ms nrmse={_NUM}"),
     "yaw_fit", ("T_s", "delay_ms", "nrmse")),
    (re.compile(
        rf"steady offset -> accel trim \[{_NUM}, {_NUM}, {_NUM}\] m/s\^2 \((\d+)/(\d+)\)"),
     "trim_update", ("tx", "ty", "tz", "k", "n")),
    (re.compile(r"round (\d+)/(\d+) flown; identifying"),
     "round_flown", ("round", "rounds_max")),
    (re.compile(rf"identification took {_NUM} s"),
     "identification_done", ("took_s",)),
    (re.compile(
        rf"{_AXIS}: alpha {_NUM} \[{_NUM}, {_NUM}\], lag {_NUM} ms \[{_NUM}, {_NUM}\] -> (.*)"),
     "identification_axis", ("axis", "alpha", "alpha_lo", "alpha_hi",
                             "lag_ms", "lag_lo", "lag_hi", "decision")),
    (re.compile(rf"{_AXIS}: applied gains validated: (.*)"),
     "gains_validated", ("axis", "detail")),
    (re.compile(r"yaw: confirmed: (.*)"),
     "yaw_confirmed", ("detail",)),
    # legacy yaw update: "yaw: median T=0.31s (n=2, spread 1.01x) -> tau 0.976 -> 1.091"
    (re.compile(rf"yaw: median T={_NUM}s \(n=(\d+), spread {_NUM}x\) -> tau {_NUM} -> {_NUM}"),
     "yaw_updated_legacy", ("T_s", "n", "spread", "tau_from", "tau_to")),
    (re.compile(r"round (\d+)/(\d+)$"),
     "round_start", ("round", "rounds_max")),
    (re.compile(r"Tuning complete\. Report: (\S+)"),
     "tuning_complete", ("report_path",)),
    (re.compile(r"Accepted the current gains as the safe set\. Report: (\S+)\."),
     "accepted", ("report_path",)),
]

_OTHER_PATTERNS: list[tuple[str, re.Pattern, str, tuple[str, ...]]] = [
    ("gain_saver", re.compile(r"Saved to (\S+) and (\S+)\."),
     "gains_saved", ("controller_override", "mavros_override")),
    ("geometric_controller_node", re.compile(r"Mass = ([\d.]+) [Kk]g"),
     "mass", ("mass_kg",)),
    ("geometric_controller_node", re.compile(r"Got (\S+)\s+= ([-\d.]+)"),
     "gain_applied", ("param", "value")),
    ("geometric_mavros_node", re.compile(r"Using max_thrust=([\d.]+) N"),
     "max_thrust", ("max_thrust_n",)),
    ("geometric_mavros_node", re.compile(r"Thrust-scale estimator OFF"),
     "estimator_off_config", ()),
    # The 09-10/09-13 sessions flew with this: gains rescale live mid-session.
    ("geometric_mavros_node", re.compile(r"Thrust-scale estimator ON: (.*)"),
     "estimator_on_config", ("detail",)),
    ("rosbag2_recorder", re.compile(r"^Recording\.\.\."),
     "recording_started", ()),
    ("rosbag2_recorder", re.compile(r"Recording stopped"),
     "recording_stopped", ()),
    ("rosbag2_storage", re.compile(r"Opened database '(\S+)' for READ_WRITE"),
     "bag_opened", ("db_path",)),
]

_FLOAT_RE = re.compile(r"^[-+]?\d+(\.\d+)?$")


def _typed(fields: tuple[str, ...], groups: tuple) -> dict:
    out = {}
    for name, val in zip(fields, groups):
        if val is None:
            out[name] = None
        elif name in ("axes",):
            out[name] = [a.strip() for a in val.split(",")]
        elif name in ("round", "rounds_max", "steps_per_axis", "k", "n"):
            out[name] = int(val)
        elif _FLOAT_RE.match(val):
            out[name] = float(val)
        else:
            out[name] = val
    return out


def parse_launch_log(path: Path) -> LaunchLog:
    path = Path(path)
    log = LaunchLog(path=path)
    pending_override = False
    with open(path, errors="replace") as fh:
        for line in fh:
            line = line.rstrip("\n")
            m = _RUN_START_RE.match(line)
            if m:
                log.run_start = _dt.datetime.fromisoformat(m.group(1)).replace(
                    tzinfo=_dt.timezone.utc
                )
                log.launch_target = m.group(2)
                continue

            # The OVERRIDE banner is a multi-line launch.user block whose
            # continuation lines carry no [node] prefix; the override file
            # path is the first indented line after the marker.
            if "*** OVERRIDE IN EFFECT ***" in line:
                pending_override = True
                continue
            if pending_override:
                text = line.strip()
                if text.startswith("/") and " " not in text:
                    log.events.append(Event(
                        None, "", "override_in_effect", {"path": text}, line))
                    pending_override = False
                    continue
                if _ROS_LINE_RE.match(line):  # block ended without a path
                    pending_override = False
                else:
                    continue

            if _LAUNCH_USER_RE.match(line):
                continue

            m = _ROS_LINE_RE.match(line)
            if not m:
                continue
            _level, epoch_s, node, msg = m.groups()
            epoch = float(epoch_s)
            short = node.rsplit(".", 1)[-1]  # strip the namespace prefix

            if short == "tuning_conductor":
                for rx, etype, fields in _CONDUCTOR_PATTERNS:
                    mm = rx.search(msg)
                    if mm:
                        f = _typed(fields, mm.groups())
                        if etype == "param_change_altitude":
                            etype, f = "param_change", {
                                "name": "min_tuning_altitude", "value": f["value"]}
                        if etype == "trim_update":
                            f = {"trim": (f["tx"], f["ty"], f["tz"]),
                                 "k": f["k"], "n": f["n"]}
                        if etype == "yaw_step":
                            f["axis"] = "yaw"
                        log.events.append(Event(epoch, short, etype, f, line))
                        break
                else:
                    # Keep unmatched conductor lines: a new conductor message
                    # must never disappear silently from the doctor's view.
                    log.events.append(Event(epoch, short, "conductor_info",
                                            {"msg": msg}, line))
                continue

            for node_suffix, rx, etype, fields in _OTHER_PATTERNS:
                if short == node_suffix or short.endswith(node_suffix):
                    mm = rx.search(msg)
                    if mm:
                        log.events.append(Event(
                            epoch, short, etype, _typed(fields, mm.groups()), line))
                        break
    return log
