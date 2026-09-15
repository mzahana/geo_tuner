"""Check catalogue, groups A (data integrity), B (preconditions and
configuration) and C (session process). PLAN.md section 5 is the spec; the
expected numbers quoted in comments are the 09-14 field session.

Each check takes the Session plus a shared Context and returns Findings;
missing inputs produce a SKIP finding with the reason, never an exception.
Thresholds come from the session's own log where it carries them (settle
cap, envelope, yaw target); otherwise the geo_tuner defaults quoted here.
"""
from __future__ import annotations

import dataclasses

import numpy as np

from . import overrides as ovr
from . import signals, timeutil
from .findings import FAIL, INFO, PASS, SKIP, WARN, Finding, skip
from .report import GEN_LEGACY, GEN_PRINCIPLED_V2
from .session import Session

# geo_tuner defaults (rounds/identification design, see docs/TUNING_GUIDE.md);
# used only where the session's own artifacts do not carry the value.
MAX_ALPHA_CI_RATIO = 1.19
ESTIMATE_CONSISTENCY = 1.35
SETTLE_TOL_POS_M = 0.06
MAX_ACCEL = 5.0  # m/s^2, controller clamp
SATURATION_MARGIN = 0.8
YAW_T_TOL = 0.25
ODOM_GAP_WARN_S = 0.5


@dataclasses.dataclass
class Context:
    window: signals.Window | None = None
    settle_cap_s: float | None = None


def build_context(s: Session) -> Context:
    ctx = Context()
    if s.log:
        ctx.window = signals.session_window(s.log)
        settles = s.log.all("settling")
        if settles:
            ctx.settle_cap_s = max(e.fields["cap_s"] for e in settles)
    return ctx


# ---------------------------------------------------------------- group A

def check_a1_artifacts(s: Session, ctx: Context) -> list[Finding]:
    art = s.artifacts
    missing = list(art.missing)
    if not missing and not s.load_errors:
        return [Finding("ARTIFACTS_PRESENT", PASS, "all session artifacts found",
                        source=[str(art.report)])]
    # Legacy sessions predate the session recording; a missing CSV there is
    # history, not a defect.
    legacy = s.report is not None and s.report.generation == GEN_LEGACY
    sev = INFO if legacy else WARN
    ev = {}
    if missing:
        ev["missing"] = missing
    if s.load_errors:
        ev["load_errors"] = dict(s.load_errors)
        sev = WARN
    return [Finding("ARTIFACTS_MISSING", sev,
                    "session artifacts missing or unreadable",
                    evidence=ev, source=[str(art.report.parent)],
                    explanation="checks that need these inputs are skipped"
                    + ("; the CSV/log predate this legacy session's tooling"
                       if legacy and missing else ""))]


def check_a2_bag_closed(s: Session, ctx: Context) -> list[Finding]:
    if s.artifacts.bag is None:
        return [skip("BAG_NOT_CLOSED", "bag shutdown", "no bag for this session")]
    if s.bag_status is not None and not s.bag_status.closed:
        return [Finding(
            "BAG_NOT_CLOSED", WARN, "bag was not closed",
            evidence={"reason": s.bag_status.reason},
            source=[str(s.artifacts.bag / "metadata.yaml")],
            explanation="the recorder did not shut down (power cut?); the bag "
                        "is readable after `ros2 bag reindex -s sqlite3` on a "
                        "COPY (ihunter fetch already does this on the drone)",
            recommendation_ids=["REC_IHUNTER_STOP"])]
    if s.log and s.log.first("recording_started") and not s.log.first("recording_stopped"):
        return [Finding(
            "BAG_NOT_CLOSED", WARN, "recorder did not shut down cleanly",
            evidence={"metadata": "present (repaired by ihunter fetch)",
                      "log_recording_stopped": False},
            source=[str(s.artifacts.log), str(s.artifacts.bag / "metadata.yaml")],
            explanation="the launch log never says the recording stopped: "
                        "power was cut before the recorder flushed (09-14 "
                        "pattern); the bag on disk was repaired by fetch",
            recommendation_ids=["REC_IHUNTER_STOP"])]
    return [Finding("BAG_NOT_CLOSED", PASS, "bag closed cleanly",
                    source=[str(s.artifacts.bag / "metadata.yaml")])]


def check_a3_odom_health(s: Session, ctx: Context) -> list[Finding]:
    if s.bag is None:
        return [skip("ODOM_HEALTH", "odometry health", "bag not loaded")]
    h = signals.odom_health(s.bag, ctx.window)
    if h is None:
        return [skip("ODOM_HEALTH", "odometry health", "no odom in the bag")]
    bad = h["max_gap_s"] > ODOM_GAP_WARN_S or h["backwards_stamps"] > 0
    return [Finding(
        "ODOM_HEALTH", WARN if bad else PASS,
        "odometry gaps or stamp disorder" if bad else "odometry healthy",
        evidence={"rate_hz": round(h["rate_hz"], 1),
                  "max_gap_s": round(h["max_gap_s"], 3),
                  "backwards_stamps": h["backwards_stamps"]},
        source=["bag geometric_controller/odom"],
        explanation=f"gap threshold {ODOM_GAP_WARN_S} s inside the tuning window")]


def check_a4_generation(s: Session, ctx: Context) -> list[Finding]:
    if s.report is None:
        return [Finding("REPORT_GENERATION", FAIL, "report missing or unreadable",
                        evidence={"error": s.load_errors.get("report", "absent")})]
    gen = s.report.generation
    notes = {
        GEN_LEGACY: "per-round identification checks (C3-C5) do not apply; "
                    "per-episode alpha/tau_lag only",
        GEN_PRINCIPLED_V2: "full schema (accepted_at, u_peak, yaw_result)",
    }.get(gen, "principled schema written before 9a7b429: accept-time fields "
               "absent, cross-check the log (A5)")
    return [Finding("REPORT_GENERATION", INFO, f"report generation: {gen}",
                    evidence={"generation": gen}, source=[str(s.report.path)],
                    explanation=notes)]


def check_a5_report_vs_log(s: Session, ctx: Context) -> list[Finding]:
    if s.report is None or s.log is None:
        return [skip("REPORT_LOG_MISMATCH", "report vs log consistency",
                     "needs both report and log")]
    mismatches = {}
    trims = s.log.all("trim_update")
    if trims and s.report.accel_trim is not None:
        log_trim = trims[-1].fields["trim"]
        if any(abs(a - b) > 0.015 for a, b in zip(log_trim, s.report.accel_trim)):
            mismatches["accel_trim"] = {"report": list(s.report.accel_trim),
                                        "log_final": list(log_trim)}
    if ctx.window is not None and s.report.session_duration_s:
        dur = s.report.session_duration_s
        if abs(ctx.window.span_s - dur) > max(15.0, 0.1 * dur):
            mismatches["duration_s"] = {"report": dur,
                                        "log_span": round(ctx.window.span_s, 1),
                                        "span_source": ctx.window.source}
    if not mismatches:
        return []  # consistent: silence, per the phase-2 acceptance
    pre_fix = s.report.generation != GEN_PRINCIPLED_V2
    return [Finding(
        "REPORT_LOG_MISMATCH", WARN, "report disagrees with the launch log",
        evidence=mismatches,
        source=[str(s.report.path), str(s.artifacts.log)],
        explanation=("report written at completion by a geo_tuner before "
                     "9a7b429, then the session continued to accept: trust "
                     "the log values" if pre_fix else
                     "report and log disagree on a post-9a7b429 session: "
                     "investigate before trusting either"),
        recommendation_ids=["REC_TRUST_LOG"] if pre_fix else [])]


# ---------------------------------------------------------------- group B

def check_b1_estimator_off(s: Session, ctx: Context) -> list[Finding]:
    if s.log is None:
        return [skip("ESTIMATOR_NOT_VERIFIED", "thrust estimator off",
                     "no launch log")]
    on = s.log.first("estimator_on_config")
    if on:
        return [Finding(
            "ESTIMATOR_NOT_VERIFIED", FAIL,
            "thrust estimator was ON during the session",
            evidence={"config_line": on.fields["detail"]},
            source=[str(s.artifacts.log)],
            explanation="the estimator rescales thrust (hence every gain) "
                        "live: the identification and the flown gains do not "
                        "describe a fixed plant (the 09-10/09-13 sessions "
                        "flew like this)",
            recommendation_ids=["REC_ESTIMATOR_PIN"])]
    if s.log.first("estimator_off_verified"):
        return [Finding("ESTIMATOR_NOT_VERIFIED", PASS,
                        "thrust estimator verified off",
                        source=[str(s.artifacts.log)],
                        evidence={"config_line": bool(s.log.first("estimator_off_config"))})]
    up = s.log.first("conductor_up")
    if up and "estimator check: OFF" in up.raw:
        return [Finding("ESTIMATOR_NOT_VERIFIED", INFO,
                        "no estimator in this rig (check disabled by config)",
                        source=[str(s.artifacts.log)])]
    return [Finding(
        "ESTIMATOR_NOT_VERIFIED", FAIL,
        "thrust estimator was NOT verified off",
        source=[str(s.artifacts.log)],
        explanation="an estimator rescaling thrust mid-session invalidates "
                    "the identification (max_thrust scales all gains)",
        recommendation_ids=["REC_ESTIMATOR_PIN"])]


def check_b2_override_provenance(s: Session, ctx: Context) -> list[Finding]:
    if s.log is None:
        return [skip("OVERRIDE_PROVENANCE", "override provenance", "no launch log")]
    loaded = s.log.all("override_in_effect")
    baselines = {e.fields["axis"]: e.fields for e in s.log.all("baseline_gains")}
    if not loaded:
        return [Finding("OVERRIDE_PROVENANCE", INFO,
                        "no override in effect (package defaults flown)",
                        source=[str(s.artifacts.log)],
                        evidence={"baselines": {a: {"kx": b["kx"], "kv": b["kv"]}
                                                for a, b in baselines.items()}})]
    # gain_saver renames the old override to .bak on save, so this session's
    # earliest .bak holds the gains it entered with. A session that saved
    # nothing (09-13 10:38) is covered by the NEXT save's .bak -- the live
    # file may have been overwritten by later sessions.
    entry_file = None
    for p in s.artifacts.override_baks:
        if "geometric_controller" in p.name:
            entry_file = p
            break
    if entry_file is None and s.artifacts.start is not None:
        cfg = s.artifacts.report.parent
        later = []
        for p in cfg.glob("geometric_controller.override.yaml.*.bak"):
            t = timeutil.parse_bak_stamp(p.name)
            if t is not None and t > s.artifacts.start:
                later.append((t, p))
        if later:
            entry_file = min(later)[1]
    if entry_file is None:
        for p in s.artifacts.overrides:
            if "geometric_controller" in p.name:
                entry_file = p  # nothing saved since: the live file is the entry file
                break
    if entry_file is None or not baselines:
        return [Finding("OVERRIDE_PROVENANCE", INFO, "override in effect",
                        evidence={"overrides": [e.fields["path"] for e in loaded]},
                        source=[str(s.artifacts.log)])]
    file_gains = ovr.controller_gains(entry_file)
    if file_gains is None:
        return [Finding("OVERRIDE_PROVENANCE", WARN,
                        "override file unreadable",
                        evidence={"file": str(entry_file)})]
    diffs = {}
    for ax, b in baselines.items():
        fg = file_gains.get(ax)
        if not fg or fg["kx"] is None:
            continue
        for key in ("kx", "kv"):
            # the thrust estimator rescales live gains by ~0.5 %; 2 % is a
            # real mismatch (a stale override from another vehicle/SITL run)
            if abs(b[key] - fg[key]) > 0.02 * max(1.0, abs(fg[key])):
                diffs.setdefault(ax, {})[key] = {"baseline": b[key], "file": fg[key]}
    if diffs:
        return [Finding(
            "OVERRIDE_PROVENANCE", WARN,
            "gains in force at start do not match the override on disk",
            evidence={"entry_file": entry_file.name, "diffs": diffs},
            source=[str(s.artifacts.log), str(entry_file)],
            explanation="stale gain-override trap: the file may be from "
                        "another vehicle or a SITL run")]
    return [Finding("OVERRIDE_PROVENANCE", PASS,
                    "baseline gains match the override history",
                    evidence={"entry_file": entry_file.name,
                              "axes_checked": sorted(baselines)},
                    source=[str(s.artifacts.log), str(entry_file)])]


def check_b3_hover_thrust(s: Session, ctx: Context) -> list[Finding]:
    if s.bag is None:
        return [skip("HOVER_THRUST_OFF", "max_thrust calibration", "bag not loaded")]
    mass_e = s.log.first("mass") if s.log else None
    thr_e = s.log.first("max_thrust") if s.log else None
    if mass_e is None or thr_e is None:
        return [skip("HOVER_THRUST_OFF", "max_thrust calibration",
                     "mass/max_thrust not in the log")]
    mass, max_thrust = mass_e.fields["mass_kg"], thr_e.fields["max_thrust_n"]
    ht = signals.hover_thrust(s.bag, ctx.window)
    if ht is None:
        return [skip("HOVER_THRUST_OFF", "max_thrust calibration",
                     "no attitude thrust stream in the bag")]
    predicted = mass * signals.G / max_thrust
    rel = abs(ht["median"] - predicted) / predicted
    ev = {"hover_thrust_median": round(ht["median"], 3),
          "predicted_mg_over_max_thrust": round(predicted, 3),
          "relative_error": round(rel, 3),
          "mass_kg": mass, "max_thrust_n": max_thrust}
    if s.report and s.report.diagnosis:
        ev["report_diagnosis"] = s.report.diagnosis
    if rel > 0.05:
        return [Finding("HOVER_THRUST_OFF", WARN,
                        "hover thrust disagrees with the thrust map",
                        evidence=ev,
                        source=["bag mavros/setpoint_raw/attitude", "log mass/max_thrust"],
                        explanation="max_thrust scales every gain; a >5 % "
                                    "error de-tunes the whole controller",
                        recommendation_ids=["REC_HOVER_TEST"])]
    return [Finding("HOVER_THRUST_OFF", PASS, "max_thrust calibration good",
                    evidence=ev,
                    source=["bag mavros/setpoint_raw/attitude", "log mass/max_thrust"])]


def check_b4_envelope(s: Session, ctx: Context) -> list[Finding]:
    if s.log is None:
        return [skip("ENVELOPE_CHANGED_LIVE", "envelope flown", "no launch log")]
    env = s.log.first("step_envelope")
    steps = s.log.all("step")
    if env is None or not steps:
        return [skip("ENVELOPE_CHANGED_LIVE", "envelope flown",
                     "no envelope/steps in the log")]
    flown_lat = max((abs(e.fields["size"]) for e in steps
                     if e.fields["axis"] in ("x", "y")), default=0.0)
    flown_vert = max((abs(e.fields["size"]) for e in steps
                      if e.fields["axis"] == "z"), default=0.0)
    changes = {e.fields["name"]: e.fields["value"] for e in s.log.all("param_change")}
    ev = {"profile": {"lateral_m": env.fields["lateral_m"],
                      "vertical_m": env.fields["vertical_m"]},
          "flown": {"lateral_m": flown_lat, "vertical_m": flown_vert}}
    if changes:
        ev["live_changes"] = changes
    gate = changes.get("min_tuning_altitude")
    if gate is None:
        ag = s.log.first("altitude_gate")
        gate = ag.fields["gate_m"] if ag else None
    if gate is not None and s.bag is not None:
        alt = signals.altitude_at(s.bag, steps[0].epoch)
        if alt is not None:
            ev["start_altitude_m"] = round(alt, 2)
            ev["gate_m"] = gate
    over = (flown_lat > env.fields["lateral_m"] + 1e-6
            or flown_vert > env.fields["vertical_m"] + 1e-6)
    if over and any(k.startswith("step_size") for k in changes):
        return [Finding("ENVELOPE_CHANGED_LIVE", INFO,
                        "step size raised live above the profile",
                        evidence=ev, source=[str(s.artifacts.log)],
                        explanation="deliberate panel change during the "
                                    "session (09-14 pattern: 1 m flown over "
                                    "a 0.5/0.4 m profile)")]
    if over:
        return [Finding("ENVELOPE_CHANGED_LIVE", WARN,
                        "steps flown exceed the profile with no live change "
                        "in the log", evidence=ev, source=[str(s.artifacts.log)])]
    return [Finding("ENVELOPE_CHANGED_LIVE", PASS, "envelope respected",
                    evidence=ev, source=[str(s.artifacts.log)])]


def check_b5_headroom(s: Session, ctx: Context) -> list[Finding]:
    max_u = MAX_ACCEL
    ev: dict = {"max_accel_assumed": max_u, "saturation_margin": SATURATION_MARGIN}
    peaks = {}
    if s.report and any(e.u_peak is not None for e in s.report.step_episodes):
        for e in s.report.step_episodes:
            if e.u_peak is not None:
                peaks[e.axis] = max(peaks.get(e.axis, 0.0), e.u_peak)
        near = [f"{e.axis} r{e.round_} rep{e.rep}" for e in s.report.step_episodes
                if e.near_saturation]
        if near:
            ev["near_saturation_episodes"] = near
        ev["source"] = "report u_peak"
        src = [str(s.report.path)]
    elif s.csv is not None:
        for i, ax in enumerate("xyz"):
            peaks[ax] = float(np.abs(s.csv.u[:, i]).max())
        ev["source"] = "session CSV peak |u|"
        src = [str(s.artifacts.csv)]
    else:
        return [skip("NEAR_SATURATION", "command headroom",
                     "no u_peak in the report and no session CSV")]
    ev["peak_cmd_accel"] = {a: round(v, 2) for a, v in peaks.items()}
    worst = max(peaks.values(), default=0.0)
    if worst > SATURATION_MARGIN * max_u:
        return [Finding("NEAR_SATURATION", WARN,
                        "commanded acceleration near the clamp",
                        evidence=ev, source=src,
                        explanation="the controller clamps before the "
                                    "identification sees it: alpha comes out "
                                    "biased low",
                        recommendation_ids=["REC_SMALLER_STEP"])]
    return [Finding("NEAR_SATURATION", PASS, "command headroom healthy",
                    evidence=ev, source=src)]


# ---------------------------------------------------------------- group C

def check_c1_time_budget(s: Session, ctx: Context) -> list[Finding]:
    if s.report is None or not s.report.step_episodes:
        return [skip("SETTLE_CAP_BOUND", "time budget", "no episodes in the report")]
    eps = s.report.step_episodes
    cap = ctx.settle_cap_s or 4.0
    settles = [e.settle_s for e in eps if e.settle_s is not None]
    records = [e.record_s for e in eps if e.record_s is not None]
    # a capped settle overshoots the cap by one 20 ms tick (4.02 on a 4.0
    # cap); 3.92 settled on its own and does not count (09-14: 27/40)
    at_cap = sum(1 for v in settles if v >= cap)
    ev = {"settle_cap_s": cap,
          "settles_at_cap": f"{at_cap}/{len(settles)}",
          "sum_settle_s": round(sum(settles), 1),
          "sum_record_s": round(sum(records), 1)}
    if s.report.session_duration_s:
        ev["session_duration_s"] = s.report.session_duration_s
    at_rec_cap = sum(1 for v in records if v >= 7.9)
    if at_rec_cap:
        ev["episodes_at_episode_time"] = at_rec_cap
    if settles and at_cap / len(settles) > 0.5:
        # A 4 s cap eats minutes (09-14: 126 s settling, 27/40 capped); the
        # 1.5 s cap of 9a7b429 makes the same air cost <= 1.5 s/step.
        sev = WARN if cap > 2.0 else INFO
        return [Finding("SETTLE_CAP_BOUND", sev,
                        "settle cap bound: hover wander above the quiet gate",
                        evidence=ev, source=[str(s.report.path)],
                        explanation="the vehicle rarely met the quiet gate "
                                    "before the settle timeout"
                        + ("; 9a7b429 caps this at 1.5 s/step" if cap > 2.0 else ""),
                        recommendation_ids=["REC_GUSTY_EXPECTED"])]
    return [Finding("SETTLE_CAP_BOUND", PASS, "time budget healthy",
                    evidence=ev, source=[str(s.report.path)])]


def check_c2_air_quality(s: Session, ctx: Context) -> list[Finding]:
    if s.bag is None:
        return [skip("HOVER_WANDER", "air quality", "bag not loaded")]
    hn = signals.hover_noise(s.bag)
    if hn is None:
        return [skip("HOVER_WANDER", "air quality",
                     "not enough parked OFFBOARD samples")]
    std_cm = [round(float(v) * 100.0, 1) for v in hn["std_m"]]
    ev = {"hover_std_cm": std_cm, "settle_tol_cm": SETTLE_TOL_POS_M * 100,
          "n_samples": hn["n_samples"]}
    above = any(v > SETTLE_TOL_POS_M * 100 for v in std_cm)
    return [Finding(
        "HOVER_WANDER", INFO,
        "hover wander above the quiet gate" if above else "air was calm",
        evidence=ev,
        source=["bag odom/setpoint/state (steps.py method)"],
        explanation=("wander above settle_tol_pos explains settle timeouts "
                     "(C1); air quality, not a controller defect" if above
                     else "wander within settle_tol_pos"))]


def check_c3_verdict_trajectory(s: Session, ctx: Context) -> list[Finding]:
    r = s.report
    if r is None:
        return [skip("VERDICT_TRAJECTORY", "verdict trajectory", "no report")]
    if r.generation == GEN_LEGACY:
        return [skip("VERDICT_TRAJECTORY", "verdict trajectory",
                     "legacy report: no per-round identification")]
    out: list[Finding] = []
    if r.status == "aborted":
        out.append(Finding("SESSION_ABORTED", FAIL, "session aborted",
                           evidence={"diagnosis": r.diagnosis},
                           source=[str(r.path)],
                           recommendation_ids=["REC_KEEP_ENTRY_GAINS"]))
    for ident in r.identifications:
        act = (ident.action or "") + " " + (ident.why or "")
        if "restor" in act or "validation failed" in act:
            out.append(Finding(
                "VALIDATION_RESTORED", WARN,
                f"{ident.axis}: applied gains failed validation and were restored",
                axis=ident.axis,
                evidence={"round": ident.round_, "action": ident.action},
                source=[str(r.path)],
                recommendation_ids=["REC_KEEP_ENTRY_GAINS"]))
    last = r.last_identification
    for ax, ident in last.items():
        if ident.verdict == "update":
            out.append(Finding(
                "UPDATE_UNFLOWN", WARN,
                f"{ax}: update indicated with no round left to validate it",
                axis=ax,
                evidence={"round": ident.round_, "why": ident.why},
                source=[str(r.path)],
                explanation="the design was computed but never applied and "
                            "validated in flight"))
    for ax, fg in (r.final_gains or {}).items():
        outcome = str(fg.get("outcome", ""))
        result = str(fg.get("result", ""))
        undecided = ("more data needed" in outcome or "inconclusive" in outcome
                     or result == "kept_inconclusive")
        if undecided:
            out.append(Finding(
                "AXIS_UNDECIDED", INFO,
                f"{ax}: kept without a confirming interval",
                axis=ax,
                evidence={"outcome": outcome, "result": result or None,
                          "validated": fg.get("validated")},
                source=[str(r.path)],
                recommendation_ids=["REC_TUNER_CONFIRM"]))
    if not out:
        out.append(Finding("VERDICT_TRAJECTORY", PASS,
                           "every axis reached a clean verdict",
                           evidence={ax: (fg.get("result") or fg.get("outcome", ""))[:60]
                                     for ax, fg in (r.final_gains or {}).items()},
                           source=[str(r.path)]))
    return out


def check_c4_knife_edge(s: Session, ctx: Context) -> list[Finding]:
    r = s.report
    if r is None or not r.identifications:
        return [skip("CI_KNIFE_EDGE", "knife-edge intervals",
                     "no identifications in the report")]
    out = []
    for ax, ident in r.last_identification.items():
        ratio = ident.ci_ratio
        if ratio is None or ident.verdict not in ("inconclusive", None):
            continue
        if ratio <= MAX_ALPHA_CI_RATIO * 1.03:
            out.append(Finding(
                "CI_KNIFE_EDGE", INFO,
                f"{ax}: interval missed confirmation by less than its own noise",
                axis=ax,
                evidence={"ci_ratio": round(ratio, 3),
                          "max_alpha_ci_ratio": MAX_ALPHA_CI_RATIO,
                          "round": ident.round_},
                source=[str(r.path)],
                explanation="not a defect: the gate is a knife edge at this "
                            "excitation level",
                recommendation_ids=["REC_TUNER_CONFIRM"]))
    return out


def check_c5_ci_growth(s: Session, ctx: Context) -> list[Finding]:
    r = s.report
    if r is None or not r.identifications:
        return []
    out = []
    by_axis: dict[str, list] = {}
    for ident in r.identifications:
        by_axis.setdefault(ident.axis, []).append(ident)
    for ax, idents in by_axis.items():
        ratios = [(i.round_, i.ci_ratio) for i in idents if i.ci_ratio]
        for (r0, a), (r1, b) in zip(ratios, ratios[1:]):
            # more data should shrink the interval ~1/sqrt(excited_s); a
            # widening round flew in noisier air (09-14 x: 1.21->1.36->1.20)
            if b > a * 1.02:
                out.append(Finding(
                    "CI_WIDENED", INFO,
                    f"{ax}: interval widened with more data in round {r1}",
                    axis=ax,
                    evidence={"ci_ratio_by_round":
                              {str(rr): round(v, 3) for rr, v in ratios}},
                    source=[str(r.path)],
                    explanation="noisier air in that round; the pooled "
                                "estimate still narrows over the session"))
                break
    return out


def check_c6_trim(s: Session, ctx: Context) -> list[Finding]:
    if s.log is None:
        return [skip("TRIM_LEARNED", "accel trim", "no launch log")]
    trims = s.log.all("trim_update")
    if not trims:
        return [Finding("TRIM_NOT_LEARNED", INFO, "no accel trim learned",
                        source=[str(s.artifacts.log)],
                        explanation="the vehicle never parked quiet long "
                                    "enough for a trim estimate (gusty air)")]
    final = trims[-1].fields["trim"]
    return [Finding("TRIM_LEARNED", PASS,
                    f"accel trim learned over {len(trims)} updates",
                    evidence={"final_trim": list(final),
                              "updates": len(trims)},
                    source=[str(s.artifacts.log)])]


def check_c7_yaw(s: Session, ctx: Context) -> list[Finding]:
    r = s.report
    if r is None or (not r.yaw_rounds and r.final_yawctrl_tau is None):
        return [skip("YAW_IDENTIFIED", "yaw identification", "no yaw data")]
    ev: dict = {}
    if r.yaw_rounds:
        ev["spread_by_round"] = {str(y.round_): y.spread for y in r.yaw_rounds}
        ev["estimate_consistency_max"] = ESTIMATE_CONSISTENCY
    if s.log:
        fits = s.log.all("yaw_fit")
        if fits:
            ev["T_median_s"] = round(float(np.median(
                [f.fields["T_s"] for f in fits])), 2)
            ev["delay_median_ms"] = round(float(np.median(
                [f.fields["delay_ms"] for f in fits])), 0)
        by = s.log.first("baseline_yaw")
        if by:
            ev["target_T_s"] = by.fields["target_T"]
    if s.log and r.generation == GEN_LEGACY:
        upd = s.log.last("yaw_updated_legacy")
        if upd:
            ev["legacy_update"] = {k: upd.fields[k]
                                   for k in ("T_s", "n", "spread", "tau_to")}
            return [Finding("YAW_IDENTIFIED", PASS,
                            "yaw updated (legacy consistency method)",
                            axis="yaw", evidence=ev,
                            source=[str(s.artifacts.log)])]
    confirmed = bool(
        (r.yaw_result and "validated" in r.yaw_result)
        or (s.log and s.log.first("yaw_confirmed")))
    if r.yaw_result:
        ev["yaw_result"] = r.yaw_result
    if confirmed:
        conf = s.log.first("yaw_confirmed") if s.log else None
        if conf:
            ev["confirmed"] = conf.fields["detail"]
        return [Finding("YAW_IDENTIFIED", PASS, "yaw identified and confirmed",
                        axis="yaw", evidence=ev,
                        source=[str(r.path)] + ([str(s.artifacts.log)] if s.log else []))]
    return [Finding("YAW_NOT_IDENTIFIED", WARN, "yaw never confirmed",
                    axis="yaw", evidence=ev, source=[str(r.path)],
                    explanation="yaw estimates stayed inconsistent across "
                                "the session")]


_C8_KEYWORDS = ("OFFBOARD lost", "pausing", "paused", "restor", "abort",
                "timed out", "timeout", "AGL lost", "leaving OFFBOARD")


def check_c8_pauses(s: Session, ctx: Context) -> list[Finding]:
    if s.log is None:
        return [skip("SESSION_INTERRUPTED", "pauses and aborts", "no launch log")]
    hits = []
    for e in s.log.all("conductor_info"):
        msg = e.fields["msg"]
        if any(k.lower() in msg.lower() for k in _C8_KEYWORDS):
            hits.append(msg)
    if hits:
        return [Finding("SESSION_INTERRUPTED", WARN,
                        "session was interrupted mid-flight",
                        evidence={"log_lines": hits[:8]},
                        source=[str(s.artifacts.log)],
                        recommendation_ids=["REC_KEEP_ENTRY_GAINS"])]
    return [Finding("SESSION_INTERRUPTED", PASS, "no pauses or aborts",
                    source=[str(s.artifacts.log)])]


def check_c9_saved(s: Session, ctx: Context) -> list[Finding]:
    r = s.report
    if r is None or s.log is None:
        return [skip("GAINS_SAVED", "saved outcome", "needs report and log")]
    accepted = s.log.first("accepted")
    saved = s.log.first("gains_saved")
    if accepted is None:
        # Can be the right call: 09-13 10:38 completed and the operator chose
        # not to accept.
        return [Finding("COMPLETE_NOT_ACCEPTED", INFO,
                        "session complete but gains never accepted/saved",
                        evidence={"status": r.status},
                        source=[str(s.artifacts.log)],
                        explanation="the entry gains still fly; accept+save "
                                    "next time if the results were better",
                        recommendation_ids=["REC_ACCEPT_NEXT_TIME"])]
    if saved is None:
        if s.artifacts.layout == "flat":
            return [Finding("GAINS_SAVED", INFO,
                            "accepted; no gain_saver in this rig (sim)",
                            source=[str(s.artifacts.log)])]
        return [Finding("GAINS_SAVED", WARN,
                        "gains accepted but never saved to the vehicle",
                        source=[str(s.artifacts.log)],
                        explanation="the accepted gains were live only; the "
                                    "next boot flies the old override")]
    # Compare the saved override with final_gains -- but only when no later
    # session has overwritten the file since.
    cfg = s.artifacts.report.parent
    all_baks = sorted(p.name for p in cfg.glob(
        "geometric_controller.override.yaml.*.bak"))
    mine = sorted(p.name for p in s.artifacts.override_baks
                  if "geometric_controller" in p.name)
    ev: dict = {"saved": True}
    if all_baks and mine and all_baks[-1] != mine[-1]:
        ev["comparison"] = "skipped: a later session overwrote the override"
        return [Finding("GAINS_SAVED", PASS, "gains accepted and saved",
                        evidence=ev, source=[str(s.artifacts.log)])]
    live = None
    for p in s.artifacts.overrides:
        if "geometric_controller" in p.name:
            live = ovr.controller_gains(p)
            break
    if live and r.final_gains:
        diffs = {}
        for ax, fg in r.final_gains.items():
            fl = live.get(ax)
            if not fl:
                continue
            for key in ("kx", "kv"):
                want, got = fg.get(key), fl.get(key)
                if want is None or got is None:
                    continue
                # the estimator rescales live gains by up to ~0.5 % on save
                if abs(got - want) > 0.005 * max(1.0, abs(want)):
                    diffs.setdefault(ax, {})[key] = {"final": want, "override": got}
        if diffs:
            return [Finding("SAVED_GAINS_MISMATCH", WARN,
                            "saved override differs from the report's final gains",
                            evidence={"diffs": diffs},
                            source=[str(r.path)])]
        ev["comparison"] = "override matches final_gains (+/-0.5 %)"
    return [Finding("GAINS_SAVED", PASS, "gains accepted, saved, and the "
                    "override matches the report",
                    evidence=ev, source=[str(s.artifacts.log)])]


ALL_CHECKS = [
    check_a1_artifacts, check_a2_bag_closed, check_a3_odom_health,
    check_a4_generation, check_a5_report_vs_log,
    check_b1_estimator_off, check_b2_override_provenance,
    check_b3_hover_thrust, check_b4_envelope, check_b5_headroom,
    check_c1_time_budget, check_c2_air_quality, check_c3_verdict_trajectory,
    check_c4_knife_edge, check_c5_ci_growth, check_c6_trim, check_c7_yaw,
    check_c8_pauses, check_c9_saved,
]


def run_checks(s: Session, ctx: Context | None = None) -> list[Finding]:
    # imported here: checks_de depends on this module's Context
    from .checks_de import DE_CHECKS
    ctx = ctx or build_context(s)
    findings: list[Finding] = []
    for check in ALL_CHECKS + DE_CHECKS:
        try:
            findings.extend(check(s, ctx))
        except Exception as e:  # a broken check must not sink the report
            findings.append(Finding(
                check.__name__.upper(), SKIP,
                f"{check.__name__} crashed",
                evidence={"error": f"{type(e).__name__}: {e}"},
                explanation="doctor bug: fix the check"))
    return findings
