"""Checks A6 (CSV vs report), D (independent verification: does the data
support the verdict?) and E (plant health). PLAN.md section 5, groups D/E;
the numbers in comments are the 09-14 session's expected outputs.

D-checks lean on the C++ geo-tuner-identify (--json/--segments): the same
2SLS core the flight ran, never a Python re-implementation. When the binary
or the bag is absent, checks skip with the reason -- INFO, never a failure.
"""
from __future__ import annotations

import numpy as np

from . import ensembles, imu_analysis, signals
from .checks import Context
from .findings import FAIL, INFO, PASS, WARN, Finding, skip
from .identify import IdentifyUnavailable, run_identify
from .session import Session

D2_RMS_PASS = 0.03
D2_RMS_WARN = 0.05


def _full_identify(s: Session, ctx: Context) -> dict | None:
    """One full-session identify --json per doctor run, cached on the ctx."""
    if not hasattr(ctx, "_identify_full"):
        ctx._identify_full = None
        if s.csv is not None:
            try:
                ctx._identify_full = run_identify(s.csv.path)
            except IdentifyUnavailable as e:
                ctx._identify_error = str(e)
    return ctx._identify_full


def check_a6_csv_vs_report(s: Session, ctx: Context) -> list[Finding]:
    if s.report is None or s.csv is None:
        return [skip("CSV_REPORT_MISMATCH", "CSV vs report identification",
                     "needs report and session CSV")]
    if not s.report.identifications:
        return [skip("CSV_REPORT_MISMATCH", "CSV vs report identification",
                     "no identifications in the report (legacy)")]
    doc = _full_identify(s, ctx)
    if doc is None:
        return [skip("CSV_REPORT_MISMATCH", "CSV vs report identification",
                     getattr(ctx, "_identify_error", "identify failed"))]
    diffs, ev = {}, {}
    for ax, ident in s.report.last_identification.items():
        got = doc["axes"].get(ax)
        if got is None or not got["ok"] or ident.alpha is None:
            continue
        ev[ax] = {"report_alpha": ident.alpha, "csv_alpha": got["alpha"],
                  "report_lag_ms": ident.lag_ms, "csv_lag_ms": got["lag_ms"]}
        # the report rounds to 3 decimals / whole ms
        if (abs(got["alpha"] - ident.alpha) > 0.005
                or abs(got["lag_ms"] - ident.lag_ms) > 2.0):
            diffs[ax] = ev[ax]
    if diffs:
        return [Finding(
            "CSV_REPORT_MISMATCH", FAIL,
            "re-identification of the session CSV contradicts the report",
            evidence={"diffs": diffs},
            source=[str(s.csv.path), str(s.report.path)],
            explanation="the report does not describe this data: wrong file "
                        "matched, or a different geo_tuner produced it")]
    return [Finding("CSV_REPORT_MISMATCH", PASS,
                    "session CSV reproduces the report's identification",
                    evidence=ev, source=[str(s.csv.path), str(s.report.path)])]


def check_d1_per_round(s: Session, ctx: Context) -> list[Finding]:
    if s.csv is None:
        return [skip("ROUNDS_DISAGREE", "per-round identification", "no session CSV")]
    if _full_identify(s, ctx) is None:
        return [skip("ROUNDS_DISAGREE", "per-round identification",
                     getattr(ctx, "_identify_error", "identify unavailable"))]
    per_round: dict[str, list] = {}
    for seg in s.csv.segments:
        try:
            doc = run_identify(s.csv.path, segments=(seg, seg))
        except IdentifyUnavailable:
            continue
        for ax, r in doc["axes"].items():
            if r["ok"]:
                per_round.setdefault(ax, []).append(
                    {"seg": seg, "alpha": r["alpha"], "ci": r["alpha_ci"]})
    out = []
    for ax, rounds in per_round.items():
        if len(rounds) < 2:
            continue
        disjoint = []
        for i in range(len(rounds)):
            for j in range(i + 1, len(rounds)):
                a, b = rounds[i], rounds[j]
                if a["ci"][1] < b["ci"][0] or b["ci"][1] < a["ci"][0]:
                    disjoint.append((a["seg"], b["seg"]))
        ev = {"alpha_by_round": {str(r["seg"]): r["alpha"] for r in rounds},
              "ci_by_round": {str(r["seg"]): r["ci"] for r in rounds}}
        if disjoint:
            out.append(Finding(
                "ROUNDS_DISAGREE", WARN,
                f"{ax}: rounds identify different plants",
                axis=ax, evidence={**ev, "disjoint_pairs": disjoint},
                source=[str(s.csv.path)],
                explanation="plant non-stationary within the session (wind, "
                            "battery, hardware); the pooled interval is "
                            "not honest",
                recommendation_ids=["REC_CHECK_HARDWARE"]))
        else:
            out.append(Finding(
                "ROUNDS_AGREE", PASS,
                f"{ax}: independent rounds agree (intervals honest)",
                axis=ax, evidence=ev, source=[str(s.csv.path)]))
    if not out:
        return [skip("ROUNDS_DISAGREE", "per-round identification",
                     "no axis identified in more than one round")]
    return out


def _models_from_identify(doc: dict) -> dict:
    models = {}
    for ax, r in doc["axes"].items():
        if r["ok"]:
            models[ax] = (r["alpha"], r["delay_ms"] / 1e3, r["tau_ms"] / 1e3)
    return models


def check_d2_model_vs_flown(s: Session, ctx: Context) -> list[Finding]:
    if s.bag is None or s.report is None:
        return [skip("MODEL_MISMATCH", "model vs flown steps",
                     "needs the bag and the report")]
    doc = _full_identify(s, ctx)
    models = _models_from_identify(doc) if doc else None
    # ensembles are built even without identify: D3's before/after A/B and
    # the ensembles plot need no model, only the flown responses
    groups = ensembles.build_ensembles(s.bag, s.report, models)
    ctx._ensembles = groups  # D3 and the plots reuse them
    if doc is None:
        return [skip("MODEL_MISMATCH", "model vs flown steps",
                     getattr(ctx, "_identify_error", "identify unavailable"))]
    out = []
    for g in groups:
        if g.rms is None:
            continue
        ev = {"kx": g.kx, "kv": g.kv, "n_steps": g.n, "rms": round(g.rms, 3),
              "measured": {k: round(v, 2) for k, v in g.measured_metrics.items()},
              "model": {k: round(v, 2) for k, v in g.model_metrics.items()}}
        if g.rms > D2_RMS_WARN:
            out.append(Finding(
                "MODEL_MISMATCH", WARN,
                f"{g.axis}: identified model does not describe the flown response",
                axis=g.axis, evidence=ev, source=["bag odom/sp + geo-tuner-identify"],
                explanation="look for saturation, pauses, wind before "
                            "trusting this session's update",
                recommendation_ids=["REC_KEEP_ENTRY_GAINS"]))
        else:
            out.append(Finding(
                "MODEL_MATCHES_FLOWN", PASS,
                f"{g.axis}: ensemble of {g.n} steps matches the model "
                f"(rms {g.rms:.3f})",
                axis=g.axis, evidence=ev,
                source=["bag odom/sp + geo-tuner-identify"]))
    if not out:
        return [skip("MODEL_MISMATCH", "model vs flown steps",
                     "no scoreable step ensembles")]
    return out


def check_d3_ab_change(s: Session, ctx: Context) -> list[Finding]:
    """When gains changed mid-session, compare the flown response before and
    after on the same axis. 09-14 z kv 2.505 -> 3.152: overshoot 1.5 ->
    -0.3 %, rise 1.38 -> 1.78 s, hover z std 7.5 -> 6.4 cm, vz std -23 %."""
    groups = getattr(ctx, "_ensembles", None)
    if not groups:
        return []
    by_axis: dict[str, list] = {}
    for g in groups:
        by_axis.setdefault(g.axis, []).append(g)
    out = []
    for ax, gs in by_axis.items():
        if len(gs) < 2:
            continue
        gs.sort(key=lambda g: min(g.t0s))
        before, after = gs[0], gs[-1]
        # the split instant is when the conductor applied the new gains (the
        # log's "Got gains.*" line), not a midpoint: the hover between the
        # rounds flies on the NEW gains and belongs to "after"
        change_t = (max(before.t0s) + min(after.t0s)) / 2.0
        if s.log:
            for e in s.log.all("gain_applied"):
                if (e.fields["param"].endswith("." + ax)
                        and max(before.t0s) < e.epoch < min(after.t0s)):
                    change_t = e.epoch
                    break
        ev = {
            "gains": {"before": [before.kx, before.kv],
                      "after": [after.kx, after.kv]},
            "overshoot_pct": [round(100 * before.measured_metrics["overshoot"], 1),
                              round(100 * after.measured_metrics["overshoot"], 1)],
            "rise_s": [round(before.measured_metrics["rise_s"], 2),
                       round(after.measured_metrics["rise_s"], 2)],
            "iae3": [round(before.measured_metrics["iae3"], 2),
                     round(after.measured_metrics["iae3"], 2)],
        }
        i = ensembles.AXIS_COL[ax]
        w = ctx.window
        # 3.0 s post-jump exclusion, the prototypes/extra.py method that
        # produced the 09-14 reference numbers (7.5 -> 6.4 cm on z)
        hn_before = signals.hover_noise(
            s.bag, exclude_s=3.0,
            t_range=(0.0, change_t)) if s.bag else None
        hn_after = signals.hover_noise(
            s.bag, exclude_s=3.0,
            t_range=(change_t, w.t1 if w else np.inf)) if s.bag else None
        if hn_before and hn_after:
            ev["hover_std_cm"] = [round(float(hn_before["std_m"][i]) * 100, 1),
                                  round(float(hn_after["std_m"][i]) * 100, 1)]
            v0, v1 = float(hn_before["vel_std"][i]), float(hn_after["vel_std"][i])
            ev["vel_std_change_pct"] = round(100 * (v1 - v0) / v0, 0)
        out.append(Finding(
            "AB_GAIN_CHANGE", INFO,
            f"{ax}: in-flight A/B of the applied gain change",
            axis=ax, evidence=ev, source=["bag odom/sp ensembles"],
            explanation="measured effect of the update the session applied, "
                        "on the same air"))
    return out


def check_d4_design_margins(s: Session, ctx: Context) -> list[Finding]:
    doc = _full_identify(s, ctx)
    if doc is None:
        return [skip("MARGINS_THIN", "design sanity",
                     getattr(ctx, "_identify_error", "identify unavailable"))]
    ev, thin = {}, {}
    for ax, r in doc["axes"].items():
        if not r["ok"]:
            continue
        ev[ax] = {"pm_nominal_deg": r["pm_now_nominal_deg"],
                  "pm_worst_deg": r["pm_now_worst_deg"]}
        # same materiality as the conductor's own accept logic: gains within
        # pm_hysteresis (5 deg) of the spec stand (new_1m x: 44.6 vs 45)
        if (r["pm_now_nominal_deg"] < 45.0 - 5.0
                or r["pm_now_worst_deg"] < 35.0 - 5.0):
            thin[ax] = ev[ax]
    if thin:
        return [Finding("MARGINS_THIN", WARN,
                        "final gains sit below the phase-margin spec",
                        evidence={"thin": thin, "spec_deg": [45.0, 35.0]},
                        source=["geo-tuner-identify on the session CSV"])]
    if not ev:
        return [skip("MARGINS_THIN", "design sanity", "no identified axis")]
    return [Finding("MARGINS_HEALTHY", PASS,
                    "final gains keep the phase-margin spec on the nominal "
                    "and worst interval plant",
                    evidence=ev, source=["geo-tuner-identify on the session CSV"])]


def check_e1_z_plant(s: Session, ctx: Context) -> list[Finding]:
    if s.bag is None or ctx.window is None:
        return [skip("Z_PLANT_SOFT", "vertical plant vs IMU",
                     "needs the bag and a session window")]
    mass_e = s.log.first("mass") if s.log else None
    thr_e = s.log.first("max_thrust") if s.log else None
    if mass_e is None or thr_e is None:
        return [skip("Z_PLANT_SOFT", "vertical plant vs IMU",
                     "mass/max_thrust not in the log")]
    try:
        res = imu_analysis.z_plant_cross_check(
            s.bag, ctx.window, mass_e.fields["mass_kg"], thr_e.fields["max_thrust_n"])
    except ImportError:
        return [skip("Z_PLANT_SOFT", "vertical plant vs IMU", "scipy unavailable")]
    if res is None:
        return [skip("Z_PLANT_SOFT", "vertical plant vs IMU",
                     "IMU/thrust streams missing or too little excitation")]
    doc = _full_identify(s, ctx)
    if doc and doc["axes"].get("z", {}).get("ok"):
        res["identified_2sls_alpha"] = doc["axes"]["z"]["alpha"]
    slope = res.get("imu_accel_per_thrust")
    linear = res["linear_map_accel_per_thrust"]
    soft = slope is not None and slope < 0.85 * linear
    return [Finding(
        "Z_PLANT_SOFT" if soft else "Z_PLANT_CONSISTENT",
        INFO,
        "thrust curve flatter than the linear map at hover" if soft
        else "vertical plant consistent with the thrust map",
        axis="z", evidence=res,
        source=["bag imu/att_thrust/odom (OLS cross-check, biased under "
                "feedback; 2SLS is the plant)"],
        explanation=("the softness is the vehicle, not EKF lag; the tuner "
                     "compensates via alpha" if soft else
                     "OLS cross-checks bracket the 2SLS estimate"))]


def check_e2_lag(s: Session, ctx: Context) -> list[Finding]:
    doc = _full_identify(s, ctx)
    if doc is None:
        return [skip("LAG_PER_AXIS", "lag per axis",
                     getattr(ctx, "_identify_error", "identify unavailable"))]
    ev = {ax: {"lag_ms": r["lag_ms"], "lag_ci_ms": r["lag_ci_ms"]}
          for ax, r in doc["axes"].items() if r["ok"]}
    if not ev:
        return [skip("LAG_PER_AXIS", "lag per axis", "no identified axis")]
    # Trend comparison against the vehicle's history arrives with phase 4
    # (--history); until then the numbers are recorded for the diff.
    return [Finding("LAG_PER_AXIS", INFO, "loop lag per axis",
                    evidence=ev, source=["geo-tuner-identify on the session CSV"],
                    explanation="09-14 vehicle history: x 126-129, y 124-128, "
                                "z 54-60 ms; a jump means props/ESC/attitude-"
                                "loop change")]


def check_e3_battery(s: Session, ctx: Context) -> list[Finding]:
    if s.bag is None or s.bag.arrays.get("bat") is None:
        return [skip("BATTERY", "battery", "no battery stream")]
    # whole recording, not the tuning window: the interesting number is what
    # the pack sagged from fresh (09-14: 16.24 -> 14.99 V)
    bat = s.bag.arrays["bat"]
    ev = {"start_v": round(float(bat[0, 1]), 2),
          "end_v": round(float(bat[-1, 1]), 2),
          "min_v": round(float(bat[:, 1].min()), 2)}
    return [Finding("BATTERY", INFO, "battery over the session",
                    evidence=ev, source=["bag mavros/battery"])]


def check_e4_vibration(s: Session, ctx: Context) -> list[Finding]:
    if s.bag is None or s.bag.arrays.get("imu_raw") is None:
        return []
    raw = s.bag.arrays["imu_raw"]
    t = raw[:, 0]
    ok = np.ones(len(t), bool)
    if ctx.window is not None:
        ok &= (t >= ctx.window.t0) & (t <= ctx.window.t1)
    for j in signals.sp_jumps(s.bag):
        ok &= ~((t >= j) & (t < j + 4.0))
    if ok.sum() < 200:
        return []
    acc = raw[ok, 5:8]
    rms = float(np.sqrt(np.mean((acc - acc.mean(axis=0)) ** 2)))
    return [Finding("VIBRATION", INFO, "hover vibration (raw IMU accel rms)",
                    evidence={"rms_m_s2": round(rms, 2),
                              "n_samples": int(ok.sum())},
                    source=["bag mavros/imu/data_raw"])]


DE_CHECKS = [
    check_a6_csv_vs_report, check_d1_per_round, check_d2_model_vs_flown,
    check_d3_ab_change, check_d4_design_margins,
    check_e1_z_plant, check_e2_lag, check_e3_battery, check_e4_vibration,
]
