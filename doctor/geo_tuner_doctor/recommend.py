"""Findings -> ordered, deduplicated recommendations (PLAN.md section 6).

Every recommendation names the finding that triggered it. None may ever say
"commit these gains to a repository": vehicle gains live on the vehicle.
"""
from __future__ import annotations

from .findings import FAIL, PASS, SKIP, WARN, Finding

RECOMMENDATIONS = {
    "REC_IHUNTER_STOP":
        "Run `ihunter stop` before cutting power so the recorder closes the "
        "bag; an unclosed bag is repaired by `ihunter fetch` (reindex).",
    "REC_TRUST_LOG":
        "Trust the launch-log values over this report's accept-time fields; "
        "deploy geo_tuner >= 9a7b429 so the report survives accept.",
    "REC_ESTIMATOR_PIN":
        "Do not fly this session's gains: the thrust estimator was not "
        "verified off. Fix the launch session pin, then re-tune.",
    "REC_HOVER_TEST":
        "Re-measure max_thrust with a hover test (`geo-tuner-hover`), update "
        "geometric_mavros on the vehicle, then re-tune.",
    "REC_SMALLER_STEP":
        "Use a smaller step, or accept knowingly: the controller clamp bites "
        "before identification sees it, PX4/motor saturation does not.",
    "REC_GUSTY_EXPECTED":
        "Settle timeouts in gusty air are expected; with geo_tuner >= "
        "9a7b429 they cost at most 1.5 s per step.",
    "REC_KEEP_ENTRY_GAINS":
        "Keep the entry gains; re-fly `tuner` in calmer air and inspect the "
        "model-vs-flown comparison (D2) before trusting an update.",
    "REC_TUNER_CONFIRM":
        "Fly `tuner_confirm` to close the interval, or use a larger step if "
        "the headroom check allows (interval width scales with 1/step).",
    "REC_CHECK_HARDWARE":
        "Check hardware (props, ESC, attitude loop, battery) before "
        "re-tuning: independent rounds identified different plants.",
    "REC_ACCEPT_NEXT_TIME":
        "If these results beat the current override, accept + save in the "
        "next session; never hand-edit the override files.",
}


def collect(findings: list[Finding]) -> list[dict]:
    """Ordered by the worst severity that triggered each recommendation."""
    sev_rank = {FAIL: 0, WARN: 1}
    by_rec: dict[str, dict] = {}
    for f in findings:
        for rid in f.recommendation_ids:
            if rid not in RECOMMENDATIONS:
                continue
            entry = by_rec.setdefault(rid, {
                "id": rid, "text": RECOMMENDATIONS[rid],
                "triggered_by": [], "rank": 2})
            entry["triggered_by"].append(f.id)
            entry["rank"] = min(entry["rank"], sev_rank.get(f.severity, 2))
    out = sorted(by_rec.values(), key=lambda e: (e["rank"], e["id"]))
    for e in out:
        del e["rank"]
    if not any(f.severity in (FAIL, WARN) for f in findings):
        ready = {
            "id": "REC_ALL_PASS",
            "text": "Gains are usable for navigation tests; note the "
                    "per-axis confidence in the axis table above.",
            "triggered_by": [f.id for f in findings
                             if f.severity not in (PASS, SKIP)][:4] or ["ALL_PASS"],
        }
        out.append(ready)
    return out
