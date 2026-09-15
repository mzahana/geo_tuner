"""Report loader tests over all three schema flavours found in the field
and sim fixtures: legacy (09-10), principled written pre-9a7b429 at accept
(the 09-14 field report), and principled at 9a7b429 (sim_sessions)."""
import pytest

from geo_tuner_doctor.report import (
    GEN_LEGACY,
    GEN_PRINCIPLED,
    GEN_PRINCIPLED_V2,
    load_report,
)


def test_field_principled_report_2026_09_14(field_logs):
    path = field_logs / "mav_controllers_config" / "geo_tuner_report_2026-09-14_08-58-27.yaml"
    if not path.is_file():
        pytest.skip(f"{path} not present")
    r = load_report(path)

    # Written at completion by a pre-9a7b429 conductor: principled schema,
    # but no accepted_at/u_peak/yaw_result (that gap is check A5's subject).
    assert r.generation == GEN_PRINCIPLED
    assert r.status == "accepted"
    assert r.rounds_flown == 3
    assert r.session_duration_s == pytest.approx(357.6)
    assert r.accepted_at is None
    assert r.accel_trim == (0.0, 0.0, 0.0)  # the stale value the log corrects
    assert r.session_recording is not None

    # 3 rounds: z,x,y then z,x,y then z,x = 8 identifications.
    assert len(r.identifications) == 8
    first_z = r.identifications_for("z")[0]
    assert first_z.alpha == pytest.approx(0.843)
    assert first_z.alpha_ci == pytest.approx((0.737, 0.964))
    assert first_z.lag_ms == 54.0
    assert first_z.verdict == "update"
    assert first_z.kv_in_force == pytest.approx(2.505)

    # z: 4 steps x 3 rounds; x same; y flew 2 rounds.
    per_axis = {}
    for e in r.step_episodes:
        per_axis[e.axis] = per_axis.get(e.axis, 0) + 1
    assert per_axis["z"] == 12 and per_axis["x"] == 12 and per_axis["y"] == 8

    assert len(r.yaw_rounds) == 2
    assert r.yaw_rounds[0].spread == pytest.approx(1.454)

    assert set(r.final_gains) == {"x", "y", "z"}
    assert r.final_gains["z"]["validated"] is True
    assert r.final_gains["x"]["validated"] is False
    ci = r.last_identification["x"].ci_ratio
    assert ci == pytest.approx(1.20, abs=0.01)  # the C4 knife-edge number


def test_field_legacy_report_2026_09_10(field_logs):
    path = field_logs / "mav_controllers_config" / "geo_tuner_report_2026-09-10_12-19-30.yaml"
    if not path.is_file():
        pytest.skip(f"{path} not present")
    r = load_report(path)
    assert r.generation == GEN_LEGACY
    assert r.status == "accepted"
    assert "max_thrust" in r.diagnosis
    assert r.step_episodes, "legacy rung episodes must parse as step episodes"
    e0 = r.step_episodes[0]
    assert e0.axis == "z" and e0.round_ == 0
    assert e0.alpha is not None and e0.tau_lag is not None  # per-episode fits
    assert not r.identifications  # no per-round identification pre-e0c1237


def test_sim_report_9a7b429(sim_sessions):
    r = load_report(sim_sessions / "new_1m" / "report.yaml")
    assert r.generation == GEN_PRINCIPLED_V2
    assert r.status == "accepted"
    assert r.accepted_at is not None
    assert r.yaw_result == "updated_validated"
    assert any(e.u_peak is not None for e in r.step_episodes)
    assert r.accel_trim is not None and r.accel_trim != (0.0, 0.0, 0.0)


@pytest.mark.parametrize("name", [
    "new_1m", "new_05m", "old_1m", "old_05m", "g2_new_1m", "g2_new_05m",
])
def test_all_sim_reports_load(sim_sessions, name):
    for fname in ("report.yaml", "report_complete.yaml"):
        r = load_report(sim_sessions / name / fname)
        assert r.status
        assert r.step_episodes


def test_all_field_reports_load(field_logs):
    cfg = field_logs / "mav_controllers_config"
    reports = sorted(cfg.glob("geo_tuner_report_*.yaml"))
    assert reports, "no field reports found"
    for p in reports:
        r = load_report(p)
        assert r.generation in (GEN_LEGACY, GEN_PRINCIPLED, GEN_PRINCIPLED_V2)
        assert r.time is not None
