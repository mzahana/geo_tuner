"""Phase-2 acceptance: the check catalogue reproduces the 09-14 analysis
numbers (PLAN.md section 8, phase 2), and behaves on the sim fixtures.

Bag-dependent checks (A2 via metadata, A3, B3, C2) need rosbag2_py; those
tests skip without a sourced ROS 2 environment.
"""
import shutil

import pytest

from geo_tuner_doctor import bag as bag_mod
from geo_tuner_doctor.checks import build_context, check_a2_bag_closed, run_checks
from geo_tuner_doctor.discover import discover
from geo_tuner_doctor.findings import INFO, PASS, WARN, banner
from geo_tuner_doctor.recommend import collect
from geo_tuner_doctor.render import render_json, render_markdown
from geo_tuner_doctor.session import Session, load_session


def _by_id(findings):
    out = {}
    for f in findings:
        out.setdefault(f.id, []).append(f)
    return out


@pytest.fixture(scope="module")
def field_0914(field_logs, tmp_path_factory):
    pytest.importorskip("rosbag2_py")
    art = discover(field_logs, latest=True)[0]
    if art.stamp != "2026-09-14_08-58-27":
        pytest.skip("logs tree has sessions newer than 09-14")
    cache = tmp_path_factory.mktemp("bagcache")
    s = load_session(art, cache_dir=cache)
    if s.bag is None:
        pytest.skip(f"bag not loadable: {s.load_errors}")
    return s


def test_0914_acceptance_numbers(field_0914):
    f = _by_id(run_checks(field_0914))

    # A2: metadata repaired by fetch, but the log never says Recording stopped.
    assert f["BAG_NOT_CLOSED"][0].severity == WARN
    # A5: both numbers, trim and duration.
    a5 = f["REPORT_LOG_MISMATCH"][0]
    assert a5.severity == WARN
    assert a5.evidence["accel_trim"]["log_final"] == [0.2, -0.1, -0.0]
    assert a5.evidence["accel_trim"]["report"] == [0.0, 0.0, 0.0]
    assert abs(a5.evidence["duration_s"]["log_span"] - 313.0) < 2.0
    # B1, B3 (0.595 vs 0.597), B4 (1 m over a 0.5/0.4 profile), B5.
    assert f["ESTIMATOR_NOT_VERIFIED"][0].severity == PASS
    b3 = f["HOVER_THRUST_OFF"][0]
    assert b3.severity == PASS
    assert b3.evidence["hover_thrust_median"] == pytest.approx(0.595, abs=0.002)
    assert b3.evidence["predicted_mg_over_max_thrust"] == pytest.approx(0.597, abs=0.001)
    b4 = f["ENVELOPE_CHANGED_LIVE"][0]
    assert b4.severity == INFO
    assert b4.evidence["flown"]["lateral_m"] == 1.0
    b5 = f["NEAR_SATURATION"][0]
    assert b5.severity == PASS
    assert all(2.0 < v < 4.0 for v in b5.evidence["peak_cmd_accel"].values())
    # C1: 27/40 settles at the 4 s cap.
    c1 = f["SETTLE_CAP_BOUND"][0]
    assert c1.severity == WARN
    assert c1.evidence["settles_at_cap"] == "27/40"
    # C2: hover wander [7.5, 7.9, 7.4] cm +/- 0.3.
    c2 = f["HOVER_WANDER"][0]
    for got, want in zip(c2.evidence["hover_std_cm"], [7.5, 7.9, 7.4]):
        assert got == pytest.approx(want, abs=0.3)
    # C4: x knife edge 1.20 vs 1.19.
    c4 = f["CI_KNIFE_EDGE"][0]
    assert c4.axis == "x" and c4.evidence["ci_ratio"] == pytest.approx(1.20, abs=0.01)
    # C7: yaw confirmed, round-0 spread 1.45.
    c7 = f["YAW_IDENTIFIED"][0]
    assert c7.severity == PASS
    assert c7.evidence["spread_by_round"]["0"] == pytest.approx(1.454, abs=0.01)
    # C9: saved override equals final_gains.
    c9 = f["GAINS_SAVED"][0]
    assert c9.severity == PASS
    assert "matches" in c9.evidence["comparison"]
    # Overall banner.
    verdict, _ = banner(run_checks(field_0914))
    assert verdict == "SUCCESS WITH WARNINGS"


def test_a2_on_truncated_metadata(field_logs, tmp_path):
    """The unclosed-bag path, tested against a copy with metadata truncated
    to 0 bytes (the state the 09-14 bag was fetched in)."""
    src = field_logs / "bags" / "tuner_20260914-055811"
    if not src.is_dir():
        pytest.skip(f"{src} not present")
    dst = tmp_path / "bag"
    dst.mkdir()
    # copy metadata only: check_closed never opens the db3.
    shutil.copy(src / "metadata.yaml", dst / "metadata.yaml")
    (dst / "metadata.yaml").write_bytes(b"")

    art = discover(field_logs, latest=True)[0]
    art.bag = dst
    s = Session(artifacts=art)
    s.bag_status = bag_mod.check_closed(dst)
    out = check_a2_bag_closed(s, build_context(s))
    assert out[0].severity == WARN
    assert out[0].evidence["reason"] == "metadata.yaml is 0 bytes"


def test_sim_new_1m_post_fix_report(sim_sessions):
    """9a7b429 report at accept: no A5 finding, trim learned and non-zero,
    result fields read."""
    s = load_session(discover(sim_sessions / "new_1m")[0])
    f = _by_id(run_checks(s))
    assert "REPORT_LOG_MISMATCH" not in f
    trim = f["TRIM_LEARNED"][0]
    assert trim.severity == PASS
    assert any(abs(v) > 0.05 for v in trim.evidence["final_trim"])
    assert s.report.final_gains["x"]["result"] == "confirmed"
    assert s.report.yaw_result == "updated_validated"
    verdict, _ = banner(run_checks(s))
    assert verdict == "SUCCESS"


def test_sim_g2_new_05m_no_trim(sim_sessions):
    s = load_session(discover(sim_sessions / "g2_new_05m")[0])
    f = _by_id(run_checks(s))
    c6 = f["TRIM_NOT_LEARNED"][0]
    assert c6.severity == INFO


def test_render_outputs(sim_sessions):
    s = load_session(discover(sim_sessions / "new_1m")[0])
    findings = run_checks(s)
    recs = collect(findings)
    md = render_markdown(s, findings, recs)
    assert md.startswith("# Tuning session new_1m — SUCCESS")
    assert "| x |" in md  # per-axis table
    assert "Recommendations" in md
    js = render_json(s, findings, recs)
    import json
    doc = json.loads(js)
    assert doc["verdict"] == "SUCCESS"
    assert all("id" in f and "severity" in f for f in doc["findings"])
    # No recommendation may ever suggest committing gains to a repo.
    assert "commit" not in js.lower() or "repo" not in js.lower()


def test_all_sessions_run_checks_without_crash(field_logs, sim_sessions):
    """A crashed check surfaces as a SKIP finding, never an exception; no
    session may produce one."""
    for art in discover(field_logs):
        s = load_session(art, read_bag=False)
        for f in run_checks(s):
            assert "crashed" not in f.title, f"{art.name}: {f}"
    for d in sorted(p for p in sim_sessions.iterdir() if p.is_dir()
                    if (p / "report.yaml").is_file()):
        s = load_session(discover(d)[0])
        for f in run_checks(s):
            assert "crashed" not in f.title, f"{d.name}: {f}"


def test_legacy_sessions_read_correctly(field_logs):
    """The 09-13 legacy sessions flew with the thrust estimator ON (the
    max_thrust ~1.08 story) -- B1 must say so precisely; their yaw was
    updated by the legacy consistency method, not 'never confirmed'; and
    B2 must compare 09-13b's baselines against the NEXT save's .bak, not
    the live override 09-14 overwrote."""
    sessions = {a.stamp: a for a in discover(field_logs) if a.stamp}
    art = sessions.get("2026-09-13_10-38-19")
    if art is None:
        pytest.skip("09-13b not in this tree")
    s = load_session(art, read_bag=False)
    f = _by_id(run_checks(s))
    b1 = f["ESTIMATOR_NOT_VERIFIED"][0]
    assert b1.severity == "FAIL"
    assert "ON" in b1.title
    yaw = f["YAW_IDENTIFIED"][0]
    assert yaw.severity == PASS and "legacy" in yaw.title
    b2 = f["OVERRIDE_PROVENANCE"][0]
    assert b2.severity == PASS
    assert b2.evidence["entry_file"].endswith(".bak")
    c9 = f["COMPLETE_NOT_ACCEPTED"][0]
    assert c9.severity == INFO


def test_0914_de_acceptance_numbers(field_0914):
    """Phase-3 acceptance on 09-14: A6 PASS; D1 per-round x alphas; D2 rms
    per axis; D3 z A/B numbers; E1 thrust slope 12.6-12.7 vs 16.4."""
    from geo_tuner_doctor.identify import find_binary
    if find_binary() is None:
        pytest.skip("geo-tuner-identify not on the path")
    f = _by_id(run_checks(field_0914))

    a6 = f["CSV_REPORT_MISMATCH"][0]
    assert a6.severity == PASS

    d1 = {x.axis: x for x in f.get("ROUNDS_AGREE", [])}
    assert set(d1) == {"x", "y", "z"}
    xa = d1["x"].evidence["alpha_by_round"]
    assert xa["0"] == pytest.approx(0.959, abs=0.005)
    assert xa["1"] == pytest.approx(1.035, abs=0.005)
    assert xa["2"] == pytest.approx(0.923, abs=0.005)
    assert "ROUNDS_DISAGREE" not in f

    d2 = {(x.axis, x.evidence["kv"]): x for x in f["MODEL_MATCHES_FLOWN"]}
    assert d2[("z", 3.15)].evidence["rms"] == pytest.approx(0.009, abs=0.005)
    assert d2[("x", 2.85)].evidence["rms"] == pytest.approx(0.015, abs=0.005)
    assert d2[("y", 3.3)].evidence["rms"] == pytest.approx(0.017, abs=0.005)
    assert "MODEL_MISMATCH" not in f

    d3 = f["AB_GAIN_CHANGE"][0]
    assert d3.axis == "z"
    ev = d3.evidence
    assert ev["overshoot_pct"] == [1.5, -0.3]
    assert ev["rise_s"] == [1.38, 1.78]
    assert ev["iae3"] == [1.09, 1.13]
    assert ev["hover_std_cm"][0] == pytest.approx(7.5, abs=0.3)
    assert ev["hover_std_cm"][1] == pytest.approx(6.4, abs=0.3)
    assert ev["vel_std_change_pct"] == pytest.approx(-23, abs=3)

    assert f["MARGINS_HEALTHY"][0].severity == PASS

    e1 = f["Z_PLANT_SOFT"][0]
    assert e1.evidence["imu_accel_per_thrust"] == pytest.approx(12.6, abs=0.3)
    assert e1.evidence["linear_map_accel_per_thrust"] == pytest.approx(16.4, abs=0.1)
    assert e1.evidence["imu_ols"]["alpha"] == pytest.approx(0.77, abs=0.02)
    assert e1.evidence["velocity_ols"]["alpha"] == pytest.approx(0.82, abs=0.02)

    e3 = f["BATTERY"][0]
    assert e3.evidence["start_v"] == pytest.approx(16.24, abs=0.05)
    assert e3.evidence["end_v"] == pytest.approx(14.99, abs=0.05)


def test_plots_render(field_0914, tmp_path):
    from geo_tuner_doctor.checks import build_context
    from geo_tuner_doctor.plots import render_plots
    pytest.importorskip("matplotlib")
    ctx = build_context(field_0914)
    run_checks(field_0914, ctx)
    names = render_plots(field_0914, ctx, tmp_path / "plots")
    assert "ensembles.png" in names and "ci_ratio.png" in names
    for n in names:
        assert (tmp_path / "plots" / n).stat().st_size > 1000
