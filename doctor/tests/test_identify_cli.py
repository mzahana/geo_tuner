"""Tests for the geo-tuner-identify --json / --segments flags (phase 3).

Skip when the binary is not on the path (an unsourced environment); the
existing gtests cover the identification core itself.
"""
import pytest

from geo_tuner_doctor.identify import find_binary, run_identify
from geo_tuner_doctor.report import load_report


@pytest.fixture(scope="module")
def csv_0914(field_logs):
    if find_binary() is None:
        pytest.skip("geo-tuner-identify not on the path")
    path = (field_logs / "mav_controllers_config"
            / "geo_tuner_session_2026-09-14_08-58-27.csv")
    if not path.is_file():
        pytest.skip(f"{path} not present")
    return path


def test_json_output_shape(csv_0914):
    doc = run_identify(csv_0914)
    assert doc["n_samples"] == 15630
    assert doc["segments"] == [0, 1, 2, 3]
    for ax in "xyz":
        r = doc["axes"][ax]
        assert r["ok"] is True
        assert r["alpha_ci"][0] < r["alpha"] < r["alpha_ci"][1]
        assert r["verdict"] in ("confirmed", "inconclusive", "update",
                                "no_estimate")
        assert "design" in r and "pm_now_nominal_deg" in r


def test_full_csv_matches_report(csv_0914, field_logs):
    """A6: the offline re-identification reproduces the flight's numbers."""
    report = load_report(field_logs / "mav_controllers_config"
                         / "geo_tuner_report_2026-09-14_08-58-27.yaml")
    doc = run_identify(csv_0914)
    for ax, ident in report.last_identification.items():
        got = doc["axes"][ax]
        assert got["alpha"] == pytest.approx(ident.alpha, abs=0.005), ax
        assert got["lag_ms"] == pytest.approx(ident.lag_ms, abs=2.0), ax


def test_segments_restrict(csv_0914):
    """D1's engine: per-round x alphas 0.959 / 1.035 / 0.923 on 09-14, and
    the trailing accept-hover segment refuses (no excitation)."""
    expected = {0: 0.959, 1: 1.035, 2: 0.923}
    for seg, alpha in expected.items():
        doc = run_identify(csv_0914, segments=(seg, seg))
        assert doc["segments"] == [seg]
        x = doc["axes"]["x"]
        assert x["ok"] is True
        assert x["alpha"] == pytest.approx(alpha, abs=0.005)
    tail = run_identify(csv_0914, segments=(3, 3))
    assert tail["axes"]["x"]["ok"] is False


def test_segments_range(csv_0914):
    doc = run_identify(csv_0914, segments=(0, 1))
    assert doc["segments"] == [0, 1]
    full = run_identify(csv_0914)
    # more data narrows the interval
    assert (full["axes"]["x"]["alpha_ci"][1] - full["axes"]["x"]["alpha_ci"][0]
            <= doc["axes"]["x"]["alpha_ci"][1] - doc["axes"]["x"]["alpha_ci"][0])
