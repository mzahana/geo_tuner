"""Phase-4 acceptance: the cross-session trend over 09-09..09-14, with the
legacy bags re-identified through geo-tuner-bag-export -> geo-tuner-identify.
The offline verdict on the old bags must say what 09-14 then confirmed in
flight: z kv 2.50 -> UPDATE toward ~3.0, z lag ~60-70 ms.
"""
import pytest

from geo_tuner_doctor.history import (
    build_history,
    find_bag_export,
    gains_timeline,
    render_history_md,
)
from geo_tuner_doctor.identify import find_binary


@pytest.fixture(scope="module")
def history(field_logs, tmp_path_factory):
    if find_binary() is None or find_bag_export() is None:
        pytest.skip("identify/bag-export binaries not on the path")
    pytest.importorskip("rosbag2_py")
    cache = field_logs / "doctor" / "history_cache"
    return build_history(field_logs, cache)


def test_trend_rows(history):
    rows, _ = history
    assert len(rows) >= 5
    by_name = {r.name: r for r in rows}
    # Legacy sessions with a bag are re-identified with the principled method.
    for name in ("2026-09-10_12-19-30", "2026-09-13_09-20-00",
                 "2026-09-13_10-38-19"):
        row = by_name.get(name)
        if row is None:
            pytest.skip(f"{name} not in this tree")
        assert row.ident_source == "bag-export", name
        z = row.axes["z"]
        assert z["verdict"] == "update", name
        assert 2.8 <= z["design_kv"] <= 3.4, (name, z["design_kv"])
        assert 55 <= z["lag_ms"] <= 75, (name, z["lag_ms"])
    # The principled session identifies from its own CSV.
    r14 = by_name.get("2026-09-14_08-58-27")
    if r14 is not None:
        assert r14.ident_source == "csv"
        assert r14.axes["z"]["verdict"] == "confirmed"


def test_no_drift_on_this_vehicle(history):
    """09-10..09-14 is one airframe in one week: the alpha/lag intervals
    overlap session to session, so no PLANT_DRIFT may fire."""
    _, findings = history
    assert findings == []


def test_gains_timeline_and_render(history, field_logs):
    rows, _ = history
    timeline = gains_timeline(field_logs)
    assert timeline and timeline[-1]["until"] == "now"
    md = render_history_md(rows, timeline)
    text = "\n".join(md)
    assert "## Session history" in text
    assert "bag-export" in text
    assert "override history" in text
