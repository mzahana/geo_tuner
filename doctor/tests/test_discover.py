"""Discovery tests: artifact matching is by time window across the two
clocks (report stamps local, bag/log names UTC), never by name equality."""
import pytest

from geo_tuner_doctor.discover import discover, find_reports
from geo_tuner_doctor.session import load_session


def test_latest_field_session_is_matched(field_logs):
    sessions = discover(field_logs, latest=True)
    assert len(sessions) == 1
    art = sessions[0]
    if art.stamp != "2026-09-14_08-58-27":
        pytest.skip("logs tree has sessions newer than 09-14")
    # The acceptance triple: report 08-58-27 local <-> log 055810 UTC <-> bag
    # 055811 UTC, matched across a 3-hour clock offset.
    assert art.log is not None and art.log.name == "tuner_20260914-055810.log"
    assert art.bag is not None and art.bag.name == "tuner_20260914-055811"
    assert art.csv is not None and art.csv.name.endswith("2026-09-14_08-58-27.csv")
    assert art.episodes_dir is not None
    assert art.missing == []
    assert any(b.name.endswith("20260914-090515.bak") for b in art.override_baks)


def test_all_field_sessions_load(field_logs):
    """Phase-1 acceptance: every field session loads without error (bags are
    exercised separately; reading them needs a sourced ROS environment)."""
    sessions = discover(field_logs)
    assert len(sessions) >= 4
    for art in sessions:
        s = load_session(art, read_bag=False)
        assert s.report is not None, f"{art.name}: {s.load_errors}"
        errors = {k: v for k, v in s.load_errors.items() if k != "bag"}
        assert not errors, f"{art.name}: {errors}"


def test_field_sessions_get_distinct_logs_and_bags(field_logs):
    """Ten tuner logs share 09-10; window matching must not hand one log to
    two sessions."""
    sessions = discover(field_logs)
    logs = [a.log for a in sessions if a.log]
    bags = [a.bag for a in sessions if a.bag]
    assert len(logs) == len(set(logs))
    assert len(bags) == len(set(bags))


def test_2026_09_09_report_has_no_matches(field_logs):
    """The 09-09 report predates session recording and stamped names: date
    only, no duration -- discovery must degrade to 'missing', not misattach
    another day's artifacts."""
    sessions = {a.report.name: a for a in discover(field_logs)}
    art = sessions.get("geo_tuner_report_2026-09-09.yaml")
    if art is None:
        pytest.skip("09-09 report not in this tree")
    assert art.stamp is None
    assert art.csv is None and art.episodes_dir is None
    assert art.log is None and art.bag is None
    assert "launch log" in art.missing and "bag" in art.missing


@pytest.mark.parametrize("name", [
    "new_1m", "new_05m", "old_1m", "old_05m", "g2_new_1m", "g2_new_05m",
])
def test_sim_sessions_load_flat(sim_sessions, name):
    sessions = discover(sim_sessions / name)
    assert len(sessions) == 1
    art = sessions[0]
    assert art.layout == "flat"
    assert art.report_complete is not None
    s = load_session(art)
    assert s.report is not None and s.report_complete is not None
    assert s.csv is not None and s.log is not None
    assert not s.load_errors, s.load_errors


def test_find_reports_ordering(field_logs):
    reports = find_reports(field_logs)
    assert reports == sorted(reports)
