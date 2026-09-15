"""Log parser tests: the committed synthetic log always runs; the real 09-14
field log (the session PLAN.md quotes its numbers from) runs where the
fetched logs tree is present."""
import datetime as dt

from conftest import FIXTURES

from geo_tuner_doctor.launch_log import parse_launch_log


def test_synthetic_log_events():
    log = parse_launch_log(FIXTURES / "tuner_synthetic.log")

    assert log.run_start == dt.datetime(2026, 1, 2, 5, 0, 0, tzinfo=dt.timezone.utc)
    assert log.launch_target == "tuner"

    ov = log.all("override_in_effect")
    assert len(ov) == 1
    assert ov[0].fields["path"].endswith("geometric_controller.override.yaml")

    assert log.first("mass").fields["mass_kg"] == 9.9
    assert log.first("max_thrust").fields["max_thrust_n"] == 99.9
    assert log.first("estimator_off_config") is not None
    assert log.first("estimator_off_verified") is not None
    assert log.first("bag_opened").fields["db_path"].endswith("_0.db3")
    assert log.first("recording_started") is not None
    assert log.first("recording_stopped") is None  # synthetic session has no stop

    up = log.first("conductor_up")
    assert up.fields["axes"] == ["z", "x", "y", "yaw"]
    assert up.fields["rounds_max"] == 3
    assert up.fields["steps_per_axis"] == 4

    base = {e.fields["axis"]: e.fields for e in log.all("baseline_gains")}
    assert set(base) == {"x", "y", "z"}
    assert base["z"]["kx"] == 1.5 and base["z"]["kv"] == 1.6
    assert log.first("baseline_yaw").fields["tau"] == 0.9

    changes = {e.fields["name"]: e.fields["value"] for e in log.all("param_change")}
    assert changes == {"min_tuning_altitude": 3.5, "step_size": 1.0}

    eng = log.first("offboard_engaged")
    assert eng.fields["round"] == 1 and eng.fields["axis"] == "z"
    settle = log.first("settling")
    assert settle.fields["cap_s"] == 4.0 and settle.fields["axis"] == "z"

    steps = log.all("step")
    assert [e.fields["size"] for e in steps] == [1.0, -1.0]
    assert log.all("yaw_step")[0].fields["axis"] == "yaw"
    flown = log.all("step_flown")
    assert flown[0].fields["overshoot_pct"] == 6.0
    yaw_fit = log.first("yaw_fit")
    assert yaw_fit.fields["T_s"] == 0.45 and yaw_fit.fields["delay_ms"] == 252.0

    trim = log.first("trim_update")
    assert trim.fields["trim"] == (0.1, 0.0, -0.13)
    assert trim.fields["k"] == 1 and trim.fields["n"] == 8

    ident = log.first("identification_axis")
    assert ident.fields["axis"] == "z"
    assert ident.fields["alpha"] == 0.8
    assert (ident.fields["alpha_lo"], ident.fields["alpha_hi"]) == (0.7, 0.9)
    assert ident.fields["lag_ms"] == 50.0
    assert ident.fields["decision"].startswith("gains outside the supported range")

    gains = {e.fields["param"]: e.fields["value"] for e in log.all("gain_applied")}
    assert gains == {"gains.pos.z": 1.5, "gains.vel.z": 1.9}

    assert log.first("round_start").fields["round"] == 2
    assert log.first("gains_validated").fields["axis"] == "z"
    assert log.first("yaw_confirmed") is not None
    assert log.first("tuning_complete").fields["report_path"].endswith(
        "geo_tuner_report_2026-01-02_08-00-00.yaml")
    assert log.first("accepted").fields["report_path"].endswith(".yaml")
    saved = log.first("gains_saved")
    assert saved.fields["controller_override"].endswith(
        "geometric_controller.override.yaml")

    # Epochs must be carried: the report/log cross-checks live off them.
    assert log.first("accepted").epoch == 1767330060.0


def test_field_log_2026_09_14(field_logs):
    """The real 09-14 log: the counts PLAN.md's check catalogue quotes."""
    path = field_logs / "logs" / "tuner_20260914-055810.log"
    if not path.is_file():
        import pytest
        pytest.skip(f"{path} not present")
    log = parse_launch_log(path)

    assert log.run_start == dt.datetime(2026, 9, 14, 5, 58, 10, tzinfo=dt.timezone.utc)
    # 40 steps flown: z 12, x 12, y 8 position steps + 8 yaw steps (C1's 27/40).
    assert len(log.all("step")) == 32
    assert len(log.all("yaw_step")) == 8
    assert len(log.all("step_flown")) == 32
    assert len(log.all("yaw_fit")) == 8
    # 8 identification lines over 3 rounds (z,x,y / z,x,y / z,x).
    idents = log.all("identification_axis")
    assert len(idents) == 8
    assert [e.fields["axis"] for e in idents[:3]] == ["z", "x", "y"]
    assert idents[0].fields["alpha"] == 0.84
    # A5: the last trim in the log is what the (pre-9a7b429) report lost.
    trims = log.all("trim_update")
    assert len(trims) == 8
    assert trims[-1].fields["trim"] == (0.2, -0.1, -0.0)
    # Session bookends.
    assert log.first("estimator_off_verified") is not None
    assert log.first("tuning_complete") is not None
    assert log.first("accepted") is not None
    assert log.first("gains_saved") is not None
    assert log.first("recording_stopped") is None  # power cut before ihunter stop
    base = {e.fields["axis"] for e in log.all("baseline_gains")}
    assert base == {"x", "y", "z"}
    assert len(log.all("override_in_effect")) == 2
    # Nothing the conductor said may be dropped: unmatched lines surface as
    # conductor_info, and the known-format lines must not land there.
    info = [e.fields["msg"] for e in log.all("conductor_info")]
    assert not any("alpha" in m or "Step " in m for m in info)
