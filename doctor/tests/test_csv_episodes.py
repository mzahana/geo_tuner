"""Session CSV and episode-dump loaders on the real 09-14 artifacts."""
import pytest

from geo_tuner_doctor.episodes import load_episode_dir
from geo_tuner_doctor.session_csv import load_session_csv


def test_field_session_csv(field_logs):
    path = (field_logs / "mav_controllers_config"
            / "geo_tuner_session_2026-09-14_08-58-27.csv")
    if not path.is_file():
        pytest.skip(f"{path} not present")
    c = load_session_csv(path)
    assert c.n > 10000  # ~5 min at 50 Hz
    assert c.rate_hz == pytest.approx(50.0, abs=1.0)
    # seg increments per recording gap, in practice one per round.
    assert len(c.segments) >= 3
    assert c.r.shape == (c.n, 3) and c.u.shape == (c.n, 3)
    # Gains-in-force columns are what geo-tuner-identify keys on; they must
    # show the mid-session z update (kvz changes value once).
    import numpy as np
    assert len(np.unique(np.round(c.kv[:, 2], 3))) >= 2


def test_field_episode_dumps(field_logs):
    d = field_logs / "mav_controllers_config" / "episodes_2026-09-14_08-58-27"
    if not d.is_dir():
        pytest.skip(f"{d} not present")
    eps = load_episode_dir(d)
    assert len(eps) == 40  # 32 position + 8 yaw steps on 09-14
    e0 = eps[0]
    assert e0.axis == "z" and e0.round_ == 0 and e0.rep == 0
    assert len(e0.t) > 10
    assert abs(e0.step) == pytest.approx(1.0)


def test_legacy_episode_header(field_logs):
    """09-10 dumps say rung= where 09-14 says round=; both must parse."""
    d = field_logs / "mav_controllers_config" / "episodes_2026-09-10_12-19-30"
    if not d.is_dir():
        pytest.skip(f"{d} not present")
    eps = load_episode_dir(d)
    assert eps and eps[0].round_ == 0


@pytest.mark.parametrize("name", [
    "new_1m", "new_05m", "old_1m", "old_05m", "g2_new_1m", "g2_new_05m",
])
def test_sim_session_csvs(sim_sessions, name):
    csvs = sorted((sim_sessions / name).glob("geo_tuner_session_*.csv"))
    assert csvs
    c = load_session_csv(csvs[-1])
    assert c.n > 1000
    assert c.rate_hz == pytest.approx(50.0, abs=2.0)
