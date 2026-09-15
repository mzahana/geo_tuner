"""Test fixtures.

Field artifacts (bags 70-105 MB, reports with this vehicle's tuned gains)
stay OUT of the repo -- gains live on the vehicle, and the logs tree lives in
~/src/ihunter_logs (host) or the shared volume (container). Tests that need
them locate the tree via GEO_TUNER_DOCTOR_FIELD_LOGS and skip when absent;
the committed fixtures are synthetic.
"""
import os
import sys
from pathlib import Path

import pytest

# Make the package importable without installation.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

FIXTURES = Path(__file__).resolve().parent / "fixtures"

_FIELD_CANDIDATES = [
    os.environ.get("GEO_TUNER_DOCTOR_FIELD_LOGS"),
    "~/src/ihunter_logs",
    "~/shared_volume/ihunter_logs",
]
_SIM_CANDIDATES = [
    os.environ.get("GEO_TUNER_DOCTOR_SIM_SESSIONS"),
    "~/src/ihunter_fixes/docs/tuner_doctor/sim_sessions",
    "~/shared_volume/tuner_doctor/sim_sessions",
]


def _first_dir(cands):
    for c in cands:
        if not c:
            continue
        p = Path(c).expanduser()
        if p.is_dir():
            return p
    return None


@pytest.fixture(scope="session")
def field_logs() -> Path:
    p = _first_dir(_FIELD_CANDIDATES)
    if p is None:
        pytest.skip("field logs tree not present (set GEO_TUNER_DOCTOR_FIELD_LOGS)")
    return p


@pytest.fixture(scope="session")
def sim_sessions() -> Path:
    p = _first_dir(_SIM_CANDIDATES)
    if p is None:
        pytest.skip("sim_sessions fixtures not present "
                    "(set GEO_TUNER_DOCTOR_SIM_SESSIONS)")
    return p
