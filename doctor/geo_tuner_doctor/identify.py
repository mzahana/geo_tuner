"""Run the C++ geo-tuner-identify on a session CSV and parse its --json.

The identification is 2SLS with the nominal command as instrument; plain OLS
under feedback is biased (low alpha, short lag in wind). Numbers the doctor
states as the plant therefore come from this binary -- the same core the
flight ran -- never from a Python re-implementation. If the binary is
absent, the dependent checks skip with INFO; they never fail over it.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path


class IdentifyUnavailable(RuntimeError):
    pass


def find_binary() -> str | None:
    env = os.environ.get("GEO_TUNER_IDENTIFY_BIN")
    if env and Path(env).is_file():
        return env
    path = shutil.which("geo-tuner-identify")
    if path:
        return path
    # A sourced ROS 2 environment does not put lib/<pkg>/ on PATH; resolve
    # through the ament prefix the way `ros2 run` would.
    prefix = os.environ.get("AMENT_PREFIX_PATH", "")
    for p in prefix.split(os.pathsep):
        cand = Path(p) / "lib" / "geo_tuner" / "geo-tuner-identify"
        if cand.is_file():
            return str(cand)
    return None


def run_identify(csv_path: Path, segments: tuple[int, int] | None = None,
                 extra: list[str] | None = None) -> dict:
    """--json result as a dict; raises IdentifyUnavailable when the binary
    is missing or refuses the file entirely."""
    binary = find_binary()
    if binary is None:
        raise IdentifyUnavailable(
            "geo-tuner-identify not found (source the workspace or set "
            "GEO_TUNER_IDENTIFY_BIN)")
    cmd = [binary, str(csv_path), "--json"]
    if segments is not None:
        cmd += ["--segments", f"{segments[0]}-{segments[1]}"]
    cmd += extra or []
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    if proc.returncode != 0:
        raise IdentifyUnavailable(
            f"geo-tuner-identify failed ({proc.returncode}): {proc.stderr.strip()}")
    try:
        return json.loads(proc.stdout)
    except json.JSONDecodeError as e:
        raise IdentifyUnavailable(f"unparsable --json output: {e}") from e
