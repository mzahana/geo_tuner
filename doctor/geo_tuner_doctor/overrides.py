"""Read gain_saver's override yamls (and their .bak history).

Layout: `<ns>/geometric_controller_node: {ros__parameters: {gains: {pos,
vel}}}`; gain_saver moves the previous file to `.<stamp>.bak` on save, so a
session's .bak holds the gains the session ENTERED with -- that is what B2
compares the log baselines against. Read-only, always: gains stay on the
vehicle.
"""
from __future__ import annotations

from pathlib import Path

import yaml


def _params(doc: dict, node_suffix: str) -> dict | None:
    if not isinstance(doc, dict):
        return None
    for key, val in doc.items():
        if key.split("/")[-1] == node_suffix and isinstance(val, dict):
            p = val.get("ros__parameters")
            if isinstance(p, dict):
                return p
    return None


def controller_gains(path: Path) -> dict | None:
    """{axis: {"kx": .., "kv": ..}} from a controller override, or None."""
    try:
        doc = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError):
        return None
    p = _params(doc, "geometric_controller_node")
    if p is None:
        return None
    gains = p.get("gains") or {}
    pos, vel = gains.get("pos") or {}, gains.get("vel") or {}
    out = {}
    for ax in ("x", "y", "z"):
        if ax in pos or ax in vel:
            out[ax] = {"kx": pos.get(ax), "kv": vel.get(ax)}
    return out or None


def mavros_params(path: Path) -> dict | None:
    """mass / max_thrust / enable_thrust_estimator from a mavros override."""
    try:
        doc = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError):
        return None
    p = _params(doc, "geometric_mavros_node")
    if p is None:
        return None
    return {k: p.get(k) for k in ("mass", "max_thrust", "enable_thrust_estimator")}
