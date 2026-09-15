"""Thin rosbag2 reader: tuner bag -> numpy arrays, cached as .npz.

A 09-14-sized bag (~105 MB sqlite3) takes ~10 s to scan, so the arrays are
cached next to the doctor's output and keyed on the db3 file's size+mtime.
rosbag2_py is imported lazily: every other loader (and their tests) must work
without a sourced ROS environment.

Signal conventions learned on 09-14 (see PLAN.md 'Notes learned the hard
way'): geometric_controller/odom twist is ENU *world*, mavros
local_position/odom twist is *body*; SE3Command.force/mass - g zhat is the
commanded acceleration, with mass read from the log/override, never
hard-coded.
"""
from __future__ import annotations

import dataclasses
import json
from pathlib import Path

import numpy as np
import yaml


class BagUnavailable(RuntimeError):
    """rosbag2_py or a message package is missing: skip bag checks with INFO,
    never fail the doctor over it."""


@dataclasses.dataclass
class BagStatus:
    path: Path
    closed: bool
    reason: str
    metadata: dict | None = None


def check_closed(bag_dir: Path) -> BagStatus:
    """A bag with a missing/0-byte/unparsable metadata.yaml was not shut down
    (power cut before `ihunter stop`, as on 09-14 before fetch repaired it).
    It is readable after `ros2 bag reindex -s sqlite3` -- on a COPY, never
    the original in place."""
    bag_dir = Path(bag_dir)
    meta = bag_dir / "metadata.yaml"
    if not meta.is_file():
        return BagStatus(bag_dir, False, "metadata.yaml missing")
    if meta.stat().st_size == 0:
        return BagStatus(bag_dir, False, "metadata.yaml is 0 bytes")
    try:
        doc = yaml.safe_load(meta.read_text())
    except yaml.YAMLError as e:
        return BagStatus(bag_dir, False, f"metadata.yaml does not parse: {e}")
    if not isinstance(doc, dict) or "rosbag2_bagfile_information" not in doc:
        return BagStatus(bag_dir, False, "metadata.yaml has no bagfile information")
    return BagStatus(bag_dir, True, "closed", doc["rosbag2_bagfile_information"])


# short name -> topic suffix (namespace-independent, matched by endswith).
# The union of what the 09-14 analysis needed (prototypes bag_export.py +
# bag_export_imu.py); loaders keep whatever subset the bag actually has.
TOPIC_SUFFIXES = {
    "odom": "geometric_controller/odom",
    "sp": "geometric_controller/multi_dof_setpoint",
    "cmd": "geometric_controller/cmd",
    "imu": "mavros/imu/data",
    "imu_raw": "mavros/imu/data_raw",
    "att_thrust": "mavros/setpoint_raw/attitude",
    "local_odom": "mavros/local_position/odom",
    "rel_alt": "mavros/global_position/rel_alt",
    "bat": "mavros/battery",
    "state": "mavros/state",
    "tuner_status": "geo_tuner/status",
    "gm_status": "geometric_mavros/status",
}


def _yaw_of(q) -> float:
    return float(np.arctan2(2 * (q.w * q.z + q.x * q.y),
                            1 - 2 * (q.y * q.y + q.z * q.z)))


@dataclasses.dataclass
class BagData:
    path: Path
    arrays: dict[str, np.ndarray]  # short name -> (n, k) float array
    strings: dict[str, list]  # short name -> [(t, text)] for String topics
    topics: dict[str, str]  # short name -> full topic present in the bag
    from_cache: bool = False

    def __contains__(self, key: str) -> bool:
        return key in self.arrays or key in self.strings


def _cache_paths(bag_dir: Path, cache_dir: Path) -> tuple[Path, Path]:
    base = cache_dir / bag_dir.name
    return base.with_suffix(".npz"), base.with_suffix(".json")


def _db3_signature(bag_dir: Path) -> str:
    sig = []
    for p in sorted(bag_dir.glob("*.db3")):
        st = p.stat()
        sig.append(f"{p.name}:{st.st_size}:{int(st.st_mtime)}")
    return ";".join(sig)


def load_bag(bag_dir: Path, cache_dir: Path) -> BagData:
    bag_dir, cache_dir = Path(bag_dir), Path(cache_dir)
    npz_path, json_path = _cache_paths(bag_dir, cache_dir)
    sig = _db3_signature(bag_dir)

    if npz_path.is_file() and json_path.is_file():
        side = json.loads(json_path.read_text())
        if side.get("signature") == sig:
            with np.load(npz_path) as z:
                arrays = {k: z[k] for k in z.files}
            return BagData(bag_dir, arrays, side.get("strings", {}),
                           side.get("topics", {}), from_cache=True)

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        raise BagUnavailable(f"rosbag2_py not importable ({e}); source ROS 2") from e

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topics: dict[str, str] = {}
    for short, suffix in TOPIC_SUFFIXES.items():
        for name in types:
            if name.endswith(suffix):
                topics[short] = name
                break
    cls = {}
    for short, name in list(topics.items()):
        try:
            cls[short] = get_message(types[name])
        except (ModuleNotFoundError, AttributeError, ValueError):
            # SE3Command needs mav_controllers_ros sourced; drop just that
            # stream rather than refusing the whole bag.
            del topics[short]
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics.values())))
    by_topic = {v: k for k, v in topics.items()}

    rows: dict[str, list] = {k: [] for k in topics}
    strings: dict[str, list] = {}
    while reader.has_next():
        name, data, t_ns = reader.read_next()
        short = by_topic[name]
        msg = deserialize_message(data, cls[short])
        t = t_ns * 1e-9
        if short in ("odom", "local_odom"):
            p, v = msg.pose.pose.position, msg.twist.twist.linear
            rows[short].append((t, p.x, p.y, p.z, v.x, v.y, v.z,
                                _yaw_of(msg.pose.pose.orientation)))
        elif short == "sp":
            if not msg.points:
                continue
            tr = msg.points[0].transforms[0]
            rows[short].append((t, tr.translation.x, tr.translation.y,
                                tr.translation.z, _yaw_of(tr.rotation)))
        elif short == "cmd":
            f = msg.force
            rows[short].append((t, f.x, f.y, f.z))
        elif short in ("imu", "imu_raw"):
            q, a = msg.orientation, msg.linear_acceleration
            rows[short].append((t, q.w, q.x, q.y, q.z, a.x, a.y, a.z))
        elif short == "att_thrust":
            rows[short].append((t, msg.thrust))
        elif short == "rel_alt":
            rows[short].append((t, msg.data))
        elif short == "bat":
            rows[short].append((t, msg.voltage, msg.current))
        elif short == "state":
            rows[short].append((t, 1.0 if msg.mode == "OFFBOARD" else 0.0,
                                1.0 if msg.armed else 0.0))
        elif short in ("tuner_status", "gm_status"):
            # tuner_status is a std_msgs String; gm_status is a
            # DiagnosticStatus (level/message + key-value pairs, carrying
            # the estimator on/off state B1 verifies).
            if hasattr(msg, "data"):
                strings.setdefault(short, []).append((t, str(msg.data)))
            elif hasattr(msg, "message"):
                text = json.dumps({
                    "level": int.from_bytes(msg.level, "little")
                    if isinstance(msg.level, bytes) else int(msg.level),
                    "name": msg.name,
                    "message": msg.message,
                    "values": {kv.key: kv.value for kv in msg.values},
                })
                strings.setdefault(short, []).append((t, text))

    arrays = {k: np.array(v, dtype=float) for k, v in rows.items() if v}

    cache_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(npz_path, **arrays)
    json_path.write_text(json.dumps(
        {"signature": sig, "topics": topics, "strings": strings}))
    return BagData(bag_dir, arrays, strings, topics)
