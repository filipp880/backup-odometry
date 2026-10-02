"""Read tram raw bags into a uniform time grid.

The C++ node consumes rosbag2 sqlite3 bags directly. This module gives the Python
pipeline the same view: absolute bag time on a uniform 50 Hz grid, with wheel
speeds in km/h as the raw VelocitySensor reports them, the driver notch, and the
GNSS master/rover fixes and velocities.

Nothing here interprets the data. Signal conditioning, physics and the observer
live in py_pipeline.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

REPO = Path(__file__).resolve().parents[2]
MSG_DIR = REPO / "data" / "tram_vehicle_msgs" / "msg"

TOPIC_FRONT = "/vehicle/front_bogie_velocity"
TOPIC_REAR = "/vehicle/rear_bogie_velocity"
TOPIC_CMD = "/vehicle/driver_position_cmd"
TOPIC_GNSS = {
    "master_fix": "/sensing/gnss/master/fix",
    "rover_fix": "/sensing/gnss/rover/fix",
    "master_vel": "/sensing/gnss/master/vel",
    "rover_vel": "/sensing/gnss/rover/vel",
}
TOPIC_IMU = "/sensing/imu/data"


def build_typestore():
    """ROS 2 Humble store plus this project's two custom messages."""
    store = get_typestore(Stores.ROS2_HUMBLE)
    for path in sorted(MSG_DIR.glob("*.msg")):
        name = f"tram_vehicle_msgs/msg/{path.stem}"
        store.register(get_types_from_msg(path.read_text(encoding="utf-8"), name))
    return store


def _reindex(grid: np.ndarray, ts: np.ndarray, values: np.ndarray, default=np.nan):
    """Place irregular samples onto a uniform grid, last value held forward.

    Deliberately not interpolated: an interpolated wheel speed between two
    encoder samples invents an acceleration that never happened, and the
    longitudinal model would integrate it.
    """
    out = np.full(grid.shape, default, dtype=np.float64)
    if ts.size == 0:
        return out
    idx = np.searchsorted(grid, ts, side="right") - 1
    ok = idx >= 0
    out[idx[ok]] = values[ok]
    # Carry the last sample forward until the next one arrives.
    last = np.searchsorted(grid, ts, side="left")
    for i, t in enumerate(ts):
        lo = last[i]
        hi = last[i + 1] if i + 1 < ts.size else grid.size
        if hi > lo:
            out[lo:hi] = values[i]
    out[grid < ts[0]] = values[0]
    return out


def read_bag(bag_dir: Path, hz: float = 50.0) -> pd.DataFrame:
    bag_dir = Path(bag_dir)
    store = build_typestore()

    cols: dict[str, list] = {}
    with AnyReader([bag_dir], default_typestore=store) as reader:
        cols = {k: [] for k in ("tf", "vf", "vr", "cmd", "imu_y")}
        t0 = None
        for conn, tstamp, raw in reader.messages(
            connections=[
                c
                for c in reader.connections
                if c.topic
                in (
                    TOPIC_FRONT,
                    TOPIC_REAR,
                    TOPIC_CMD,
                    TOPIC_IMU,
                )
            ]
        ):
            msg = reader.deserialize(raw, conn.msgtype)
            if t0 is None:
                t0 = tstamp
            ts = (tstamp - t0) / 1e9
            if conn.topic == TOPIC_FRONT:
                cols["vf"].append((ts, msg.velocity))
            elif conn.topic == TOPIC_REAR:
                cols["vr"].append((ts, msg.velocity))
            elif conn.topic == TOPIC_CMD:
                cols["cmd"].append((ts, float(msg.position)))
            elif conn.topic == TOPIC_IMU:
                cols["imu_y"].append((ts, msg.orientation.z))

    if t0 is None:
        raise ValueError(f"{bag_dir}: none of the expected topics are present")

    def arr(key, col=1):
        rows = sorted(cols[key])
        if not rows:
            return np.empty(0), np.empty(0)
        a = np.array(rows, dtype=np.float64)
        return a[:, 0], a[:, col]

    dur = max(
        (a[0].max() for a in (arr("vf"), arr("vr"), arr("cmd")) if a[0].size),
        default=0.0,
    )
    grid = np.arange(0.0, dur, 1.0 / hz)

    t_f, v_f = arr("vf")
    t_r, v_r = arr("vr")
    t_c, v_c = arr("cmd")
    t_i, v_i = arr("imu_y")

    df = pd.DataFrame(
        {
            # Absolute bag time in seconds, matching the C++ dump convention.
            "t": t0 / 1e9 + grid,
            "dt": 1.0 / hz,
            "v_front_kmh": _reindex(grid, t_f, v_f),
            "v_rear_kmh": _reindex(grid, t_r, v_r),
            "cmd_position": _reindex(grid, t_c, v_c),
            "imu_yaw_rate": _reindex(grid, t_i, v_i),
        }
    )

    gnss = _read_gnss(bag_dir, store)
    for key in (
        "master_fix_lat", "master_fix_lon", "master_fix_alt", "master_fix_quality",
        "master_vel_x", "master_vel_y",
        "rover_fix_lat", "rover_fix_lon", "rover_fix_alt", "rover_fix_quality",
        "rover_vel_x", "rover_vel_y",
    ):
        # 35 of the 122 bags carry no GNSS topic at all, so the family may be
        # absent entirely. Emit the column as NaN anyway: a downstream KeyError
        # is a crash, and "this run has no reference" is a value.
        if key in gnss:
            ts, val, default = gnss[key]
            df[key] = _reindex(grid, ts, val, default=default)
        else:
            df[key] = np.full(grid.shape, np.nan)

    df.insert(0, "bag_id", bag_dir.name)
    return df


def _read_gnss(bag_dir: Path, store) -> dict:
    """GNSS fixes and velocities, returned as {column: (t, value, default)}.

    Fix and velocity are kept in separate column families rather than merged, so
    a missing velocity never blanks a good position.
    """
    wanted = {v: k for k, v in TOPIC_GNSS.items()}
    acc: dict[str, list] = {}
    with AnyReader([bag_dir], default_typestore=store) as reader:
        t0 = None
        for conn, tstamp, raw in reader.messages(
            connections=[c for c in reader.connections if c.topic in wanted]
        ):
            msg = reader.deserialize(raw, conn.msgtype)
            if t0 is None:
                t0 = tstamp
            key = wanted[conn.topic]
            ts = (tstamp - t0) / 1e9
            if key.endswith("_fix"):
                acc.setdefault(key, []).append(
                    (
                        ts,
                        float(msg.latitude),
                        float(msg.longitude),
                        float(msg.altitude),
                        float(getattr(msg, "position_covariance_type", 0) or 0),
                    )
                )
            else:
                acc.setdefault(key, []).append(
                    (ts, float(msg.twist.linear.x), float(msg.twist.linear.y), 0.0)
                )

    out: dict = {}
    for key, rows in acc.items():
        a = np.array(sorted(rows), dtype=np.float64)
        names = (
            ("lat", "lon", "alt", "quality") if key.endswith("_fix") else ("x", "y", "q")
        )
        for i, name in enumerate(names):
            out[f"{key}_{name}"] = (a[:, 0], a[:, 1 + i], np.nan)
    return out
