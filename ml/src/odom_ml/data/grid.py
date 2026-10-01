from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd

from .. import config as C
from .bag import FIELDS, RawBag

COLUMNS = (
    "t",
    "u",
    "v_front",
    "v_rear",
    "mlat",
    "mlon",
    "malt",
    "mstatus",
    "mvx",
    "mvy",
    "mvz",
    "rlat",
    "rlon",
    "ralt",
    "rstatus",
    "rvx",
    "rvy",
    "rvz",
)

_TOPIC_COLUMN = {
    C.TOPIC_DRIVER: ("u",),
    C.TOPIC_FRONT: ("v_front",),
    C.TOPIC_REAR: ("v_rear",),
    C.TOPIC_MASTER_FIX: ("mlat", "mlon", "malt", "mstatus"),
    C.TOPIC_ROVER_FIX: ("rlat", "rlon", "ralt", "rstatus"),
    C.TOPIC_MASTER_VEL: ("mvx", "mvy", "mvz"),
    C.TOPIC_ROVER_VEL: ("rvx", "rvy", "rvz"),
}


def make_grid(t0: float, t1: float, hz: float) -> np.ndarray:
    dt = 1.0 / hz
    n = int(np.floor((t1 - t0) / dt)) + 1
    return t0 + dt * np.arange(n, dtype=np.float64)


def _resample(
    src_t: np.ndarray,
    src_v: np.ndarray,
    grid: np.ndarray,
    max_lag: float,
    mode: str,
) -> np.ndarray:
    out = np.full((grid.size, src_v.shape[1]), np.nan, dtype=np.float64)
    if src_t.size == 0:
        return out
    rel = src_t
    idx = np.searchsorted(rel, grid, side="right" if mode == "causal" else "left")
    if mode == "causal":
        idx = np.clip(idx - 1, 0, rel.size - 1)
    else:
        idx = np.clip(idx, 0, rel.size - 1)
        left = np.clip(idx - 1, 0, rel.size - 1)
        pick_right = np.abs(rel[idx] - grid) > np.abs(rel[left] - grid)
        idx = np.where(pick_right, idx, left)
    lag = np.abs(rel[idx] - grid)
    valid = lag <= max_lag
    out[valid] = src_v[idx[valid]]
    return out


@dataclass
class Grid:
    bag_id: str
    vehicle: str
    frame: pd.DataFrame

    def __len__(self) -> int:
        return len(self.frame)

    @property
    def duration(self) -> float:
        return float(self.frame["t"].iloc[-1])


def grid_from_bag(
    raw: RawBag,
    hz: float = 50.0,
    max_lag: float = 0.12,
    mode: str = "causal",
) -> Grid:
    max_lag = max_lag if max_lag is not None else 1.5 / hz
    t0 = min(float(v[0]) for v in raw.t.values() if v.size)
    t1 = max(float(v[-1]) for v in raw.t.values() if v.size)
    grid = make_grid((t0 - raw.t0_ns) / 1e9, (t1 - raw.t0_ns) / 1e9, hz)

    frame = pd.DataFrame({"t": grid})
    for topic, cols in _TOPIC_COLUMN.items():
        rel_t = (raw.t[topic].astype(np.float64) - float(raw.t0_ns)) / 1e9
        values = _resample(rel_t, raw.values[topic], grid, max_lag, mode)
        for i, name in enumerate(cols):
            frame[name] = values[:, i]

    if "v_front" in frame:
        frame["v_front"] = frame["v_front"] * C.KPH_TO_MS
        frame["v_rear"] = frame["v_rear"] * C.KPH_TO_MS
    return Grid(bag_id=raw.bag_id, vehicle=raw.vehicle, frame=frame)
