from __future__ import annotations

import numpy as np
from scipy.signal import savgol_filter

from ..geo import latlon_to_local_enu
from .grid import Grid

MAX_SPEED = 40.0
SPIKE_JUMP = 8.0


def _interp_nan(x: np.ndarray, t: np.ndarray) -> np.ndarray:
    out = x.copy()
    good = np.isfinite(out)
    if good.sum() < 2:
        return np.zeros_like(out)
    out[~good] = np.interp(t[~good], t[good], out[good])
    return out


def nanmean2(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    stack = np.vstack([a, b])
    with np.errstate(invalid="ignore"):
        cnt = np.isfinite(stack).sum(axis=0)
        tot = np.nansum(stack, axis=0)
    return np.where(cnt > 0, tot / np.maximum(cnt, 1), np.nan)


def gnss_local_enu(
    frame, fix_cols=("mlat", "mlon", "malt"), vel_cols=("mvx", "mvy")
) -> dict[str, np.ndarray]:
    t = frame["t"].to_numpy()
    lat = frame[fix_cols[0]].to_numpy()
    lon = frame[fix_cols[1]].to_numpy()
    alt = frame[fix_cols[2]].to_numpy()
    vx = frame[vel_cols[0]].to_numpy()
    vy = frame[vel_cols[1]].to_numpy()
    ok = np.isfinite(lat) & np.isfinite(lon)
    out: dict[str, np.ndarray] = {}
    if ok.sum() < 10:
        for k in ("gx", "gy", "gz", "speed", "heading", "yaw_rate", "accel"):
            out[k] = np.full(t.shape, np.nan)
        return out

    lat0 = float(np.nanmedian(lat[ok]))
    lon0 = float(np.nanmedian(lon[ok]))
    alt0 = float(np.nanmedian(alt[ok]))
    e = _interp_nan(lat, t)
    n = _interp_nan(lon, t)
    a = _interp_nan(alt, t)
    enu = latlon_to_local_enu(e, n, a, lat0, lon0, alt0)
    gx, gy, gz = enu[:, 0], enu[:, 1], enu[:, 2]

    valid = ok.copy()
    speed = np.hypot(_interp_nan(vx, t), _interp_nan(vy, t))
    valid &= np.isfinite(speed) & (speed < MAX_SPEED)
    for arr in (gx, gy, gz):
        bad = ~valid
        arr[bad] = np.nan
    for arr in (gx, gy, gz):
        d = np.abs(np.diff(arr))
        jump = np.zeros_like(d, dtype=bool)
        jump[d > SPIKE_JUMP] = True
        if jump.any():
            idx = np.nonzero(jump)[0]
            arr[idx] = np.nan
            arr[idx + 1] = np.nan
    gx = _interp_nan(gx, t)
    gy = _interp_nan(gy, t)
    gz = _interp_nan(gz, t)
    speed = np.where(np.isfinite(speed) & (speed < MAX_SPEED), speed, np.nan)
    speed = _interp_nan(speed, t)

    win = 31 if t.size > 60 else max(5, (t.size // 2) * 2 + 1)
    poly = 3 if win > 5 else 2
    gs = savgol_filter(speed, win, poly, mode="interp")
    gx_s = savgol_filter(gx, win, poly, mode="interp")
    gy_s = savgol_filter(gy, win, poly, mode="interp")
    gz_s = savgol_filter(gz, win, poly, mode="interp")

    heading = np.unwrap(np.arctan2(gy_s, gx_s))
    accel_win = 81 if t.size > 200 else max(9, (t.size // 2) * 2 + 1)
    accel = savgol_filter(speed, accel_win, 3, deriv=1, delta=1.0 / 50.0, mode="interp")
    yaw_rate = savgol_filter(heading, accel_win, 3, deriv=1, delta=1.0 / 50.0, mode="interp")

    out = {
        "gx": gx_s,
        "gy": gy_s,
        "gz": gz_s,
        "gx_raw": np.where(valid, gx, np.nan),
        "gy_raw": np.where(valid, gy, np.nan),
        "speed": np.clip(gs, 0.0, None),
        "heading": heading,
        "yaw_rate": yaw_rate,
        "accel": accel,
        "lat0": np.array(lat0),
        "lon0": np.array(lon0),
        "alt0": np.array(alt0),
    }
    return out


def label_grid(grid: Grid, hz: float = 50.0) -> dict[str, np.ndarray]:
    frame = grid.frame
    t = frame["t"].to_numpy()
    dt = 1.0 / hz
    u = frame["u"].to_numpy()
    vf = frame["v_front"].to_numpy()
    vr = frame["v_rear"].to_numpy()
    # the master antenna carries the primary labels; the rover is 12.4 m away on
    # the same rigid body, so it is an independent measurement of the same track
    master = gnss_local_enu(frame)
    rover = gnss_local_enu(
        frame, fix_cols=("rlat", "rlon", "ralt"), vel_cols=("rvx", "rvy")
    )

    labels = {
        "t": t,
        "u": u,
        "v_front": vf,
        "v_rear": vr,
        "v_wheel_mean": nanmean2(vf, vr),
        "v_wheel_diff": vf - vr,
        "u_valid": np.isfinite(u).astype(np.float32),
        "vf_valid": np.isfinite(vf).astype(np.float32),
        "vr_valid": np.isfinite(vr).astype(np.float32),
        "dt": np.full(t.shape, dt),
    }
    for k, v in master.items():
        labels[k if k not in ("lat0", "lon0", "alt0") else f"gnss_{k}"] = v
    for k, v in rover.items():
        if k in ("gx_raw", "gy_raw"):
            continue
        name = f"r{k}" if k not in ("lat0", "lon0", "alt0") else f"gnss_r{k}"
        labels[name] = v
    return labels
