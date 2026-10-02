"""The 16 ML features, in the compiled-in order of ml_features.hpp.

The order is the contract. C++ reads kFeatureNames and kOutputNames from the
header and the descriptor is rejected if they differ, so this list must match
kNumFeatures exactly and in sequence:

    0  u              normalised driver command, [-1, 1]
    1  v              body speed, m/s
    2  a_model        physics acceleration, m/s^2, no learned residual
    3  grade          path inclination, rad
    4  mu             adhesion estimate
    5  omega_front    front bogie angular rate, rad/s
    6  omega_rear     rear bogie angular rate, rad/s
    7  b_scale        wheel scale from the observer
    8  slip_index     slip detector output, [0, 1]
    9  trust          slip detector confidence, [0, 1]
   10  cmd_rate       |du/dt|, 1/s
   11  dt             step, s
   12  abs_u          |u|
   13  u_sq           u^2
   14  v_sq           v^2
   15  force_ratio    |F_net| / adhesion limit

`a_model` is the pure physics value. The learned residual is added at runtime by
the C++ node, so feeding it into its own input would close a loop.

The label is exported unclipped, with a separate target_clipped flag, because
clipping at export teaches the model a distribution the runtime never applies.
"""

from __future__ import annotations

import numpy as np

FEATURE_NAMES: tuple[str, ...] = (
    "u",
    "v",
    "a_model",
    "grade",
    "mu",
    "omega_front",
    "omega_rear",
    "b_scale",
    "slip_index",
    "trust",
    "cmd_rate",
    "dt",
    "abs_u",
    "u_sq",
    "v_sq",
    "force_ratio",
)

TARGET_ENV = 1.5  # ml.max_a_residual


def build_features(
    *,
    u: np.ndarray,
    v: np.ndarray,
    a_model: np.ndarray,
    omega_front: np.ndarray,
    omega_rear: np.ndarray,
    b_scale: np.ndarray,
    slip_index: np.ndarray,
    trust: np.ndarray,
    mu: np.ndarray,
    dt: np.ndarray,
    grade: np.ndarray,
    f_net: np.ndarray,
    f_limit: np.ndarray,
    cmd_rate: np.ndarray,
) -> np.ndarray:
    """Return an (n, 16) array in FEATURE_NAMES order.

    cmd_rate is an input rather than derived here on purpose: it is a derivative
    along the series, so it cannot be computed one row at a time, and computing
    it per row silently produced an all-zero column.
    """
    cols = {
        "u": u,
        "v": v,
        "a_model": a_model,
        "grade": grade,
        "mu": mu,
        "omega_front": omega_front,
        "omega_rear": omega_rear,
        "b_scale": b_scale,
        "slip_index": slip_index,
        "trust": trust,
        "cmd_rate": cmd_rate,
        "dt": dt,
        "abs_u": np.abs(u),
        "u_sq": u * u,
        "v_sq": v * v,
        "force_ratio": np.abs(f_net) / np.maximum(1.0, f_limit),
    }
    return np.column_stack([np.asarray(cols[k], dtype=np.float64) for k in FEATURE_NAMES])


def build_targets(a_lim: np.ndarray, a_phys: np.ndarray):
    """Unclipped residual plus the flag that says whether it was clipped.

    Returns (target, target_clipped). The runtime limit still applies at
    inference; what changes is that a saturated row is now identifiable instead
    of silently taught as a real correction.
    """
    target = np.asarray(a_lim, dtype=np.float64) - np.asarray(a_phys, dtype=np.float64)
    return target, (np.abs(target) > TARGET_ENV).astype(np.int8)
