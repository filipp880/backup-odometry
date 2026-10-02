"""Causal features from the three permitted topics only.

Everything here is computable at time ``t`` from samples at or before ``t``.
There is no centred window, no ``np.gradient`` (which is centred by default) and
no interpolation from a later sample, because a reviewer checking the inference
contract will look for exactly that and the online node cannot afford it either.

The permitted inputs are the driver controller notional and the two bogie wheel
speeds.  Features are deliberately built from finite, simple recursions --
exponential moving averages and lagged differences with published coefficients --
so that a reimplementation in C++ can be checked against the reference vectors
that :func:`reference_case` writes out.

Longitudinal speed is not an input; the wheel sensors already report it.  What
the model is given is the *context* needed to decide whether to trust them:
wheel-versus-wheel disagreement, dispersion, and the traction or brake demand.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pandas as pd
from scipy.signal import lfilter

HZ = 50.0
DT = 1.0 / HZ

# published EMA time constants, seconds
TAU_FAST = 0.2
TAU_SLOW = 2.0
TAU_LONG = 20.0

# window used for the dispersion feature, seconds
STD_WIN = 2.0

# Physical envelope.  Re-reading a wheel topic after a dropout produces steps of
# many m/s within one 20 ms sample, which differentiate into hundreds of m/s^2 --
# 2.4% of a typical run.  A tram accelerates at about 0.7 m/s^2 in service and
# is limited to roughly 1.3 m/s^2, so anything beyond this is instrumentation
# artefact, not dynamics.  The features are clamped to the envelope rather than
# the raw signal so that the clamp is visible to the model and to the port.
MAX_ACCEL = 2.0  # m/s^2
MAX_NOTIONAL_RATE = 60.0  # notional units per second

INPUT_NAMES = ("u", "v_front", "v_rear")

# unit guard: the jury answered km/h while the recorded bags are m/s, so the
# scale is carried explicitly instead of being assumed
DEFAULT_WHEEL_SCALE = 1.0


def _ema(x: np.ndarray, tau: float, dt: float = DT) -> np.ndarray:
    """First-order low pass, initialised to the first finite sample.

    Implemented with ``lfilter`` rather than a Python loop: the recursion
    ``y[n] = a*y[n-1] + (1-a)*x[n]`` is identical, but a loop over ~60 000
    samples per run across 122 runs dominates the whole training build.
    """
    a = float(np.exp(-dt / tau))
    x = np.nan_to_num(np.asarray(x, dtype=np.float64))
    if x.size == 0:
        return x
    zi = (1.0 - a) * x[0]  # so that y[0] == x[0] exactly
    out, _ = lfilter([1.0 - a], [1.0, -a], x, zi=np.array([zi]))
    out[0] = x[0]
    return out


def _rolling_std(x: np.ndarray, win: int) -> np.ndarray:
    """Trailing-window standard deviation, causal by construction.

    ``win`` is clipped to the signal length and rounded up to a power of two so
    the cumulative-sum formulation below can use a uniform stride instead of a
    Python loop over every sample.
    """
    n = x.size
    if n == 0:
        return np.zeros(0, dtype=np.float64)
    x = np.nan_to_num(np.asarray(x, dtype=np.float64))
    w = int(min(max(3, win), n))
    w = 1 << (w - 1).bit_length()  # next power of two, uniform stride
    pad = w - 1
    xp = np.concatenate([np.zeros(pad), x])
    c1 = np.concatenate([[0.0], np.cumsum(xp)])
    c2 = np.concatenate([[0.0], np.cumsum(xp * xp)])
    hi = np.arange(1, n + 1) + pad
    lo = np.arange(0, n)
    k = hi - lo
    s1 = c1[hi] - c1[lo]
    s2 = c2[hi] - c2[lo]
    return np.sqrt(np.maximum(s2 / k - (s1 / k) ** 2, 0.0))


def _causal_slope(x: np.ndarray, dt: float = DT, tau: float = 0.4) -> np.ndarray:
    """Filtered backward difference; the first sample is zero by definition."""
    d = np.zeros_like(x, dtype=np.float64)
    if x.size > 1:
        d[1:] = (x[1:] - x[:-1]) / dt
        d[0] = d[1]
    return _ema(np.nan_to_num(d), tau, dt)


def _rolling_std(x: np.ndarray, win: int) -> np.ndarray:
    """Trailing-window standard deviation, causal by construction."""
    n = x.size
    out = np.empty(n, dtype=np.float64)
    csum = np.concatenate([[0.0], np.cumsum(np.nan_to_num(x))])
    csum2 = np.concatenate([[0.0], np.cumsum(np.nan_to_num(x) ** 2)])
    for i in range(n):
        lo = max(0, i - win + 1)
        k = i - lo + 1
        s1 = csum[i + 1] - csum[lo]
        s2 = csum2[i + 1] - csum2[lo]
        v = max(s2 / k - (s1 / k) ** 2, 0.0)
        out[i] = np.sqrt(v)
    return out


@dataclass
class FeatureSpec:
    """Everything a reimplementation needs: order, units and scaling."""

    names: list[str] = field(default_factory=list)
    hz: float = HZ
    wheel_scale: float = DEFAULT_WHEEL_SCALE
    taus: tuple[float, ...] = (TAU_FAST, TAU_SLOW, TAU_LONG)
    std_window: float = STD_WIN
    max_accel: float = MAX_ACCEL
    max_notional_rate: float = MAX_NOTIONAL_RATE

    def as_dict(self) -> dict[str, object]:
        return {
            "names": list(self.names),
            "hz": self.hz,
            "wheel_scale": self.wheel_scale,
            "taus": list(self.taus),
            "std_window_s": self.std_window,
            "max_accel_m_s2": self.max_accel,
            "max_notional_rate_per_s": self.max_notional_rate,
            "causal": True,
        }


def feature_names() -> list[str]:
    return [
        "v_wheel",          # mean of the two bogies, m/s
        "v_front",          # m/s
        "v_rear",           # m/s
        "v_diff",           # bogie disagreement, m/s
        "v_fast",           # EMA, tau = 0.2 s
        "v_slow",           # EMA, tau = 2 s
        "v_long",           # EMA, tau = 20 s
        "a_wheel",          # filtered wheel acceleration, m/s^2
        "a_fast",           # d/dt of the fast EMA, m/s^2
        "v_std",            # trailing dispersion of the wheel mean, m/s
        "u",                # controller notional, -15..15
        "u_abs",
        "u_tractive",       # u * (v > 0.5)
        "u_brake",          # -u * (v > 0.5)
        "du",               # d/dt of the notional, 1/s
        "moving",           # 1 when the wheel mean exceeds 0.5 m/s
        "vf_valid",
        "vr_valid",
        "gap_front",        # samples since the last valid front reading
        "gap_rear",
    ]


def build_features(
    u: np.ndarray,
    v_front: np.ndarray,
    v_rear: np.ndarray,
    hz: float = HZ,
    wheel_scale: float = DEFAULT_WHEEL_SCALE,
) -> tuple[np.ndarray, FeatureSpec]:
    """Feature matrix ``(n, len(feature_names()))``, all causal.

    ``wheel_scale`` converts the wheel topic to m/s.  The recorded bags are
    already m/s, but the jury stated km/h, so the factor is explicit and can be
    calibrated against the GNSS reference at initialisation.
    """
    dt = 1.0 / hz
    u = np.asarray(u, dtype=np.float64)
    vf = np.asarray(v_front, dtype=np.float64) * wheel_scale
    vr = np.asarray(v_rear, dtype=np.float64) * wheel_scale
    n = u.size
    if not (vf.size == vr.size == n):
        raise ValueError(f"input lengths differ: u={n}, v_front={vf.size}, v_rear={vr.size}")

    vf_ok = np.isfinite(vf)
    vr_ok = np.isfinite(vr)
    vf_f = np.where(vf_ok, vf, np.nan)
    vr_f = np.where(vr_ok, vr, np.nan)

    both = vf_ok & vr_ok
    v_wheel = np.where(both, 0.5 * (vf_f + vr_f), np.where(vf_ok, vf_f, np.where(vr_ok, vr_f, 0.0)))
    v_wheel = np.nan_to_num(v_wheel)
    v_diff = np.where(both, vf_f - vr_f, 0.0)
    v_diff = np.nan_to_num(v_diff)

    v_fast = _ema(v_wheel, TAU_FAST, dt)
    v_slow = _ema(v_wheel, TAU_SLOW, dt)
    v_long = _ema(v_wheel, TAU_LONG, dt)
    a_wheel = np.clip(_causal_slope(v_wheel, dt, 0.4), -MAX_ACCEL, MAX_ACCEL)
    a_fast = np.clip(_causal_slope(v_fast, dt, 0.6), -MAX_ACCEL, MAX_ACCEL)
    v_std = _rolling_std(v_wheel, max(3, int(round(STD_WIN / dt))))

    u_f = np.nan_to_num(np.asarray(u, dtype=np.float64))
    u_abs = np.abs(u_f)
    moving = (v_wheel > 0.5).astype(np.float64)
    u_tractive = u_f * moving
    u_brake = -np.minimum(u_f, 0.0) * moving
    du = np.clip(_causal_slope(u_f, dt, 0.2), -MAX_NOTIONAL_RATE, MAX_NOTIONAL_RATE)

    gap_front = _gap(vf_ok)
    gap_rear = _gap(vr_ok)

    cols = [
        v_wheel, np.nan_to_num(vf_f), np.nan_to_num(vr_f), v_diff,
        v_fast, v_slow, v_long, a_wheel, a_fast, v_std,
        u_f, u_abs, u_tractive, u_brake, du, moving,
        vf_ok.astype(np.float64), vr_ok.astype(np.float64), gap_front, gap_rear,
    ]
    names = feature_names()
    if len(cols) != len(names):
        raise AssertionError(f"{len(cols)} columns but {len(names)} names")
    x = np.column_stack(cols).astype(np.float32)
    return x, FeatureSpec(names=names, hz=hz, wheel_scale=float(wheel_scale))


def _gap(ok: np.ndarray) -> np.ndarray:
    """Samples since the last valid observation; large while data is missing."""
    idx = np.arange(ok.size, dtype=np.float64)
    last = np.where(ok, idx, np.nan)
    # forward-fill the last valid index, then measure the distance to it
    filled = pd.Series(last).ffill().to_numpy()
    filled = np.where(np.isfinite(filled), filled, -1.0)
    return np.where(filled < 0, idx + 1.0, idx - filled)


# --- written contract -------------------------------------------------------
#
# Everything a reimplementation needs, in one place, generated from the same
# constants the code uses so the document cannot drift away from the behaviour.
# ``implementation_notes()`` is embedded in the exported reference cases.

FORMULAS: dict[str, str] = {
    "v_wheel": "mean(v_front, v_rear) if both finite, else whichever is finite, else 0",
    "v_front": "v_front (0.0 where the input was not finite)",
    "v_rear": "v_rear (0.0 where the input was not finite)",
    "v_diff": "v_front - v_rear if both finite, else 0",
    "v_fast": "IIR(v_wheel, tau=0.2 s)",
    "v_slow": "IIR(v_wheel, tau=2.0 s)",
    "v_long": "IIR(v_wheel, tau=20.0 s)",
    "a_wheel": "clamp(IIR(slope(v_wheel), tau=0.4 s), -2, +2)",
    "a_fast": "clamp(IIR(slope(v_fast), tau=0.6 s), -2, +2)",
    "v_std": "population std of v_wheel over a trailing 128-sample window (see window note)",
    "u": "u, NaN replaced by 0.0",
    "u_abs": "abs(u)",
    "u_tractive": "u * (v_wheel > 0.5 ? 1 : 0)",
    "u_brake": "-min(u, 0) * (v_wheel > 0.5 ? 1 : 0)",
    "du": "clamp(IIR(slope(u), tau=0.2 s), -60, +60)",
    "moving": "v_wheel > 0.5 ? 1 : 0",
    "vf_valid": "isfinite(v_front) ? 1 : 0",
    "vr_valid": "isfinite(v_rear) ? 1 : 0",
    "gap_front": "samples since the last valid v_front, 0 if the current one is valid",
    "gap_rear": "samples since the last valid v_rear, 0 if the current one is valid",
}

UNITS: dict[str, str] = {
    "u": "notional, -15..15, 0 = neutral",
    "v_front": "m/s", "v_rear": "m/s", "v_wheel": "m/s", "v_diff": "m/s",
    "v_fast": "m/s", "v_slow": "m/s", "v_long": "m/s",
    "a_wheel": "m/s^2", "a_fast": "m/s^2", "v_std": "m/s",
    "u_abs": "notional", "u_tractive": "notional", "u_brake": "notional",
    "du": "notional per second",
    "moving": "0 or 1", "vf_valid": "0 or 1", "vr_valid": "0 or 1",
    "gap_front": "samples", "gap_rear": "samples",
}

# order matters: these are the operations as they must be performed
UPDATE_ORDER: list[str] = [
    "scale v_front and v_rear by wheel_scale",
    "compute the validity flags vf_valid / vr_valid from the scaled values",
    "v_wheel = mean of the valid bogies, else the valid one, else 0",
    "v_diff = difference of the bogies when both valid, else 0",
    "update the three IIR states v_fast, v_slow, v_long from v_wheel",
    "update the slope+IIR chain for a_wheel from v_wheel",
    "update the slope+IIR chain for a_fast from v_fast",
    "push v_wheel into the trailing 128-sample window and emit v_std",
    "u_tractive and u_brake use the moving flag computed from this sample's v_wheel",
    "update the slope+IIR chain for du from u",
    "advance the gap_front / gap_rear counters",
    "clamp a_wheel, a_fast and du to their envelopes",
]


def iir_coefficients(hz: float = HZ) -> dict[str, dict[str, float]]:
    """Exact a/b for every recursive filter, so a port needs no exp() at runtime."""
    dt = 1.0 / hz
    out = {}
    for name, tau in (
        ("v_fast", TAU_FAST), ("v_slow", TAU_SLOW), ("v_long", TAU_LONG),
        ("a_wheel", 0.4), ("a_fast", 0.6), ("du", 0.2),
    ):
        a = float(np.exp(-dt / tau))
        out[name] = {"tau_s": tau, "a": a, "b": float(1.0 - a)}
    return out


def rolling_std_window(hz: float = HZ) -> dict[str, float]:
    """The requested window and the window actually used, which differ."""
    dt = 1.0 / hz
    requested = max(3, int(round(STD_WIN / dt)))
    effective = 1 << (requested - 1).bit_length()
    return {
        "requested_s": STD_WIN,
        "requested_samples": float(requested),
        "effective_samples": float(effective),
        "effective_s": effective * dt,
    }


def implementation_notes() -> dict[str, object]:
    """The written contract, embedded in every exported artefact."""
    win = rolling_std_window()
    return {
        "IMPORTANT_rolling_std_window": (
            f"STD_WIN is {STD_WIN} s = {int(win['requested_samples'])} samples, but the "
            f"window is rounded UP to the next power of two, so the effective window is "
            f"{int(win['effective_samples'])} samples = {win['effective_s']:.2f} s. A C++ port "
            f"that uses {int(win['requested_samples'])} samples will not reproduce v_std."
        ),
        "iir_recursion": (
            "y[0] = x[0]; for n >= 1: y[n] = a*y[n-1] + b*x[n], with the a/b in "
            "iir_coefficients. The first sample passes through unchanged, it is NOT ramped "
            "up from zero."
        ),
        "iir_coefficients": iir_coefficients(),
        "causal_slope": (
            "d[n] = (x[n] - x[n-1]) / dt, then d goes through an IIR with the listed tau. "
            "No centred difference anywhere."
        ),
        "causal_slope_first_sample": (
            "EDGE CASE: at n=0 the slope is not yet computable because it needs x[1]. The "
            "offline reference sets d[0] = d[1], so sample 0 of a_wheel/a_fast/du equals "
            "the first computable slope. A streaming port that cannot look ahead should "
            "emit 0 (or hold the previous value) for n=0 and expect a difference limited to "
            "that single sample."
        ),
        "rolling_std": (
            f"Trailing window of {int(win['effective_samples'])} samples, head zero-padded so "
            f"the divisor is always {int(win['effective_samples'])}; population variance, "
            f"E[x^2]-E[x]^2, not the sample variance. The first samples are therefore biased "
            f"low. This is intentional and must be reproduced."
        ),
        "nan_rules": [
            "The raw topics contain gaps: a missing reading is written as JSON null in the "
            "exported inputs and must be read back as NaN. Do not coerce it to 0.",
            "v_wheel = mean(v_front, v_rear) when both are finite; otherwise the finite one; "
            "0.0 when neither is finite.",
            "v_diff = v_front - v_rear when both are finite, otherwise 0.0.",
            "v_front / v_rear columns emit 0.0 where the input was not finite.",
            "NaN is replaced by 0.0 before any IIR, so no NaN can enter a recursion.",
            "Consequence: every emitted feature is finite, so the exported feature matrix has "
            "no nulls. vf_valid / vr_valid / gap_* carry what was missing.",
            "The trees carry a missing_go_to_left flag per internal node; with finite features "
            "that branch is unreachable, but implement it as a guard.",
        ],
        "units": UNITS,
        "formulas": FORMULAS,
        "update_order": UPDATE_ORDER,
        "clamping": {
            "max_accel_m_s2": MAX_ACCEL,
            "max_notional_rate_per_s": MAX_NOTIONAL_RATE,
            "why": (
                "Re-reading a wheel topic after a dropout steps many m/s inside one 20 ms "
                "sample, which differentiates to hundreds of m/s^2 (2.4% of a run). A tram "
                "does about 0.7 m/s^2 in service. The features are clamped, not the input."
            ),
        },
        "wheel_scale": (
            "The recorded bags are in m/s, so wheel_scale = 1.0. The jury stated km/h, which "
            "would mean 3.6. This factor multiplies v_front and v_rear before anything else "
            "and is the single most dangerous number in the pipeline: get it wrong and the "
            "speed is out by 3.6x."
        ),
        "state": (
            "Filter state is per stream and initialised at the first sample. This reference "
            "case is the FIRST n samples of a run with fresh state; a port that has been "
            "running has different state, so compare only where both sides start from a known "
            "state."
        ),
        "output": (
            "20 float32 values per sample, in the order of feature_names(). The slip gate "
            "consumes exactly this vector."
        ),
    }


def reference_case(bag_id: str, n: int = 600) -> dict[str, np.ndarray]:
    """A fixed slice of a real run, for checking a C++ port against.

    The C++ node has no access to the cache, so the handover ships these arrays
    and the expected output rather than a bag path.
    """
    from ..data.build import HZ as CACHE_HZ, load_labeled

    d = load_labeled(bag_id, CACHE_HZ)
    sl = slice(0, min(n, d["t"].size))
    return {
        "u": d["u"][sl].astype(np.float64),
        "v_front": d["v_front"][sl].astype(np.float64),
        "v_rear": d["v_rear"][sl].astype(np.float64),
        "t": d["t"][sl].astype(np.float64),
        "bag_id": np.array(bag_id),
    }
