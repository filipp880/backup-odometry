"""Python port of the C++ signal filters and the acceleration label.

Ported from, in order:
  cpp/tram_odometry/src/signal_filter.cpp       HampelFilter, LowPassFilter
  cpp/tram_odometry/src/longitudinal_model.cpp  resistance, adhesionLimit
  cpp/tram_odometry/src/estimator.cpp:675-704   the label itself

The label in the C++ dump is recoverable without the traction model or the
observer, because estimator.cpp writes both halves:

    target_a_residual = clamp(a_lim - a_physics, -1.5, +1.5)
    a_model           = a_physics

so a_lim = a_model + target_a_residual. That is the quantity compared against
the port below, and it is why this module can be validated before the harder
parts exist.
"""

from __future__ import annotations

import numpy as np

# --- params.hpp, verified against config/params.yaml -------------------------
MASS_KG = 38000.0
G = 9.80665
WHEEL_RADIUS_M = 0.30
GEAR_RATIO = 1.0
ROT_INERTIA_KGM2 = 22.0
N_WHEELS_DRIVEN = 4
C_RR = 0.0060
RHO_AIR = 1.225
CD_A = 12.0
CURVE_RESISTANCE_COEFF = 0.0015

WHEEL_HAMPEL_WINDOW = 7
WHEEL_HAMPEL_SIGMA = 3.0
WHEEL_CUTOFF_HZ = 25.0
LABEL_CUTOFF_HZ = 1.0  # estimator.hpp:183, LowPassFilter label_wheel_lp_{1.0, 0.02}

K_ACCEL_ENV = 2.0  # estimator.cpp:701
K_TARGET_ENV = 1.5  # estimator.cpp:702

M_EFF = MASS_KG + ROT_INERTIA_KGM2 * GEAR_RATIO**2 / (WHEEL_RADIUS_M**2)


class Biquad:
    """Transposed direct form II, matching signal_filter.hpp bit for bit."""

    __slots__ = ("b0", "b1", "b2", "a1", "a2", "z1", "z2")

    def __init__(self, b0=1.0, b1=0.0, b2=0.0, a1=0.0, a2=0.0) -> None:
        self.b0, self.b1, self.b2 = b0, b1, b2
        self.a1, self.a2 = a1, a2
        self.z1 = self.z2 = 0.0

    def step(self, x: float) -> float:
        y = self.b0 * x + self.z1
        self.z1 = self.b1 * x - self.a1 * y + self.z2
        self.z2 = self.b2 * x - self.a2 * y
        return y

    def run(self, x: np.ndarray) -> np.ndarray:
        out = np.empty_like(x, dtype=np.float64)
        for i, v in enumerate(x):
            out[i] = self.step(float(v))
        return out


class LowPassFilter:
    """Two cascaded single-pole RC sections, as designed in signal_filter.cpp.

    Not a Butterworth: a 4th-order Butterworth overshoots on a step, and a speed
    filter that overshoots injects a false acceleration into the derivative.
    """

    def __init__(self, cutoff_hz: float, nominal_dt: float = 0.02) -> None:
        self.nominal_dt = nominal_dt if nominal_dt > 1e-4 else 0.02
        self.cutoff = cutoff_hz
        self._designed_dt = -1.0
        self._designed_cutoff = -1.0
        self.s1 = Biquad()
        self.s2 = Biquad()
        self.design(cutoff_hz, self.nominal_dt)

    def design(self, cutoff_hz: float, dt: float) -> None:
        fs = 1.0 / dt
        fc = cutoff_hz
        nyq = 0.5 * fs
        if fc >= nyq:
            fc = 0.9 * nyq
        if fc < 1e-3:
            fc = 1e-3
        K = np.pi * fc * dt
        norm = 1.0 / (1.0 + K)
        q = Biquad(b0=K * norm, b1=K * norm, b2=0.0, a1=(K - 1.0) * norm, a2=0.0)
        self.s1, self.s2 = q, Biquad(b0=q.b0, b1=q.b1, b2=q.b2, a1=q.a1, a2=q.a2)
        self._designed_dt = dt
        self._designed_cutoff = cutoff_hz

    def run(self, x: np.ndarray, dt: float) -> np.ndarray:
        if dt < 1e-5:
            dt = self.nominal_dt
        if (
            abs(dt - self._designed_dt) > 0.25 * self._designed_dt
            or abs(self.cutoff - self._designed_cutoff) > 1e-6
        ):
            self.design(self.cutoff, dt)
        return self.s2.run(self.s1.run(x))


def hampel(x: np.ndarray, window: int = WHEEL_HAMPEL_WINDOW, sigma: float = WHEEL_HAMPEL_SIGMA):
    """HampelFilter::push over the whole series, returning (filtered, is_outlier).

    Same rule as the C++: a non-finite sample is always an outlier, a constant
    window only accepts samples within 1e-3 of the median, and nothing is judged
    before three samples are in.
    """
    n = x.size
    out = np.empty(n, dtype=np.float64)
    outlier = np.zeros(n, dtype=bool)
    last = 0.0
    buf: list[float] = []
    for i in range(n):
        v = float(x[i])
        if not np.isfinite(v):
            out[i] = last
            outlier[i] = True
            continue
        buf.append(v)
        if len(buf) > window:
            buf.pop(0)
        srt = np.sort(np.asarray(buf, dtype=np.float64))
        m = len(srt)
        med = srt[m // 2] if m % 2 == 1 else 0.5 * (srt[m // 2 - 1] + srt[m // 2])
        dev = np.sort(np.abs(np.asarray(buf, dtype=np.float64) - med))
        mad = dev[m // 2] if m % 2 == 1 else 0.5 * (dev[m // 2 - 1] + dev[m // 2])
        scale = 1.4826 * mad
        bad = False
        if m >= 3:
            if scale > 1e-6:
                bad = abs(v - med) > sigma * scale
            else:
                bad = abs(v - med) > 1e-3
        if not bad:
            last = v
            out[i] = v
        else:
            out[i] = last
            outlier[i] = True
    return out, outlier


def wheel_speed_mps(v_front_kmh: np.ndarray, v_rear_kmh: np.ndarray) -> np.ndarray:
    """Mean of the two bogies in m/s, Hampel-filtered at 25 Hz.

    km/h -> m/s: the raw VelocitySensor reports km/h, and the C++ conversion is
    what the cached runs were built with.
    """
    vf, _ = hampel(np.asarray(v_front_kmh, dtype=np.float64) / 3.6)
    vr, _ = hampel(np.asarray(v_rear_kmh, dtype=np.float64) / 3.6)
    mean = 0.5 * (vf + vr)
    return LowPassFilter(WHEEL_CUTOFF_HZ).run(mean, 0.02)


def accel_label(v_mps: np.ndarray, dt: float = 0.02, cutoff_hz: float = LABEL_CUTOFF_HZ):
    """estimator.cpp:690-703, wheel branch.

    a_label = d/dt of LPF(v), then clamped to +/-2.0. The first sample has no
    predecessor, so it is 0, matching label_wheel_prev_valid_.
    """
    lp = LowPassFilter(cutoff_hz, dt).run(v_mps, dt)
    a = np.zeros_like(lp)
    a[1:] = np.diff(lp) / dt
    a_lim = np.clip(a, -K_ACCEL_ENV, K_ACCEL_ENV)
    return a, a_lim, lp


def resistance(v: np.ndarray, grade: np.ndarray, curvature: np.ndarray) -> np.ndarray:
    f_roll = C_RR * MASS_KG * G * np.cos(grade)
    f_aero = 0.5 * RHO_AIR * CD_A * v * np.abs(v)
    f_grade = MASS_KG * G * np.sin(grade)
    f_curve = CURVE_RESISTANCE_COEFF * MASS_KG * G * np.abs(curvature)
    return f_roll + f_aero + f_grade + f_curve


def adhesion_limit(mu: np.ndarray) -> np.ndarray:
    fz = MASS_KG * G / 4.0 * max(1, N_WHEELS_DRIVEN)
    return np.maximum(0.0, mu) * fz


def accel_physics(v, drive_force, brake_force, grade, curvature, mu, residual=None):
    """LongitudinalModel::acceleration, without the learned part."""
    mu = np.clip(mu, 0.02, 1.5)
    f_limit = adhesion_limit(mu)
    f_net = drive_force - brake_force - resistance(v, grade, curvature)
    f_net = np.clip(f_net, -f_limit, f_limit)
    a = f_net / M_EFF
    if residual is not None:
        a = a + residual
    return a


# --- traction_model.cpp ------------------------------------------------------

# config/params.yaml, section traction
TORQUE_CURVE_U = (0.0, 0.05, 0.15, 0.3, 0.5, 0.7, 0.85, 1.0)
TORQUE_CURVE_TAU = (0.0, 0.35, 0.62, 0.8, 0.91, 0.97, 1.0, 1.0)
MAX_TRACTIVE_EFFORT_N = 90000.0
MAX_BRAKE_EFFORT_N = 120000.0
RATED_POWER_W = 300000.0
CONSTANT_POWER_LIMIT = True
ACTUATOR_TAU_S = 0.20
BRAKE_TAU_S = 0.20
U_MIN, U_MAX = -1.0, 1.0
DRIVELINE_EFFICIENCY = 0.9


class FirstOrderLag:
    """signal_filter.hpp, exact."""

    __slots__ = ("tau", "y")

    def __init__(self, tau_s: float) -> None:
        self.tau = tau_s
        self.y = 0.0

    def step(self, target: float, dt: float) -> float:
        # A single non-finite target would otherwise poison y permanently: every
        # later step keeps the NaN, and callers that guard with max(0.0, y) turn
        # that into a silent 0.0 rather than an error. Hold the last value.
        if not np.isfinite(target):
            return self.y
        if self.tau <= 1e-6:
            self.y = target
            return self.y
        a = dt / (self.tau + dt)
        self.y += a * (target - self.y)
        return self.y

    def run(self, target: np.ndarray, dt: float) -> np.ndarray:
        out = np.empty(target.size, dtype=np.float64)
        for i, t in enumerate(target):
            out[i] = self.step(float(t), dt)
        return out


def shape_factor(u: np.ndarray) -> np.ndarray:
    """TractionModel::shapeFactor, linear interpolation of the torque curve."""
    x = np.clip(u, 0.0, 1.0)
    return np.interp(x, TORQUE_CURVE_U, TORQUE_CURVE_TAU)


def power_limit(v: np.ndarray) -> np.ndarray:
    if not CONSTANT_POWER_LIMIT:
        return np.ones_like(v)
    vv = np.maximum(np.abs(v), 0.5)
    if RATED_POWER_W <= 0.0:
        return np.ones_like(v)
    return np.minimum(1.0, RATED_POWER_W / (MAX_TRACTIVE_EFFORT_N * vv))


def traction(u: np.ndarray, v: np.ndarray, dt: float = 0.02):
    """TractionModel::step over a series. Returns (drive, brake, net).

    u is the normalised driver command in [-1, 1]; u >= 0 drives, u < 0 brakes.
    The two lags are separate objects in C++ and only one is stepped per sample,
    so the idle one is not dragged along.
    """
    u = np.asarray(u, dtype=np.float64)
    u = np.where(np.isfinite(u), np.clip(u, U_MIN, U_MAX), 0.0)
    v = np.asarray(v, dtype=np.float64)
    n = u.size
    drive = np.zeros(n, dtype=np.float64)
    brake = np.zeros(n, dtype=np.float64)
    lag_d = FirstOrderLag(ACTUATOR_TAU_S)
    lag_b = FirstOrderLag(BRAKE_TAU_S)
    for i in range(n):
        if u[i] >= 0.0:
            f_cmd = shape_factor(np.array([u[i]]))[0] * MAX_TRACTIVE_EFFORT_N * power_limit(
                np.array([v[i]])
            )[0]
            drive[i] = max(0.0, lag_d.step(f_cmd, dt))
            brake[i] = 0.0
        else:
            f_cmd = shape_factor(np.array([-u[i]]))[0] * MAX_BRAKE_EFFORT_N
            brake[i] = max(0.0, lag_b.step(f_cmd, dt))
            drive[i] = 0.0
    return drive, brake, drive - brake


def notch_to_u(position: np.ndarray) -> np.ndarray:
    """DriverControllerCommand.position (-15..15) to normalised u in [-1, 1].

    Non-finite samples are forward filled and then zeroed. Left as NaN they
    would poison np.gradient over the whole series, and a NaN in `u` is a NaN
    training row: the wheel odometry has nothing to compare it against, so the
    command is the last known value by definition.
    """
    p = np.asarray(position, dtype=np.float64)
    finite = np.isfinite(p)
    if not finite.all():
        idx = np.where(finite, np.arange(p.size), 0)
        np.maximum.accumulate(idx, out=idx)
        p = p[idx]
        p[~np.isfinite(p)] = 0.0
    return np.clip(p / 15.0, -1.0, 1.0)
