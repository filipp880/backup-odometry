"""Python port of the C++ slip detector.

Ported from cpp/tram_odometry/src/slip_detector.cpp. Produces the two features
that the ML vector cannot do without: trust and slip_index.

The five detectors, in the order the C++ evaluates them (the source numbers them
0, 1, 2, 3, 4, 5 with 0 sitting in the middle):

  1 dropout / invalid sensor      unconditional penalty 1.0
  2 frozen tachometer             only when motion is expected
  3 front/rear bogie mismatch     ramp between warn 0.25 and fault 0.8
  4 measured acceleration beyond the adhesion ceiling, and wheelspin under torque
  5 wheel lock under braking

The C++ comment for the frozen test is the reason for the `motion_expected`
guard and is worth keeping: the model acceleration is never exactly zero, because
rolling and aerodynamic resistance hold it near -0.07 m/s^2 even in a steady
cruise. Gating on the model instead of the measurement condemns every
constant-speed run, and a tram holding speed emits a bit-exactly constant
tachometer. That mistake zeroed trust for a whole bag.
"""

from __future__ import annotations

import numpy as np

# adhesion section of config/params.yaml
MU_PEAK = 0.22
MU_WET_FACTOR = 0.65
SLIP_PEAK = 0.12
SLIP_HARD = 0.45
BOGIE_MISMATCH_WARN = 0.25
BOGIE_MISMATCH_FAULT = 0.8

WHEELBASE_M = 7.55  # per spec: расстояние между осями вращения тележек

# SlipDetector ctor: trust_lp_(2.0, 0.02), slip_lp_(5.0, 0.02)
TRUST_TAU_S = 2.0
SLIP_TAU_S = 5.0

# kAccelBeyondAdhesion
A_BEYOND_HARD = 2.5
EXCESS_GATE = 0.15
EXCESS_FULL = 0.8

# test 4, wheelspin
UTILISATION_GATE = 0.85
K_ERR_GATE = 0.6
K_ERR_FULL = 1.5
TRANSIENT_A = 1.5
TRANSIENT_FACTOR = 0.5

# test 5, lock
LOCK_FULL = 0.6
K_DEV_GATE = 0.6
K_DEV_FULL = 1.8
BRAKE_GATE_N = 100.0

REASONS = (
    "none",
    "dropout",
    "frozen",
    "bogie_mismatch",
    "yaw_mismatch",
    "accel_beyond_adhesion",
    "torque_accel_conflict",
)


def ramp(x: float, lo: float, hi: float) -> float:
    """Linear ramp, matching the anonymous-namespace helper in the C++."""
    if hi <= lo:
        return 1.0 if x > hi else 0.0
    return float(np.clip((x - lo) / (hi - lo), 0.0, 1.0))


def _lp(tau: float, y: float, target: float, dt: float) -> float:
    if tau <= 1e-6:
        return target
    a = dt / (tau + dt)
    return y + a * (target - y)


class SlipDetector:
    def __init__(self, mu: float = MU_PEAK) -> None:
        self.reset()
        self.mu = mu

    def reset(self) -> None:
        self.trust = 0.5
        self.slip_index = 0.0
        self.slip_ratio = 0.0
        self.reason = 0
        self._t = TRUST_TAU_S
        self._s = SLIP_TAU_S

    def update(
        self,
        dt: float,
        *,
        v_front: float,
        v_rear: float,
        a_wheel: float,
        a_model: float,
        drive_force: float,
        brake_force: float,
        adhesion_limit: float,
        effective_mass: float,
        yaw_rate: float = 0.0,
        front_valid: bool = True,
        rear_valid: bool = True,
        driver_valid: bool = True,
        dropout: bool = False,
        frozen_front: bool = False,
        frozen_rear: bool = False,
    ) -> float:
        if dt <= 0.0:
            dt = 0.02

        penalty = 0.0
        slip_evidence = 0.0
        reason = 0

        def note(p: float, r: int, slip_p: float = 0.0) -> None:
            nonlocal penalty, slip_evidence, reason
            if p > penalty:
                penalty = p
                reason = r
            slip_evidence = max(slip_evidence, slip_p)

        # --- 1. dropout / missing sensors
        if dropout or not front_valid or not rear_valid:
            note(1.0, 1 if dropout else 0)

        # --- 2. frozen sensors, only when motion is expected
        motion_expected = abs(a_wheel) > 0.05 or drive_force > 1.0 or brake_force > 1.0
        if motion_expected:
            if frozen_front:
                note(1.0, 2)
            if frozen_rear:
                note(1.0, 2)

        # --- 3. bogie mismatch
        if front_valid and rear_valid:
            mismatch = abs(v_front - v_rear)
            kinematic = 2.0 * WHEELBASE_M * abs(yaw_rate)
            note(ramp(mismatch, BOGIE_MISMATCH_WARN, BOGIE_MISMATCH_FAULT), 3)
            if abs(yaw_rate) > 1e-3:
                p = ramp(
                    abs(mismatch - kinematic), BOGIE_MISMATCH_WARN, BOGIE_MISMATCH_FAULT
                )
                note(p, 4, p)

        # --- 0. kinematic envelope, mu-independent
        if abs(a_wheel) > A_BEYOND_HARD:
            note(1.0, 5, 1.0)

        # --- 4. measured acceleration beyond the adhesion ceiling
        if front_valid and rear_valid and adhesion_limit > 1.0:
            a_ceiling = adhesion_limit / max(1.0, effective_mass)
            excess = abs(a_wheel) - a_ceiling
            if excess > EXCESS_GATE:
                p = ramp(excess, EXCESS_GATE, EXCESS_FULL)
                note(p, 5, p)

        # --- 4b. wheelspin: high torque, little acceleration
        if driver_valid and adhesion_limit > 1.0:
            utilisation = drive_force / adhesion_limit
            if utilisation > UTILISATION_GATE:
                gain_error = a_model - a_wheel
                if abs(gain_error) > K_ERR_GATE and v_front > 0.5:
                    p = ramp(abs(gain_error), K_ERR_GATE, K_ERR_FULL) * ramp(
                        utilisation, UTILISATION_GATE, 1.0
                    )
                    if abs(a_wheel) > TRANSIENT_A:
                        p *= TRANSIENT_FACTOR
                    note(p, 6, p)

        # --- 5. wheel lock under braking
        if brake_force > BRAKE_GATE_N and front_valid and rear_valid and v_front > 0.5:
            lock = ramp(brake_force / max(1.0, adhesion_limit) - 1.0, 0.0, LOCK_FULL)
            dev = ramp(a_model - a_wheel, K_DEV_GATE, K_DEV_FULL)
            p = lock * dev
            if abs(a_wheel) > TRANSIENT_A:
                p *= TRANSIENT_FACTOR
            note(p, 6, p)

        raw_trust = 1.0 - penalty
        raw_slip = max(slip_evidence, penalty)
        self.trust = float(np.clip(_lp(self._t, self.trust, raw_trust, dt), 0.0, 1.0))
        self.slip_index = float(
            np.clip(_lp(self._s, self.slip_index, raw_slip, dt), 0.0, 1.0)
        )
        self.reason = reason

        if adhesion_limit > 1.0 and drive_force > adhesion_limit:
            excess = drive_force / adhesion_limit - 1.0
            self.slip_ratio = min(SLIP_HARD, SLIP_PEAK + excess * 0.3)
        elif brake_force > adhesion_limit and v_front > 0.5:
            excess = brake_force / adhesion_limit - 1.0
            self.slip_ratio = min(SLIP_HARD, SLIP_PEAK + excess * 0.3)
        else:
            self.slip_ratio *= float(np.exp(-dt / 0.5))
        self.slip_ratio = float(np.clip(self.slip_ratio, 0.0, SLIP_HARD))

        return self.trust


def frozen_flags(v: np.ndarray, window: int = 20) -> tuple[np.ndarray, np.ndarray]:
    """Constant-value detector: a stuck sensor holds one value for a whole window."""
    n = v.size
    out = np.zeros(n, dtype=bool)
    if n < window:
        return out, out.copy()
    for i in range(window - 1, n):
        seg = v[i - window + 1 : i + 1]
        if np.isfinite(seg).all() and np.ptp(seg) < 1e-9:
            out[i] = True
    return out, out.copy()
