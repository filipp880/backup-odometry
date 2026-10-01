"""Python port of the C++ ESKF observer.

Ported from cpp/tram_odometry/src/observer.cpp. State vector, in the same order
as the C++ kV..kKr indices:

    0 V  filtered body speed, m/s
    1 A  model acceleration, lightly smoothed, m/s^2
    2 S  distance travelled along the path, m
    3 B  wheel scale, b such that z = R*omega reads v/b
    4 Kf front bogie longitudinal slip
    5 Kr rear bogie longitudinal slip

Two details in the C++ are load-bearing and are reproduced deliberately:

  * the slip states are NOT in the wheel measurement equation. They come from
    slipFromExcess(), i.e. from the adhesion model, so putting them in h asserts
    a heuristic is exactly right. With mu mis-estimated the target slip came out
    near 0.2, and h = v/(b(1-k)) would then be a hard 20% speed error, not a
    confidence loss. Slip widens the measurement noise instead.

  * the scale update is formulated as a measurement (y = v_ref - b*v_wheel) and
    not as `b *= ratio`. The reference arrives at 50 Hz, so hand-applying the
    ratio multiplied b by 0.9709 a hundred times and slammed it into the 0.80
    clamp. Through the update the innovation decays to zero, P(kB,kB) collapses,
    the gain vanishes, and further calls are no-ops.
"""

from __future__ import annotations

import numpy as np

# observer section of config/params.yaml
SIGMA_V = 0.28
SIGMA_A = 1.10
SIGMA_SCALE = 0.010
SIGMA_SLIP = 0.35
R_WHEEL = 0.045
R_WHEEL_DEGRADED = 4.0
R_GNSS_POS = 0.8
R_GNSS_VEL = 0.25
GATE_CHI2 = 5.99
GNSS_MAX_AGE_S = 0.35
GNSS_MIN_SATS = 6
GNSS_INIT_WINDOW_S = 2.5
SCALE_ADAPTIVE_RATE = 0.02
SLIP_ADAPTIVE_RATE = 0.05

# adhesion section
SLIP_PEAK = 0.12
SLIP_HARD = 0.45
C_SLIP = 1.55
B_SLIP = 8.0
E_SLIP = 0.97

N = 6
K_V, K_A, K_S, K_B, K_F, K_R = range(N)


def slip_from_excess(force_demand: float, force_limit: float) -> float:
    """LongitudinalModel::slipFromExcess."""
    if force_limit <= 1e-6:
        return SLIP_HARD
    excess = force_demand / force_limit - 1.0
    if excess <= 0.0:
        return 0.0
    span = max(1e-3, SLIP_HARD - SLIP_PEAK)
    return min(SLIP_HARD, SLIP_PEAK + excess * span)


class Observer:
    def __init__(self, wheel_radius: float = 0.30, mass: float = 38000.0, m_eff: float = 38244.44) -> None:
        self.R = wheel_radius
        self.mass = mass
        self.m_eff = m_eff
        self.reset()

    def reset(self) -> None:
        self.x = np.zeros(N)
        self.x[K_B] = 1.0
        self.P = np.zeros((N, N))
        for i, v in enumerate((25.0, 9.0, 100.0, 0.01, 0.05, 0.05)):
            self.P[i, i] = v
        self.last_innovation = 0.0

    def set_initial_velocity(self, v: float) -> None:
        self.x[K_V] = v
        self.P[K_V, K_V] = max(1e-3, R_GNSS_VEL)

    def set_initial_position(self, s: float) -> None:
        self.x[K_S] = s
        self.P[K_S, K_S] = max(1e-3, R_GNSS_POS)

    def predict(self, dt, a_model, v_new, drive, brake, mu, f_limit) -> None:
        """Observer::predict, with the model call hoisted out.

        The C++ builds a LongitudinalModel::Input and calls acceleration() and
        stepVelocity() internally. Both are pure functions of (v, drive, brake,
        grade, curvature, mu), so the caller computes a_model and v_new and this
        method only does the state propagation, which is the part that matters.
        """
        if dt <= 0.0 or dt > 1.0:
            return
        v_prev = self.x[K_V]
        self.x[K_V] = v_new
        self.x[K_A] = 0.5 * (self.x[K_A] + a_model)
        self.x[K_S] += 0.5 * (v_prev + self.x[K_V]) * dt

        kappa_target = slip_from_excess(max(drive, brake), f_limit)
        relax = min(1.0, dt / 0.2)
        self.x[K_F] += (kappa_target - self.x[K_F]) * relax
        self.x[K_R] += (kappa_target - self.x[K_R]) * relax

        F = np.eye(N)
        F[K_S, K_V] = dt
        F[K_F, K_V] = -0.05 * dt
        F[K_R, K_V] = -0.05 * dt

        Q = np.zeros((N, N))
        for i, s in ((K_V, SIGMA_V), (K_A, SIGMA_A), (K_S, SIGMA_V), (K_B, SIGMA_SCALE), (K_F, SIGMA_SLIP), (K_R, SIGMA_SLIP)):
            Q[i, i] = s * s * dt

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

        self.x[K_V] = np.clip(self.x[K_V], -20.0, 30.0)
        self.x[K_B] = np.clip(self.x[K_B], 0.80, 1.25)
        self.x[K_F] = np.clip(self.x[K_F], -1.0, 1.0)
        self.x[K_R] = np.clip(self.x[K_R], -1.0, 1.0)

    def scalar_update(self, index, innovation, h, r) -> bool:
        hp = self.P @ h
        S = float(r + h @ hp)
        if not (S > 1e-12):
            return False
        # Physical bound applied before the chi-square gate. S carries the whole
        # covariance, so while P is large the gate opens and NIS stays small even
        # for an absurd residual: the filter would believe anything because it
        # believes it knows nothing.
        max_innov = 50.0 if index == K_S else 10.0
        if abs(innovation) > max_innov:
            self.last_innovation = innovation * innovation / S
            return False
        nis = innovation * innovation / S
        self.last_innovation = nis
        if nis > GATE_CHI2:
            return False
        K = hp / S
        self.x += K * innovation
        KH = np.outer(h, h) / S
        A = np.eye(N) - KH
        self.P = A @ self.P @ A.T + (KH * r) @ KH.T
        self.P = 0.5 * (self.P + self.P.T)
        self.x[K_B] = np.clip(self.x[K_B], 0.80, 1.25)
        return True

    def update_wheels(self, omega_front, omega_rear, trust, slip_index) -> bool:
        zf = self.R * omega_front
        zr = self.R * omega_rear
        b = self.x[K_B]
        eps = 1e-3
        safe_b = np.copysign(max(abs(b), eps), b)
        hf = hr = self.x[K_V] / safe_b
        yf = zf - hf
        yr = zr - hr
        h = np.zeros(N)
        h[K_V] = 1.0 / safe_b  # dh/db deliberately zero
        w = float(np.clip(trust, 0.0, 1.0))
        base = R_WHEEL * w + R_WHEEL_DEGRADED * (1.0 - w)
        r = base * (1.0 + 4.0 * slip_index)
        ok_f = self.scalar_update(K_V, yf, h, r)
        ok_r = self.scalar_update(K_V, yr, h, r)
        return ok_f or ok_r

    def calibrate_scale_from_velocity(self, v_ref, v_wheel) -> None:
        if not (np.isfinite(v_ref) and np.isfinite(v_wheel)):
            return
        if abs(v_wheel) < 1.0:
            return
        ratio = v_ref / v_wheel
        if not (0.5 < ratio < 2.0):
            return
        y = v_ref - self.x[K_B] * v_wheel
        h = np.zeros(N)
        h[K_B] = v_wheel
        self.scalar_update(K_B, y, h, R_GNSS_VEL)

    def update_gnss_velocity(self, v_meas) -> bool:
        h = np.zeros(N)
        h[K_V] = 1.0
        return self.scalar_update(K_V, v_meas - self.x[K_V], h, R_GNSS_VEL)

    def update_gnss_path(self, s_meas, s_meas_cov) -> bool:
        h = np.zeros(N)
        h[K_S] = 1.0
        return self.scalar_update(K_S, s_meas - self.x[K_S], h, max(1e-3, s_meas_cov))

    def calibrate_scale(self, s_odometry, s_reference) -> None:
        if abs(s_odometry) < 1.0:
            return
        ratio = s_reference / s_odometry
        if not (0.5 < ratio < 2.0):
            return
        self.x[K_B] = float(np.clip(self.x[K_B] * ratio, 0.80, 1.25))
        self.P[K_B, K_B] = SIGMA_SCALE * SIGMA_SCALE
        self.x[K_S] = s_reference
        self.P[K_S, K_S] = max(1e-3, R_GNSS_POS)

    def apply_scale_bias(self, log_scale, dt) -> None:
        if not np.isfinite(log_scale) or dt <= 0.0:
            return
        rate = float(np.clip(dt / 0.5, 0.0, 1.0))
        if rate <= 0.0:
            return
        b_prev = self.x[K_B]
        self.x[K_B] = float(np.clip(b_prev * np.exp(log_scale * rate), 0.80, 1.25))
        applied = abs(np.log(self.x[K_B] / max(1e-6, b_prev)))
        if applied > 1e-9:
            self.P[K_B, K_B] = max(self.P[K_B, K_B] + applied * applied, 1e-8)


def adapt_friction(mu, trust, slip_index, demand, dt, a_wheel, f_limit, mu_peak=0.22):
    """Estimator::adaptFriction, in place on mu (a float).

    The gate is a kinematic steady state, not a trust threshold. Requiring high
    trust to adapt mu is self-defeating: a mis-estimated mu produces phantom
    slip, the phantom slip drops the trust, and mu is then never corrected. And
    slipping is impossible when the wheels are not accelerating, so a steady
    state is a clean trigger.
    """
    if abs(a_wheel) > 0.2:
        return mu
    if trust < 0.3:
        return mu
    if f_limit < 1.0:
        return mu
    utilisation = demand / f_limit
    rate = SLIP_ADAPTIVE_RATE * dt
    if slip_index > 0.25 and utilisation > 0.9:
        mu *= 1.0 - min(0.5, rate)
    elif slip_index < 0.05 and utilisation > 0.85:
        mu *= 1.0 + min(0.5, rate)
    return float(np.clip(mu, 0.05, mu_peak * 1.2))
