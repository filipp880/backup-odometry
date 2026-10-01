from __future__ import annotations

import numpy as np


class VelocityEKF:
    """State x = [v, b, k].

    v true longitudinal speed [m/s]
    b additive acceleration bias (rolling+aero resistance, grade) [m/s^2]
    k wheel-speed scale (effective radius / calibration error)

    process:  v' = v + (kt * A_pos(u, v) - A_neg(u, v) + b) * dt
    measure: z = k * v  (mean of front/rear wheel speeds)
    """

    def __init__(
        self,
        dt: float = 0.02,
        v0: float = 0.0,
        kt: float = 1.0,
        q_v: float = 1e-3,
        q_b: float = 3e-4,
        q_k: float = 2e-6,
        sigma_v: float = 0.4,
        sigma_b: float = 0.1,
        sigma_k: float = 0.01,
        sigma_z: float = 0.05,
    ) -> None:
        self.dt = dt
        self.q_v = q_v
        self.q_b = q_b
        self.q_k = q_k
        self.sigma_z = sigma_z
        self.x = np.array([v0, 0.0, kt])
        self.P = np.diag([sigma_v**2, sigma_b**2, sigma_k**2])
        self.I = np.eye(3)
        self.last_innovation = 0.0
        self.last_innovation_n = 0.0

    def step(
        self,
        a_traction: float,
        a_brake: float,
        z: float,
        z_valid: bool,
        r_scale: float = 1.0,
    ) -> None:
        dt = self.dt
        F = np.eye(3)
        F[1, 1] = 1.0
        Q = np.diag([self.q_v, self.q_b, self.q_k]) * dt
        self.x = self.x + np.array([(a_traction - a_brake) * dt, 0.0, 0.0])
        self.P = F @ self.P @ F.T + Q
        if not z_valid:
            return
        H = np.array([[self.x[2], 0.0, 0.0]])
        R = np.array([[max((self.sigma_z * r_scale) ** 2, 1e-8)]])
        y = float(z - H @ self.x)
        S = float(H @ self.P @ H.T + R)
        K = (self.P @ H.T) / S
        self.x = self.x + (K.ravel() * y)
        A = self.I - np.outer(K.ravel(), H.ravel())
        self.P = A @ self.P @ A.T + np.outer(K.ravel(), K.ravel()) * R[0, 0]
        self.P = 0.5 * (self.P + self.P.T)
        self.x[0] = max(self.x[0], -1.0)
        self.x[2] = float(np.clip(self.x[2], 0.85, 1.15))
        self.last_innovation = y
        self.last_innovation_n = y / np.sqrt(S)

    @property
    def v(self) -> float:
        return float(self.x[0])

    @property
    def b(self) -> float:
        return float(self.x[1])

    @property
    def k(self) -> float:
        return float(self.x[2])
