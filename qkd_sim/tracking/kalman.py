"""Constant-velocity Kalman filter in world angles (ported from src/vision.py ``KalmanCV``).

Measurements are world angles = gimbal / mount angle at the measurement's row time minus the pixel offset,
stamped with their rolling-shutter row times, so the filter runs on irregular time steps. A measurement whose
Mahalanobis distance exceeds ``mahalanobis_gate`` sigma is rejected (it is not the beacon).

Rig behaviour: rig_fixed uses the constant nominal mount angles from ``FixedMount.read()``; rig_gimbal adds the
gimbal readback, which is IMU based with noise (white, smoothed here) and a slow pan drift (0.2 deg/min random
walk, no magnetometer). The drift moves the world frame itself, so it shows up in ``drift_mrad`` (not in the
residual, which is measured in the image).
Config: ``tracking.yaml: kalman.*``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class TrackState:
    """Filter state: x = [az, el, az_rate, el_rate] (rad, rad/s; az positive left, el positive up)."""
    t: float
    x: np.ndarray
    P: np.ndarray

    @property
    def position(self) -> tuple[float, float]:
        return float(self.x[0]), float(self.x[1])

    @property
    def velocity(self) -> tuple[float, float]:
        return float(self.x[2]), float(self.x[3])


class AngleKalman:
    """Constant-velocity Kalman filter on (az, el, az_rate, el_rate)."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        k = cfg.get("kalman", cfg)
        self.q = float(k["process_noise_rad2_per_s3"])
        self.r = float(k["measurement_noise_rad"]) ** 2
        self.gate = float(k["mahalanobis_gate"])
        self.coast_tau = float(k.get("coast_velocity_tau_s", 0.1))
        self.t: float | None = None
        self.x: np.ndarray | None = None
        self.P: np.ndarray | None = None

    @property
    def initialised(self) -> bool:
        return self.x is not None

    def reset(self) -> None:
        self.t = self.x = self.P = None

    def init(self, t: float, az: float, el: float) -> None:
        self.t = float(t)
        self.x = np.array([az, el, 0.0, 0.0])
        self.P = np.diag([self.r, self.r, 1e-2, 1e-2])

    def predict(self, t: float) -> TrackState:
        """Propagate the state to time ``t`` (no-op for t <= the filter time)."""
        dt = float(t) - self.t
        if dt > 0:
            F = np.eye(4)
            F[0, 2] = F[1, 3] = dt
            qb = np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]]) * self.q
            Q = np.zeros((4, 4))
            Q[np.ix_([0, 2], [0, 2])] = qb
            Q[np.ix_([1, 3], [1, 3])] = qb
            self.x = F @ self.x
            self.P = F @ self.P @ F.T + Q
            self.t = float(t)
        return self.state()

    def update(self, t: float, z_rad: np.ndarray) -> bool:
        """Apply a measurement at time ``t``; return False if rejected by the Mahalanobis gate."""
        self.predict(t)
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1.0
        S = H @ self.P @ H.T + self.r * np.eye(2)
        y = np.asarray(z_rad, dtype=float) - H @ self.x
        if float(y @ np.linalg.solve(S, y)) > self.gate ** 2:
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        return True

    def damp_velocity(self, dt: float) -> None:
        """While coasting: decay the velocity so a guess is not extrapolated far."""
        if self.x is not None and dt > 0:
            self.x[2:] *= math.exp(-dt / self.coast_tau)

    def state(self) -> TrackState:
        return TrackState(float(self.t), self.x.copy(), self.P.copy())
