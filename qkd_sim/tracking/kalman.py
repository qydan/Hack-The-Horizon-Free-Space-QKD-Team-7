"""[OPEN] Constant-velocity Kalman filter in world angles.

Inputs:  measurements (world angle = mount/gimbal angle + pixel offset) with their row times.
Outputs: :class:`TrackState` (azimuth/elevation, rates, covariance).
Config:  ``tracking.yaml: kalman.*`` (process_noise_rad2_per_s3, measurement_noise_rad, mahalanobis_gate).

Rig behaviour: rig_fixed uses the constant nominal mount angles from ``FixedMount.read()``; rig_gimbal adds the
gimbal readback angles, which are IMU based with noise and a slow pan drift (0.2 deg/min random walk, no
magnetometer) that the filter design has to tolerate.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class TrackState:
    """Filter state."""
    t: float
    x: np.ndarray
    P: np.ndarray


class AngleKalman:
    """Constant-velocity Kalman filter on (az, el, az_rate, el_rate)."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        """Store configuration."""
        raise NotImplementedError

    def predict(self, t: float) -> TrackState:
        """Propagate the state to time ``t``."""
        raise NotImplementedError

    def update(self, t: float, z_rad: np.ndarray) -> bool:
        """Apply a measurement; return False if rejected by the Mahalanobis gate."""
        raise NotImplementedError

    def state(self) -> TrackState:
        """Current state."""
        raise NotImplementedError
