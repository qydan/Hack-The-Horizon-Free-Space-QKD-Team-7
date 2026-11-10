"""[OPEN] Motion estimates for telemetry: drift, jitter, residual, gimbal lag, lean, range.

Inputs:  track history (measurements, filter states, gimbal readback), optional tag pose.
Outputs: telemetry fields drift_mrad, jitter_mrad, residual_mrad, gimbal_lag_mrad, lean_deg, range_m.
Config:  ``tracking.yaml: motion_estimate.*`` (drift_window_s, jitter_window_s).
Rig behaviour: gimbal_lag_mrad is null in rig_fixed.
"""
from __future__ import annotations

from typing import Any


class MotionEstimator:
    """Sliding-window drift / jitter / residual estimator."""

    def __init__(self, cfg: dict[str, Any], rig: str) -> None:
        """Store configuration."""
        raise NotImplementedError

    def push(self, t: float, measurement: Any, track_state: Any, gimbal_state: Any) -> None:
        """Add the data of one frame."""
        raise NotImplementedError

    def fields(self) -> dict[str, float | None]:
        """Return the motion telemetry fields."""
        raise NotImplementedError
