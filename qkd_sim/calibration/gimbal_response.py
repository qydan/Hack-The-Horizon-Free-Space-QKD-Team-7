"""[OPEN] Config B only: step response, rate limit, readback noise and pan drift of the gimbal.

Sim support: ``scenarios/calib_gimbal.yaml``. Output keys: ``config/calibration.yaml: gimbal_response.*``.
Rig behaviour: rig_gimbal only; raise a clear error in rig_fixed.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any


def record(gimbal: Any, duration_s: float) -> list[Any]:
    """Record gimbal readback while the scenario plays its command schedule."""
    raise NotImplementedError


def fit(records: list[Any]) -> dict[str, Any]:
    """Fit natural frequency, damping, rate limit, readback noise and pan drift."""
    raise NotImplementedError


def save(result: dict[str, Any], path: Path) -> None:
    """Write the gimbal_response section."""
    raise NotImplementedError
