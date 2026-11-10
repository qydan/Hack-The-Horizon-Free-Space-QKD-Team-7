"""[OPEN] Known-range brightness reference (I_cal), with the exposure gain recorded.

Sim support: ``scenarios/calib_brightness.yaml``. Output keys: ``config/calibration.yaml: preflight_brightness.*``.
Rig behaviour: same for both rigs.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any


def measure_reference(source: Any, range_m: float, n_frames: int) -> dict[str, Any]:
    """Measure the beacon intensity at a known range; return I_cal and the exposure gain estimate."""
    raise NotImplementedError


def save(result: dict[str, Any], path: Path) -> None:
    """Write the preflight_brightness section."""
    raise NotImplementedError
