"""[OPEN] Rolling-shutter readout time from a fast-moving beacon (spinning arm).

Sim support: ``scenarios/calib_rolling_shutter.yaml``. Output keys: ``config/calibration.yaml: rolling_shutter.readout_s``.
Rig behaviour: same for both rigs.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any


def estimate_readout(frames: list[Any], cfg: dict[str, Any]) -> float:
    """Estimate the readout time (s) from the row-dependent skew of the moving beacon."""
    raise NotImplementedError


def save(readout_s: float, path: Path) -> None:
    """Write the rolling_shutter section."""
    raise NotImplementedError
