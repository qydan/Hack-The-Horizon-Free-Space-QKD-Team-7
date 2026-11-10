"""[OPEN] Capture-to-frame latency from a beacon switched at a known sim time.

Sim support: ``scenarios/calib_latency.yaml``. Output keys: ``config/calibration.yaml: latency.capture_to_frame_s``.
Rig behaviour: same for both rigs.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any


def estimate_latency(frames: list[Any], switch_times_s: list[float]) -> float:
    """Estimate the latency from the first frame showing each switch."""
    raise NotImplementedError


def save(latency_s: float, path: Path) -> None:
    """Write the latency section."""
    raise NotImplementedError
