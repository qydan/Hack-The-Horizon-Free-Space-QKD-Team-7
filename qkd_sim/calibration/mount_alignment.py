"""[OPEN] Camera-to-mount (rig_fixed) or camera-to-gimbal (rig_gimbal) alignment from a static beacon at known positions.

Sim support: ``scenarios/calib_alignment.yaml``. Output keys: ``config/calibration.yaml: mount_alignment.*``.
Rig behaviour: rig_fixed estimates the tripod mount error; rig_gimbal estimates the camera-to-tilt-link offset using gimbal readback.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any


def measure(source: Any, gimbal: Any, beacon_positions_m: list[list[float]]) -> list[dict[str, Any]]:
    """Measure the beacon direction for each known position (and the gimbal attitude)."""
    raise NotImplementedError


def solve(measurements: list[dict[str, Any]], intrinsics: dict[str, Any]) -> dict[str, Any]:
    """Solve for the yaw / pitch / roll alignment angles."""
    raise NotImplementedError


def save(result: dict[str, Any], path: Path) -> None:
    """Write the mount_alignment section."""
    raise NotImplementedError
