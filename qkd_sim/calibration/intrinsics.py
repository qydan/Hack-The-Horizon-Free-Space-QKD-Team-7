"""[OPEN] Fisheye intrinsics (cv2.fisheye model) from checkerboard views across the whole 140 deg field.

Sim support: ``scenarios/calib_intrinsics.yaml``. Output keys: ``config/calibration.yaml: intrinsics.*``.
Rig behaviour: same for both rigs (the camera is shared).
"""
from __future__ import annotations

from pathlib import Path
from typing import Any


def collect_views(source: Any, n_views: int) -> list[Any]:
    """Grab frames with a detected 9x6 checkerboard."""
    raise NotImplementedError


def calibrate(views: list[Any], cfg: dict[str, Any]) -> dict[str, Any]:
    """Run cv2.fisheye.calibrate; return K, D, image size and RMS reprojection error."""
    raise NotImplementedError


def save(result: dict[str, Any], path: Path) -> None:
    """Write the intrinsics section of config/calibration.yaml."""
    raise NotImplementedError
