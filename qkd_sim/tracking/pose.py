"""[OPEN] Tag pose (lean and range) from the ArUco tag.

Inputs:  a frame and the calibrated fisheye camera model.
Outputs: :class:`TagPose` (lean_deg, range_m, rvec, tvec) or None.
Config:  ``tracking.yaml: pose.tag_size_m``.
Known limit: at 3 m the 8 cm tag spans about 21 px, too few for a reliable lean estimate (see README).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from sim_bridge.frame_source import Frame


@dataclass
class TagPose:
    """Tag pose in the camera frame."""
    lean_deg: float
    range_m: float
    rvec: np.ndarray
    tvec: np.ndarray


def estimate_tag_pose(frame: Frame, camera_model: Any, cfg: dict[str, Any]) -> TagPose | None:
    """Detect the tag and estimate its pose."""
    raise NotImplementedError
