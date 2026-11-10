"""[OPEN] Acquisition: find the beacon in the full 140 deg frame.

Inputs:  frames from a ``FrameSource`` (BGR uint8, sim/host timestamps).
Outputs: :class:`Candidate` (azimuth/elevation in the camera frame, pixel position, confidence).
Config:  ``config/tracking.yaml: acquire.*`` (lockin_frequency_hz, lockin_window_frames, red_channel_threshold,
         aruco_dictionary, aruco_id, min_confidence).

Method (to be written): 7 Hz lock-in detection on the red channel over a window of frames (at 30 fps, 7 Hz is
well below Nyquist; mains flicker aliases to 10 Hz / 0 Hz). ArUco (DICT_4X4_50) fallback for the tag target.
Rig behaviour: identical in rig_fixed and rig_gimbal (acquisition is image based; in rig_gimbal the gimbal is
held still while acquiring).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from sim_bridge.frame_source import Frame, FrameSource


@dataclass
class Candidate:
    """Detected beacon candidate."""
    az_rad: float
    el_rad: float
    u_px: float
    v_px: float
    confidence: float
    t: float


class LockInDetector:
    """Per-pixel 7 Hz lock-in over a sliding window of frames (red channel)."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        """Store configuration (tracking.yaml: acquire)."""
        raise NotImplementedError

    def push(self, frame: Frame) -> None:
        """Add one frame to the window, using ``frame.t_capture_sim`` (never the nominal fps)."""
        raise NotImplementedError

    def candidates(self) -> list[Candidate]:
        """Return candidates ranked by lock-in amplitude / confidence."""
        raise NotImplementedError


def detect_aruco(image: np.ndarray, cfg: dict[str, Any]) -> list[Candidate]:
    """ArUco fallback for the tag target."""
    raise NotImplementedError


def acquire(source: FrameSource, cfg: dict[str, Any], timeout_s: float) -> Candidate | None:
    """Run acquisition until a candidate exceeds ``min_confidence`` or the timeout expires."""
    raise NotImplementedError
