"""[OPEN] Fine tracking inside a region of interest.

Inputs:  frames from a ``FrameSource`` plus the predicted beacon position.
Outputs: :class:`Measurement` (pixel and angular offset from the boresight, intensity, spot FWHM, SNR,
         saturation flag).
Config:  ``tracking.yaml: track.*`` (roi_size_px, saturation_level, rolling_shutter_readout_s).

Must handle: saturation guard (the camera clips at 255), auto-exposure changes (normalise intensity by the
estimated exposure gain), rolling shutter (row time = t_capture + row / H * readout).
Rig behaviour: same in both rigs.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from sim_bridge.frame_source import Frame


@dataclass
class Measurement:
    """One ROI measurement."""
    t_row: float
    u_px: float
    v_px: float
    dx_rad: float
    dy_rad: float
    intensity: float
    spot_fwhm_px: float
    snr: float
    saturated: bool


class RoiTracker:
    """Centroid tracker in a region of interest."""

    def __init__(self, cfg: dict[str, Any], camera_model: Any) -> None:
        """Store configuration and the calibrated camera model (from config/calibration.yaml)."""
        raise NotImplementedError

    def measure(self, frame: Frame, predicted_uv: tuple[float, float]) -> Measurement | None:
        """Measure the beacon near ``predicted_uv``; None if not found."""
        raise NotImplementedError

    def row_time(self, frame: Frame, v_px: float) -> float:
        """Capture time of image row ``v_px`` given the calibrated rolling-shutter readout."""
        raise NotImplementedError
