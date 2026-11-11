"""Fine tracking inside a region of interest (ported from src/vision.py ``refine_centroid`` + the TRACK stage of
src/tracker.py).

Per frame: the lock-in map confirms that the blinking beacon is inside the ROI around the prediction; the lock-in
blob lags a moving beacon (it averages ~0.67 s), so the position is taken from *this* frame: the brightest red
point near the prediction and its background-subtracted, brightness-weighted centroid (the beacon never switches
fully off, 100% <-> 40%, so every frame holds the spot). Then:

* rolling shutter: the measurement time is the row time t_capture + v / H * readout
* saturation guard: ``saturated`` when the spot reaches ``saturation_level`` (the centroid of a clipped spot is
  still fine; its brightness is not)
* auto-exposure: intensity uses the lock-in amplitude (blink-phase independent, linear light) divided by the
  estimated exposure gain (:class:`ExposureEstimator`)

Tag mode (ArUco target): the marker centre in the ROI is the measurement.
Config: ``config/tracking.yaml: track.*``. Rig behaviour: same in both rigs.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from sim_bridge.frame_source import Frame


@dataclass
class Measurement:
    """One ROI measurement."""
    t_row: float
    u_px: float
    v_px: float
    dx_rad: float            # offset from the optical axis, x right +
    dy_rad: float            # y down +
    intensity: float         # raw: lock-in amplitude (LED) or 1.0 (tag); normalised by the tracker
    spot_fwhm_px: float
    snr: float
    saturated: bool
    kind: str = "led"
    corners: np.ndarray | None = None


def refine_centroid(channel: np.ndarray, cx: float, cy: float, search_r: int = 16, centroid_r: int = 8,
                    min_contrast: float = 30.0):
    """Sub-pixel centre of the spot near (cx, cy) in one channel; returns (x, y, stats) or None.
    stats: peak and background grey levels, background noise (MAD), FWHM estimate."""
    h, w = channel.shape
    x0, x1 = max(0, int(cx) - search_r), min(w, int(cx) + search_r + 1)
    y0, y1 = max(0, int(cy) - search_r), min(h, int(cy) + search_r + 1)
    if x1 - x0 < 3 or y1 - y0 < 3:
        return None
    crop = channel[y0:y1, x0:x1].astype(np.float32)
    py, px = np.unravel_index(int(np.argmax(cv2.GaussianBlur(crop, (3, 3), 0))), crop.shape)
    bg = float(np.median(crop))
    if float(crop[py, px]) - bg < min_contrast:
        return None
    bx, by = x0 + px, y0 + py
    xa, xb = max(0, bx - centroid_r), min(w, bx + centroid_r + 1)
    ya, yb = max(0, by - centroid_r), min(h, by + centroid_r + 1)
    sub = channel[ya:yb, xa:xb].astype(np.float64)
    wts = np.clip(sub - bg, 0.0, None)
    tot = float(wts.sum())
    if tot <= 0:
        return None
    ys, xs = np.mgrid[ya:yb, xa:xb]
    mx, my = float((xs * wts).sum() / tot), float((ys * wts).sum() / tot)
    var = float((((xs - mx) ** 2 + (ys - my) ** 2) * wts).sum() / tot) / 2.0
    mad = float(np.median(np.abs(crop - bg))) * 1.4826
    return mx, my, {"peak": float(sub.max()), "bg": bg, "noise": max(mad, 0.5),
                    "fwhm": 2.3548 * float(np.sqrt(max(var, 1e-6)))}


class ExposureEstimator:
    """Relative exposure gain of an auto-exposure camera, from the frames alone.

    With AE on, the background is held near the target brightness, so the gain cannot be read off the image;
    but the AE control law can be replayed on the observed output: g <- g * (target / mean)^(gamma dt / tau)
    (first-order loop in log gain on the linear-light mean, the model the Action 5 emulation uses). Replaying it
    gives the gain up to its unknown starting value (assumed ``gain_initial``). ``mode: locked`` reports a
    constant. Identify ``target_mean`` / ``tau_s`` on the real camera (step the room light, watch the mean)."""

    def __init__(self, cfg: dict[str, Any]):
        e = cfg.get("track", cfg).get("exposure_model", {})
        self.mode = e.get("mode", "auto")
        self.target = float(e.get("target_mean", 0.45))
        self.tau = float(e.get("tau_s", 0.5))
        self.gamma = float(e.get("gamma", 2.2))
        self.gmin, self.gmax = float(e.get("gain_min", 0.125)), float(e.get("gain_max", 16.0))
        self.gain = float(e.get("gain_initial", 1.0))
        self._t_last: float | None = None

    def update(self, image: np.ndarray, t: float) -> float:
        """Gain that produced this frame; then advance the loop with this frame's mean.

        The replay is open loop, so it integrates any bias between the camera's metering and ours at
        gamma / tau ~ 4.4 per second: the mean must be metered exactly as the camera does (full frame, all
        channels; a strided subsample drifted 9% in 14 s). That holds for the emulation; on a real camera whose
        AE law is unknown, lock the exposure (``mode: locked``) or identify the law first."""
        g = self.gain
        if self.mode != "locked":
            m = cv2.mean(image)
            mean = max(sum(m[:3]) / (3.0 if image.ndim == 3 else 1.0) / 255.0, 1e-3)
            dt = 0.0 if self._t_last is None else max(0.0, t - self._t_last)
            if dt > 0:
                self.gain = float(np.clip(self.gain * (self.target / mean) ** (self.gamma * dt / self.tau),
                                          self.gmin, self.gmax))
        self._t_last = t
        return g


class RoiTracker:
    """Centroid tracker in a region of interest."""

    def __init__(self, cfg: dict[str, Any], camera_model: Any, readout_s: float | None = None) -> None:
        """``cfg``: config/tracking.yaml (or its ``track`` section); ``camera_model``: tracking.camera.CameraModel;
        ``readout_s``: calibrated rolling-shutter readout (default ``track.rolling_shutter_readout_s``)."""
        t = cfg.get("track", cfg)
        self.cfg_all = cfg
        self.cam = camera_model
        self.roi = int(t["roi_size_px"])
        self.search_r = int(t.get("refine_search_px", 16))
        self.centroid_r = int(t.get("refine_centroid_px", 8))
        self.min_contrast = float(t.get("min_contrast", 30.0))
        self.sat_level = float(t["saturation_level"])
        self.readout = float(readout_s if readout_s is not None else t["rolling_shutter_readout_s"])
        self.confirm_fraction = float(t.get("lockin_confirm_fraction", 0.5))

    def roi_box(self, uv: tuple[float, float], size: int | None = None) -> tuple[float, float, float, float]:
        h = (size or self.roi) / 2.0
        return uv[0] - h, uv[1] - h, uv[0] + h, uv[1] + h

    def row_time(self, frame: Frame, v_px: float) -> float:
        """Capture time of image row ``v_px`` given the calibrated rolling-shutter readout."""
        h = frame.image.shape[0]
        return frame.t_capture_sim + min(max(v_px, 0.0), h - 1) / h * self.readout

    def measure(self, frame: Frame, predicted_uv: tuple[float, float], lockin=None, kind: str = "led",
                channel: np.ndarray | None = None, min_contrast: float | None = None) -> Measurement | None:
        """Measure the beacon near ``predicted_uv``; None if not found. With ``lockin`` the blinking spot must be
        inside the ROI (rejects lamps and reflections); ``kind="tag"`` measures the ArUco marker instead.
        ``channel``: the single-channel image to centroid on (default the red channel; steady mode passes R - max(G, B))."""
        if kind == "tag":
            return self._measure_tag(frame, predicted_uv)
        img = frame.image
        centres = [predicted_uv]
        amp = 0.0
        if lockin is not None:
            spot = lockin.detect(self.roi_box(predicted_uv), lockin.min_amplitude * self.confirm_fraction)
            if spot is None:
                return None
            centres.append((spot.x, spot.y))
            amp = spot.amplitude
        red = channel if channel is not None else (img[:, :, 2] if img.ndim == 3 else img)
        res = None
        mc = self.min_contrast if min_contrast is None else min_contrast
        for c in centres:
            res = refine_centroid(red, c[0], c[1], self.search_r, self.centroid_r, mc)
            if res is not None:
                break
        if res is None:
            return None
        u, v, st = res
        r = self.centroid_r
        hh, ww = red.shape
        patch = img[max(0, int(v) - r):min(hh, int(v) + r + 1), max(0, int(u) - r):min(ww, int(u) + r + 1)]
        dx, dy = self.cam.pixel_to_angles(u, v)
        if lockin is None:            # steady beacon: linear-light peak above the background (0-255 scale)
            amp = 255.0 * ((st["peak"] / 255.0) ** 2.2 - (st["bg"] / 255.0) ** 2.2)
        return Measurement(self.row_time(frame, v), u, v, dx, dy, amp,
                           st["fwhm"], (st["peak"] - st["bg"]) / st["noise"],
                           bool(patch.size and float(patch.max()) >= self.sat_level), "led")

    def _measure_tag(self, frame: Frame, predicted_uv) -> Measurement | None:
        from tracking.acquire import find_tag_corners
        c = find_tag_corners(frame.image, self.cfg_all, self.roi_box(predicted_uv))
        if c is None:
            return None
        u, v = (float(x) for x in c.mean(axis=0))
        side = float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))
        gray = cv2.cvtColor(frame.image, cv2.COLOR_BGR2GRAY) if frame.image.ndim == 3 else frame.image
        x0, y0 = (int(max(0, m)) for m in c.min(axis=0))
        x1, y1 = (int(m) + 1 for m in c.max(axis=0))
        crop = gray[y0:y1, x0:x1]
        contrast = float(np.percentile(crop, 95) - np.percentile(crop, 5)) if crop.size else 0.0
        dx, dy = self.cam.pixel_to_angles(u, v)
        return Measurement(self.row_time(frame, v), u, v, dx, dy, 1.0, side, contrast / 2.0,
                           bool(crop.size and crop.max() >= self.sat_level), "tag", c)
