"""Action 5 Pro post-processing chain applied to rendered images (section 8.1, steps 1-5).

  1. Rolling shutter: each output frame takes row band k from the sub-frame rendered nearest the band's
     readout time (``bands`` bands over ``readout_s``). ``fidelity: fast`` bypasses this.
  2. Auto-exposure: slow gain loop toward a target mean brightness (applied in linear light), clip at 255.
  3. MJPG: JPEG encode at the configured quality, then decode.
  4. Latency: frames are held until t_capture + latency (sim time).
  5. Output BGR uint8.

Pure numpy/OpenCV; no Gazebo. ``GzAction5FrameSource`` feeds it rendered images.

Timestamp convention: ``t_capture_sim`` is the readout start of the first row (band 0) of the frame.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


@dataclass
class SubFrame:
    image_rgb: np.ndarray        # H x W x 3 uint8 (as rendered, RGB)
    t_sim: float


class RollingShutterAssembler:
    """Collects sub-frames and emits assembled frames at the nominal frame rate.

    Output frame k nominally starts readout at ``t0 + k / fps``. Band b (rows b*H/bands ... (b+1)*H/bands)
    is taken from the sub-frame closest in time to ``t_k + b * readout / bands``.
    """

    def __init__(self, fps: float, readout_s: float, bands: int, height: int, t0: float | None = None):
        self.period = 1.0 / fps
        self.readout = readout_s
        self.bands = int(bands)
        self.band_dt = readout_s / bands
        self.rows = [(round(b * height / bands), round((b + 1) * height / bands)) for b in range(bands)]
        self.t0 = t0
        self.k = 0
        self.buf: deque[SubFrame] = deque(maxlen=4 * bands + 64)

    def band_times(self, k: int) -> list[float]:
        tk = self.t0 + k * self.period
        return [tk + b * self.band_dt for b in range(self.bands)]

    def push(self, sub: SubFrame) -> list[tuple[np.ndarray, float]]:
        if self.t0 is None:
            self.t0 = sub.t_sim
        self.buf.append(sub)
        out = []
        while True:
            times = self.band_times(self.k)
            # wait until a sub-frame at or past the last band time has arrived (so "nearest" is decidable)
            if self.buf[-1].t_sim + 1e-9 < times[-1]:
                break
            frame = np.empty_like(self.buf[-1].image_rgb)
            for (r0, r1), tb in zip(self.rows, times):
                nearest = min(self.buf, key=lambda s: abs(s.t_sim - tb))
                frame[r0:r1] = nearest.image_rgb[r0:r1]
            out.append((frame, times[0]))
            self.k += 1
            # drop sub-frames that can no longer be needed
            t_keep = self.band_times(self.k)[0] - 2 * self.band_dt
            while len(self.buf) > 1 and self.buf[1].t_sim < t_keep:
                self.buf.popleft()
        return out


class FramePacer:
    """Fast fidelity: one render per frame; keeps renders on the nominal 1/fps grid (drops extras)."""

    def __init__(self, fps: float):
        self.period = 1.0 / fps
        self.t_next: float | None = None

    def push(self, sub: SubFrame) -> list[tuple[np.ndarray, float]]:
        if self.t_next is None:
            self.t_next = sub.t_sim
        if sub.t_sim + 0.5 * self.period < self.t_next:
            return []
        self.t_next = max(self.t_next + self.period, sub.t_sim + 0.5 * self.period)
        return [(sub.image_rgb, sub.t_sim)]


@dataclass
class AutoExposure:
    """First-order loop in log-gain: g <- g * (target / mean)^(dt / tau), gain applied in linear light."""
    target_mean: float
    tau_s: float
    gain_min: float
    gain_max: float
    gamma: float = 2.2
    gain: float = 1.0
    locked: bool = False
    _t_last: float | None = None
    _lut_cache: dict = field(default_factory=dict)

    @classmethod
    def from_cfg(cls, e: dict[str, Any]) -> "AutoExposure":
        locked = e["mode"] == "locked"
        return cls(float(e["target_mean"]), float(e["tau_s"]), float(e["gain_min"]), float(e["gain_max"]),
                   float(e.get("gamma", 2.2)), float(e["locked_gain"] if locked else e["gain_initial"]), locked)

    def _lut(self, gain: float) -> tuple[np.ndarray, np.ndarray]:
        key = round(gain, 4)
        if key not in self._lut_cache:
            if len(self._lut_cache) > 512:
                self._lut_cache.clear()
            lin = (np.arange(256) / 255.0) ** self.gamma * gain
            sat = lin > 1.0 + 1e-9
            out = np.clip(lin, 0.0, 1.0) ** (1.0 / self.gamma) * 255.0
            self._lut_cache[key] = (np.round(out).astype(np.uint8), sat)
        return self._lut_cache[key]

    def apply(self, image: np.ndarray, t_sim: float) -> tuple[np.ndarray, float, bool]:
        """Return (image_out, gain_used, saturated). Saturated = some value would exceed 255 after gain."""
        gain = self.gain
        lut, sat_lut = self._lut(gain)
        # OpenCV (fast, releases the GIL) instead of numpy fancy indexing on 6 MB frames
        out = cv2.LUT(image, lut) if cv2 is not None else lut[image]
        saturated = False
        if sat_lut.any():                    # saturated iff some input value is at/above the first clipping level
            v_sat = int(np.argmax(sat_lut))
            saturated = bool(int(image.max()) >= v_sat)
        if not self.locked:
            dt = 0.0 if self._t_last is None else max(0.0, t_sim - self._t_last)
            m = cv2.mean(out)[:3] if cv2 is not None else (out.mean(),)
            mean = max(float(sum(m) / len(m)) / 255.0, 1e-3)
            # loop acts on the linear-light mean so the target is reached exactly at steady state
            err = (self.target_mean / mean) ** self.gamma
            self.gain = float(np.clip(self.gain * err ** (dt / self.tau_s) if dt > 0 else self.gain,
                                      self.gain_min, self.gain_max))
        self._t_last = t_sim
        return out, gain, saturated


def mjpg_roundtrip(image_bgr: np.ndarray, quality: int) -> np.ndarray:
    ok, buf = cv2.imencode(".jpg", image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


@dataclass
class ProcessedFrame:
    image_bgr: np.ndarray
    t_capture_sim: float
    t_delivered_sim: float
    seq: int
    exposure_gain: float
    saturated: bool


class Action5Pipeline:
    """Rendered sub-frames in -> delivered BGR frames out (with latency queue)."""

    def __init__(self, cam_cfg: dict[str, Any], fidelity: str, *, pp_offset_px=(0.0, 0.0), remap=None):
        self.cfg = cam_cfg
        self.fidelity = fidelity
        fps = float(cam_cfg["stream"]["fps"])
        h = int(cam_cfg["stream"]["height_px"])
        rs = cam_cfg["rolling_shutter"]
        if fidelity == "full":
            self.assembler = RollingShutterAssembler(fps, float(rs["readout_s"]), int(rs["bands"]), h)
            self.use_ae = True
            self.use_mjpg = True
        elif fidelity == "fast":
            self.assembler = FramePacer(fps)
            self.use_ae = True           # cheap; keeps brightness behaviour comparable
            self.use_mjpg = False
        else:
            raise ValueError(f"fidelity must be 'full' or 'fast', got {fidelity!r}")
        self.ae = AutoExposure.from_cfg(cam_cfg["exposure"])
        self.quality = int(cam_cfg["compression"]["mjpg_quality"])
        self.latency = float(cam_cfg["latency"]["capture_to_frame_s"])
        self.pp_offset = tuple(map(float, pp_offset_px))
        self.remap = remap                       # (map_x, map_y) for rectilinear_remap, else None
        self.seq = 0
        self.pending: deque[ProcessedFrame] = deque()

    @staticmethod
    def required_render_rate_hz(cam_cfg: dict[str, Any], fidelity: str) -> float:
        if fidelity == "full":
            rs = cam_cfg["rolling_shutter"]
            return float(rs["bands"]) / float(rs["readout_s"])
        return float(cam_cfg["stream"]["fps"])

    def _geometry(self, rgb: np.ndarray) -> np.ndarray:
        if self.remap is not None:
            rgb = cv2.remap(rgb, self.remap[0], self.remap[1], interpolation=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        dx, dy = self.pp_offset
        if dx or dy:
            m = np.float32([[1, 0, dx], [0, 1, dy]])
            rgb = cv2.warpAffine(rgb, m, (rgb.shape[1], rgb.shape[0]), flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        return rgb

    def push_render(self, image_rgb: np.ndarray, t_sim: float) -> list[ProcessedFrame]:
        """Feed one rendered image; returns frames newly *captured* (not yet delivered)."""
        new = []
        for frame_rgb, t_cap in self.assembler.push(SubFrame(image_rgb, t_sim)):
            img = self._geometry(frame_rgb)
            gain, sat = 1.0, False
            if self.use_ae:
                img, gain, sat = self.ae.apply(img, t_cap)
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            if self.use_mjpg:
                bgr = mjpg_roundtrip(bgr, self.quality)
            pf = ProcessedFrame(bgr, t_cap, t_cap + self.latency, self.seq, gain, sat)
            self.seq += 1
            self.pending.append(pf)
            new.append(pf)
        return new

    def pop_delivered(self, t_now_sim: float) -> list[ProcessedFrame]:
        out = []
        while self.pending and self.pending[0].t_delivered_sim <= t_now_sim + 1e-9:
            out.append(self.pending.popleft())
        return out


def lin_brightness(image: np.ndarray, gamma: float = 2.2) -> float:
    return float(np.mean((image / 255.0) ** gamma))


def nyquist_alias_hz(f: float, fs: float) -> float:
    """Apparent frequency of a tone f sampled at fs."""
    return abs(f - fs * round(f / fs))


def frame_count_expected(duration_s: float, fps: float) -> int:
    return int(math.floor(duration_s * fps))
