"""Acquisition: find the beacon in the full 140 deg frame (ported from src/vision.py ``LockInDetector``).

Lock-in detection: over a sliding window of ``lockin_window_frames`` the detector correlates every pixel of the
(downscaled) red channel with cos / sin at the blink frequency, using each frame's ``t_capture_sim`` (never the
nominal fps), after removing the pixel's mean so steady light cancels exactly. The magnitude is the modulation
amplitude; the phase is irrelevant, so the beacon clock need not be synchronised with the camera (and the
rolling-shutter row offset is a constant phase per pixel). The red channel is converted to linear light first,
so the amplitude scales with the camera's exposure gain (track.ExposureEstimator divides it back out).

At 30 fps the 7 Hz beacon is well below Nyquist; mains flicker (100 / 120 Hz) aliases to 10 / 0 Hz and the 20-frame
window cancels both exactly (see config/tracking.yaml). The 2 Hz red indicator and steady red lamps don't blink
at 7 Hz and are ignored. ArUco (DICT_4X4_50) is the fallback for the tag target.

Inputs:  frames from a ``FrameSource`` (BGR uint8). Outputs: :class:`Candidate`.
Config:  ``config/tracking.yaml: acquire.*``. Rig behaviour: identical in both rigs (image based; in rig_gimbal
the gimbal is held still while acquiring).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from sim_bridge.frame_source import Frame, FrameSource


@dataclass
class Candidate:
    """Detected beacon candidate. az / el: camera-frame angles from the optical axis, rad, positive left / up
    (the gimbal convention); u / v: full-resolution pixel position."""
    az_rad: float
    el_rad: float
    u_px: float
    v_px: float
    confidence: float
    t: float
    amplitude: float = 0.0
    kind: str = "led"


@dataclass
class Spot:
    """A lock-in blob in full-resolution pixels."""
    x: float
    y: float
    amplitude: float     # summed modulation amplitude of the blob (linear-light red, 0-255 scale, processing pixels)
    peak: float


class LockInDetector:
    """Per-pixel lock-in over a sliding window of frames (linear-light red channel)."""

    def __init__(self, cfg: dict[str, Any], gamma: float = 2.2) -> None:
        """``cfg``: config/tracking.yaml (or its ``acquire`` section)."""
        a = cfg.get("acquire", cfg)
        self.f = float(a["lockin_frequency_hz"])
        self.n = int(a["lockin_window_frames"])
        self.min_amplitude = float(a["red_channel_threshold"])
        self.rel = float(a.get("rel_threshold", 0.5))
        self.proc_width = int(a.get("proc_width_px") or 0)
        self.min_confidence = float(a.get("min_confidence", 0.5))
        self._lut = ((np.arange(256) / 255.0) ** gamma * 255.0).astype(np.float32)
        self.scale = 1.0
        self.reset()

    def reset(self) -> None:
        self._buf: deque = deque()
        self._I = self._Q = self._S = None
        self._sc = self._ss = 0.0
        self._amp: np.ndarray | None = None
        self.t_last: float | None = None

    @property
    def ready(self) -> bool:
        return len(self._buf) >= self.n

    def _prepare(self, image: np.ndarray) -> np.ndarray:
        red = cv2.extractChannel(image, 2) if image.ndim == 3 else image      # contiguous copy (fast)
        h, w = red.shape
        if self.proc_width and w > self.proc_width:
            self.scale = self.proc_width / w
            red = cv2.resize(red, (self.proc_width, int(round(h * self.scale))), interpolation=cv2.INTER_AREA)
        else:
            self.scale = 1.0
        return cv2.LUT(red, self._lut).reshape(red.shape)                     # uint8 -> linear float32

    def push(self, frame: Frame | tuple[np.ndarray, float]) -> None:
        """Add one frame to the window, using ``frame.t_capture_sim``."""
        image, t = (frame.image, frame.t_capture_sim) if isinstance(frame, Frame) else frame
        x = self._prepare(image)
        if self._I is not None and self._I.shape != x.shape:
            self.reset()
        ph = 2.0 * math.pi * self.f * t
        c, s = math.cos(ph), math.sin(ph)
        if self._I is None:
            self._I, self._Q, self._S = np.zeros_like(x), np.zeros_like(x), np.zeros_like(x)
            self._tmp = np.empty_like(x)
        tmp = self._tmp
        np.add(self._I, np.multiply(x, c, out=tmp), out=self._I)        # in place: no temporaries
        np.add(self._Q, np.multiply(x, s, out=tmp), out=self._Q)
        np.add(self._S, x, out=self._S)
        self._sc += c
        self._ss += s
        self._buf.append((x, c, s))
        if len(self._buf) > self.n:
            ox, oc, os_ = self._buf.popleft()
            np.subtract(self._I, np.multiply(ox, oc, out=tmp), out=self._I)
            np.subtract(self._Q, np.multiply(ox, os_, out=tmp), out=self._Q)
            np.subtract(self._S, ox, out=self._S)
            self._sc -= oc
            self._ss -= os_
        self._amp = None
        self.t_last = t

    def amplitude(self, x0: int = 0, y0: int = 0, x1: int | None = None, y1: int | None = None) -> np.ndarray | None:
        """Modulation amplitude of the processing pixels [y0:y1, x0:x1] (only that region is computed: the ROI
        while tracking, the whole frame while searching)."""
        if not self.ready:
            return None
        m = len(self._buf)
        sl = (slice(y0, y1), slice(x0, x1))
        S = self._S[sl]
        i = self._I[sl] - S * np.float32(self._sc / m)
        q = self._Q[sl] - S * np.float32(self._ss / m)
        return cv2.magnitude(i, q) * np.float32(2.0 / m)

    @property
    def amp_map(self) -> np.ndarray | None:
        """Full-frame modulation amplitude per processing pixel (computed on demand, cached until the next push)."""
        if self._amp is None and self.ready:
            self._amp = self.amplitude()
        return self._amp

    # ------------------------------------------------------------------ detection
    def _to_proc_roi(self, roi):
        h, w = self._S.shape
        if roi is None:
            return 0, 0, w, h
        x0, y0, x1, y1 = (int(math.floor(roi[0] * self.scale)), int(math.floor(roi[1] * self.scale)),
                          int(math.ceil(roi[2] * self.scale)), int(math.ceil(roi[3] * self.scale)))
        return max(0, x0), max(0, y0), min(w, x1), min(h, y1)

    def _full(self, xp: float, yp: float) -> tuple[float, float]:
        return (xp + 0.5) / self.scale - 0.5, (yp + 0.5) / self.scale - 0.5

    def _blobs(self, roi=None, max_n: int = 5, min_amplitude: float | None = None) -> list[Spot]:
        if not self.ready:
            return []
        thr = self.min_amplitude if min_amplitude is None else min_amplitude
        x0, y0, x1, y1 = self._to_proc_roi(roi)
        if x1 - x0 < 1 or y1 - y0 < 1:
            return []
        sub = self.amp_map[y0:y1, x0:x1] if (roi is None or self._amp is not None) else self.amplitude(x0, y0, x1, y1)
        peak = float(sub.max())
        if peak < thr:
            return []
        mask = (sub >= max(thr * self.rel, self.rel * peak)).astype(np.uint8)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        out = []
        for lab in range(1, n):
            bx, by, bw, bh = (int(stats[lab, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                                                         cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
            patch = sub[by:by + bh, bx:bx + bw]
            sel = labels[by:by + bh, bx:bx + bw] == lab
            pk = float(patch[sel].max())
            if pk < thr:
                continue
            ys, xs = np.nonzero(sel)
            wts = patch[ys, xs].astype(np.float64)
            tot = float(wts.sum())
            fx, fy = self._full(float((xs * wts).sum() / tot) + bx + x0, float((ys * wts).sum() / tot) + by + y0)
            out.append(Spot(fx, fy, tot, pk))
        out.sort(key=lambda s: -s.peak)
        return out[:max_n]

    def detect(self, roi: tuple[float, float, float, float] | None = None,
               min_amplitude: float | None = None) -> Spot | None:
        """The strongest blinking spot (in ``roi`` = full-resolution (x0, y0, x1, y1) if given); ``min_amplitude``
        overrides the acquisition threshold (the ROI confirmation while tracking uses a lower one)."""
        b = self._blobs(roi, 1, min_amplitude)
        return b[0] if b else None

    def candidates(self, roi=None, camera=None, max_n: int = 5) -> list[Candidate]:
        """Candidates ranked by lock-in amplitude. confidence = peak / (peak + threshold): 0.5 at the threshold."""
        out = []
        for s in self._blobs(roi, max_n):
            dx, dy = camera.pixel_to_angles(s.x, s.y) if camera is not None else (0.0, 0.0)
            out.append(Candidate(-dx, -dy, s.x, s.y, s.peak / (s.peak + self.min_amplitude),
                                 float(self.t_last or 0.0), s.amplitude, "led"))
        return out


# ----------------------------------------------------------------------------- steady beacon
class SteadySpotDetector:
    """A constantly-lit LED as a small bright red spot against a dimmer background (ported from src/vision.py).

    For ``beacon.yaml red_led.modulation.enabled: false``. It cannot tell the LED from any other small bright red
    thing (the steady red lamp distractor, a reflection), so keep the scene dim and restrict the search with
    ``acquire.search_zone``. Candidates are pixels ``steady_min_contrast`` grey levels above the background; each is
    re-thresholded at ``rel_threshold`` of its own peak and kept if its core is ``steady_min_area_px`` ..
    ``steady_max_area_px`` (processing pixels). The brightest wins. Runs on the downscaled red channel."""

    def __init__(self, cfg: dict[str, Any]):
        a = cfg.get("acquire", cfg)
        self.min_contrast = float(a.get("steady_min_contrast", 40.0))
        self.rel = float(a.get("rel_threshold", 0.5))
        self.min_area = int(a.get("steady_min_area_px", 1))
        self.max_area = int(a.get("steady_max_area_px", 400))
        self.proc_width = int(a.get("steady_proc_width_px") or 0)     # 0 = full resolution (a 4 px LED)
        self.channel = str(a.get("steady_channel", "red_excess"))

    def redness(self, image: np.ndarray) -> np.ndarray:
        """``red_excess``: R - max(G, B), so white glints, warm-white panels and the white fan don't count, only
        red things do. ``red``: the raw red channel (src/ used grey)."""
        if image.ndim != 3:
            return image
        r = cv2.extractChannel(image, 2)
        if self.channel != "red_excess":
            return r
        return cv2.subtract(r, cv2.max(cv2.extractChannel(image, 0), cv2.extractChannel(image, 1)))

    def detect(self, image: np.ndarray, roi=None) -> Spot | None:
        """The brightest qualifying spot."""
        c = self.detect_all(image, roi, 1)
        return c[0] if c else None

    def detect_all(self, image: np.ndarray, roi=None, max_n: int = 8) -> list[Spot]:
        """Every qualifying spot, brightest first (the tracker keeps the ones that stay lit: a steady LED does,
        the 2 Hz indicator doesn't)."""
        if roi is not None and not self.proc_width:          # only the zone's pixels need the channel
            h0, w0 = image.shape[:2]
            x0, y0 = max(0, int(roi[0])), max(0, int(roi[1]))
            x1, y1 = min(w0, int(math.ceil(roi[2]))), min(h0, int(math.ceil(roi[3])))
            if x1 - x0 < 3 or y1 - y0 < 3:
                return []
            return [Spot(s.x + x0, s.y + y0, s.amplitude, s.peak) for s in self.detect_all(image[y0:y1, x0:x1], None, max_n)]
        red = self.redness(image)
        h, w = red.shape
        s = self.proc_width / w if self.proc_width and w > self.proc_width else 1.0
        if s < 1.0:
            red = cv2.resize(red, (self.proc_width, int(round(h * s))), interpolation=cv2.INTER_AREA)
        ph, pw = red.shape
        x0, y0, x1, y1 = (0, 0, pw, ph) if roi is None else (
            max(0, int(roi[0] * s)), max(0, int(roi[1] * s)), min(pw, int(math.ceil(roi[2] * s))),
            min(ph, int(math.ceil(roi[3] * s))))
        if x1 - x0 < 3 or y1 - y0 < 3:
            return []
        blur = cv2.GaussianBlur(red[y0:y1, x0:x1].astype(np.float32), (3, 3), 0)
        bg = float(np.median(blur[::4, ::4]))
        if float(blur.max()) - bg < self.min_contrast:
            return []
        n, labels, stats, _ = cv2.connectedComponentsWithStats((blur >= bg + self.min_contrast).astype(np.uint8), 8)
        out = []
        for i in np.argsort(-stats[1:, cv2.CC_STAT_AREA])[:32] + 1:
            bx, by, bw, bh = (int(stats[i, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                                                       cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
            comp = labels[by:by + bh, bx:bx + bw] == i
            patch = blur[by:by + bh, bx:bx + bw]
            pk = float(patch[comp].max())
            core = comp & (patch >= bg + self.rel * (pk - bg))
            if not self.min_area <= int(core.sum()) <= self.max_area:
                continue
            ys, xs = np.nonzero(core)
            wts = (patch[ys, xs] - bg).astype(np.float64)
            tot = float(wts.sum())
            xp, yp = float((xs * wts).sum() / tot) + bx + x0, float((ys * wts).sum() / tot) + by + y0
            out.append(Spot((xp + 0.5) / s - 0.5, (yp + 0.5) / s - 0.5, tot, pk - bg))
        out.sort(key=lambda sp: -sp.peak)
        return out[:max_n]


def zone_box(cfg: dict[str, Any], width: int, height: int):
    """``acquire.search_zone`` (x0, y0, x1, y1 as 0-1 fractions of the frame) in pixels, or None."""
    z = cfg.get("acquire", cfg).get("search_zone")
    if not z:
        return None
    return z[0] * width, z[1] * height, z[2] * width, z[3] * height


# ----------------------------------------------------------------------------- ArUco fallback
def _aruco_detector(cfg: dict[str, Any]):
    a = cfg.get("acquire", cfg)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, str(a.get("aruco_dictionary", "DICT_4X4_50"))))
    return cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())


def find_tag_corners(image: np.ndarray, cfg: dict[str, Any], roi=None) -> np.ndarray | None:
    """Corners (4 x 2, full-resolution px) of the configured ArUco id, optionally inside ``roi``."""
    a = cfg.get("acquire", cfg)
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    x0 = y0 = 0
    if roi is not None:
        h, w = gray.shape
        x0, y0 = max(0, int(roi[0])), max(0, int(roi[1]))
        x1, y1 = min(w, int(roi[2])), min(h, int(roi[3]))
        if x1 - x0 < 8 or y1 - y0 < 8:
            return None
        gray = gray[y0:y1, x0:x1]
    det = cfg.get("_aruco") or _aruco_detector(cfg)
    corners, ids, _ = det.detectMarkers(gray)
    if ids is None:
        return None
    for c, i in zip(corners, ids.flatten()):
        if int(i) == int(a.get("aruco_id", 0)):
            return c.reshape(4, 2).astype(np.float64) + np.array([x0, y0], np.float64)
    return None


def detect_aruco(image: np.ndarray, cfg: dict[str, Any], camera=None, t: float = 0.0, roi=None) -> list[Candidate]:
    """ArUco fallback for the tag target: the configured marker as a candidate (confidence 1)."""
    c = find_tag_corners(image, cfg, roi)
    if c is None:
        return []
    u, v = (float(x) for x in c.mean(axis=0))
    dx, dy = camera.pixel_to_angles(u, v) if camera is not None else (0.0, 0.0)
    return [Candidate(-dx, -dy, u, v, 1.0, t, 0.0, "tag")]


def acquire(source: FrameSource, cfg: dict[str, Any], timeout_s: float, camera=None) -> Candidate | None:
    """Run acquisition until a candidate exceeds ``min_confidence`` or the timeout (frame time) expires.
    ``cfg`` is the full tracking config (load_tracking_cfg)."""
    from tracking.camera import CameraModel
    camera = camera or CameraModel.from_config(cfg)
    det = LockInDetector(cfg)
    a = cfg["acquire"]
    t0 = None
    while True:
        fr = source.read(timeout_s=max(1.0, timeout_s))
        if fr is None:
            return None
        t0 = fr.t_capture_sim if t0 is None else t0
        if a.get("target", "auto") != "tag":
            det.push(fr)
            c = det.candidates(camera=camera, max_n=1)
            if c and c[0].confidence >= det.min_confidence:
                return c[0]
        if a.get("target", "auto") in ("tag", "auto"):
            c = detect_aruco(fr.image, cfg, camera, fr.t_capture_sim)
            if c:
                return c[0]
        if fr.t_capture_sim - t0 > timeout_s:
            return None
