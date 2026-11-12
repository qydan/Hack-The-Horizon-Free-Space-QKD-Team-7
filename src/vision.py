"""Vision building blocks for the ground tracker.

* ``LockInDetector``   finds pixels that pulse at the beacon's blink rate (13 Hz). Room
                       lights, windows and reflections don't, so they are ignored.
* ``SteadySpotDetector`` finds a constantly-lit LED as a small, isolated red spot (red excess,
                       local-background top-hat, shape and isolation checks); ``ClutterMap``
                       masks known LED-like spots (lamps, indicator LEDs) from the search.
* ``KalmanCV``         constant-velocity filter: smooths the beacon direction and predicts it
                       (used to coast through a brief loss).
* ``tag_pose`` / ``AprilTagPose``  drone tilt (lean) and range from an AprilTag, no drone
                       telemetry needed.
* Search-stage detectors (``BrightBlobDetector``, ``YoloDnnDetector``) that box the drone
  anywhere in the wide view.
* ``VideoCaptureSource`` wraps a real camera (run on the OS that owns it).

All angles are milliradians, pixel coordinates are source pixels.
"""
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


# --------------------------------------------------------------------------- camera model

@dataclass
class CameraModel:
    width: int = 320
    height: int = 240
    hfov_deg: float = 70.0     # stock lens; narrow camera ~ 12 mm lens

    @property
    def mrad_per_px(self) -> float:
        """Angle per pixel at the image centre (pinhole model: 1 / focal length in px).
        Off-centre pixels subtend less; px_to_mrad / mrad_to_px handle that exactly."""
        return 1000.0 / self.focal_px

    @property
    def focal_px(self) -> float:
        return (self.width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

    @property
    def matrix(self) -> np.ndarray:
        f = self.focal_px
        return np.array([[f, 0, self.width / 2.0], [0, f, self.height / 2.0], [0, 0, 1]], dtype=np.float64)

    def px_to_mrad(self, x: float, y: float) -> Tuple[float, float]:
        """Offset of pixel (x, y) from the image centre, in mrad (pan right +, tilt down +)."""
        f = self.focal_px
        return (math.atan((x - self.width / 2.0) / f) * 1000.0,
                math.atan((y - self.height / 2.0) / f) * 1000.0)

    def mrad_to_px(self, pan: float, tilt: float) -> Tuple[float, float]:
        """Inverse of px_to_mrad: pixel position of an angular offset from the image centre."""
        f = self.focal_px
        return (self.width / 2.0 + f * math.tan(pan / 1000.0),
                self.height / 2.0 + f * math.tan(tilt / 1000.0))


# --------------------------------------------------------------------------- lock-in

@dataclass
class Beacon:
    x: float            # sub-pixel centroid, source pixels
    y: float
    amplitude: float    # summed modulation amplitude of the blob (grey levels)
    peak: float


class LockInDetector:
    """Lock-in detection of a pixel that pulses at ``blink_hz``.

    Over a sliding window of ``periods`` blink periods the detector correlates every pixel
    with cos/sin at the blink frequency (after removing the pixel's mean, so steady light
    cancels exactly). The magnitude is the modulation amplitude; the phase is irrelevant,
    so the beacon's clock need not be synchronised with the camera's.
    """

    def __init__(self, blink_hz: float = 13.0, fps: float = 120.0, periods: int = 3,
                 min_amplitude: float = 8.0, rel_threshold: float = 0.5):
        self.f = blink_hz
        self.n = max(4, int(round(periods * fps / blink_hz)))
        self.min_amplitude = min_amplitude
        self.rel_threshold = rel_threshold
        self._buf: deque = deque()
        self._I = self._Q = self._S = None
        self._sc = self._ss = 0.0
        self.amp_map: Optional[np.ndarray] = None

    @property
    def ready(self) -> bool:
        return len(self._buf) >= self.n

    def reset(self) -> None:
        self._buf.clear()
        self._I = self._Q = self._S = None
        self._sc = self._ss = 0.0
        self.amp_map = None

    def push(self, frame_bgr: np.ndarray, t: float) -> None:
        gray = (frame_bgr if frame_bgr.ndim == 2 else cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)).astype(np.float32)
        ph = 2.0 * math.pi * self.f * t
        c, s = math.cos(ph), math.sin(ph)
        if self._I is None:
            self._I = np.zeros_like(gray)
            self._Q = np.zeros_like(gray)
            self._S = np.zeros_like(gray)
        self._I += gray * c
        self._Q += gray * s
        self._S += gray
        self._sc += c
        self._ss += s
        self._buf.append((gray, c, s))
        if len(self._buf) > self.n:
            og, oc, os_ = self._buf.popleft()
            self._I -= og * oc
            self._Q -= og * os_
            self._S -= og
            self._sc -= oc
            self._ss -= os_
        if self.ready:
            m = len(self._buf)
            i = self._I - self._S * (self._sc / m)
            q = self._Q - self._S * (self._ss / m)
            self.amp_map = np.hypot(i, q) * (2.0 / m)

    def detect(self, roi: Optional[Tuple[int, int, int, int]] = None) -> Optional[Beacon]:
        """Find the blinking spot. ``roi`` = (x0, y0, x1, y1) restricts the search."""
        if self.amp_map is None:
            return None
        h, w = self.amp_map.shape
        x0, y0, x1, y1 = (0, 0, w, h) if roi is None else roi
        x0, y0, x1, y1 = max(0, x0), max(0, y0), min(w, x1), min(h, y1)
        if x1 - x0 < 1 or y1 - y0 < 1:
            return None
        sub = self.amp_map[y0:y1, x0:x1]
        peak = float(sub.max())
        if peak < self.min_amplitude:
            return None
        mask = (sub >= self.rel_threshold * peak).astype(np.uint8)
        _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        py, px = np.unravel_index(int(np.argmax(sub)), sub.shape)
        lab = labels[py, px]
        ys, xs = np.nonzero(labels == lab)
        wts = sub[ys, xs].astype(np.float64)
        total = float(wts.sum())
        return Beacon(x=float((xs * wts).sum() / total) + x0, y=float((ys * wts).sum() / total) + y0,
                      amplitude=total, peak=peak)


@dataclass
class SpotCandidate:
    """One compact bright spot seen by :class:`SteadySpotDetector` (full-frame pixels)."""
    x: float
    y: float
    contrast: float          # peak above the local background (feature units: red excess or grey levels)
    amplitude: float         # summed contrast over the core
    area: int                # core pixels
    bbox: Tuple[int, int, int, int]   # x, y, w, h of the core
    score: float = 0.0
    reason: str = ""         # why it was rejected ("" = valid)

    @property
    def valid(self) -> bool:
        return not self.reason

    def as_beacon(self) -> Beacon:
        return Beacon(self.x, self.y, self.amplitude, self.contrast)


class SteadySpotDetector:
    """Finds a constantly-lit LED as a small, isolated, (red) spot. Works in lit rooms.

    A steady LED has no blink to tell it apart, so the detector piles up every other cue:

    * **Colour** (``color="red"``, default for BGR frames): works on the red excess
      ``R - max(G, B)``. White lamps, windows, screens and reflections have R ~ G ~ B and
      nearly vanish. A clipped (white) LED core is filled in from its red ring.
    * **Local background and line suppression** (``line_angles`` > 0, default): anything containing a
      straight segment of ``bg_kernel_px`` (default ~1/31 of the frame width) in any of ``line_angles``
      directions is background and is subtracted: painted lines (a squash court's red lines), edges,
      line ends, corners and large bright areas (windows, lit walls, posters) all drop out, and an LED
      sitting on a line keeps its contrast above the line. ``line_angles=0`` falls back to a square
      top-hat (large areas only) plus an isolation check.
    * **LED on a red line**: on strongly red pixels the brightness residue is used as well, since the red
      excess barely rises where the red channel is already near clipping.
    * **Emitter test**: the spot must be at least ``min_emission`` brighter (red channel) than the median of
      its surroundings. An LED gives off light; a short piece of red line on a white wall is darker.
    * **Noise floor**: the threshold is at least ``noise_k`` x the robust (MAD) noise of the
      top-hat image, so a grainy high-gain image doesn't produce false spots.
    * **Shape**: core (``rel_threshold`` of the spot's own peak) of ``min_area``..``max_area`` px,
      elongation at most ``max_elongation`` (rejects edges and lines).
    * **Isolation**: a spot whose surroundings are nearly as bright as it is (the corner of a
      window, a gap between bright things) is rejected.
    * **Priors** from the caller: ``exclude`` (a mask of known clutter, see :class:`ClutterMap`)
      and ``prefer`` (the tracker's predicted position, a Gaussian weight on distance).

    Among valid spots the highest ``contrast x prior`` wins.
    """

    def __init__(self, min_contrast: float = 40.0, rel_threshold: float = 0.5,
                 min_area: int = 2, max_area: int = 600, max_candidates: int = 16,
                 color: str = "auto", bg_kernel_px: Optional[int] = None, noise_k: float = 6.0,
                 max_elongation: float = 4.0, max_surround: float = 0.3,
                 clip_level: int = 250, clip_fill_px: int = 2,
                 line_angles: int = 12, red_ratio: float = 0.35, min_emission: float = 10.0,
                 min_shape_area: int = 10):
        self.line_angles, self.red_ratio, self.min_emission = line_angles, red_ratio, min_emission
        self.min_shape_area = min_shape_area
        self._lk_cache: dict = {}
        self.min_contrast, self.rel_threshold = min_contrast, rel_threshold
        self.min_area, self.max_area, self.max_candidates = min_area, max_area, max_candidates
        if color not in ("auto", "red", "gray"):
            raise ValueError(f"color must be auto, red or gray, not {color!r}")
        self.color, self.bg_kernel_px, self.noise_k = color, bg_kernel_px, noise_k
        self.max_elongation, self.max_surround = max_elongation, max_surround
        self.clip_level, self.clip_fill_px = clip_level, clip_fill_px

    # ------------------------------------------------------------------ feature image

    def uses_color(self, frame: np.ndarray) -> bool:
        return frame.ndim == 3 and self.color in ("auto", "red")

    def feature(self, frame: np.ndarray) -> np.ndarray:
        """What the detector looks at (float32, same size as the frame): red excess with clipped
        cores filled for colour frames, plain grey otherwise. Before background removal."""
        if not self.uses_color(frame):
            gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return gray.astype(np.float32)
        b, g, r = cv2.split(frame)
        red = cv2.subtract(r, cv2.max(g, b)).astype(np.float32)        # saturating: negative -> 0
        clipped = (r >= self.clip_level).astype(np.uint8)       # red channel saturated: its red excess is unreliable
        if self.clip_fill_px > 0 and clipped.any():
            self._fill_clipped(red, clipped)
        return red

    def _fill_clipped(self, red: np.ndarray, clipped: np.ndarray) -> None:
        """A red LED bright enough to clip is white or yellow in the middle (red channel saturated) with a red ring. Each
        clipped region whose surrounding ring is mostly red takes the ring's mean red excess; a white lamp
        (no red ring) stays 0 even when it touches a red line or poster on one side."""
        n, labels = cv2.connectedComponents(clipped, connectivity=8)
        if n <= 1:
            return
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * self.clip_fill_px + 1,) * 2)   # the halo right next to it
        ring = (cv2.dilate(clipped, k) > 0) & (clipped == 0)
        owner = cv2.dilate(labels.astype(np.float32), k)[ring].astype(np.int64)   # the region each ring pixel borders
        vals = red[ring]
        hot = (vals >= 0.5 * self.min_contrast).astype(np.float64)
        n_all = np.bincount(owner, minlength=n)
        n_red = np.bincount(owner, weights=hot, minlength=n)
        s_red = np.bincount(owner, weights=vals * hot, minlength=n)
        fill = np.where(n_red >= 0.6 * np.maximum(n_all, 1), s_red / np.maximum(n_red, 1), 0.0)
        fill[0] = 0.0
        sel = clipped > 0
        red[sel] = np.maximum(red[sel], fill[labels[sel]].astype(np.float32))

    def _kernel(self, frame_w: int) -> int:
        k = self.bg_kernel_px if self.bg_kernel_px else max(9, int(round(frame_w / 31.0)))
        return k | 1

    def _line_kernels(self, length: int) -> List[np.ndarray]:
        key = (length, self.line_angles)
        if key not in self._lk_cache:
            ks, c = [], length // 2
            for i in range(self.line_angles):
                th = math.pi * i / self.line_angles
                dx, dy = math.cos(th) * c, math.sin(th) * c
                k = np.zeros((length, length), np.uint8)
                cv2.line(k, (int(round(c - dx)), int(round(c - dy))), (int(round(c + dx)), int(round(c + dy))), 1, 1)
                ks.append(k)
            self._lk_cache[key] = ks
        return self._lk_cache[key]

    def _residue(self, img: np.ndarray, k: int) -> np.ndarray:
        """What is left of ``img`` after removing its local background: with line suppression, everything
        that contains a straight segment of ``k`` px in some direction (court lines, edges, line ends,
        corners, large areas); otherwise a k x k square top-hat (large areas only)."""
        if self.line_angles <= 0:
            return cv2.morphologyEx(img, cv2.MORPH_TOPHAT, cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))
        u = img if img.dtype == np.uint8 else np.clip(img, 0, 255).astype(np.uint8)
        bg = np.zeros_like(u)
        for lk in self._line_kernels(k):            # supremum of openings by line segments
            cv2.max(bg, cv2.morphologyEx(u, cv2.MORPH_OPEN, lk), dst=bg)
        return cv2.subtract(u, bg).astype(np.float32)     # 8-bit throughout: saturating, fast

    def contrast_map(self, frame: np.ndarray, kernel_px: Optional[int] = None) -> Tuple[np.ndarray, np.ndarray]:
        raw, con, _ = self._maps(frame, kernel_px)
        return raw, con

    def _maps(self, frame: np.ndarray, kernel_px: Optional[int] = None):
        """(raw, contrast) for a frame or crop, both float32: ``raw`` is the blurred :meth:`feature`,
        ``contrast`` what the detector thresholds (spot contrast above the local background).

        Colour frames: the red-excess residue, plus, on strongly red pixels only, the brightness residue.
        The second term finds the LED where it sits on a red line (the red channel is near clipping there,
        so the red excess hardly rises, but the LED is still much brighter); restricting it to strongly red
        pixels keeps white glints (a varnished floor, a lamp) out. ``kernel_px``: background scale (default
        from the frame width; pass it when ``frame`` is a crop)."""
        k = kernel_px or self._kernel(frame.shape[1])
        raw = cv2.GaussianBlur(self.feature(frame), (5, 5), 0)
        if not self.uses_color(frame):
            return raw, self._residue(raw, k), raw
        r = cv2.GaussianBlur(frame[..., 2], (5, 5), 0).astype(np.float32)     # brightness for the emitter test
        con = self._residue(raw, k)
        if self.line_angles > 0:
            # remove lines from brightness and from red excess separately, then combine. (Gating the grey
            # image first and removing lines once is cheaper, but the gate's edges along each red line pick
            # up the LED's light and split it into two blobs, one on each edge of the line.)
            gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
            res_g = self._residue(gray, k)
            res_g[raw < self.red_ratio * r] = 0.0
            np.maximum(con, res_g, out=con)
        return raw, con, r

    # ------------------------------------------------------------------ candidates

    def candidates(self, frame: np.ndarray, roi: Optional[Tuple[int, int, int, int]] = None,
                   exclude: Optional[np.ndarray] = None, prefer: Optional[Tuple[float, float, float]] = None,
                   keep_rejected: bool = False) -> List[SpotCandidate]:
        """Spots in ``roi`` (x0, y0, x1, y1), best first. ``exclude``: full-frame bool mask of known clutter.
        ``prefer``: (x, y, sigma_px), weights spots by distance to an expected position."""
        h, w = frame.shape[:2]
        x0, y0, x1, y1 = (0, 0, w, h) if roi is None else roi
        x0, y0, x1, y1 = max(0, int(x0)), max(0, int(y0)), min(w, int(x1)), min(h, int(y1))
        if x1 - x0 < 3 or y1 - y0 < 3:
            return []
        k = self._kernel(w)
        m = k // 2 + 3                  # margin so the background near the ROI edge is estimated from real pixels
        mx0, my0, mx1, my1 = max(0, x0 - m), max(0, y0 - m), min(w, x1 + m), min(h, y1 + m)
        raw_full, th_full, bright_full = self._maps(frame[my0:my1, mx0:mx1], k)
        sl = (slice(y0 - my0, y1 - my0), slice(x0 - mx0, x1 - mx0))
        raw, th, bright = raw_full[sl], th_full[sl], bright_full[sl]

        sample = th[::2, ::2]
        med = float(np.median(sample))
        noise = 1.4826 * float(np.median(np.abs(sample - med)))
        thr = max(self.min_contrast, med + self.noise_k * noise)
        if float(th.max()) < thr:
            return []
        mask = (th >= thr).astype(np.uint8)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        # look at the strongest blobs first (by their peak), not the biggest
        peaks = np.zeros(n, np.float32)
        on = labels > 0
        np.maximum.at(peaks, labels[on], th[on])
        order = np.argsort(-peaks[1:])[:self.max_candidates] + 1
        out: List[SpotCandidate] = []
        rh, rw = th.shape
        for i in order:
            bx, by = int(stats[i, cv2.CC_STAT_LEFT]), int(stats[i, cv2.CC_STAT_TOP])
            bw, bh = int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT])
            comp = labels[by:by + bh, bx:bx + bw] == i
            patch = th[by:by + bh, bx:bx + bw]
            pk = float(peaks[i])
            core = comp & (patch >= self.rel_threshold * pk)
            ys, xs = np.nonzero(core)
            area = len(xs)
            wts = patch[ys, xs].astype(np.float64)
            total = float(wts.sum())
            cx, cy = float((xs * wts).sum() / total) + bx, float((ys * wts).sum() / total) + by
            cw, ch = int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1)
            c = SpotCandidate(cx + x0, cy + y0, pk, total, area,
                              (int(xs.min()) + bx + x0, int(ys.min()) + by + y0, cw, ch))
            if area < self.min_area:
                c.reason = "small"
            elif area > self.max_area:
                c.reason = "large"
            elif area >= self.min_shape_area and self._elongation(xs, ys) > self.max_elongation:
                # (a few-pixel blob has no reliable shape: the LED on a line leaves a short bar there)
                c.reason = "elongated"
            elif self._emission(bright, cx, cy, area, rw, rh) < self.min_emission:
                c.reason = "not a light"     # not brighter than its surroundings: paint (a piece of court line)
            elif exclude is not None and exclude[min(h - 1, int(round(c.y))), min(w - 1, int(round(c.x)))]:
                c.reason = "clutter"
            elif self.line_angles <= 0 and self._surround(raw, cx, cy, area, pk, rw, rh) > self.max_surround:
                # (line suppression already removes corners and edges; this check would also reject
                # the LED where it sits on a red line, so it only runs with the square top-hat)
                c.reason = "not isolated"
            c.score = pk
            if prefer is not None:
                d2 = (c.x - prefer[0]) ** 2 + (c.y - prefer[1]) ** 2
                c.score *= math.exp(-d2 / (2.0 * prefer[2] ** 2))
            if c.valid or keep_rejected:
                out.append(c)
        out.sort(key=lambda c: (not c.valid, -c.score))
        return out

    @staticmethod
    def _elongation(xs: np.ndarray, ys: np.ndarray) -> float:
        if len(xs) < 3:
            return 1.0
        cov = np.cov(np.vstack([xs, ys]).astype(np.float64)) + np.eye(2) * (1.0 / 12.0)   # pixel extent
        ev = np.linalg.eigvalsh(cov)
        return float(math.sqrt(ev[1] / max(ev[0], 1e-9)))

    @staticmethod
    def _ring(img, cx, cy, area, w, h) -> Optional[np.ndarray]:
        """Values of ``img`` on a ring just outside a spot of ``area`` px centred on (cx, cy)."""
        r_in = max(3.0, 2.0 * math.sqrt(area / math.pi))
        r_out = r_in + 4.0
        xa, xb = max(0, int(cx - r_out)), min(w, int(cx + r_out) + 2)
        ya, yb = max(0, int(cy - r_out)), min(h, int(cy + r_out) + 2)
        if xb <= xa or yb <= ya:
            return None
        yy, xx = np.mgrid[ya:yb, xa:xb]
        d2 = (xx - cx) ** 2 + (yy - cy) ** 2
        ring = (d2 >= r_in * r_in) & (d2 <= r_out * r_out)
        return img[ya:yb, xa:xb][ring] if ring.any() else None

    @classmethod
    def _surround(cls, raw, cx, cy, area, contrast, w, h) -> float:
        """Fraction of a ring around the spot that is nearly as bright as the spot's peak
        (window corners and edges: high; a real LED with bloom: low)."""
        vals = cls._ring(raw, cx, cy, area, w, h)
        if vals is None:
            return 0.0
        top = float(raw[min(h - 1, int(round(cy))), min(w - 1, int(round(cx)))])
        return float((vals >= top - 0.5 * contrast).mean())

    @classmethod
    def _emission(cls, bright, cx, cy, area, w, h) -> float:
        """How much brighter (red channel, or grey) the spot is than the median of its surroundings. An LED
        gives off light, so it is brighter than whatever is behind it; red paint on a white wall is darker."""
        vals = cls._ring(bright, cx, cy, area, w, h)
        if vals is None:
            return float("inf")
        return float(bright[min(h - 1, int(round(cy))), min(w - 1, int(round(cx)))]) - float(np.median(vals))

    def detect(self, frame: np.ndarray, roi: Optional[Tuple[int, int, int, int]] = None,
               exclude: Optional[np.ndarray] = None,
               prefer: Optional[Tuple[float, float, float]] = None) -> Optional[Beacon]:
        c = self.candidates(frame, roi, exclude, prefer)
        return c[0].as_beacon() if c else None


class ClutterMap:
    """Places in a fixed camera's view where something LED-like sits when the beacon is not there
    (lamps, indicator LEDs, glints). Learn it with the beacon off or out of view; the search stage
    then ignores spots there. Tracking does not use it (the prediction gate handles a lamp the
    drone flies past)."""

    def __init__(self, shape: Tuple[int, int], pad_px: int = 4):
        self.mask = np.zeros(shape[:2], np.uint8)
        self.pad_px = pad_px

    def learn(self, cands: List[SpotCandidate]) -> None:
        for c in cands:
            x, y, bw, bh = c.bbox
            cv2.rectangle(self.mask, (x - self.pad_px, y - self.pad_px),
                          (x + bw + self.pad_px, y + bh + self.pad_px), 1, -1)

    def clear(self) -> None:
        self.mask[:] = 0

    @property
    def count(self) -> int:
        """Number of separate masked regions."""
        return cv2.connectedComponents(self.mask, connectivity=8)[0] - 1

    @property
    def excluded(self) -> np.ndarray:
        return self.mask.astype(bool)


def refine_centroid(frame: np.ndarray, cx: float, cy: float, search_r: int = 12,
                    centroid_r: int = 5, min_contrast: float = 30.0) -> Optional[Tuple[float, float]]:
    """Sub-pixel centre of the beacon in the *current* frame, near (cx, cy).

    The lock-in map averages over a few blink periods, so its position lags a moving
    beacon. The beacon never switches fully off (100% <-> 40%), so every frame still holds
    the spot: find the brightest point near the prediction and take the brightness-weighted
    centroid around it (background removed)."""
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    x0, x1 = max(0, int(cx) - search_r), min(w, int(cx) + search_r + 1)
    y0, y1 = max(0, int(cy) - search_r), min(h, int(cy) + search_r + 1)
    if x1 - x0 < 3 or y1 - y0 < 3:
        return None
    crop = gray[y0:y1, x0:x1].astype(np.float32)
    py, px = np.unravel_index(int(np.argmax(cv2.GaussianBlur(crop, (3, 3), 0))), crop.shape)
    if float(crop[py, px]) - float(np.median(crop)) < min_contrast:
        return None          # no bright spot here: the beacon is dark right now
    bx, by = x0 + px, y0 + py
    xa, xb = max(0, bx - centroid_r), min(w, bx + centroid_r + 1)
    ya, yb = max(0, by - centroid_r), min(h, by + centroid_r + 1)
    sub = gray[ya:yb, xa:xb].astype(np.float64)
    wts = np.clip(sub - float(np.median(crop)), 0.0, None)
    total = float(wts.sum())
    if total <= 0:
        return None
    ys, xs = np.mgrid[ya:yb, xa:xb]
    return float((xs * wts).sum() / total), float((ys * wts).sum() / total)


# --------------------------------------------------------------------------- Kalman

class KalmanCV:
    """Constant-velocity Kalman filter for the beacon direction (two axes, mrad)."""

    def __init__(self, accel_sigma: float = 400.0, meas_sigma: float = 0.2):
        self.q = accel_sigma ** 2      # mrad/s^2 process noise
        self.r = meas_sigma ** 2
        self.x: Optional[np.ndarray] = None   # [px, py, vx, vy]
        self.P: Optional[np.ndarray] = None

    @property
    def initialised(self) -> bool:
        return self.x is not None

    def reset(self) -> None:
        self.x = self.P = None

    def init(self, px: float, py: float) -> None:
        self.x = np.array([px, py, 0.0, 0.0])
        self.P = np.diag([self.r, self.r, 1e4, 1e4])

    def predict(self, dt: float) -> Tuple[float, float]:
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        dt2, dt3 = dt * dt, dt ** 3
        Qb = np.array([[dt3 / 3, dt2 / 2], [dt2 / 2, dt]]) * self.q
        Q = np.zeros((4, 4))
        Q[np.ix_([0, 2], [0, 2])] = Qb
        Q[np.ix_([1, 3], [1, 3])] = Qb
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        return float(self.x[0]), float(self.x[1])

    def update(self, px: float, py: float) -> Tuple[float, float]:
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1.0
        S = H @ self.P @ H.T + self.r * np.eye(2)
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (np.array([px, py]) - H @ self.x)
        self.P = (np.eye(4) - K @ H) @ self.P
        return float(self.x[0]), float(self.x[1])

    @property
    def position(self) -> Tuple[float, float]:
        return float(self.x[0]), float(self.x[1])

    @property
    def velocity(self) -> Tuple[float, float]:
        return float(self.x[2]), float(self.x[3])


# --------------------------------------------------------------------------- pose

@dataclass
class Pose:
    lean_deg: float
    range_m: float


def tag_pose(corners_px: np.ndarray, tag_size_m: float, camera_matrix: np.ndarray) -> Optional[Pose]:
    """Drone tilt and range from the four tag corners (cv2.solvePnP).

    lean_deg is the angle between the tag's normal (the beacon axis, fixed to the drone body)
    and the line of sight to the camera: how far the body-mounted transmitter points away
    from the ground station. range_m is the distance to the tag.
    """
    h = tag_size_m / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    img = np.asarray(corners_px, dtype=np.float64).reshape(4, 2)
    ok, rvec, tvec = cv2.solvePnP(obj, img, camera_matrix, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    normal = R[:, 2]
    t = tvec.reshape(3)
    rng = float(np.linalg.norm(t))
    cosang = abs(float(np.dot(normal, -t / rng)))
    return Pose(lean_deg=math.degrees(math.acos(min(1.0, cosang))), range_m=rng)


def range_from_tag_pixels(focal_px: float, tag_size_m: float, tag_px: float) -> float:
    """range = focal length x tag size / pixels (turns angular drift into metres)."""
    return focal_px * tag_size_m / tag_px


class AprilTagPose:
    """Detects an AprilTag (36h11) with OpenCV's aruco module and returns its pose.
    Needs opencv-contrib-python. Use the narrow camera: an 8-10 cm tag spans about 90 px
    at 3 m on a 12 mm lens, but only about 25 px on the wide camera (too few for pose)."""

    def __init__(self, camera: CameraModel, tag_size_m: float = 0.09):
        self.camera = camera
        self.tag_size_m = tag_size_m
        self._det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
                                            cv2.aruco.DetectorParameters())

    def __call__(self, frame: np.ndarray) -> Optional[Pose]:
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._det.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return None
        return tag_pose(corners[0], self.tag_size_m, self.camera.matrix)


# --------------------------------------------------------------------------- search stage

@dataclass
class Box:
    cx: float
    cy: float
    w: float
    h: float
    score: float = 1.0


class BrightBlobDetector:
    """Stand-in for the drone detection model: boxes the largest blob brighter than
    ``threshold`` (works on the synthetic scene; the real search stage uses YoloDnnDetector)."""

    def __init__(self, threshold: int = 60, min_area: int = 30):
        self.threshold, self.min_area = threshold, min_area

    def __call__(self, frame: np.ndarray) -> Optional[Box]:
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mask = (gray >= self.threshold).astype(np.uint8)
        n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if n <= 1:
            return None
        best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        if stats[best, cv2.CC_STAT_AREA] < self.min_area:
            return None
        return Box(float(cents[best][0]), float(cents[best][1]), float(stats[best, cv2.CC_STAT_WIDTH]),
                   float(stats[best, cv2.CC_STAT_HEIGHT]))


def parse_yolo_output(out: np.ndarray, in_size: int, frame_w: int, frame_h: int,
                      conf: float = 0.4, nms: float = 0.45) -> Optional[Box]:
    """Parse a YOLOv5 ([1, N, 5+C]) or YOLOv8 ([1, 4+C, N]) ONNX output into the best box."""
    o = np.squeeze(out, 0)
    if o.shape[0] < o.shape[1]:          # v8 layout: [4+C, N] -> [N, 4+C]
        o = o.T
        scores = o[:, 4:].max(axis=1)
    else:                                # v5 layout: [N, 5+C] with objectness
        scores = o[:, 4] * (o[:, 5:].max(axis=1) if o.shape[1] > 5 else 1.0)
    keep = scores >= conf
    if not keep.any():
        return None
    o, scores = o[keep], scores[keep]
    xs, ys = frame_w / in_size, frame_h / in_size
    boxes = [[float((r[0] - r[2] / 2) * xs), float((r[1] - r[3] / 2) * ys), float(r[2] * xs), float(r[3] * ys)]
             for r in o]
    idx = cv2.dnn.NMSBoxes(boxes, scores.astype(float).tolist(), conf, nms)
    if len(idx) == 0:
        return None
    i = int(np.array(idx).flatten()[0])
    x, y, w, h = boxes[i]
    return Box(x + w / 2, y + h / 2, w, h, float(scores[i]))


class YoloDnnDetector:
    """Small YOLO-style drone detector run through OpenCV's DNN module (ONNX file)."""

    def __init__(self, model_path: str, in_size: int = 640, conf: float = 0.4):
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.in_size, self.conf = in_size, conf

    def __call__(self, frame: np.ndarray) -> Optional[Box]:
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (self.in_size, self.in_size), swapRB=True)
        self.net.setInput(blob)
        return parse_yolo_output(self.net.forward(), self.in_size, w, h, self.conf)


# --------------------------------------------------------------------------- real camera

class VideoCaptureSource:
    """Wraps cv2.VideoCapture. Run on the OS that owns the camera (Windows Python, not WSL).
    Lock exposure, or auto-exposure will hide intensity changes."""

    def __init__(self, device=0, width=1280, height=720, fps=120, manual_exposure: Optional[float] = -6,
                 backend: Optional[int] = None, fourcc: Optional[str] = "MJPG",
                 proc_width: Optional[int] = None, duration: Optional[float] = None):
        self.cap = cv2.VideoCapture(device) if backend is None else cv2.VideoCapture(device, backend)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open camera {device}")
        self.proc_width = proc_width          # downscale each frame to this width (None = keep)
        self.duration = duration              # stop after this many seconds (None = until closed)
        self._t0: Optional[float] = None
        if fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        if manual_exposure is not None:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)   # V4L2 manual
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)      # DirectShow manual
            self.cap.set(cv2.CAP_PROP_EXPOSURE, manual_exposure)

    def restart(self, duration: Optional[float] = None) -> None:
        """Restart the capture clock with a new run length (e.g. after a calibration pass)."""
        self.duration = duration
        self._t0 = None

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        t = time.monotonic()
        if self._t0 is None:
            self._t0 = t
        if self.duration is not None and t - self._t0 > self.duration:
            return None
        if self.proc_width and frame.shape[1] > self.proc_width:
            s = self.proc_width / frame.shape[1]
            frame = cv2.resize(frame, (self.proc_width, int(round(frame.shape[0] * s))),
                               interpolation=cv2.INTER_AREA)
        return frame, t

    def release(self):
        self.cap.release()


class VideoFileSource:
    """Replays a recorded flight video through the tracker, frame by frame.

    Timestamps come from the file (frame index / file fps), not the wall clock, so a replay
    gives the same telemetry however fast the machine runs it. ``restart`` does not rewind:
    a ``--calibrate`` or ``--clutter-learn`` pass uses the start of the clip and tracking
    carries on from there (record the calm hover / beacon-off part first)."""

    def __init__(self, path: str, proc_width: Optional[int] = None, duration: Optional[float] = None,
                 start_s: float = 0.0):
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open video {path}")
        self.fps = float(self.cap.get(cv2.CAP_PROP_FPS)) or 30.0
        self.n_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if start_s > 0:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, int(round(start_s * self.fps)))
        self._i = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES))
        self.proc_width = proc_width
        self.duration = duration
        self._t0: Optional[float] = None

    def restart(self, duration: Optional[float] = None) -> None:
        self.duration = duration
        self._t0 = None

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        t = self._i / self.fps
        self._i += 1
        if self._t0 is None:
            self._t0 = t
        if self.duration is not None and t - self._t0 > self.duration:
            return None
        if self.proc_width and frame.shape[1] > self.proc_width:
            s = self.proc_width / frame.shape[1]
            frame = cv2.resize(frame, (self.proc_width, int(round(frame.shape[0] * s))),
                               interpolation=cv2.INTER_AREA)
        return frame, t

    def release(self):
        self.cap.release()
