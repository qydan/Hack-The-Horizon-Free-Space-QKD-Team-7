"""Camera-based telemetry: LED detection, acquisition/tracking state machine, calibration.

Pipeline per frame
    frame -> LEDDetector.detect()  (green blob centroid + brightness)
          -> PATTracker.update()   (SEARCH -> ACQUIRE -> TRACK, with reacquire)
          -> per-frame row {timestamp, drift, jitter, intensity}
    OpenCVCameraProvider.get_metrics() aggregates rows over a time window.

Coordinates are normalised to ``norm_width`` pixels wide, so the constants in the
transfer function don't depend on the camera resolution.

Frame sources return ``(frame_bgr, timestamp_seconds)`` or ``None`` at end of stream.
Use ``VideoCaptureSource`` for a real camera and ``SyntheticLEDSource`` for development.
"""
import csv
import json
import math
import time
from dataclasses import asdict, dataclass
from typing import Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np

from telemetry import TelemetryExhausted, TelemetryProvider, aggregate_window

SEARCH, ACQUIRE, TRACK = "SEARCH", "ACQUIRE", "TRACK"


# --------------------------------------------------------------------------- detection

@dataclass
class Detection:
    x: float            # normalised pixels
    y: float
    intensity: float    # summed brightness of the blob (resolution-normalised)


class LEDDetector:
    """Finds the brightest green blob. HSV thresholds are the main thing to tune on the rig."""

    def __init__(self, hsv_low=(40, 60, 100), hsv_high=(90, 255, 255),
                 core_v: int = 245, min_area: int = 3, norm_width: int = 640):
        self.hsv_low = np.array(hsv_low, dtype=np.uint8)
        self.hsv_high = np.array(hsv_high, dtype=np.uint8)
        self.core_v = core_v
        self.min_area = min_area
        self.norm_width = norm_width

    def detect(self, frame_bgr: np.ndarray,
               roi: Optional[Tuple[int, int, int, int]] = None) -> Optional[Detection]:
        """``roi`` is (x0, y0, x1, y1) in source pixels. Returns None if no LED is found."""
        h, w = frame_bgr.shape[:2]
        x0, y0, x1, y1 = (0, 0, w, h) if roi is None else roi
        x0, y0 = max(0, x0), max(0, y0)
        x1, y1 = min(w, x1), min(h, y1)
        if x1 - x0 < 1 or y1 - y0 < 1:
            return None
        sub = frame_bgr[y0:y1, x0:x1]
        hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)

        green = cv2.inRange(hsv, self.hsv_low, self.hsv_high)
        # A saturated LED core looks white; keep it if it sits inside the green halo
        core = cv2.inRange(hsv[:, :, 2], self.core_v, 255)
        halo = cv2.dilate(green, np.ones((5, 5), np.uint8))
        mask = green | (core & halo)

        n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if n_labels <= 1:
            return None
        areas = stats[1:, cv2.CC_STAT_AREA]
        best = int(np.argmax(areas)) + 1
        if stats[best, cv2.CC_STAT_AREA] < self.min_area:
            return None

        ys, xs = np.nonzero(labels == best)
        weights = hsv[:, :, 2][ys, xs].astype(np.float64)
        total = weights.sum()
        if total <= 0:
            return None
        cx = float((xs * weights).sum() / total) + x0
        cy = float((ys * weights).sum() / total) + y0

        scale = self.norm_width / w
        return Detection(x=cx * scale, y=cy * scale, intensity=float(total / 255.0) * scale ** 2)


# --------------------------------------------------------------------------- calibration

@dataclass
class Calibration:
    """Reference values measured during a calm hover."""
    boresight_x: float
    boresight_y: float
    i_ref: float            # blob brightness at full signal

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)

    @classmethod
    def load(cls, path: str) -> "Calibration":
        with open(path) as f:
            return cls(**json.load(f))

    @classmethod
    def from_detections(cls, dets: List[Detection]) -> "Calibration":
        if not dets:
            raise ValueError("no detections to calibrate from")
        return cls(float(np.mean([d.x for d in dets])),
                   float(np.mean([d.y for d in dets])),
                   float(np.mean([d.intensity for d in dets])))


# --------------------------------------------------------------------------- tracker

@dataclass
class FrameSample:
    t: float
    state: str
    locked: bool
    drift: float
    jitter: float
    intensity: float


class PATTracker:
    """Pointing, Acquisition and Tracking state machine.

    SEARCH   scan the whole frame for the LED
    ACQUIRE  LED seen; require ``acquire_frames`` consecutive hits near the last position
    TRACK    follow the LED inside a region of interest; after ``lost_frames`` misses
             in a row drop back to SEARCH (reacquire)
    """

    def __init__(self, detector: LEDDetector, calibration: Calibration,
                 acquire_frames: int = 5, lost_frames: int = 15, roi_half: int = 80,
                 lost_drift: Optional[float] = None):
        self.detector = detector
        self.cal = calibration
        self.acquire_frames = acquire_frames
        self.lost_frames = lost_frames
        self.roi_half = roi_half
        self.lost_drift = lost_drift if lost_drift is not None else detector.norm_width / 2

        self.state = SEARCH
        self._hits = 0
        self._misses = 0
        self._last: Optional[Detection] = None
        self._prev_t: Optional[float] = None
        self._search_started: Optional[float] = None

        self.acquisition_times: List[float] = []   # seconds from search start to lock, per lock
        self.lock_losses = 0

    @property
    def acquisition_time(self) -> Optional[float]:
        """Time to the first lock (the D1 'under 60 seconds' number)."""
        return self.acquisition_times[0] if self.acquisition_times else None

    def _roi(self, frame_shape) -> Optional[Tuple[int, int, int, int]]:
        if self._last is None:
            return None
        h, w = frame_shape[:2]
        scale = w / self.detector.norm_width
        cx, cy, r = self._last.x * scale, self._last.y * scale, self.roi_half * scale
        return int(cx - r), int(cy - r), int(cx + r), int(cy + r)

    def update(self, frame: np.ndarray, t: float) -> FrameSample:
        if self._search_started is None:
            self._search_started = t
        dt = (t - self._prev_t) if self._prev_t is not None else None
        self._prev_t = t

        roi = None if self.state == SEARCH else self._roi(frame.shape)
        det = self.detector.detect(frame, roi)
        prev = self._last

        if self.state == SEARCH:
            if det:
                self.state, self._hits, self._last = ACQUIRE, 1, det
        elif self.state == ACQUIRE:
            if det:
                self._hits += 1
                self._last = det
                if self._hits >= self.acquire_frames:
                    self.state, self._misses = TRACK, 0
                    self.acquisition_times.append(t - self._search_started)
            else:
                self.state, self._hits, self._last = SEARCH, 0, None
        else:  # TRACK
            if det:
                self._misses = 0
                self._last = det
            else:
                self._misses += 1
                if self._misses >= self.lost_frames:
                    self.state, self._hits, self._last = SEARCH, 0, None
                    self._search_started = t
                    self.lock_losses += 1

        if self.state == TRACK and det and prev is not None and dt and dt > 0:
            drift = math.hypot(det.x - self.cal.boresight_x, det.y - self.cal.boresight_y)
            jitter = math.hypot(det.x - prev.x, det.y - prev.y) / dt
            intensity = float(np.clip(det.intensity / self.cal.i_ref, 0.0, 1.0))
            return FrameSample(t, self.state, True, drift, jitter, intensity)
        if self.state == TRACK and det:   # first tracked frame: no previous position
            drift = math.hypot(det.x - self.cal.boresight_x, det.y - self.cal.boresight_y)
            return FrameSample(t, self.state, True, drift, 0.0,
                               float(np.clip(det.intensity / self.cal.i_ref, 0.0, 1.0)))
        # Not locked (searching, acquiring or a momentary miss): treat the link as lost
        return FrameSample(t, self.state, False, self.lost_drift, 0.0, 0.0)


# --------------------------------------------------------------------------- frame sources

class VideoCaptureSource:
    """Wraps cv2.VideoCapture. Run this on the OS that owns the camera (e.g. Windows Python,
    not WSL). Exposure must be locked or intensity changes get hidden by auto-exposure."""

    def __init__(self, device=0, width=1280, height=720, fps=60, manual_exposure: Optional[float] = -6,
                 backend: Optional[int] = None):
        self.cap = cv2.VideoCapture(device) if backend is None else cv2.VideoCapture(device, backend)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open camera {device}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        if manual_exposure is not None:
            # Backend-specific: V4L2 uses 0.25 for manual, DirectShow uses 0. Try both.
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, manual_exposure)
        self.last_frame: Optional[np.ndarray] = None

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            return None
        self.last_frame = frame
        return frame, time.monotonic()

    def release(self):
        self.cap.release()


Scene = Callable[[float], Optional[Tuple[float, float, float]]]
"""t -> (x, y, brightness 0..1) in source pixels, or None if the LED is not visible."""


class SyntheticLEDSource:
    """Renders a green LED on a noisy dark background. Stand-in for the camera while the
    hardware isn't available, and a deterministic fixture for tests."""

    def __init__(self, scene: Scene, width=640, height=480, fps=60, duration=120.0,
                 sigma=2.5, noise=6, seed=0):
        self.scene, self.w, self.h, self.fps, self.duration = scene, width, height, fps, duration
        self.sigma, self.noise = sigma, noise
        self.rng = np.random.default_rng(seed)
        self._i = 0
        self.last_frame: Optional[np.ndarray] = None

    def read(self):
        t = self._i / self.fps
        if t >= self.duration:
            return None
        self._i += 1
        frame = self.rng.integers(0, self.noise, size=(self.h, self.w, 3)).astype(np.float32)
        led = self.scene(t)
        if led is not None:
            x, y, b = led
            r = int(self.sigma * 4)
            xs = np.arange(max(0, int(x) - r), min(self.w, int(x) + r + 1))
            ys = np.arange(max(0, int(y) - r), min(self.h, int(y) + r + 1))
            if len(xs) and len(ys):
                gx, gy = np.meshgrid(xs, ys)
                g = np.exp(-((gx - x) ** 2 + (gy - y) ** 2) / (2 * self.sigma ** 2)) * b
                sub = frame[ys[0]:ys[-1] + 1, xs[0]:xs[-1] + 1]
                sub[:, :, 1] += 255 * 1.6 * g          # green, saturating in the core
                sub[:, :, 0] += 255 * 0.7 * g * 1.6    # blue/red leak -> whitish core
                sub[:, :, 2] += 255 * 0.7 * g * 1.6
        self.last_frame = np.clip(frame, 0, 255).astype(np.uint8)
        return self.last_frame, t


def demo_flight_scene(nominal=(400.0, 260.0), appear_at=3.0, seed=0) -> Scene:
    """120 s scripted flight mirroring tools/generate_fake_telemetry.py:
    not visible until ``appear_at``; calm; moderate fan; gust (dim, wild); calm again."""
    rng = np.random.default_rng(seed)
    nx, ny = nominal
    # (t_start, t_end, wander_px, shake_px, brightness)
    phases = [(0, 45, 2.0, 0.05, 1.0), (45, 80, 5.0, 0.3, 0.85),
              (80, 92, 25.0, 6.0, 0.3), (92, 1e9, 2.0, 0.05, 1.0)]

    def scene(t):
        if t < appear_at:
            return None
        for a, b, wander, shake, bright in phases:
            if a <= t < b:
                x = nx + wander * math.sin(2 * math.pi * t / 4.0) + rng.normal(0, shake)
                y = ny + 0.6 * wander * math.cos(2 * math.pi * t / 5.0) + rng.normal(0, shake)
                return x, y, bright
        return None
    return scene


# --------------------------------------------------------------------------- provider

class OpenCVCameraProvider(TelemetryProvider):
    """TelemetryProvider backed by a frame source + PATTracker."""

    def __init__(self, source, tracker: PATTracker, window_s: float = 1.0,
                 record_path: Optional[str] = None):
        self.source = source
        self.tracker = tracker
        self.window_s = window_s
        self._t0: Optional[float] = None
        self._pending = None   # first frame of the next window
        self._start: Optional[float] = None   # session start, for recorded timestamps
        self._rec_file = None
        self._rec = None
        if record_path:
            self._rec_file = open(record_path, "w", newline="")
            self._rec = csv.writer(self._rec_file)
            self._rec.writerow(["timestamp", "drift", "jitter", "intensity"])

    @property
    def state(self) -> str:
        return self.tracker.state

    def calibrate(self, seconds: float = 10.0) -> Calibration:
        """Hold the drone in calm hover, then call this. Sets boresight and I_ref from the
        mean tracked LED position and brightness, and returns the calibration."""
        dets: List[Detection] = []
        start = None
        while True:
            item = self.source.read()
            if item is None:
                break
            frame, t = item
            start = t if start is None else start
            self.tracker.update(frame, t)
            if self.tracker.state == TRACK and self.tracker._last is not None:
                dets.append(self.tracker._last)
            if t - start >= seconds:
                break
        cal = Calibration.from_detections(dets)
        self.tracker.cal = cal
        return cal

    def get_metrics(self) -> Dict[str, float]:
        rows = []
        while True:
            item, self._pending = (self._pending, None) if self._pending else (self.source.read(), None)
            if item is None:
                break
            frame, t = item
            if self._start is None:
                self._start = t
            if self._t0 is None:
                self._t0 = t
            if t - self._t0 >= self.window_s and rows:
                self._pending = item      # belongs to the next window
                self._t0 = None
                break
            sample = self.tracker.update(frame, t)
            rows.append({"drift": sample.drift, "jitter": sample.jitter, "intensity": sample.intensity})
            if self._rec:
                self._rec.writerow([f"{t - self._start:.4f}", f"{sample.drift:.3f}",
                                    f"{sample.jitter:.3f}", f"{sample.intensity:.4f}"])
        if not rows:
            raise TelemetryExhausted("camera stream ended")
        return aggregate_window(rows)

    def close(self) -> None:
        if self._rec_file:
            self._rec_file.close()
        rel = getattr(self.source, "release", None)
        if rel:
            rel()
