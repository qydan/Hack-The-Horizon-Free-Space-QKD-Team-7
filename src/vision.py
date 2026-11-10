"""Vision building blocks for the ground tracker.

* ``LockInDetector``   finds pixels that pulse at the beacon's blink rate (13 Hz). Room
                       lights, windows and reflections don't, so they are ignored.
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
from typing import Optional, Tuple

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


class SteadySpotDetector:
    """Finds a constantly-lit LED as a small bright spot against a dimmer background.

    Use when the beacon does not blink. It has no way to tell the LED from any other small
    bright thing (a lamp, a reflection, a phone screen), so the scene must be dark and the
    LED the brightest compact spot. Candidates are pixels at least ``min_contrast`` grey
    levels above the background; each is re-thresholded at ``rel_threshold`` of its own peak
    (so bloom and the drone body don't count) and kept only if its core is
    ``min_area``..``max_area`` pixels. The brightest such spot wins.
    """

    def __init__(self, min_contrast: float = 40.0, rel_threshold: float = 0.5,
                 min_area: int = 3, max_area: int = 600, max_candidates: int = 16):
        self.min_contrast, self.rel_threshold = min_contrast, rel_threshold
        self.min_area, self.max_area, self.max_candidates = min_area, max_area, max_candidates

    def detect(self, frame: np.ndarray, roi: Optional[Tuple[int, int, int, int]] = None) -> Optional[Beacon]:
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        x0, y0, x1, y1 = (0, 0, w, h) if roi is None else roi
        x0, y0, x1, y1 = max(0, x0), max(0, y0), min(w, x1), min(h, y1)
        if x1 - x0 < 3 or y1 - y0 < 3:
            return None
        blur = cv2.GaussianBlur(gray[y0:y1, x0:x1].astype(np.float32), (5, 5), 0)
        bg = float(np.median(blur[::4, ::4]))
        if float(blur.max()) - bg < self.min_contrast:
            return None
        mask = (blur >= bg + self.min_contrast).astype(np.uint8)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        order = np.argsort(-stats[1:, cv2.CC_STAT_AREA])[:self.max_candidates] + 1
        best = None
        for i in order:
            bx, by = int(stats[i, cv2.CC_STAT_LEFT]), int(stats[i, cv2.CC_STAT_TOP])
            bw, bh = int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT])
            comp = labels[by:by + bh, bx:bx + bw] == i
            patch = blur[by:by + bh, bx:bx + bw]
            pk = float(patch[comp].max())
            core = comp & (patch >= bg + self.rel_threshold * (pk - bg))
            area = int(core.sum())
            if self.min_area <= area <= self.max_area and (best is None or pk > best[0]):
                best = (pk, bx, by, core, patch)
        if best is None:
            return None
        pk, bx, by, core, patch = best
        ys, xs = np.nonzero(core)
        wts = (patch[ys, xs] - bg).astype(np.float64)
        total = float(wts.sum())
        return Beacon(x=float((xs * wts).sum() / total) + bx + x0, y=float((ys * wts).sum() / total) + by + y0,
                      amplitude=total, peak=pk - bg)


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
