"""Synthetic rig: a wind-blown drone with a 13 Hz blinking beacon, room-light distractors, a
simulated gimbal and a simulated camera. Lets the whole tracker run without hardware.

``SimRig.read()`` returns ``(frame, t)`` like a camera. The tracker drives the same
``SimGimbal`` the rig renders from, so gimbal lag, step size and latency show up in the
measured tracking residual just as they would on the real rig.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from gimbal import SimGimbal
from vision import CameraModel, Pose


@dataclass
class Phase:
    """A stretch of the scripted flight."""
    t0: float
    t1: float
    wander_mrad: float      # slow drift amplitude (wind)
    shake_mrad: float       # fast jitter RMS
    lean_deg: float         # drone tilt into the wind
    brightness: float = 1.0
    name: str = ""


def demo_phases() -> List[Phase]:
    """120 s: fan off, low, high, a gust, then calm again."""
    return [Phase(0, 30, 3, 0.3, 0.5, name="fan off"),
            Phase(30, 60, 15, 1.5, 2.5, name="fan low"),
            Phase(60, 90, 25, 3.0, 3.5, name="fan high"),
            Phase(90, 100, 60, 12.0, 6.0, 0.5, name="gust"),
            Phase(100, 1e9, 3, 0.3, 0.5, name="recovery")]


class SimRig:
    def __init__(self, camera: Optional[CameraModel] = None, gimbal: Optional[SimGimbal] = None,
                 phases: Optional[List[Phase]] = None, fps: float = 120.0, duration: float = 120.0,
                 appear_at: float = 2.0, blink_hz: float = 13.0, start_offset=(300.0, 90.0),
                 range_m: float = 3.0, seed: int = 0, noise: int = 5,
                 steady_beacon: bool = False, distractors: bool = True,
                 color: bool = False, bright_scene: bool = False, court_lines: bool = False):
        self.steady_beacon = steady_beacon      # beacon constantly on (no blink)
        self.distractors = distractors          # window light + mains lamp in the scene
        self.court_lines = court_lines          # squash court: red painted lines on a white wall, one of them
                                                # right through the hover point (the beacon sits on it)
        self.color = color or bright_scene or court_lines   # BGR frames: red beacon, white lights
        self.bright_scene = bright_scene        # lit room: ambient light, a blown-out window, a white lamp
                                                # brighter than the beacon next to the drone, a red poster
        self.cam = camera or CameraModel(480, 270, 70.0)
        self.gimbal = gimbal or SimGimbal()
        self.phases = phases or demo_phases()
        self.fps, self.duration, self.appear_at, self.blink_hz = fps, duration, appear_at, blink_hz
        self.range_m = range_m
        self.noise = noise
        self.rng = np.random.default_rng(seed)
        self._i = 0
        self.t = 0.0
        self.gimbal.place(-start_offset[0], -start_offset[1])    # camera starts pointing away
        rs = np.random.default_rng(seed + 1)
        self._ph = rs.uniform(0, 2 * np.pi, size=(2, 3))
        self._fr = np.array([0.11, 0.23, 0.37])                  # Hz, slow wind wander
        self._ar = np.zeros(2)                                    # AR(1) fast-shake state
        self.drone = (0.0, 0.0)                                   # true direction, mrad
        self.lean_deg = 0.0
        self.brightness = 1.0
        self.visible = False
        self.window_blob = (150.0, -60.0)                         # steady room light, world mrad
        self.flicker_blob = (-200.0, 40.0)                        # 100 Hz mains lamp
        self.last_frame: Optional[np.ndarray] = None

    # -------------------------------------------------------------- world

    def phase(self, t: float) -> Phase:
        for p in self.phases:
            if p.t0 <= t < p.t1:
                return p
        return self.phases[-1]

    def _world(self, t: float, dt: float) -> None:
        p = self.phase(t)
        slow = np.sin(2 * np.pi * self._fr[None, :] * t + self._ph).sum(axis=1) / 3.0 * 1.5
        a = math.exp(-dt * 2 * math.pi * 4.0)                     # ~4 Hz shake bandwidth
        self._ar = a * self._ar + math.sqrt(1 - a * a) * self.rng.standard_normal(2)
        self.drone = (p.wander_mrad * slow[0] + p.shake_mrad * self._ar[0],
                      0.6 * p.wander_mrad * slow[1] + p.shake_mrad * self._ar[1])
        self.lean_deg = p.lean_deg
        self.brightness = p.brightness
        self.visible = t >= self.appear_at

    def truth(self) -> dict:
        return {"t": self.t, "drone_mrad": self.drone, "lean_deg": self.lean_deg,
                "range_m": self.range_m, "visible": self.visible}

    # -------------------------------------------------------------- rendering

    def _blob(self, img, cx, cy, sigma, peak, bgr=(1.0, 1.0, 1.0)):
        r = int(sigma * 4)
        x0, x1 = max(0, int(cx) - r), min(self.cam.width, int(cx) + r + 1)
        y0, y1 = max(0, int(cy) - r), min(self.cam.height, int(cy) + r + 1)
        if x0 >= x1 or y0 >= y1:
            return
        gx, gy = np.meshgrid(np.arange(x0, x1), np.arange(y0, y1))
        g = peak * np.exp(-((gx - cx) ** 2 + (gy - cy) ** 2) / (2 * sigma ** 2))
        if img.ndim == 2:
            img[y0:y1, x0:x1] += g
        else:
            img[y0:y1, x0:x1] += g[..., None] * np.asarray(bgr, np.float32)

    def _rect(self, img, cx, cy, half_w, half_h, value, bgr=(1.0, 1.0, 1.0)):
        x0, x1 = max(0, int(cx - half_w)), max(0, int(cx + half_w))
        y0, y1 = max(0, int(cy - half_h)), max(0, int(cy + half_h))
        img[y0:y1, x0:x1] += value if img.ndim == 2 else value * np.asarray(bgr, np.float32)

    def _to_px(self, world_mrad):
        pan, tilt = self.gimbal.angles()
        return self.cam.mrad_to_px(world_mrad[0] - pan, world_mrad[1] - tilt)

    def read(self) -> Optional[Tuple[np.ndarray, float]]:
        t = self._i / self.fps
        if t >= self.duration:
            return None
        dt = 1.0 / self.fps
        self._i += 1
        self.t = t
        self.gimbal.step_time(dt)
        self._world(t, dt)

        shape = (self.cam.height, self.cam.width) + ((3,) if self.color else ())
        img = self.rng.integers(0, self.noise, size=shape).astype(np.float32)
        if self.bright_scene:
            img += 60.0 + np.linspace(0.0, 30.0, self.cam.height, dtype=np.float32)[:, None, None]   # lit room
            wx, wy = self._to_px((-250.0, -60.0))
            self._rect(img, wx, wy, 30, 22, 255.0)                                     # blown-out window
            px, py = self._to_px((200.0, 60.0))
            self._rect(img, px, py, 22, 22, 110.0, bgr=(0.1, 0.15, 1.0))               # red poster
            lx, ly = self._to_px((60.0, -20.0))
            self._blob(img, lx, ly, 3.0, 400.0)                                        # white lamp, clips
        if self.court_lines:
            img += 110.0                                                               # white wall
            m = np.zeros((self.cam.height, self.cam.width), np.uint8)
            for (a, b, wd) in [((-900.0, 0.0), (900.0, 0.0), 4),                       # through the hover point
                               ((-900.0, 80.0), (900.0, 80.0), 4),
                               ((-120.0, -500.0), (-100.0, 500.0), 3),                 # near-vertical
                               ((40.0, -300.0), (400.0, 300.0), 3),                    # diagonal
                               ((-60.0, 150.0), (-40.0, 160.0), 4)]:                   # short stub
                pa, pb = self._to_px(a), self._to_px(b)
                cv2.line(m, (int(round(pa[0])), int(round(pa[1]))), (int(round(pb[0])), int(round(pb[1]))), 1, wd)
            img[m > 0] = img[m > 0] * 0.25 + np.array([10.0, 10.0, 110.0], np.float32)  # red paint
        if self.distractors:
            wx, wy = self._to_px(self.window_blob)
            self._blob(img, wx, wy, 3.0, 200.0)                                        # steady window
            fx, fy = self._to_px(self.flicker_blob)
            self._blob(img, fx, fy, 3.0, 160.0 * (0.5 + 0.5 * math.cos(2 * math.pi * 100.0 * t)))  # mains lamp
        if self.visible:
            bx, by = self._to_px(self.drone)
            x0, x1 = int(bx - 18), int(bx + 18)
            y0, y1 = int(by - 4), int(by + 16)
            body = img[max(0, y0):max(0, y1), max(0, x0):max(0, x1)]
            if self.color:
                body[...] = 45.0 + self.rng.integers(0, self.noise, size=body.shape)  # opaque dark drone body
            else:
                body += 80.0                                                       # (dark-room grey scenes)
            blink = 1.0 if self.steady_beacon else \
                0.4 + 0.6 * (1.0 if math.sin(2 * math.pi * self.blink_hz * t) >= 0 else 0.0)
            self._blob(img, bx, by, 1.6, 150.0 * blink * self.brightness,
                       bgr=(0.15, 0.2, 1.0))                                           # red beacon 100% <-> 40%
        self.last_frame = np.clip(img, 0, 255).astype(np.uint8)
        return self.last_frame, t

    def pose(self, frame=None) -> Pose:
        """Stand-in for the AprilTag pose: ground truth plus about 1 degree of pose noise."""
        return Pose(lean_deg=abs(self.lean_deg + self.rng.normal(0, 1.0)),
                    range_m=self.range_m * (1 + self.rng.normal(0, 0.01)))

    def release(self) -> None:
        pass
