"""Offline closed-loop stand-in for Gazebo, for testing tracking/ without the simulator (the qkd_sim version of
src/sim.py ``SimRig``).

Renders the hover scene as the Action 5 Pro sees it (the active lens of camera_action5.yaml, camera
at the rig's lens height, the rig's deliberate mount error) and pushes each render through the *real* camera
emulation (``sim_bridge.image_pipeline.Action5Pipeline``: principal-point offset, auto-exposure, latency queue;
MJPG + rolling shutter at ``fidelity="full"``). rig_gimbal uses the real ``Storm32GimbalSim`` dynamics
(latency, second-order axes, holding jitter, noisy drifting readback), stepped on the rig's clock.

Scene: a 7 Hz red beacon (100% / 40%) on a wandering drone, a dark drone body, and the lighting distractors of
lighting.yaml: a steady red lamp, the 2 Hz red indicator and a ceiling panel flickering at 100 Hz (10% depth).
``truth`` holds the beacon's true angular offset from the optical axis for every captured frame.

Test helper only: it lives in tests/ and may use sim_bridge internals; tracking/ never imports it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from sim_bridge import config as cfgmod
from sim_bridge.camera_model import LINK_TO_OPTICAL, make_lens, rot_zyx
from sim_bridge.frame_source import Frame
from sim_bridge.gimbal_io import FixedMount, Storm32GimbalSim
from sim_bridge.image_pipeline import Action5Pipeline, mjpg_roundtrip

DEG = math.pi / 180.0


@dataclass
class Phase:
    t0: float
    t1: float
    wander_m: float          # slow drift amplitude (wind)
    shake_m: float = 0.0     # fast jitter RMS (gusts)


@dataclass
class SceneOptions:
    phases: list[Phase] = field(default_factory=lambda: [Phase(0, 1e9, 0.02, 0.002)])
    start_offset_m: tuple[float, float] = (0.0, 0.0)     # (y, z) offset of the drone from the station
    appear_at_s: float = 0.0
    occlusions: list[tuple[float, float]] = field(default_factory=list)
    blink_hz: float = 7.0
    steady: bool = False                                  # beacon.yaml red_led.modulation.enabled: false
    beacon_size_m: float = 0.015                          # apparent size of the bare LED (beacon.yaml render size)
    distractors: bool = True
    background: int = 110                                 # sRGB grey level of the room (office-lit)
    # Light the LED adds to its core pixel, linear RGB. Matched to a real Gazebo render (1920x1080, 3 m, steady,
    # before AE): core BGR ~ (172, 182, 238), i.e. pale pink, R - max(G, B) ~ 56, because the 15 mm render blends
    # with the background at its edges. On this rig's grey 110 background that is (0.70, 0.30, 0.26).
    beacon_rgb_lin: tuple[float, float, float] = (0.70, 0.30, 0.26)
    mjpg: bool = True                                     # MJPG round trip (4:2:0 chroma) as the real stream
    takeoff_s: float = 0.0                                # >0: the drone rises from the floor at t = 0 (simple_flight)
    sweep_mps: float = 0.0                                # >0: sideways sine of amplitude sweep_amp_m at this peak speed
    sweep_amp_m: float = 0.5                              #   (as scenarios/tracking_sweep.yaml)


class SyntheticRig:
    def __init__(self, rig: str = "rig_fixed", *, duration_s: float = 20.0, seed: int = 0, fidelity: str = "fast",
                 scene: SceneOptions | None = None, exposure_mode: str | None = None):
        cfgs = cfgmod.load_all({"camera_action5": {"exposure": {"mode": exposure_mode}}} if exposure_mode else None)
        self.cfgs = cfgs
        cam = cfgs["camera_action5"]
        self.rig = rig
        self.cfg_rig = cfgs["rigs"][rig]
        self.scene = scene or SceneOptions()
        self.lens = make_lens(cam)                           # the active lens (rectilinear by default)
        self.pipeline = Action5Pipeline(cam, fidelity, pp_offset_px=self.cfg_rig["principal_point_offset_px"])
        self.quality = int(cam["compression"]["mjpg_quality"])     # fast fidelity skips MJPG: applied in read()
        self.fps = float(cam["stream"]["fps"])
        self.W, self.H = self.lens.width, self.lens.height
        self.duration = duration_s
        self.rng = np.random.default_rng(seed)
        self.t = 0.0
        self._k = 0
        self.cam_pos = np.array([0.0, 0.0, float(cfgs["world"]["camera_mount"]["lens_height_m"])])
        self.station = np.array(cfgs["world"]["hover_station_m"], dtype=float)
        e = self.cfg_rig["mount_error_deg"]
        self.R_err = rot_zyx(float(e["yaw"]) * DEG, float(e["pitch"]) * DEG, float(e["roll"]) * DEG)
        clock = lambda: self.t  # noqa: E731
        if rig == "rig_gimbal":
            self.gimbal = Storm32GimbalSim(cfgs["gimbal_storm32"], self.cfg_rig, world=None, clock=clock, seed=seed)
        else:
            self.gimbal = FixedMount(self.cfg_rig, clock)
        self._ph = self.rng.uniform(0, 2 * np.pi, size=(2, 3))
        self._fr = np.array([0.11, 0.23, 0.37])
        self._ar = np.zeros(2)
        self._noise = [np.clip(self._base() + self.rng.normal(0, 2.0, (self.H, self.W, 1)), 0, 255).astype(np.uint8)
                       .repeat(3, axis=2) for _ in range(3)]
        lt = cfgs["lighting"]
        self.distractors = lt["distractors"]
        self.truth: list[dict] = []
        self.true_gain: dict[float, float] = {}       # t_capture -> exposure gain the emulation applied
        self.drone = self.station.copy()

    # ------------------------------------------------------------------ scene
    def _base(self) -> np.ndarray:
        y = np.linspace(-1, 1, self.H)[:, None, None]
        return np.full((self.H, self.W, 1), float(self.scene.background)) * (1.0 - 0.25 * y)

    def phase(self, t: float) -> Phase:
        for p in self.scene.phases:
            if p.t0 <= t < p.t1:
                return p
        return self.scene.phases[-1]

    def _move(self, t: float, dt: float) -> None:
        p = self.phase(t)
        slow = np.sin(2 * np.pi * self._fr[None, :] * t + self._ph).sum(axis=1) / 3.0 * 1.5
        a = math.exp(-dt * 2 * math.pi * 3.0)
        self._ar = a * self._ar + math.sqrt(1 - a * a) * self.rng.standard_normal(2)
        off = np.array([0.0, self.scene.start_offset_m[0] + p.wander_m * slow[0] + p.shake_m * self._ar[0],
                        self.scene.start_offset_m[1] + 0.6 * p.wander_m * slow[1] + p.shake_m * self._ar[1]])
        if self.scene.sweep_mps:          # scripted sideways sine: peak speed sweep_mps, accel sweep_mps^2 / amp
            off[1] += self.scene.sweep_amp_m * math.sin(self.scene.sweep_mps / self.scene.sweep_amp_m * t)
        if self.scene.takeoff_s:          # simple_flight: lifts off the floor at t = 0, hovering by takeoff_s
            x = min(1.0, t / self.scene.takeoff_s)
            off[2] -= (1.0 - x * x * (3 - 2 * x)) * (self.station[2] - 0.05)
            # PID overshoot as in Gazebo t5 (peaks ~0.15 m above the station ~3.5 s after take-off, settles by ~8 s)
            off[2] += 0.15 * math.exp(-((t - self.scene.takeoff_s * 0.9 - 3.5) / 1.5) ** 2)
        self.drone = self.station + off

    def camera_rotation(self) -> np.ndarray:
        """Camera link -> world. Gimbal: pan about +z (left), tilt positive up (= negative pitch)."""
        if self.rig == "rig_gimbal":
            pan, tilt, roll = self.gimbal.true_attitude()
        else:
            pan, tilt, roll = (float(self.cfg_rig[k]) * DEG for k in ("nominal_pan_deg", "nominal_tilt_deg",
                                                                       "nominal_roll_deg"))
        return rot_zyx(pan, -tilt, roll) @ self.R_err

    def _optical(self, p_world: np.ndarray, R: np.ndarray) -> np.ndarray:
        d = LINK_TO_OPTICAL @ (R.T @ (np.asarray(p_world, float) - self.cam_pos))
        return d / np.linalg.norm(d)

    def _pixel(self, d_opt: np.ndarray) -> np.ndarray:
        return self.lens.project(d_opt)[0]

    @staticmethod
    def _angles(d_opt: np.ndarray) -> tuple[float, float]:
        """Equidistant components of the offset from the optical axis (x right, y down), rad."""
        r = math.hypot(d_opt[0], d_opt[1])
        th = math.atan2(r, d_opt[2])
        return (th * d_opt[0] / r, th * d_opt[1] / r) if r > 1e-12 else (0.0, 0.0)

    def _spot(self, img_lin: np.ndarray, uv, radius_px: float, rgb_lin, x0y0) -> None:
        cx, cy = uv[0] - x0y0[0], uv[1] - x0y0[1]
        h, w = img_lin.shape[:2]
        sig = max(0.7, radius_px / 1.4)
        r = int(4 * sig) + 1
        xa, xb, ya, yb = max(0, int(cx) - r), min(w, int(cx) + r + 1), max(0, int(cy) - r), min(h, int(cy) + r + 1)
        if xa >= xb or ya >= yb:
            return
        gx, gy = np.meshgrid(np.arange(xa, xb), np.arange(ya, yb))
        g = np.exp(-((gx - cx) ** 2 + (gy - cy) ** 2) / (2 * sig * sig))[..., None]
        img_lin[ya:yb, xa:xb] += g * np.asarray(rgb_lin, float)[None, None, :]

    def _draw(self, img: np.ndarray, uv, radius_px: float, rgb_lin) -> None:
        """Add a light (linear) on top of the sRGB image around ``uv``."""
        r = int(4 * max(0.7, radius_px / 1.4)) + 2
        x0, x1 = max(0, int(uv[0]) - r), min(self.W, int(uv[0]) + r + 1)
        y0, y1 = max(0, int(uv[1]) - r), min(self.H, int(uv[1]) + r + 1)
        if x0 >= x1 or y0 >= y1:
            return
        patch = (img[y0:y1, x0:x1].astype(np.float64) / 255.0) ** 2.2
        self._spot(patch, uv, radius_px, rgb_lin, (x0, y0))
        img[y0:y1, x0:x1] = np.clip(patch, 0, 1) ** (1 / 2.2) * 255.0

    def render(self, t: float) -> np.ndarray:
        """RGB image at time t (before the camera emulation)."""
        img = self._noise[self._k % len(self._noise)].copy()
        R = self.camera_rotation()
        if self.scene.distractors:
            d = self.distractors
            panel = slice(0, self.H // 6), slice(self.W // 3, 2 * self.W // 3)        # flickering ceiling panel
            img[panel] = np.clip(img[panel].astype(np.float32) * 2.2 * (1 + 0.1 * math.cos(2 * math.pi * 100.0 * t)),
                                 0, 255).astype(np.uint8)
            for name, level in (("steady_red_lamp", 1.0),
                                ("blinking_red_indicator",
                                 1.0 if math.sin(2 * math.pi * float(d["blinking_red_indicator"]["blink_hz"]) * t) >= 0 else 0.0)):
                o = d[name]
                dd = self._optical(o["position_m"], R)
                if dd[2] > 0.1 and level > 0:
                    rng = float(np.linalg.norm(np.asarray(o["position_m"]) - self.cam_pos))
                    self._draw(img, self._pixel(dd), self.lens.f_px * float(o["diameter_m"]) / 2 / rng,
                               0.8 * level * np.asarray(o["emissive_rgb"], float))
            for g in d.get("glints") or []:                                            # small bright white glints
                dd = self._optical(g["position_m"], R)
                if dd[2] > 0.1:
                    rng = float(np.linalg.norm(np.asarray(g["position_m"]) - self.cam_pos))
                    self._draw(img, self._pixel(dd), self.lens.f_px * float(np.mean(g["size_m"])) / 2 / rng,
                               0.9 * np.asarray(g["emissive"], float))
        dd = self._optical(self.drone, R)
        rng = float(np.linalg.norm(self.drone - self.cam_pos))
        visible = t >= self.scene.appear_at_s and not any(a <= t < b for a, b in self.scene.occlusions)
        if dd[2] > 0.1:
            u, v = self._pixel(dd)
            bw, bh = int(self.lens.f_px * 0.10 / rng), int(self.lens.f_px * 0.03 / rng)       # drone body (dark)
            img[max(0, int(v) - bh):max(0, int(v) + bh), max(0, int(u) - bw):max(0, int(u) + bw)] //= 2
            if visible:
                level = 1.0 if (self.scene.steady or math.sin(2 * math.pi * self.scene.blink_hz * t) >= 0) else 0.4
                self._draw(img, (u, v), self.lens.f_px * self.scene.beacon_size_m / 2 / rng,
                           level * np.asarray(self.scene.beacon_rgb_lin, float))
        ax, ay = self._angles(dd)
        self.truth.append({"t": t, "err_x_rad": ax, "err_y_rad": ay, "range_m": rng, "visible": visible,
                           "drone": self.drone.copy()})
        return img

    # ------------------------------------------------------------------ FrameSource
    def read(self, timeout_s: float = 5.0) -> Frame | None:
        while True:
            t = self._k / self.fps
            if t >= self.duration:
                return None
            self.t = t
            if self.rig == "rig_gimbal":
                self.gimbal.advance_to(t)
            self._move(t, 1.0 / self.fps)
            self.pipeline.push_render(self.render(t), t)
            self._k += 1
            out = self.pipeline.pop_delivered(t)
            for pf in out:
                self.true_gain[round(pf.t_capture_sim, 6)] = pf.exposure_gain
            if out:
                pf = out[-1]
                img = mjpg_roundtrip(pf.image_bgr, self.quality) if self.scene.mjpg else pf.image_bgr
                return Frame(img, pf.t_capture_sim, t, "synthetic", pf.seq)

    @property
    def finished(self) -> bool:
        return self._k / self.fps >= self.duration

    def close(self) -> None:
        pass

    def truth_at(self, t: float) -> dict:
        i = int(round(t * self.fps))
        return self.truth[min(max(i, 0), len(self.truth) - 1)]
