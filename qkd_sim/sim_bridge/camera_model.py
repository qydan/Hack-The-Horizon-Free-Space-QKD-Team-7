"""Action 5 Pro webcam-mode lens model (equidistant fisheye) derived from camera_action5.yaml.

This module is simulator-side: it is used to *generate* the camera and to write evaluation/camera_truth.yaml.
It must never be imported by tracking/ or calibration/ (they have to estimate these numbers).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class EquidistantLens:
    width: int
    height: int
    hfov_rad: float
    pp_offset_px: tuple[float, float] = (0.0, 0.0)

    @classmethod
    def from_cfg(cls, cam: dict[str, Any], pp_offset_px=(0.0, 0.0)) -> "EquidistantLens":
        return cls(int(cam["stream"]["width_px"]), int(cam["stream"]["height_px"]),
                   math.radians(float(cam["lens"]["hfov_deg"])), tuple(map(float, pp_offset_px)))

    @property
    def f_px(self) -> float:
        """f = (W/2) / (HFOV/2)."""
        return (self.width / 2.0) / (self.hfov_rad / 2.0)

    @property
    def cx(self) -> float:
        return (self.width - 1) / 2.0 + self.pp_offset_px[0]

    @property
    def cy(self) -> float:
        return (self.height - 1) / 2.0 + self.pp_offset_px[1]

    @property
    def vfov_rad(self) -> float:
        return 2.0 * (self.height / 2.0) / self.f_px

    @property
    def ifov_rad(self) -> float:
        return 1.0 / self.f_px

    def project(self, dirs_cam: np.ndarray) -> np.ndarray:
        """Project unit direction vectors in the *optical* frame (x right, y down, z forward) to pixels."""
        d = np.atleast_2d(np.asarray(dirs_cam, float))
        r_xy = np.hypot(d[:, 0], d[:, 1])
        theta = np.arctan2(r_xy, d[:, 2])
        scale = np.where(r_xy > 1e-12, self.f_px * theta / np.maximum(r_xy, 1e-12), self.f_px)
        return np.stack([self.cx + scale * d[:, 0], self.cy + scale * d[:, 1]], axis=1)

    def unproject(self, uv: np.ndarray) -> np.ndarray:
        uv = np.atleast_2d(np.asarray(uv, float))
        x = (uv[:, 0] - self.cx) / self.f_px
        y = (uv[:, 1] - self.cy) / self.f_px
        theta = np.hypot(x, y)
        s = np.where(theta > 1e-12, np.sin(theta) / np.maximum(theta, 1e-12), 1.0)
        return np.stack([x * s, y * s, np.cos(theta)], axis=1)

    def K(self) -> list[list[float]]:
        return [[self.f_px, 0.0, self.cx], [0.0, self.f_px, self.cy], [0.0, 0.0, 1.0]]

    def truth_dict(self) -> dict[str, Any]:
        """cv2.fisheye with D = 0 is exactly the equidistant model r = f * theta."""
        return {
            "model": "cv2.fisheye (equidistant, D = 0)",
            "projection": "equidistant",
            "image_size_px": [self.width, self.height],
            "K": self.K(),
            "D": [0.0, 0.0, 0.0, 0.0],
            "f_px": self.f_px,
            "hfov_deg": math.degrees(self.hfov_rad),
            "vfov_deg": math.degrees(self.vfov_rad),
            "ifov_mrad_per_px_centre": 1e3 * self.ifov_rad,
            "principal_point_offset_px": list(self.pp_offset_px),
        }


@dataclass(frozen=True)
class RectilinearLens:
    """Pinhole (rectilinear) lens: the Action 5 Pro's dewarped modes. r = f tan(theta), f = (W/2) / tan(HFOV/2)."""
    width: int
    height: int
    hfov_rad: float
    pp_offset_px: tuple[float, float] = (0.0, 0.0)

    @classmethod
    def from_cfg(cls, cam: dict[str, Any], pp_offset_px=(0.0, 0.0)) -> "RectilinearLens":
        return cls(int(cam["stream"]["width_px"]), int(cam["stream"]["height_px"]),
                   math.radians(float(cam["lens"]["hfov_deg"])), tuple(map(float, pp_offset_px)))

    @property
    def f_px(self) -> float:
        return (self.width / 2.0) / math.tan(self.hfov_rad / 2.0)

    @property
    def cx(self) -> float:
        return (self.width - 1) / 2.0 + self.pp_offset_px[0]

    @property
    def cy(self) -> float:
        return (self.height - 1) / 2.0 + self.pp_offset_px[1]

    @property
    def vfov_rad(self) -> float:
        return 2.0 * math.atan((self.height / 2.0) / self.f_px)

    @property
    def ifov_rad(self) -> float:
        return 1.0 / self.f_px

    def project(self, dirs_cam: np.ndarray) -> np.ndarray:
        d = np.atleast_2d(np.asarray(dirs_cam, float))
        z = np.where(d[:, 2] > 1e-9, d[:, 2], np.nan)          # behind the camera -> NaN
        return np.stack([self.cx + self.f_px * d[:, 0] / z, self.cy + self.f_px * d[:, 1] / z], axis=1)

    def unproject(self, uv: np.ndarray) -> np.ndarray:
        uv = np.atleast_2d(np.asarray(uv, float))
        v = np.stack([(uv[:, 0] - self.cx) / self.f_px, (uv[:, 1] - self.cy) / self.f_px, np.ones(len(uv))], axis=1)
        return v / np.linalg.norm(v, axis=1, keepdims=True)

    def K(self) -> list[list[float]]:
        return [[self.f_px, 0.0, self.cx], [0.0, self.f_px, self.cy], [0.0, 0.0, 1.0]]

    def truth_dict(self) -> dict[str, Any]:
        return {
            "model": "pinhole (rectilinear), cv2 standard model with D = 0",
            "projection": "rectilinear",
            "image_size_px": [self.width, self.height],
            "K": self.K(),
            "D": [0.0, 0.0, 0.0, 0.0, 0.0],
            "f_px": self.f_px,
            "hfov_deg": math.degrees(self.hfov_rad),
            "vfov_deg": math.degrees(self.vfov_rad),
            "ifov_mrad_per_px_centre": 1e3 * self.ifov_rad,
            "principal_point_offset_px": list(self.pp_offset_px),
        }


def make_lens(cam: dict[str, Any], pp_offset_px=(0.0, 0.0)):
    """The active lens from camera_action5.yaml: RectilinearLens (default) or EquidistantLens (fisheye option)."""
    proj = cam["lens"]["projection"]
    if proj == "rectilinear":
        return RectilinearLens.from_cfg(cam, pp_offset_px)
    if proj == "equidistant":
        return EquidistantLens.from_cfg(cam, pp_offset_px)
    raise ValueError(f"camera_action5.yaml lens.projection must be rectilinear or equidistant, got {proj!r}")


def render_sensor_type(cam: dict[str, Any]) -> str:
    """Resolve render.sensor_type 'auto' against the active lens."""
    st = cam["render"]["sensor_type"]
    proj = cam["lens"]["projection"]
    if st == "auto":
        return "camera" if proj == "rectilinear" else "wideanglecamera"
    if proj == "rectilinear" and st != "camera":
        raise ValueError(f"render.sensor_type {st!r} only applies to the equidistant fisheye; use auto")
    return st


def rectilinear_to_equidistant_maps(lens: EquidistantLens, rect_w: int, rect_h: int, rect_hfov_rad: float):
    """cv2.remap maps that turn a rectilinear render (pinhole, hfov) into the equidistant output image.

    Output pixels whose ray falls outside the rectilinear frustum map to -1 (black). With a 120 deg render and
    a 140 deg output, the outer ~10 deg on each side stay black: this is the documented limit of the fallback.
    """
    vv, uu = np.mgrid[0:lens.height, 0:lens.width].astype(np.float64)
    d = lens.unproject(np.stack([uu.ravel(), vv.ravel()], axis=1))
    fr = (rect_w / 2.0) / math.tan(rect_hfov_rad / 2.0)
    cxr, cyr = (rect_w - 1) / 2.0, (rect_h - 1) / 2.0
    z = d[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        mx = np.where(z > 1e-6, cxr + fr * d[:, 0] / z, -1.0)
        my = np.where(z > 1e-6, cyr + fr * d[:, 1] / z, -1.0)
    bad = (mx < 0) | (mx > rect_w - 1) | (my < 0) | (my > rect_h - 1)
    mx[bad] = -1.0
    my[bad] = -1.0
    return mx.reshape(lens.height, lens.width).astype(np.float32), my.reshape(lens.height, lens.width).astype(np.float32)


def rot_zyx(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """Body-to-world rotation R = Rz(yaw) Ry(pitch) Rx(roll) (ENU, x forward)."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


# camera link frame (x forward, y left, z up) -> optical frame (x right, y down, z forward)
LINK_TO_OPTICAL = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def project_world_point(lens: EquidistantLens, cam_pos: np.ndarray, R_cam_to_world: np.ndarray,
                        p_world: np.ndarray) -> np.ndarray:
    """Pixel of a world point for a camera link at ``cam_pos`` with orientation ``R_cam_to_world``."""
    d_link = R_cam_to_world.T @ (np.asarray(p_world, float) - np.asarray(cam_pos, float))
    d_opt = LINK_TO_OPTICAL @ d_link
    return lens.project(d_opt / np.linalg.norm(d_opt))[0]
