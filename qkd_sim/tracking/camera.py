"""Camera model for tracking: the Action 5 Pro's lens, rectilinear (the default dewarped "standard" mode) or
equidistant fisheye (the ultra-wide option), as set in config/camera_action5.yaml ``lens.projection``.

Uses ``config/calibration.yaml: intrinsics`` once ``calibration/intrinsics.py`` has written it (``model``
``cv2.fisheye`` -> fisheye distortion D[4]; anything else -> OpenCV pinhole distortion); until then the nominal
datasheet lens: ``hfov_deg`` with the principal point at the image centre. The nominal model does not know the
rig's principal-point offset or mount error: those show up as a constant bias in err_x / err_y (drift is measured
against a boresight and the residual against the receiver, so both cancel it).

Angles are the components of the offset from the optical axis, theta * unit direction, in rad: x right, y down
(src/'s pinhole ``px_to_mrad`` generalised to both projections).
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np

PROJECTIONS = ("rectilinear", "equidistant")


class CameraModel:
    def __init__(self, K: np.ndarray, D: np.ndarray, width: int, height: int, projection: str = "rectilinear",
                 source: str = "nominal"):
        if projection not in PROJECTIONS:
            raise ValueError(f"projection must be one of {PROJECTIONS}, got {projection!r}")
        self.K = np.asarray(K, dtype=np.float64).reshape(3, 3)
        self.D = np.asarray(D if D is not None else [], dtype=np.float64).reshape(-1)
        self.width, self.height = int(width), int(height)
        self.projection = projection
        self.source = source
        self._plain = not np.any(self.D)

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> "CameraModel":
        intr = (cfg.get("calibration") or {}).get("intrinsics") or {}
        nom = cfg["camera_nominal"]
        if intr.get("K") is not None:
            w, h = intr.get("image_size_px") or (nom["width_px"], nom["height_px"])
            proj = "equidistant" if "fisheye" in str(intr.get("model") or "") else "rectilinear"
            return cls(np.array(intr["K"]), np.array(intr.get("D") or []), w, h, proj, "calibration.yaml")
        return cls.nominal(nom)

    @classmethod
    def nominal(cls, nom: dict[str, Any]) -> "CameraModel":
        w, h = int(nom["width_px"]), int(nom["height_px"])
        proj = str(nom.get("projection", "rectilinear"))
        half = math.radians(float(nom["hfov_deg"])) / 2.0
        f = (w / 2.0) / (math.tan(half) if proj == "rectilinear" else half)
        K = [[f, 0.0, (w - 1) / 2.0], [0.0, f, (h - 1) / 2.0], [0.0, 0.0, 1.0]]
        return cls(np.array(K), np.zeros(0), w, h, proj, f"nominal {proj} {float(nom['hfov_deg']):g} deg lens")

    @property
    def f_px(self) -> float:
        return float(self.K[0, 0])

    @property
    def mrad_per_px(self) -> float:
        """At the image centre (both projections: 1 / f)."""
        return 1000.0 / self.f_px

    # ------------------------------------------------------------------ projections
    def normalized(self, u: float, v: float) -> tuple[float, float]:
        """Pinhole-normalised coordinates (direction = (a, b, 1)) of pixel (u, v), distortion removed."""
        if self._plain:
            x = (u - self.K[0, 2]) / self.K[0, 0]
            y = (v - self.K[1, 2]) / self.K[1, 1]
            if self.projection == "rectilinear":
                return x, y
            th = math.hypot(x, y)                         # equidistant: r = f theta
            if th < 1e-12:
                return 0.0, 0.0
            s = math.tan(min(th, 1.55)) / th
            return x * s, y * s
        import cv2
        pt = np.array([[[u, v]]], np.float64)
        n = (cv2.fisheye.undistortPoints(pt, self.K, self.D) if self.projection == "equidistant"
             else cv2.undistortPoints(pt, self.K, self.D))[0, 0]
        return float(n[0]), float(n[1])

    def pixel_to_angles(self, u: float, v: float) -> tuple[float, float]:
        """Offset of pixel (u, v) from the optical axis, rad (x right +, y down +): theta * unit direction."""
        a, b = self.normalized(u, v)
        r = math.hypot(a, b)
        if r < 1e-12:
            return 0.0, 0.0
        th = math.atan(r)
        return th * a / r, th * b / r

    def angles_to_pixel(self, ax: float, ay: float) -> tuple[float, float]:
        """Inverse of pixel_to_angles."""
        th = math.hypot(ax, ay)
        if th < 1e-12:
            return float(self.K[0, 2]), float(self.K[1, 2])
        if self._plain:
            r = math.tan(min(th, 1.55)) if self.projection == "rectilinear" else th
            return self.K[0, 2] + self.K[0, 0] * r * ax / th, self.K[1, 2] + self.K[1, 1] * r * ay / th
        import cv2
        t = math.tan(min(th, 1.55))
        a, b = t * ax / th, t * ay / th
        if self.projection == "equidistant":
            p = cv2.fisheye.distortPoints(np.array([[[a, b]]], np.float64), self.K, self.D)[0, 0]
        else:
            p = cv2.projectPoints(np.array([[a, b, 1.0]]), np.zeros(3), np.zeros(3), self.K, self.D)[0][0, 0]
        return float(p[0]), float(p[1])
