"""Tag pose (lean and range) from the ArUco tag (ported from src/vision.py ``tag_pose`` / ``AprilTagPose``,
moved to DICT_4X4_50 and the Action 5 Pro lens, rectilinear or fisheye).

The four marker corners are undistorted to pinhole-normalised coordinates with the camera model, then
``cv2.solvePnP(IPPE_SQUARE)`` with an identity camera matrix. lean_deg is the angle between the tag's normal (the
beacon axis, fixed to the drone body) and the line of sight to the camera: how far the body-mounted transmitter
points away from the ground station. range_m is the distance to the tag.

Known limit: at 3 m the 8 cm tag spans about 21 px, too few for a reliable lean estimate (see README).
Config: ``tracking.yaml: pose.tag_size_m``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from sim_bridge.frame_source import Frame


@dataclass
class TagPose:
    """Tag pose in the camera frame."""
    lean_deg: float
    range_m: float
    rvec: np.ndarray
    tvec: np.ndarray


def pose_from_corners(corners_px: np.ndarray, tag_size_m: float, camera_model: Any) -> TagPose | None:
    """Pose from the four marker corners (ArUco order: top-left, top-right, bottom-right, bottom-left)."""
    h = tag_size_m / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    # undistorted corners on a virtual pinhole camera of the same focal length (well-conditioned pixel units)
    f = float(camera_model.f_px)
    K = np.array([[f, 0.0, 0.0], [0.0, f, 0.0], [0.0, 0.0, 1.0]])
    img = f * np.array([camera_model.normalized(float(u), float(v)) for u, v in np.asarray(corners_px).reshape(4, 2)],
                       dtype=np.float64)
    # IPPE_SQUARE alone returned a wrong (identity) pose with 15 px reprojection error on OpenCV 5.0; take the
    # best-reprojecting solution of IPPE and the iterative solver
    best = None
    for flag in (cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_ITERATIVE):
        try:
            n, rvs, tvs, errs = cv2.solvePnPGeneric(obj, img, K, None, flags=flag)
        except cv2.error:
            continue
        for rv, tv, e in zip(rvs, tvs, np.asarray(errs).reshape(-1)):
            if best is None or float(e) < best[0]:
                best = (float(e), rv, tv)
    if best is None or best[2].reshape(3)[2] <= 0:
        return None
    _, rvec, tvec = best
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)
    rng = float(np.linalg.norm(t))
    if rng <= 0:
        return None
    cosang = abs(float(np.dot(R[:, 2], -t / rng)))
    return TagPose(math.degrees(math.acos(min(1.0, cosang))), rng, rvec.reshape(3), t)


def estimate_tag_pose(frame: Frame, camera_model: Any, cfg: dict[str, Any], roi=None) -> TagPose | None:
    """Detect the tag (optionally only inside ``roi`` = (x0, y0, x1, y1) px) and estimate its pose."""
    from tracking.acquire import find_tag_corners
    corners = find_tag_corners(frame.image, cfg, roi)
    if corners is None:
        return None
    return pose_from_corners(corners, float(cfg.get("pose", cfg)["tag_size_m"]), camera_model)
