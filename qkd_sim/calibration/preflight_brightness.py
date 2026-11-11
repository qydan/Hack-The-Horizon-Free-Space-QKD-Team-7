"""Known-range brightness reference (I_cal), with the exposure gain recorded (ported from the brightness half of
src/tracker.py ``GroundTracker.calibrate``).

Over ``n_frames`` the beacon is found with the tracker's own lock-in detector and ROI tracker, and its lock-in
amplitude is divided by the estimated exposure gain (``tracking.track.ExposureEstimator``). The median is I_cal:
the tracker's ``intensity`` is the beacon's amplitude / gain / I_cal, scaled from ``range_m`` to the hover
station's ``motion_estimate.reference_range_m`` by the inverse-square law. Without this file entry the tracker
references itself on its first lock (calm hover).

    python -m calibration.preflight_brightness --rig rig_fixed --range 4.0 --frames 300   # during calib_brightness

Sim support: ``scenarios/calib_brightness.yaml``. Output keys: ``config/calibration.yaml: preflight_brightness.*``.
Rig behaviour: same for both rigs.
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np


def measure_reference(source: Any, range_m: float, n_frames: int, cfg: dict[str, Any] | None = None) -> dict[str, Any]:
    """Measure the beacon intensity at a known range; return I_cal and the exposure gain estimate."""
    from tracking.acquire import LockInDetector
    from tracking.camera import CameraModel
    from tracking.config import load_tracking_cfg
    from tracking.track import ExposureEstimator, RoiTracker
    cfg = cfg or load_tracking_cfg()
    cam = CameraModel.from_config(cfg)
    det, expo, roi = LockInDetector(cfg), ExposureEstimator(cfg), RoiTracker(cfg, cam)
    amps, gains, n_read = [], [], 0
    while len(amps) < n_frames and n_read < 4 * n_frames + 10 * det.n:
        fr = source.read(timeout_s=5.0)
        if fr is None:
            if getattr(source, "finished", False):
                break
            continue
        n_read += 1
        g = expo.update(fr.image, fr.t_capture_sim)
        det.push(fr)
        c = det.candidates(camera=cam, max_n=1)
        if not c or c[0].confidence < det.min_confidence:
            continue
        m = roi.measure(fr, (c[0].u_px, c[0].v_px), det)
        if m is not None and m.intensity > 0:
            amps.append(m.intensity / g)
            gains.append(g)
    if not amps:
        raise RuntimeError("brightness calibration failed: the beacon was never found")
    return {"i_cal": float(np.median(amps)), "range_m": float(range_m), "exposure_gain": float(np.mean(gains)),
            "n_frames": len(amps), "spread": float(np.std(amps) / np.median(amps))}


def save(result: dict[str, Any], path: Path | None = None) -> None:
    """Write the preflight_brightness section."""
    from calibration.yaml_io import update_section
    update_section("preflight_brightness", {k: result[k] for k in ("i_cal", "range_m", "exposure_gain")}, path)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rig", default="rig_fixed", choices=["rig_fixed", "rig_gimbal"])
    ap.add_argument("--range", type=float, default=4.0, help="known beacon range, m (calib_brightness: 4 m)")
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--hardware", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="print, don't write config/calibration.yaml")
    a = ap.parse_args()
    from sim_bridge.frame_source import make_frame_source
    src = make_frame_source(hardware=a.hardware, rig=a.rig)
    try:
        res = measure_reference(src, a.range, a.frames)
    finally:
        src.close()
    print(res)
    if not a.dry_run:
        save(res)
        print("written to config/calibration.yaml: preflight_brightness")


if __name__ == "__main__":
    main()
