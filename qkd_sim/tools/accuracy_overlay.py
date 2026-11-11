"""Tracker-vs-truth overlay for the live camera view (``run_scenario --view``). EVALUATION SIDE: uses ground truth.

For every camera frame it draws
  * GREEN ring  : where the LED really is (truth beacon position projected through the exact simulated camera:
                  lens model, principal-point offset, mount error),
  * MAGENTA cross: where the tracker measured it (telemetry ``spot_u_px`` / ``spot_v_px`` for that same frame),
  * a line between them and the error in px and mrad, plus the tracker state and a running RMS over TRACK frames.

Frames are matched to the tracker's telemetry record for the *same* capture time (the tracker publishes after
processing, so frames are held briefly, ``hold_s``, waiting for their record). Truth is taken at the frame's
capture time.
"""
from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from typing import Any

import numpy as np

from sim_bridge.camera_model import make_lens, project_world_point, rot_zyx

DEG = math.pi / 180.0


class TelemetryTap:
    """Background subscriber to the I2 telemetry port; keeps recent records by capture time."""

    def __init__(self, bus: dict[str, Any], keep: int = 600):
        import zmq
        self.sock = zmq.Context.instance().socket(zmq.SUB)
        self.sock.connect(f"tcp://{bus['host']}:{bus['ports']['telemetry']}")
        self.sock.setsockopt(zmq.SUBSCRIBE, b"")
        self.records: deque[dict[str, Any]] = deque(maxlen=keep)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True, name="telemetry-tap")
        self.thread.start()

    def _run(self) -> None:
        while not self.stop.is_set():
            if self.sock.poll(200):
                try:
                    rec = json.loads(self.sock.recv_multipart()[-1])
                except (ValueError, IndexError):
                    continue
                with self.lock:
                    self.records.append(rec)

    def find(self, t: float, tol: float = 0.002) -> dict[str, Any] | None:
        with self.lock:
            for rec in reversed(self.records):
                if abs(float(rec.get("t", -1e9)) - t) <= tol:
                    return rec
        return None

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=2.0)                # the poll loop wakes every 200 ms; close only after it exits
        self.sock.close(linger=0)


def truth_pixel(lens, truth: dict[str, Any]) -> np.ndarray | None:
    R = rot_zyx(truth["cam_yaw_deg"] * DEG, truth["cam_pitch_deg"] * DEG, truth["cam_roll_deg"] * DEG)
    uv = project_world_point(lens, np.asarray(truth["lens_pos_m"]), R, np.asarray(truth["beacon_pos_m"]))
    return None if not np.all(np.isfinite(uv)) else uv


def angle_between_px(lens, a, b) -> float:
    da, db = lens.unproject(np.asarray([a, b], float))
    return math.acos(max(-1.0, min(1.0, float(da @ db))))


class AccuracyOverlay:
    def __init__(self, cfgs: dict[str, dict[str, Any]], rig: str, truth_history: deque, telemetry: TelemetryTap | None,
                 *, hold_s: float = 0.6):
        self.lens = make_lens(cfgs["camera_action5"], cfgs["rigs"][rig]["principal_point_offset_px"])
        self.truth = truth_history
        self.tel = telemetry
        self.hold_s = hold_s
        self.pending: deque[tuple[float, Any, float]] = deque()    # (wall time, frame, ...)
        self.err_sq: list[float] = []
        self.n_track = 0
        self.n_frames = 0

    def _truth_at(self, t: float) -> dict[str, Any] | None:
        best, bdt = None, 0.05
        for rec in reversed(self.truth):
            dt = abs(rec["t_sim"] - t)
            if dt < bdt:
                best, bdt = rec, dt
            elif rec["t_sim"] < t - 0.05:
                break
        return best

    def push(self, frame) -> list[tuple[Any, list[str]]]:
        """Queue a frame; return frames that are ready to show, each as (annotated image, text lines)."""
        self.pending.append((time.monotonic(), frame))
        out = []
        while self.pending:
            t_wall, fr = self.pending[0]
            rec = self.tel.find(fr.t_capture_sim) if self.tel is not None else None
            if rec is None and time.monotonic() - t_wall < self.hold_s and self.tel is not None:
                break
            self.pending.popleft()
            out.append(self.annotate(fr, rec))
        return out

    def annotate(self, fr, rec: dict[str, Any] | None) -> tuple[Any, list[str]]:
        import cv2
        img = fr.image.copy()
        self.n_frames += 1
        truth = self._truth_at(fr.t_capture_sim)
        tp = truth_pixel(self.lens, truth) if truth else None
        lines = []
        state = rec.get("state") if rec else "no tracker record"
        if state == "TRACK":
            self.n_track += 1
        sp = None
        if rec and rec.get("spot_u_px") is not None and rec.get("spot_v_px") is not None:
            sp = (float(rec["spot_u_px"]), float(rec["spot_v_px"]))
        if sp is not None and tp is not None:                      # error line first, markers on top
            cv2.line(img, tuple(int(round(v)) for v in tp), tuple(int(round(v)) for v in sp), (0, 255, 255), 2, cv2.LINE_AA)
        if tp is not None:
            c = tuple(int(round(v)) for v in tp)
            cv2.circle(img, c, 22, (0, 255, 0), 3, cv2.LINE_AA)
            cv2.circle(img, c, 3, (0, 255, 0), -1, cv2.LINE_AA)
        if sp is not None:
            cv2.drawMarker(img, tuple(int(round(v)) for v in sp), (255, 0, 255), cv2.MARKER_CROSS, 40, 3, cv2.LINE_AA)
        if sp is not None and tp is not None:
            err_px = float(np.hypot(sp[0] - tp[0], sp[1] - tp[1]))
            err_mrad = 1e3 * angle_between_px(self.lens, sp, tp)
            if state == "TRACK":
                self.err_sq.append(err_mrad ** 2)
            rms = math.sqrt(sum(self.err_sq[-900:]) / len(self.err_sq[-900:])) if self.err_sq else float("nan")
            lines.append(f"tracker {state}: error {err_px:.1f} px = {err_mrad:.2f} mrad   RMS(TRACK, last 30 s) {rms:.2f} mrad")
        elif rec and rec.get("spot_u_px", "missing") == "missing":
            lines.append(f"tracker {state}: (telemetry has no spot_u_px/spot_v_px yet: truth only)")
        else:
            lines.append(f"tracker {state}: no spot this frame")
        frac = self.n_track / max(1, self.n_frames)
        lines.append(f"GREEN = truth LED   MAGENTA = tracker   TRACK {100 * frac:.0f}% of frames")
        return img, lines
