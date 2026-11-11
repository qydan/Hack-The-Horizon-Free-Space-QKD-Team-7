"""One frame through the whole tracker (the src/tracker.py ``GroundTracker.update`` loop, on the sim interfaces).

    SEARCH / ACQUIRE  full-frame lock-in (ArUco every few frames as the tag fallback); a candidate is refined on
                      the frame, and after ``acquire_frames`` consecutive consistent hits the filter starts (TRACK)
    TRACK / COAST     predict -> ROI around the predicted pixel -> lock-in confirms the blinking spot -> sub-pixel
                      centroid at its rolling-shutter row time -> world angle = gimbal readback at that row time
                      minus the pixel offset -> gated Kalman update; no measurement = COAST on the prediction
    then              pointing (gimbal command / virtual receiver), residual, drift / jitter, intensity (lock-in
                      amplitude / exposure gain / reference), tag pose every few frames

The gimbal readback is interpolated to each measurement's row time from a short history (the frame arrives
``latency`` after capture, when the gimbal is already elsewhere: without this a fast slew shows up as false
drone jitter, as the src/ gimbal docstring warned). Returns the telemetry fields for ``telemetry_out``.
"""
from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np

from sim_bridge.frame_source import Frame
from sim_bridge.gimbal_io import GimbalIO
from tracking.acquire import LockInDetector, Spot, SteadySpotDetector, detect_aruco, zone_box
from tracking.camera import CameraModel
from tracking.config import calib
from tracking.kalman import AngleKalman
from tracking.motion_estimate import MotionEstimator
from tracking.pointing import Pointer
from tracking.pose import TagPose, pose_from_corners
from tracking.state_machine import State, StateMachine
from tracking.track import ExposureEstimator, Measurement, RoiTracker


class Tracker:
    def __init__(self, cfg: dict[str, Any], rig: str, gimbal: GimbalIO, camera: CameraModel | None = None):
        self.cfg, self.rig, self.gimbal = cfg, rig, gimbal
        self.cam = camera or CameraModel.from_config(cfg)
        a = cfg["acquire"]
        self.target = str(a.get("target", "red_led"))
        self.steady = str(a.get("beacon", "blink")) == "steady"
        self.aruco_every = int(a.get("aruco_every_n_frames", 5))
        self.lockin = LockInDetector(cfg, float(cfg["track"].get("exposure_model", {}).get("gamma", 2.2)))
        self.spot = SteadySpotDetector(cfg)
        self.steady_persist = int(a.get("steady_persist_frames", 15))
        # blink mode: the lock-in has already put the beacon in the ROI, so the per-frame centroid (on redness, as
        # in steady mode: the LED sits on the dark drone body, where its red can fall below the room's) may use a
        # lower contrast and still catch the 40% phase
        self.confirmed_contrast = float(cfg["track"].get("confirmed_min_contrast", 4.0))
        self._steady_tracks: list = []
        self._last_spot: tuple[float, float] | None = None      # last measured centroid (full-resolution px)
        self.zone = zone_box(cfg, self.cam.width, self.cam.height)
        self.roi = RoiTracker(cfg, self.cam, float(calib(cfg, "rolling_shutter", "readout_s",
                                                         cfg["track"]["rolling_shutter_readout_s"])))
        self.kf = AngleKalman(cfg)
        self.sm = StateMachine(cfg)
        self.motion = MotionEstimator(cfg, rig)
        self.pointer = Pointer(cfg, rig, gimbal)
        self.exposure = ExposureEstimator(cfg)
        self.pose_every = int(cfg["pose"].get("every_n_frames", 3))
        self.tag_size = float(cfg["pose"]["tag_size_m"])
        self.cal_latency = calib(cfg, "latency", "capture_to_frame_s", None)
        self._hist: deque = deque(maxlen=128)          # gimbal readback (t, pan, tilt)
        self.kind: str | None = None                    # "led" | "tag" once acquired
        self._cand_uv: tuple[float, float] | None = None
        self._pose: TagPose | None = None
        self._pose_t = 0.0
        self.pose_max_age_s = float(cfg["pose"].get("max_age_s", 1.0))
        self._t_prev: float | None = None
        self.frame_i = 0
        self.acquisition_times: list[float] = []
        self.lock_losses = 0
        self.aborts = 0
        self.last_fields: dict[str, Any] = {}

    # ------------------------------------------------------------------ gimbal history
    def _record_gimbal(self):
        g = self.gimbal.read()
        if not self._hist or g.t_sim > self._hist[-1][0]:
            self._hist.append((g.t_sim, g.pan_rad, g.tilt_rad))
        return g

    def gimbal_at(self, t: float) -> tuple[float, float]:
        """Gimbal / mount angles at time ``t`` (linear interpolation of the readback history)."""
        h = np.array(self._hist)
        if len(h) == 1 or t <= h[0, 0]:
            return float(h[0, 1]), float(h[0, 2])
        return float(np.interp(t, h[:, 0], h[:, 1])), float(np.interp(t, h[:, 0], h[:, 2]))

    # ------------------------------------------------------------------ control
    def reacquire(self, t: float | None = None) -> None:
        """Gate abort (I8): drop the lock and search again."""
        t = self._t_prev if t is None else t
        if self.sm.state in (State.TRACK, State.COAST):
            self.lock_losses += 1
            if self.steady and self._last_spot is not None:
                # the spot we were tracking has already proved it stays lit: re-acquisition re-checks it with the
                # usual acquire_frames instead of the full steady persistence (in tracking_sweep 25 of 26 lock
                # losses were gate aborts, each costing ~0.65 s of re-proving a spot that never went away)
                u, v = self._last_spot
                self._steady_tracks = [(Spot(u, v, 0.0, 0.0), self.steady_persist)]
        self.aborts += 1
        self.sm.on_event(t or 0.0, "abort")
        self.kf.reset()
        self.kind = None
        self._cand_uv = None

    @property
    def acquisition_time(self) -> float | None:
        """Seconds from the start of the search to the first lock (the D1 '< 60 s' number)."""
        return self.acquisition_times[0] if self.acquisition_times else None

    # ------------------------------------------------------------------ search
    def _find_candidate(self, frame: Frame) -> Measurement | None:
        if self.target != "tag" and self.steady:
            s = self._persistent_spot(frame)
            if s is not None:
                m = self.roi.measure(frame, (s.x, s.y), None, channel=self.spot.redness(frame.image),
                                     min_contrast=self.spot.min_contrast)
                if m is not None:
                    return m
        elif self.target != "tag":
            c = self.lockin.candidates(self.zone, camera=self.cam, max_n=1)
            if c and c[0].confidence >= self.lockin.min_confidence:
                m = self.roi.measure(frame, (c[0].u_px, c[0].v_px), self.lockin, channel=self.spot.redness(frame.image),
                                     min_contrast=self.confirmed_contrast)
                if m is not None:
                    return m
        if self.target in ("tag", "auto") and self.frame_i % self.aruco_every == 0:
            c = detect_aruco(frame.image, self.cfg, self.cam, frame.t_capture_sim, self.zone)
            if c:
                return self.roi.measure(frame, (c[0].u_px, c[0].v_px), kind="tag")
        return None

    def _persistent_spot(self, frame: Frame):
        """Steady mode: follow every candidate spot frame to frame and offer only one that has stayed lit for
        ``steady_persist_frames`` in a row (the steadiest first, then the brightest). A steady LED qualifies; the
        2 Hz red indicator (on 0.25 s at a time) and passing glints never do, however red they are."""
        spots = self.spot.detect_all(frame.image, self.zone)
        gate = self.roi.roi / 4.0
        tracks = []
        for s in spots:
            prev = min(self._steady_tracks, key=lambda tr: np.hypot(tr[0].x - s.x, tr[0].y - s.y), default=None)
            hits = prev[1] + 1 if prev is not None and np.hypot(prev[0].x - s.x, prev[0].y - s.y) <= gate else 1
            tracks.append((s, hits))
        self._steady_tracks = tracks                          # a spot missing for one frame starts over
        ok = [tr for tr in tracks if tr[1] >= self.steady_persist]
        return max(ok, key=lambda tr: (tr[1], tr[0].peak))[0] if ok else None

    def _search(self, frame: Frame, t: float, g) -> Measurement | None:
        m = self._find_candidate(frame)
        if m is not None and self._cand_uv is not None and self.sm.state == State.ACQUIRE:
            if np.hypot(m.u_px - self._cand_uv[0], m.v_px - self._cand_uv[1]) > self.roi.roi / 2:
                self.sm.on_event(t, "no_candidate")          # a different spot: start counting again
        self._cand_uv = (m.u_px, m.v_px) if m is not None else None
        self.sm.on_event(t, "candidate" if m is not None else "no_candidate")
        if self.sm.locked_now:
            pan, tilt = self.gimbal_at(m.t_row)
            az, el = pan - m.dx_rad, tilt - m.dy_rad
            self.kf.init(m.t_row, az, el)
            self.kind = m.kind
            self.motion.on_lock()
            self.pointer.on_lock(t, az, el)
            since = self.sm.search_since
            self.acquisition_times.append(t - (t if since is None else since))
            return m
        self.pointer.search_step(t, self.sm.searching_for(t), g)
        return None

    # ------------------------------------------------------------------ main step
    def step(self, frame: Frame) -> dict[str, Any]:
        """Process one frame; return the telemetry fields."""
        t = frame.t_capture_sim
        dt = 0.0 if self._t_prev is None else max(0.0, t - self._t_prev)
        self._t_prev = t
        self.frame_i += 1
        g = self._record_gimbal()
        latency = frame.t_delivered_sim - t
        if latency <= 0 and self.cal_latency is not None:      # real camera: host stamps both at grab
            latency = float(self.cal_latency)
        gain = self.exposure.update(frame.image, t)
        if self.target != "tag" and self.kind != "tag" and not self.steady:
            self.lockin.push(frame)

        meas: Measurement | None = None
        z: np.ndarray | None = None
        if self.sm.state in (State.SEARCH, State.ACQUIRE):
            meas = self._search(frame, t, g)
            if meas is not None:
                pan, tilt = self.gimbal_at(meas.t_row)
                z = np.array([pan - meas.dx_rad, tilt - meas.dy_rad])
        else:
            pred = self.kf.predict(t)
            pan, tilt = self.gimbal_at(t)
            uv = self.cam.angles_to_pixel(pan - pred.position[0], tilt - pred.position[1])
            confirm = None if (self.kind == "tag" or self.steady) else self.lockin     # blink: lock-in confirms
            led = self.kind != "tag"
            m = self.roi.measure(frame, uv, confirm, self.kind or "led",
                                 channel=self.spot.redness(frame.image) if led else None,
                                 min_contrast=(self.spot.min_contrast if self.steady else self.confirmed_contrast)
                                 if led else None)
            if m is not None:
                pan, tilt = self.gimbal_at(m.t_row)
                zz = np.array([pan - m.dx_rad, tilt - m.dy_rad])
                if self.kf.update(m.t_row, zz):
                    meas, z = m, zz
            self.sm.on_event(t, "measurement" if meas is not None else "miss")
            if self.sm.lost_now:
                self.lock_losses += 1
                self.kf.reset()
                self.kind = None
            elif self.sm.state == State.COAST:
                self.kf.damp_velocity(dt)

        if meas is not None:
            self._last_spot = (meas.u_px, meas.v_px)
        fields: dict[str, Any] = {
            "t": t, "state": self.sm.state.value, "err_x_urad": 0.0, "err_y_urad": 0.0, "residual_mrad": 0.0,
            "drift_mrad": 0.0, "jitter_mrad": 0.0, "gimbal_lag_mrad": self.pointer.lag_mrad(g),
            "lean_deg": None, "range_m": None, "intensity": 0.0, "exposure_gain": gain,
            "spot_fwhm_px": 0.0, "snr": 0.0, "saturated": False, "latency_ms": 1e3 * max(0.0, latency),
            "gimbal_pan_deg": float(np.degrees(g.pan_rad)), "gimbal_tilt_deg": float(np.degrees(g.tilt_rad)),
            # optional schema fields for tools/accuracy_overlay.py: the centroid used this frame, full-resolution
            # pixels of the delivered frame (refine always runs at full resolution); null without a measurement
            "spot_u_px": None if meas is None else float(meas.u_px),
            "spot_v_px": None if meas is None else float(meas.v_px)}

        if self.sm.state in (State.TRACK, State.COAST) and self.kf.initialised:
            ks = self.kf.state()
            d = z if z is not None else np.array(ks.position)
            amp_norm = meas.intensity / max(gain, 1e-6) if (meas is not None and meas.kind == "led") else None
            self.motion.push(t, d, amp_norm)
            rx = self.pointer.step(t, ks, g, latency, None if meas is None else (meas.dx_rad, meas.dy_rad),
                                   t_now=max(frame.t_delivered_sim, g.t_sim), dt=dt)
            if self.pointer.mode == "gimbal":
                pan_t, tilt_t = self.gimbal_at(meas.t_row if meas is not None else t)
                res = (meas.dx_rad, meas.dy_rad) if meas is not None else (pan_t - d[0], tilt_t - d[1])
            else:
                if self.pointer.mode == "fixed_boresight":
                    b = self.motion.reference.current_boresight()
                    rx = tuple(b) if b is not None else rx
                res = self.pointer.residual_rad((float(d[0]), float(d[1])), rx)
            pan_t, tilt_t = self.gimbal_at(meas.t_row if meas is not None else t)
            err = (meas.dx_rad, meas.dy_rad) if meas is not None else (pan_t - d[0], tilt_t - d[1])
            fields.update(self.motion.fields())
            fields.update(err_x_urad=1e6 * err[0], err_y_urad=1e6 * err[1],
                          residual_mrad=1e3 * float(np.hypot(*res)), gimbal_lag_mrad=self.pointer.lag_mrad(g))
            if meas is not None:
                i_ref = self.motion.reference.current_i_ref()
                if meas.kind == "led":
                    fields["intensity"] = float(np.clip(amp_norm / i_ref, 0.0, 1.0)) if i_ref else 1.0
                else:
                    fields["intensity"] = 1.0
                fields.update(spot_fwhm_px=meas.spot_fwhm_px, snr=meas.snr, saturated=meas.saturated)
                if self.frame_i % self.pose_every == 0:
                    self._update_pose(frame, meas)
            if self._pose is not None and t - self._pose_t <= self.pose_max_age_s:
                fields.update(lean_deg=self._pose.lean_deg, range_m=self._pose.range_m)
        self.last_fields = fields
        return fields

    def _update_pose(self, frame: Frame, meas: Measurement) -> None:
        if meas.kind == "tag" and meas.corners is not None:
            corners = meas.corners
        else:
            from tracking.acquire import find_tag_corners
            corners = find_tag_corners(frame.image, self.cfg, self.roi.roi_box((meas.u_px, meas.v_px)))
        if corners is not None:
            p = pose_from_corners(corners, self.tag_size, self.cam)
            if p is not None:
                self._pose, self._pose_t = p, frame.t_capture_sim
