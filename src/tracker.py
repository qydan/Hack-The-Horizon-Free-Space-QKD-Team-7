"""Ground tracker (ground segment): finds the drone, locks onto the beacon, measures its motion.

    Stage 1  SEARCH   a drone-detection model boxes the drone anywhere in the wide view; the
                      box offset from the image centre becomes pan/tilt and the gimbal turns
                      to it. When the drone is near the centre and the 13 Hz beacon is seen,
                      control passes to fine track.
    Stage 2  TRACK    watch a small window around the beacon, find the blinking spot's centre
                      to a fraction of a pixel (lock-in detection), smooth and predict with a
                      Kalman filter, and steer the gimbal to keep the spot centred. If the
                      beacon vanishes for a moment keep following the prediction (COAST); if
                      it stays lost, go back to SEARCH.

Per frame it emits one ``TelemetryMessage`` (drift, jitter, residual, gimbal lag, lean, range,
intensity). The drone's direction is the gimbal encoder angle plus where the spot sits in the
image, so no drone telemetry is needed.

Run as its own program (publishes over ZeroMQ; ``demo_runner.py --source zmq`` is the
sandbox end):
    python src/tracker.py --source sim --realtime
    python src/tracker.py --source camera --fps 60 --calibrate 15    # real fixed camera

``--source camera`` is the fixed-camera mode (``FixedGimbal``): no search detector, no
steering and no AprilTag pose, so lean stays 0 and range stays at ``nominal_range_m``.
"""
import argparse
import json
import math
import os
import sys
import time
from collections import deque
from dataclasses import asdict, dataclass
from typing import Callable, Deque, Iterator, List, Optional, Tuple

import numpy as np

from gimbal import Gimbal, SimGimbal
from telemetry import (COAST, FINE_TRACK, SEARCH, TELEMETRY_ENDPOINT, ControlSubscriber, TelemetryMessage,
                       ZmqPublisher)
from vision import (Beacon, BrightBlobDetector, CameraModel, ClutterMap, KalmanCV, LockInDetector, Pose,
                    SteadySpotDetector, refine_centroid)


@dataclass
class Calibration:
    """Preflight reference: the calm-hover direction (boresight) and beacon brightness."""
    boresight_pan: float = 0.0      # mrad
    boresight_tilt: float = 0.0
    i_ref: float = 1.0              # lock-in amplitude of the beacon at full signal

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)

    @classmethod
    def load(cls, path: str) -> "Calibration":
        with open(path) as f:
            return cls(**json.load(f))


@dataclass
class TrackerConfig:
    fps: float = 120.0
    blink_hz: float = 13.0
    lockin_periods: int = 4            # blink periods per lock-in window (longer = better rejection)
    min_amplitude: float = 25.0        # grey levels of 13 Hz modulation needed to call it the beacon
    centre_tol_mrad: float = 60.0      # search -> fine track hand-over distance
    acquire_frames: int = 6            # consecutive beacon hits required
    coast_frames: int = 72             # frames of prediction before giving up (~0.6 s; the lock-in window needs ~0.3 s to refill after a dropout)
    roi_half_px: int = 40
    max_jump_mrad: float = 150.0       # reject a measurement this far from the prediction
    drift_cutoff_hz: float = 0.5       # drift = below this, jitter = above
    jitter_window_s: float = 1.0
    lead_s: float = 0.03               # aim ahead by the control latency
    pose_every: int = 6                # run the (slower) pose estimator every N frames
    nominal_range_m: float = 3.0
    scan_rate_mrad_s: float = 300.0    # pan sweep while nothing is found
    scan_range_mrad: float = 500.0
    fixed_camera: bool = False         # no gimbal: acquire the beacon anywhere in view, never steer or scan
    virtual_mount: bool = False        # fixed camera only: a software pan-tilt mount follows the drone and the residual
                                       # is measured against it (what a real gimbal would leave over)
    mount_latency_s: float = 0.06      # virtual mount lag: ~30 ms servo loop + frame interval and exposure at 30 fps
    steady_beacon: bool = False        # beacon does not blink: find it as a bright spot (min_amplitude = contrast)
    spot_max_area: int = 600           # steady mode: largest core (px) still counted as the LED
    spot_color: str = "auto"           # steady mode: "red" = red excess R - max(G, B) (ignores white lights),
                                       # "gray" = brightness; "auto" = red on colour frames
    spot_bg_kernel_px: Optional[int] = None   # steady mode: local-background window (default ~ frame width / 31)
    search_zone: Optional[Tuple[float, float, float, float]] = None   # (x0, y0, x1, y1) as 0-1 fractions of the frame:
                                                                      # only look for the beacon here (the hover area)


class GroundTracker:
    def __init__(self, camera: CameraModel, gimbal: Gimbal, calibration: Optional[Calibration] = None,
                 search_detector: Optional[Callable] = None, pose_fn: Optional[Callable] = None,
                 config: Optional[TrackerConfig] = None):
        self.cam = camera
        self.gimbal = gimbal
        self.cal = calibration or Calibration()
        self.cfg = config or TrackerConfig()
        self.search_detector = search_detector or BrightBlobDetector()
        self.pose_fn = pose_fn
        self.lockin = LockInDetector(self.cfg.blink_hz, self.cfg.fps, self.cfg.lockin_periods,
                                     self.cfg.min_amplitude)
        self.spot = SteadySpotDetector(self.cfg.min_amplitude, max_area=self.cfg.spot_max_area,
                                       color=self.cfg.spot_color, bg_kernel_px=self.cfg.spot_bg_kernel_px)
        self.clutter: Optional[ClutterMap] = None     # steady mode: known non-beacon spots (learn_clutter)
        self._acq_px: Optional[Tuple[float, float]] = None   # where the current run of acquisition hits is
        self._last_beacon: Optional[Beacon] = None
        self.kf = KalmanCV()
        self.vmount: Optional[SimGimbal] = (SimGimbal(latency_s=self.cfg.mount_latency_s)
                                            if self.cfg.virtual_mount and self.cfg.fixed_camera else None)

        self.state = SEARCH
        self._hits = 0
        self._misses = 0
        self._prev_t: Optional[float] = None
        self._search_started: Optional[float] = None
        self._frame_i = 0
        self._pose = Pose(0.0, self.cfg.nominal_range_m)
        self._lp: Optional[np.ndarray] = None
        self._hp: Deque = deque()
        self._last_dir = np.zeros(2)
        self._have_dir = False
        self._scan_dir = 1.0
        self._scan_origin: Optional[float] = None

        self.acquisition_times: List[float] = []   # seconds from search start to lock, per lock
        self.lock_losses = 0

    # ------------------------------------------------------------------ control hooks

    @property
    def acquisition_time(self) -> Optional[float]:
        """Time to the first lock (the D1 'under 60 seconds' number)."""
        return self.acquisition_times[0] if self.acquisition_times else None

    def reacquire(self) -> None:
        """Gate abort (I8): drop the lock and search again."""
        if self.state != SEARCH:
            self.lock_losses += 1
        self._to_search(self._prev_t if self._prev_t is not None else 0.0)

    def _to_search(self, t: float) -> None:
        self.state = SEARCH
        self._hits = self._misses = 0
        self._acq_px = None
        self.kf.reset()
        self._search_started = t
        self._scan_origin = None
        if self._have_dir:                       # look where the drone was last seen
            self.gimbal.command(*self._last_dir)

    def _find(self, frame: np.ndarray, roi=None, prefer=None) -> Optional[Beacon]:
        """The beacon in the frame (or in ``roi``): lock-in on a blinking LED, bright spot on a steady one.
        Steady mode: ``prefer`` = (x, y, sigma_px) favours spots near the prediction; a full-frame search
        skips learned clutter."""
        searching = roi is None
        if roi is None and self.cfg.search_zone is not None:
            roi = self._zone_roi(frame)
        if self.cfg.steady_beacon:
            exclude = self.clutter.excluded if (searching and self.clutter is not None) else None
            return self.spot.detect(frame, roi, exclude=exclude, prefer=prefer)
        if roi is None and not self.lockin.ready:
            return None
        return self.lockin.detect(roi)

    def _zone_roi(self, frame: np.ndarray):
        if self.cfg.search_zone is None:
            return None
        h, w = frame.shape[:2]
        zx0, zy0, zx1, zy1 = self.cfg.search_zone
        return int(zx0 * w), int(zy0 * h), int(zx1 * w), int(zy1 * h)

    def learn_clutter(self, source, seconds: float) -> int:
        """Steady mode, fixed camera: watch the scene with the beacon OFF (or out of view) and remember
        every LED-like spot (lamps, indicator LEDs, glints) so the search never locks onto one.
        Returns the number of masked regions."""
        start = None
        while True:
            item = source.read()
            if item is None:
                break
            frame, t = item
            start = t if start is None else start
            if self.clutter is None:
                self.clutter = ClutterMap(frame.shape)
            self.clutter.learn(self.spot.candidates(frame, self._zone_roi(frame)))
            if t - start >= seconds:
                break
        return self.clutter.count if self.clutter is not None else 0

    # ------------------------------------------------------------------ main step

    def update(self, frame: np.ndarray, t: float) -> TelemetryMessage:
        cfg = self.cfg
        dt = (t - self._prev_t) if self._prev_t is not None else 1.0 / cfg.fps
        self._prev_t = t
        self._frame_i += 1
        if self._search_started is None:
            self._search_started = t
        self._last_beacon = None

        if not cfg.steady_beacon:
            self.lockin.push(frame, t)
        enc = np.array(self.gimbal.angles(), dtype=float)       # camera pointing (0 for a fixed camera)
        cmd = np.array(self.gimbal.commanded, dtype=float)
        if self.vmount is not None:
            self.vmount.step_time(dt)
            mount = np.array(self.vmount.angles(), dtype=float)   # where the virtual receiver points
            lag = float(np.hypot(*(np.array(self.vmount.commanded) - mount)))
        else:
            mount = enc
            lag = float(np.hypot(*(cmd - enc)))

        beacon: Optional[Beacon] = None
        resid = 0.0
        measured_dir: Optional[np.ndarray] = None

        if self.state == SEARCH:
            self._search_step(frame, t, enc)
        else:
            # predict where the drone is and look for the beacon in a small window around it
            pred = np.array(self.kf.predict(dt))
            px, py = self.cam.mrad_to_px(pred[0] - enc[0], pred[1] - enc[1])
            r = cfg.roi_half_px
            beacon = self._find(frame, (int(px - r), int(py - r), int(px + r), int(py + r)),
                                prefer=(px, py, 0.5 * r))
            if beacon is not None:
                # lock-in averages over a few blink periods (lags a moving beacon): take this frame's exact
                # position. The steady detector already measured this frame, on its own (colour) image.
                fine = (beacon.x, beacon.y) if cfg.steady_beacon else refine_centroid(frame, px, py)
                if fine is not None:
                    off = np.array(self.cam.px_to_mrad(*fine))
                    d_new = enc + off
                    if np.hypot(*(d_new - pred)) > cfg.max_jump_mrad:   # outlier: not the beacon
                        fine = None
                if fine is None:
                    beacon = None
            if beacon is not None:
                self._last_beacon = beacon
                measured_dir = d_new
                resid = float(np.hypot(*(measured_dir - mount)))
                self.kf.update(*measured_dir)
                self.state, self._misses = FINE_TRACK, 0
            else:
                self._misses += 1
                self.state = COAST
                if self._misses >= cfg.coast_frames:
                    self.lock_losses += 1
                    self._to_search(t)
            if self.state == COAST:
                self.kf.x[2:] *= math.exp(-dt / 0.1)        # don't extrapolate far on a guess
            if self.state != SEARCH:
                vx, vy = self.kf.velocity
                tx, ty = self.kf.position
                self.gimbal.command(tx + vx * cfg.lead_s, ty + vy * cfg.lead_s)
                if self.vmount is not None:
                    self.vmount.command(tx + vx * cfg.lead_s, ty + vy * cfg.lead_s)

        # ---- measurements
        if self.state != SEARCH:
            d = measured_dir if measured_dir is not None else np.array(self.kf.position)
            self._last_dir = d
            self._have_dir = True
            self._update_motion(d, dt)
            if self.pose_fn is not None and self._frame_i % cfg.pose_every == 0:
                p = self.pose_fn(frame)
                if p is not None:
                    self._pose = p
        drift, jitter = self._drift_jitter()
        intensity = float(np.clip(beacon.amplitude / self.cal.i_ref, 0.0, 1.0)) if beacon else 0.0

        return TelemetryMessage(
            t=round(t, 5), state=self.state, drift_mrad=drift, jitter_mrad=jitter, residual_mrad=resid,
            gimbal_lag_mrad=lag, lean_deg=self._pose.lean_deg, range_m=self._pose.range_m, intensity=intensity)

    # ------------------------------------------------------------------ search stage

    def _search_step(self, frame, t, enc) -> None:
        cfg = self.cfg
        beacon = self._find(frame)
        if cfg.fixed_camera:
            # nothing to steer: the blinking spot alone says where the drone is
            if beacon is None:
                self._hits, self._acq_px = 0, None
                return
            fine = ((beacon.x, beacon.y) if cfg.steady_beacon else
                    refine_centroid(frame, beacon.x, beacon.y) or (beacon.x, beacon.y))
            # the hits must be the same spot, not whichever spot won in each frame
            if self._acq_px is not None and math.hypot(fine[0] - self._acq_px[0],
                                                       fine[1] - self._acq_px[1]) > 0.5 * cfg.roi_half_px:
                self._hits = 0
            self._acq_px = fine
            self._hits += 1
            if self._hits >= cfg.acquire_frames:
                self._last_beacon = beacon
                first = enc + np.array(self.cam.px_to_mrad(*fine))
                self.kf.init(*first)
                if self.vmount is not None:
                    self.vmount.place(*first)           # the virtual mount starts on the drone
                self.state, self._misses = FINE_TRACK, 0
                self.acquisition_times.append(t - self._search_started)
                self._lp, self._hp = None, deque()
            return
        box = self.search_detector(frame)
        if box is None and beacon is None:
            self._hits = 0
            self._scan(t, enc)
            return
        if box is not None:
            off = np.array(self.cam.px_to_mrad(box.cx, box.cy))
            self.gimbal.command(*(enc + off))
            near = float(np.hypot(*off)) < cfg.centre_tol_mrad
        else:
            near = False
        if near and beacon is not None:
            fine = refine_centroid(frame, beacon.x, beacon.y) or (beacon.x, beacon.y)
            boff = np.array(self.cam.px_to_mrad(*fine))
            if np.hypot(*boff) < cfg.centre_tol_mrad:
                self._hits += 1
                if self._hits >= cfg.acquire_frames:
                    self.kf.init(*(enc + boff))
                    self.state, self._misses = FINE_TRACK, 0
                    self.acquisition_times.append(t - self._search_started)
                    self._lp, self._hp = None, deque()
                return
        self._hits = 0

    def _scan(self, t: float, enc) -> None:
        """Nothing in view: sweep the pan axis back and forth."""
        cfg = self.cfg
        if self._scan_origin is None:
            self._scan_origin = float(enc[0])
        target = float(enc[0]) + self._scan_dir * cfg.scan_rate_mrad_s / cfg.fps * 4
        if abs(target - self._scan_origin) > cfg.scan_range_mrad:
            self._scan_dir *= -1
        self.gimbal.command(target, float(enc[1]))

    # ------------------------------------------------------------------ motion measures

    def _update_motion(self, d: np.ndarray, dt: float) -> None:
        if self._lp is None:
            self._lp = d.copy()
        a = 1.0 - math.exp(-2.0 * math.pi * self.cfg.drift_cutoff_hz * dt)
        self._lp = self._lp + a * (d - self._lp)
        self._hp.append(d - self._lp)
        while len(self._hp) > self.cfg.jitter_window_s * self.cfg.fps:
            self._hp.popleft()

    def _drift_jitter(self) -> Tuple[float, float]:
        if self._lp is None:
            return 0.0, 0.0
        drift = float(np.hypot(self._lp[0] - self.cal.boresight_pan, self._lp[1] - self.cal.boresight_tilt))
        h = np.array(self._hp)
        jitter = float(np.sqrt(np.mean(np.sum(h ** 2, axis=1)))) if len(h) else 0.0
        return drift, jitter

    # ------------------------------------------------------------------ calibration

    def calibrate(self, source, seconds: float = 10.0) -> Calibration:
        """Hold the drone in calm hover, then call this. Sets the boresight (mean tracked
        direction) and the full-signal beacon brightness, and returns the calibration."""
        dirs, amps, start = [], [], None
        while True:
            item = source.read()
            if item is None:
                break
            frame, t = item
            start = t if start is None else start
            msg = self.update(frame, t)
            if msg.state == FINE_TRACK:
                b = self._last_beacon       # the spot this frame's measurement came from
                if b is not None:
                    dirs.append(self._last_dir.copy())
                    amps.append(b.amplitude)
            if t - start >= seconds:
                break
        if not dirs:
            raise RuntimeError("calibration failed: beacon never tracked")
        use = slice(len(dirs) // 3, None)          # skip the settling period
        m = np.mean(dirs[use], axis=0)
        self.cal = Calibration(float(m[0]), float(m[1]), float(np.mean(amps[use])))
        return self.cal

    def run(self, source, control: Optional[ControlSubscriber] = None) -> Iterator[TelemetryMessage]:
        while True:
            item = source.read()
            if item is None:
                return
            if control is not None:
                for cmd in control.poll():
                    if cmd.get("cmd") == "reacquire":
                        self.reacquire()
            frame, t = item
            yield self.update(frame, t)


# --------------------------------------------------------------------------- program

def build_sim(args):
    from gimbal import SimGimbal
    from sim import Phase, SimRig
    cam = CameraModel(480, 270, 70.0)
    rig = SimRig(cam, SimGimbal(), duration=args.duration, fps=args.fps)
    tracker = GroundTracker(cam, rig.gimbal, pose_fn=rig.pose, config=TrackerConfig(fps=args.fps))
    # preflight calibration against a calm full-brightness reference
    cal_rig = SimRig(cam, SimGimbal(), phases=[Phase(0, 1e9, 2, 0.2, 0.5)], duration=15,
                     fps=args.fps, appear_at=0.0)
    cal_tracker = GroundTracker(cam, cal_rig.gimbal, config=TrackerConfig(fps=args.fps))
    tracker.cal = cal_tracker.calibrate(cal_rig, seconds=12.0)
    return tracker, rig


def build_camera(args):
    """Fixed camera (no gimbal) on the real rig. Run on Windows Python."""
    import cv2
    from gimbal import FixedGimbal
    from vision import VideoCaptureSource
    backend = cv2.CAP_DSHOW if sys.platform == "win32" else None
    src = VideoCaptureSource(args.device, args.cap_width, args.cap_height, args.fps, manual_exposure=args.exposure,
                             backend=backend, proc_width=args.proc_width, duration=args.duration)
    first = src.read()
    if first is None:
        raise SystemExit("camera gave no frames")
    h, w = first[0].shape[:2]
    cam = CameraModel(w, h, args.hfov)
    # the frame-count settings are tuned for 120 fps; rescale them to the real frame rate
    s = args.fps / 120.0
    cfg = TrackerConfig(fps=args.fps, blink_hz=args.blink_hz, min_amplitude=args.min_amplitude,
                        fixed_camera=True, virtual_mount=args.virtual_mount, mount_latency_s=args.mount_latency,
                        acquire_frames=max(3, round(6 * s)),
                        coast_frames=max(12, round(72 * s)), roi_half_px=max(20, round(40 * w / 480)),
                        max_jump_mrad=args.max_jump_mrad, steady_beacon=args.beacon == "steady",
                        spot_max_area=max(50, round(600 * (w / 960) ** 2)), spot_color=args.spot_color,
                        search_zone=tuple(float(v) for v in args.zone.split(",")) if args.zone else None)
    pose_fn = None
    if args.tag_size:
        from vision import AprilTagPose
        pose_fn = AprilTagPose(cam, args.tag_size)
    tracker = GroundTracker(cam, FixedGimbal(), pose_fn=pose_fn, config=cfg)
    print(f"camera {w}x{h} at {args.hfov:.0f} deg hfov ({cam.mrad_per_px:.2f} mrad/px), "
          f"{args.fps:.0f} fps, blink {args.blink_hz} Hz, "
          f"{'virtual mount, lag %.0f ms' % (1000 * args.mount_latency) if args.virtual_mount else 'no virtual mount'}")
    if args.clutter_learn > 0:
        if args.beacon != "steady":
            print("--clutter-learn only applies to --beacon steady (the lock-in ignores steady lights); skipped")
        else:
            print(f"learning clutter for {args.clutter_learn:.0f}s: the beacon must be OFF or out of view")
            src.restart(None)
            n = tracker.learn_clutter(src, args.clutter_learn)
            print(f"clutter: {n} LED-like spot(s) masked from the search; switch the beacon ON now")
            src.restart(args.duration)
    if args.calibrate > 0:
        print(f"calibrating for {args.calibrate:.0f}s: keep the drone in calm hover with the beacon blinking")
        src.restart(None)
        tracker.cal = tracker.calibrate(src, seconds=args.calibrate)
        tracker.cal.save(args.calibration)
        print(f"saved {args.calibration}: boresight ({tracker.cal.boresight_pan:.1f}, "
              f"{tracker.cal.boresight_tilt:.1f}) mrad, i_ref {tracker.cal.i_ref:.1f}")
        src.restart(args.duration)
        tracker.state = SEARCH
        tracker.kf.reset()
    elif os.path.exists(args.calibration):
        tracker.cal = Calibration.load(args.calibration)
        print(f"loaded {args.calibration}")
    else:
        print("no calibration file: boresight 0, intensity unscaled (run once with --calibrate 15)")
    return tracker, src


def main():
    # show output as it happens (even when piped) and never crash on a console that can't encode a character
    sys.stdout.reconfigure(line_buffering=True, errors="replace")
    ap = argparse.ArgumentParser(description="Ground tracker: publishes telemetry for the sandbox")
    ap.add_argument("--source", choices=["sim", "camera"], default="sim")
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--fps", type=float, default=120.0)
    ap.add_argument("--endpoint", default=TELEMETRY_ENDPOINT)
    ap.add_argument("--realtime", action="store_true", help="pace the simulated rig at its frame rate")
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--cap-width", type=int, default=1920, help="camera capture size (camera source)")
    ap.add_argument("--cap-height", type=int, default=1080)
    ap.add_argument("--proc-width", type=int, default=960, help="downscale frames to this width for tracking")
    ap.add_argument("--hfov", type=float, default=70.0, help="horizontal FOV in degrees after dewarping")
    ap.add_argument("--beacon", choices=["blink", "steady"], default="blink",
                    help="blink: lock-in on a blinking LED (robust to room lights). steady: constantly-lit LED "
                         "found as the strongest small, isolated red spot (see --spot-color, --clutter-learn)")
    ap.add_argument("--zone", default=None,
                    help="only look for the beacon in this part of the frame: x0,y0,x1,y1 as 0-1 fractions "
                         "(e.g. 0.25,0.1,0.75,0.9 = middle half); keeps lamps and windows outside it from stealing the lock")
    ap.add_argument("--spot-color", choices=["red", "gray"], default="red",
                    help="steady: red = detect on the red excess R - max(G, B), so white lamps, windows and screens "
                         "are ignored (red LED); gray = brightness only (needs a dark scene)")
    ap.add_argument("--clutter-learn", type=float, default=0.0,
                    help="steady: first watch the scene this many seconds with the beacon OFF and mask every "
                         "LED-like spot (lamps, indicator LEDs, glints) from the search")
    ap.add_argument("--blink-hz", type=float, default=13.0, help="beacon blink rate (e.g. 7.5 at 30 fps)")
    ap.add_argument("--min-amplitude", type=float, default=25.0,
                    help="blink: lock-in grey levels; steady: grey levels the LED must stand above the background")
    ap.add_argument("--max-jump-mrad", type=float, default=150.0)
    ap.add_argument("--exposure", type=float, default=None,
                    help="camera source: manual exposure value (DirectShow: 2^value seconds, e.g. -9 = 1/512 s). "
                         "Omit if exposure is already locked on the camera")
    ap.add_argument("--tag-size", type=float, default=None,
                    help="camera source: side of the AprilTag on the drone in metres (e.g. 0.09); enables lean and "
                         "range. Omit if there is no tag (lean 0, range 3 m)")
    ap.add_argument("--virtual-mount", action=argparse.BooleanOptionalAction, default=True,
                    help="camera source: measure the residual against a software pan-tilt mount that follows the "
                         "drone (what a real gimbal would leave over). --no-virtual-mount charges the drone's whole "
                         "wander as pointing error")
    ap.add_argument("--mount-latency", type=float, default=0.06,
                    help="virtual mount lag in seconds (servo loop plus frame interval and exposure)")
    ap.add_argument("--calibration", default="calibration.json")
    ap.add_argument("--calibrate", type=float, default=0.0, help="seconds of calm hover to calibrate on (camera)")
    ap.add_argument("--wait", type=float, default=1.0, help="seconds to wait for subscribers to connect")
    args = ap.parse_args()

    pub = ZmqPublisher(args.endpoint)
    from telemetry import CONTROL_ENDPOINT
    ctl = ControlSubscriber(CONTROL_ENDPOINT)
    if args.source == "sim":
        tracker, rig = build_sim(args)
    else:
        tracker, rig = build_camera(args)
    time.sleep(args.wait)
    t0 = time.monotonic()
    n = 0
    for msg in tracker.run(rig, ctl):
        pub.send(msg)
        n += 1
        if args.realtime:
            lag = msg.t - (time.monotonic() - t0)
            if lag > 0:
                time.sleep(lag)
    print(f"tracker: sent {n} messages; first lock after "
          f"{tracker.acquisition_time if tracker.acquisition_time is not None else float('nan'):.2f}s; "
          f"lock losses: {tracker.lock_losses}")
    time.sleep(0.5)
    pub.close()
    ctl.close()


if __name__ == "__main__":
    main()
