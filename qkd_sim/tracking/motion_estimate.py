"""Motion estimates for telemetry: drift and jitter (ported from src/tracker.py ``_update_motion`` /
``_drift_jitter``), plus the boresight / brightness reference they are measured against.

* drift_mrad   slow (< ~0.5 Hz) wander of the drone's direction from the reference boresight: a first-order
               low-pass with time constant ``drift_window_s``
* jitter_mrad  RMS of the fast part (direction minus the low-pass) over ``jitter_window_s``

Reference (src/ ``Calibration``: boresight + full-signal brightness). ``calibration/preflight_brightness.py``
writes the brightness reference to config/calibration.yaml; without it, and always for the boresight, the
tracker references itself once the drone is *hovering*: after ``boresight_settle_s`` it slides a
``boresight_window_s`` window and fixes the reference only when the direction shows no trend across it (the two
halves' means within ``boresight_max_trend_mrad``). src/ asked for a calm hover at a ``--calibrate`` step; in the
sim the drone takes off at t = 0 and the tracker locks while it is still on the floor, and a reference taken
during the climb read as 100-180 mrad of drift for the whole run (the gate aborted every block). Until the
reference is fixed, the window's mean serves as a provisional one.

The residual and gimbal lag come from ``pointing``; lean and range from ``pose`` (null without a tag).
Config: ``tracking.yaml: motion_estimate.*``.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Any

import numpy as np


class Reference:
    """Boresight (world az, el) and brightness reference i_ref (lock-in amplitude / exposure gain)."""

    def __init__(self, cfg: dict[str, Any]):
        m = cfg.get("motion_estimate", cfg)
        self.settle = float(m["boresight_settle_s"])
        self.window = float(m["boresight_window_s"])
        self.max_trend = float(m.get("boresight_max_trend_mrad", 30.0)) * 1e-3
        self.follow_tau = float(m.get("boresight_follow_tau_s", 0.0) or 0.0)
        self._t_last: float | None = None
        pb = (cfg.get("calibration") or {}).get("preflight_brightness") or {}
        self.i_ref: float | None = None
        self.i_ref_source = "in-run (first lock)"
        if pb.get("i_cal") is not None:
            r_cal, r_ref = pb.get("range_m"), m.get("reference_range_m")
            scale = (float(r_cal) / float(r_ref)) ** 2 if (r_cal and r_ref) else 1.0
            self.i_ref = float(pb["i_cal"]) * scale
            self.i_ref_source = "calibration.yaml preflight_brightness"
        self.boresight: np.ndarray | None = None
        self.t_fixed: float | None = None
        self._t_lock: float | None = None
        self._win: deque = deque()                    # (t, direction, amp or None) over the last window
        self._prov_dir: np.ndarray | None = None      # latest sample during the settle time
        self._prov_amp: float | None = None
        self._done = False

    @property
    def complete(self) -> bool:
        return self._done

    def push(self, t: float, direction: np.ndarray, amp_norm: float | None) -> None:
        if self._done:
            # follow the long-run hover position slowly: the station is the mean of the hover, and a window that
            # happened to be fixed on a take-off overshoot or a wander excursion would bias drift for the whole run
            if self.follow_tau > 0 and self._t_last is not None and t > self._t_last:
                a = 1.0 - math.exp(-(t - self._t_last) / self.follow_tau)
                self.boresight = self.boresight + a * (np.asarray(direction, float) - self.boresight)
            self._t_last = t
            return
        if self._t_lock is None:
            self._t_lock = t
        d = np.asarray(direction, float)
        amp = float(amp_norm) if (amp_norm is not None and amp_norm > 0) else None
        self._prov_dir = d
        if amp is not None:
            self._prov_amp = amp
        if t - self._t_lock < self.settle:
            return
        self._win.append((t, d, amp))
        while self._win and t - self._win[0][0] > self.window:
            self._win.popleft()
        if t - self._win[0][0] >= self.window - 0.05:
            dirs = np.array([w[1] for w in self._win])
            half = len(dirs) // 2
            trend = float(np.hypot(*(dirs[:half].mean(axis=0) - dirs[half:].mean(axis=0))))
            if trend <= self.max_trend:              # hovering: no climb or transit across the window
                self.boresight = dirs.mean(axis=0)
                amps = [w[2] for w in self._win if w[2] is not None]
                if self.i_ref is None and amps:
                    self.i_ref = float(np.median(amps))
                self.t_fixed = self._t_last = t
                self._done = True

    def current_boresight(self) -> np.ndarray | None:
        if self.boresight is not None:
            return self.boresight
        if self._win:
            return np.mean([w[1] for w in self._win], axis=0)
        return self._prov_dir

    def current_i_ref(self) -> float | None:
        if self.i_ref is not None:
            return self.i_ref
        amps = [w[2] for w in self._win if w[2] is not None]
        return float(np.median(amps)) if amps else self._prov_amp


class MotionEstimator:
    """Sliding-window drift / jitter estimator."""

    def __init__(self, cfg: dict[str, Any], rig: str) -> None:
        m = cfg.get("motion_estimate", cfg)
        self.rig = rig
        self.tau = float(m["drift_window_s"])
        self.jitter_window = float(m["jitter_window_s"])
        self.reference = Reference(cfg)
        self._lp: np.ndarray | None = None
        self._hp: deque = deque()
        self._t: float | None = None

    def on_lock(self) -> None:
        """New lock: restart the filters (the previous track's history is not this one's)."""
        self._lp, self._t = None, None
        self._hp.clear()

    def push(self, t: float, direction: Any, amp_norm: float | None = None) -> None:
        """Add the direction (az, el) of one frame (measured, or the filter's while coasting)."""
        d = np.asarray(direction, float)
        dt = 0.0 if self._t is None else max(0.0, t - self._t)
        self._t = t
        if self._lp is None:
            self._lp = d.copy()
        else:
            self._lp = self._lp + (1.0 - math.exp(-dt / self.tau)) * (d - self._lp)
        self._hp.append((t, d - self._lp))
        while self._hp and t - self._hp[0][0] > self.jitter_window:
            self._hp.popleft()
        self.reference.push(t, d, amp_norm)

    def fields(self) -> dict[str, float | None]:
        """drift_mrad and jitter_mrad (0 before the first lock)."""
        if self._lp is None:
            return {"drift_mrad": 0.0, "jitter_mrad": 0.0}
        b = self.reference.current_boresight()
        drift = 0.0 if b is None else 1e3 * float(np.hypot(*(self._lp - b)))
        h = np.array([v for _, v in self._hp])
        jitter = 1e3 * float(np.sqrt(np.mean(np.sum(h ** 2, axis=1)))) if len(h) else 0.0
        return {"drift_mrad": drift, "jitter_mrad": jitter}
