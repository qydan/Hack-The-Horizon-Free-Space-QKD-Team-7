"""Receiver pointing (ported from src/tracker.py gimbal steering and src/gimbal.py ``SimGimbal`` virtual mount).

* rig_gimbal: ``GimbalIO.set_target(pan, tilt)`` at most ``command_rate_hz``: the filtered direction plus a
  velocity lead of (frame latency + command latency) x ``feedforward_gain``, minus a PI term on the offset
  measured in the image (removes steady readback / alignment bias). The receiver shares the camera's boresight,
  so the residual is the measured image offset itself.
* rig_fixed, ``virtual_receiver`` (default): nothing to steer (``FixedMount.set_target`` is never called), so a
  software receiver follows the drone the way a mount would: first-order servo, slew limit, optional step
  quantisation, commanded with the same lead and applied after the real frame latency + ``virtual_receiver_lag_s``.
  The residual is measured against it: what a real mount would leave over.
* rig_fixed, ``fixed_boresight``: the receiver stays on the calibrated boresight (residual = drone offset from it).

Angles: rad; az / pan positive left, el / tilt positive up. Residual (x, y) = receiver boresight minus beacon
direction = the beacon's image offset convention (x right +, y down +).
Config: ``tracking.yaml: pointing.*``.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Any

import numpy as np

from sim_bridge.gimbal_io import GimbalIO, GimbalState


class VirtualReceiver:
    """Software pan-tilt mount in sim / capture time: latency queue, first-order servo, slew limit, steps."""

    def __init__(self, gain_per_s: float = 25.0, max_rate: float = 2.5, step: float = 0.0):
        self.gain, self.max_rate, self.step = gain_per_s, max_rate, step
        self._a = np.zeros(2)
        self._applied = np.zeros(2)
        self.commanded = np.zeros(2)
        self._queue: deque = deque()
        self.t: float | None = None

    def place(self, t: float, pan: float, tilt: float) -> None:
        self._a[:] = self._applied[:] = self.commanded[:] = (pan, tilt)
        self._queue.clear()
        self.t = t

    def command(self, t_apply: float, pan: float, tilt: float) -> None:
        self.commanded = np.array([pan, tilt])
        self._queue.append((t_apply, pan, tilt))

    def _integrate(self, dt: float) -> None:
        if dt <= 0:
            return
        err = self._applied - self._a
        move = err * (1.0 - math.exp(-self.gain * dt))
        lim = self.max_rate * dt
        self._a += np.clip(move, -lim, lim)

    def advance(self, t: float) -> None:
        if self.t is None:
            self.t = t
        while self._queue and self._queue[0][0] <= t:
            ta, p, q = self._queue.popleft()
            self._integrate(ta - self.t)
            self.t = max(self.t, ta)
            self._applied = np.array([p, q])
        self._integrate(t - self.t)
        self.t = max(self.t, t)

    def angles(self) -> np.ndarray:
        return np.round(self._a / self.step) * self.step if self.step > 0 else self._a.copy()


class Pointer:
    """Common pointing interface for both rigs."""

    def __init__(self, cfg: dict[str, Any], rig: str, gimbal: GimbalIO) -> None:
        p = cfg.get("pointing", cfg)
        self.rig, self.gimbal = rig, gimbal
        self.mode = "gimbal" if rig == "rig_gimbal" else str(p["mode_rig_fixed"])
        if self.mode not in ("gimbal", "virtual_receiver", "fixed_boresight"):
            raise ValueError(f"pointing.mode_rig_fixed must be fixed_boresight or virtual_receiver, got {self.mode!r}")
        self.ff = float(p["feedforward_gain"])
        self.kp, self.ki = float(p["pi_kp"]), float(p["pi_ki"])
        self.i_lim = float(p.get("integrator_limit_rad", 0.01))
        self.cmd_period = 1.0 / float(p["command_rate_hz"])
        self.cmd_latency = float(p.get("command_latency_s", 0.0))
        self.servo_lag = float(p.get("gimbal_servo_lag_s", 0.0))      # velocity lag of the gimbal's own loop
        self.vr_lag = float(p["virtual_receiver_lag_s"])
        self.vr = VirtualReceiver(float(p.get("virtual_receiver_gain_per_s", 25.0)),
                                  float(p.get("virtual_receiver_max_rate_rad_s", 2.5)),
                                  float(p.get("virtual_receiver_step_rad", 0.0)))
        self.lost_timeout = float(cfg.get("state_machine", {}).get("lost_timeout_s", 5.0))
        self._integ = np.zeros(2)
        self._last_cmd_t: float | None = None
        self._commanded: np.ndarray | None = None
        self.home: tuple[float, float] | None = None
        self._homed = False
        self.boresight: np.ndarray | None = None     # fixed_boresight mode (set by the tracker's reference)

    # ------------------------------------------------------------------ events
    def on_lock(self, t: float, az: float, el: float) -> None:
        self._integ[:] = 0.0
        self._homed = False
        if self.mode == "virtual_receiver":
            self.vr.place(t, az, el)           # the virtual mount starts on the drone

    def search_step(self, t: float, searching_for_s: float, g: GimbalState) -> None:
        """While searching: rig_gimbal holds still, and returns home after ``lost_timeout_s``."""
        if self.home is None:
            self.home = (g.pan_rad, g.tilt_rad)
        if self.mode == "gimbal" and searching_for_s >= self.lost_timeout and not self._homed:
            self.gimbal.set_target(*self.home)
            self._commanded = np.array(self.home)
            self._homed = True

    # ------------------------------------------------------------------ per frame while locked
    def step(self, t: float, track_state: Any, gimbal_state: GimbalState, latency_s: float = 0.0,
             offset: tuple[float, float] | None = None, t_now: float | None = None, dt: float = 0.0) -> tuple[float, float]:
        """Update pointing for a frame captured at ``t``; return the receiver boresight (az, el) in rad.
        ``offset``: the measured image offset (dx, dy) this frame (None while coasting)."""
        if self.home is None:
            self.home = (gimbal_state.pan_rad, gimbal_state.tilt_rad)
        pos = np.array(track_state.position)
        vel = np.array(track_state.velocity)
        if self.mode == "gimbal":
            if offset is not None and self.ki:
                self._integ = np.clip(self._integ + np.array(offset) * dt, -self.i_lim, self.i_lim)
            now = t if t_now is None else t_now
            if self._last_cmd_t is None or now - self._last_cmd_t >= self.cmd_period - 1e-9:
                lead = self.ff * (max(0.0, now - t) + self.cmd_latency + self.servo_lag)
                corr = self.kp * (np.array(offset) if offset is not None else 0.0) + self.ki * self._integ
                tgt = pos + vel * lead - corr
                self.gimbal.set_target(float(tgt[0]), float(tgt[1]))
                self._commanded, self._last_cmd_t = tgt, now
            return gimbal_state.pan_rad, gimbal_state.tilt_rad
        if self.mode == "virtual_receiver":
            self.vr.advance(t)
            rx = self.vr.angles()
            delay = latency_s + self.vr_lag
            # lead over the frame latency and the command lag, plus the servo's own velocity lag (a first-order
            # servo trails a ramp by 1 / gain): without it a 1 m/s sweep left ~13 mrad of residual
            tgt = pos + vel * self.ff * (delay + 1.0 / self.vr.gain)
            self.vr.command(t + delay, float(tgt[0]), float(tgt[1]))
            return float(rx[0]), float(rx[1])
        b = self.boresight if self.boresight is not None else pos
        return float(b[0]), float(b[1])

    def residual_rad(self, beacon_dir: tuple[float, float], receiver: tuple[float, float]) -> tuple[float, float]:
        """Receiver boresight minus beacon direction (x, y) in rad = image-offset convention (x right, y down)."""
        return receiver[0] - beacon_dir[0], receiver[1] - beacon_dir[1]

    def lag_mrad(self, gimbal_state: GimbalState) -> float | None:
        """Commanded minus actual receiver angle (the lag part of the residual); None for a fixed boresight."""
        if self.mode == "gimbal":
            if self._commanded is None:
                return 0.0
            return 1e3 * float(np.hypot(self._commanded[0] - gimbal_state.pan_rad,
                                        self._commanded[1] - gimbal_state.tilt_rad))
        if self.mode == "virtual_receiver":
            return 1e3 * float(np.hypot(*(self.vr.commanded - self.vr.angles())))
        return None
