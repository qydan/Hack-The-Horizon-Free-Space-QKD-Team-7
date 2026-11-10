"""Gimbal / mount interface shared by the simulator and the real hardware (section 8).

``GimbalIO`` is one of the two sim_bridge modules tracking/ may import. Implementations:
  * :class:`FixedMount`        config A (rig_fixed): tripod + ball head, nothing to steer.
  * :class:`Storm32GimbalSim`  config B (rig_gimbal): emulated STorM32 3-axis gimbal driving Gazebo joints.
  * :class:`Storm32Gimbal`     config B real hardware over serial (MAVLink Gimbal Protocol v2). Stub.

Angle conventions (all radians): pan positive = rotate left (counter-clockwise seen from above, about +z);
tilt positive = camera looks up; roll positive = right-hand about the optical axis. Zero pan/tilt looks
along world +x.

Use :func:`make_gimbal_io` to build the implementation for a rig; tracking code should depend only on the
``GimbalIO`` protocol.
"""
from __future__ import annotations

import logging
import math
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Protocol, runtime_checkable

import numpy as np

from sim_bridge import config as cfgmod
from sim_bridge.random_processes import BandLimitedNoise, RandomWalk

log = logging.getLogger("gimbal_io")

DEG = math.pi / 180.0


@dataclass(frozen=True)
class GimbalState:
    pan_rad: float
    tilt_rad: float
    roll_rad: float
    t_sim: float


@runtime_checkable
class GimbalIO(Protocol):
    def set_target(self, pan_rad: float, tilt_rad: float) -> None:
        """Command a pan/tilt angle setpoint (roll is always held level)."""

    def read(self) -> GimbalState:
        """Latest attitude readback, stamped with sim time (host time on real hardware)."""


# ---------------------------------------------------------------------------------------------------------
# Config A
# ---------------------------------------------------------------------------------------------------------
class FixedMount:
    """Config A: nothing to steer. ``read()`` returns the *nominal* mount angles (not the deliberate mount
    error, which calibration must find) with the current sim time; ``set_target()`` only logs a warning."""

    def __init__(self, cfg_rig: dict[str, Any], clock: Callable[[], float]):
        self._pan = float(cfg_rig["nominal_pan_deg"]) * DEG
        self._tilt = float(cfg_rig["nominal_tilt_deg"]) * DEG
        self._roll = float(cfg_rig.get("nominal_roll_deg", 0.0)) * DEG
        self._clock = clock
        self.warnings = 0

    def set_target(self, pan_rad: float, tilt_rad: float) -> None:
        self.warnings += 1
        if self.warnings <= 3 or self.warnings % 1000 == 0:
            log.warning("FixedMount.set_target(%.4f, %.4f) ignored: rig_fixed has no pan-tilt (call #%d)",
                        pan_rad, tilt_rad, self.warnings)

    def read(self) -> GimbalState:
        return GimbalState(self._pan, self._tilt, self._roll, float(self._clock()))


# ---------------------------------------------------------------------------------------------------------
# Config B emulation: pure dynamics (no Gazebo), then the Gazebo-driving wrapper
# ---------------------------------------------------------------------------------------------------------
@dataclass
class _Axis:
    theta: float = 0.0
    omega: float = 0.0


@dataclass
class Storm32Dynamics:
    """STorM32 emulation core, integrated at a fixed step in sim time.

    * Commands are sampled at ``command.max_rate_hz`` (latest wins) and take effect ``command.latency_s``
      later.
    * Each axis follows a second-order response (wn, zeta) with a symmetric rate limit.
    * ``offset_schedule`` (T6) adds pan/tilt offset steps after the controller target.
    * Holding jitter (band-limited) is added to the physical camera attitude.
    * Readback (IMU-based, no encoders) = true attitude + white noise + random-walk pan bias.
    """
    cfg: dict[str, Any]
    rng: np.random.Generator
    t: float = 0.0
    pan: _Axis = field(default_factory=_Axis)
    tilt: _Axis = field(default_factory=_Axis)

    def __post_init__(self) -> None:
        c = self.cfg
        self.dt = float(c["response"]["integration_dt_s"])
        self.wn = 2 * math.pi * float(c["response"]["natural_frequency_hz"])
        self.zeta = float(c["response"]["damping_ratio"])
        self.rate_limit = float(c["response"]["rate_limit_dps"]) * DEG
        self.cmd_period = 1.0 / float(c["command"]["max_rate_hz"])
        self.latency = float(c["command"]["latency_s"])
        self.pan_lim = tuple(v * DEG for v in c["limits_deg"]["pan"])
        self.tilt_lim = tuple(v * DEG for v in c["limits_deg"]["tilt"])
        jit = c["holding_jitter"]
        fs = 1.0 / self.dt
        self._jit = [BandLimitedNoise(float(jit["rms_deg"]) * DEG, float(jit["band_hz"][0]),
                                      float(jit["band_hz"][1]), fs, self.rng) for _ in range(3)]
        self.jitter = np.zeros(3)                       # pan, tilt, roll jitter (rad)
        rb = c["readback"]
        self.rb_noise = float(rb["noise_rms_deg"]) * DEG
        self.rb_period = 1.0 / float(rb["rate_hz"])
        drift = float(rb["pan_drift_deg_per_min"]) * DEG          # 1-sigma after 60 s
        self._drift = RandomWalk(drift / math.sqrt(60.0), self.rng)
        self.offsets = sorted(c.get("offset_schedule") or [], key=lambda d: float(d["t_s"]))
        self._pending: tuple[float, float] | None = None          # latest command not yet sampled
        self._queue: deque[tuple[float, float, float]] = deque()  # (t_effective, pan, tilt)
        self._next_sample = 0.0
        self.target = (0.0, 0.0)
        self._last_readback: GimbalState | None = None
        self._next_readback = 0.0
        self.active = True                                         # False = motors off (T1 "gimbal off")

    def set_active(self, active: bool) -> None:
        self.active = bool(active)

    def reset(self, pan: float, tilt: float, t: float = 0.0) -> None:
        self.pan, self.tilt = _Axis(pan), _Axis(tilt)
        self.target = (pan, tilt)
        self.t = t
        self._next_sample = t
        self._next_readback = t

    def command(self, pan: float, tilt: float) -> None:
        self._pending = (float(pan), float(tilt))

    def offset_at(self, t: float) -> tuple[float, float]:
        pan = tilt = 0.0
        for o in self.offsets:
            if t >= float(o["t_s"]):
                pan, tilt = float(o.get("pan_mrad", 0.0)) * 1e-3, float(o.get("tilt_mrad", 0.0)) * 1e-3
        return pan, tilt

    def _axis_step(self, ax: _Axis, ref: float) -> None:
        acc = self.wn * self.wn * (ref - ax.theta) - 2.0 * self.zeta * self.wn * ax.omega
        ax.omega += acc * self.dt
        ax.omega = max(-self.rate_limit, min(self.rate_limit, ax.omega))
        ax.theta += ax.omega * self.dt

    def step(self) -> None:
        t = self.t + self.dt
        if t + 1e-12 >= self._next_sample:
            if self._pending is not None:
                self._queue.append((t + self.latency, *self._pending))
                self._pending = None
            self._next_sample += self.cmd_period
        while self._queue and self._queue[0][0] <= t + 1e-12:
            _, p, q = self._queue.popleft()
            self.target = (min(max(p, self.pan_lim[0]), self.pan_lim[1]),
                           min(max(q, self.tilt_lim[0]), self.tilt_lim[1]))
        op, ot = self.offset_at(t)
        jit = np.array([j.sample() for j in self._jit])      # keep the noise streams advancing either way
        if self.active:
            self._axis_step(self.pan, self.target[0] + op)
            self._axis_step(self.tilt, self.target[1] + ot)
            self.jitter = jit
        else:                                                 # motors off (T1): camera rests, no jitter
            self.pan.omega = self.tilt.omega = 0.0
            self.jitter = np.zeros(3)
        self._drift.step(self.dt)
        self.t = t
        if t + 1e-12 >= self._next_readback:
            self._last_readback = GimbalState(
                self.pan.theta + self.jitter[0] + self._drift.x + self.rb_noise * float(self.rng.standard_normal()),
                self.tilt.theta + self.jitter[1] + self.rb_noise * float(self.rng.standard_normal()),
                self.jitter[2] + self.rb_noise * float(self.rng.standard_normal()),
                t)
            self._next_readback += self.rb_period

    def advance_to(self, t_target: float) -> None:
        while self.t + self.dt <= t_target + 1e-12:
            self.step()

    def true_attitude(self) -> tuple[float, float, float]:
        """Physical camera attitude (pan, tilt, roll) including holding jitter."""
        return (self.pan.theta + self.jitter[0], self.tilt.theta + self.jitter[1], self.jitter[2])

    def readback(self) -> GimbalState:
        if self._last_readback is None:
            p, q, r = self.true_attitude()
            return GimbalState(p, q, r, self.t)
        return self._last_readback


class Storm32GimbalSim:
    """Config B emulation. Steps :class:`Storm32Dynamics` on the Gazebo sim clock in a background thread and
    publishes the true attitude as JointPositionController targets for the pan/roll/tilt joints.

    Parameters
    ----------
    world: a :class:`sim_bridge.gz_io.GzWorld` (or any object with ``sim_time``, ``wait_until`` and
        ``publish_double``). Pass ``world=None`` together with ``clock`` for an offline instance that is
        advanced manually with :meth:`advance_to` (used by tests A12).
    """

    def __init__(self, cfg_gimbal: dict[str, Any], cfg_rig: dict[str, Any], *, world=None,
                 clock: Callable[[], float] | None = None, seed: int = 0, publish_rate_hz: float = 250.0):
        self.cfg = cfg_gimbal
        self.dyn = Storm32Dynamics(cfg_gimbal, np.random.default_rng(seed))
        self.world = world
        self._clock = clock or (lambda: world.sim_time)
        self.dyn.reset(float(cfg_rig["initial_pan_deg"]) * DEG, float(cfg_rig["initial_tilt_deg"]) * DEG,
                       float(self._clock()))
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.publish_period = 1.0 / publish_rate_hz
        self.topics = cfg_gimbal["joint_control"]["topics"]

    # GimbalIO ------------------------------------------------------------------------------------------
    def set_target(self, pan_rad: float, tilt_rad: float) -> None:
        with self._lock:
            self.dyn.command(pan_rad, tilt_rad)

    def read(self) -> GimbalState:
        with self._lock:
            return self.dyn.readback()

    # Offline / threaded stepping ------------------------------------------------------------------------
    def advance_to(self, t_sim: float) -> None:
        with self._lock:
            self.dyn.advance_to(t_sim)

    def true_attitude(self) -> tuple[float, float, float]:
        with self._lock:
            return self.dyn.true_attitude()

    def _publish_joints(self) -> None:  # pragma: no cover - needs Gazebo
        pan, tilt, roll = self.true_attitude()
        self.world.publish_double(self.topics["pan"], pan)
        self.world.publish_double(self.topics["tilt"], tilt)
        self.world.publish_double(self.topics["roll"], roll)

    def _run(self) -> None:  # pragma: no cover - needs Gazebo
        t_next = self.world.sim_time
        while not self._stop.is_set():
            if not self.world.wait_until(t_next, timeout_wall_s=1.0):
                continue
            self.advance_to(self.world.sim_time)
            self._publish_joints()
            t_next = self.world.sim_time + self.publish_period

    def start(self) -> "Storm32GimbalSim":  # pragma: no cover - needs Gazebo
        if self.world is None:
            raise RuntimeError("start() needs a running world; use advance_to() offline")
        self._thread = threading.Thread(target=self._run, name="storm32-sim", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)


class Storm32GimbalServer:  # pragma: no cover - needs Gazebo
    """Runs inside the bridge host (tools/run_scenario.py), which owns the one Storm32GimbalSim. Publishes
    readback at the configured readback rate and accepts set_target commands from :class:`Storm32GimbalRemote`."""

    def __init__(self, sim: Storm32GimbalSim, bus: dict[str, Any]):
        import zmq
        self.sim = sim
        ctx = zmq.Context.instance()
        self.pub = ctx.socket(zmq.PUB)
        self.pub.bind(f"tcp://{bus['host']}:{bus['internal_ports']['gimbal_state']}")
        self.sub = ctx.socket(zmq.SUB)
        self.sub.bind(f"tcp://{bus['host']}:{bus['internal_ports']['gimbal_cmd']}")
        self.sub.setsockopt(zmq.SUBSCRIBE, b"")
        self.period = 1.0 / float(sim.cfg["readback"]["rate_hz"])

    def run(self, stop=lambda: False) -> None:
        import json

        import zmq
        world = self.sim.world
        t_next = world.sim_time
        while not stop():
            while True:
                try:
                    msg = self.sub.recv_json(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break
                self.sim.set_target(float(msg["pan_rad"]), float(msg["tilt_rad"]))
            if world.wait_until(t_next, timeout_wall_s=0.2):
                s = self.sim.read()
                self.pub.send_multipart([b"gimbal", json.dumps(s.__dict__).encode()])
                t_next = world.sim_time + self.period


class Storm32GimbalRemote:
    """GimbalIO client for a Storm32GimbalSim owned by another process (the bridge host)."""

    def __init__(self, bus: dict[str, Any]):
        import zmq
        ctx = zmq.Context.instance()
        self.cmd = ctx.socket(zmq.PUB)
        self.cmd.connect(f"tcp://{bus['host']}:{bus['internal_ports']['gimbal_cmd']}")
        self.state = ctx.socket(zmq.SUB)
        self.state.connect(f"tcp://{bus['host']}:{bus['internal_ports']['gimbal_state']}")
        self.state.setsockopt(zmq.SUBSCRIBE, b"gimbal")
        self._last: GimbalState | None = None

    def set_target(self, pan_rad: float, tilt_rad: float) -> None:
        self.cmd.send_json({"pan_rad": float(pan_rad), "tilt_rad": float(tilt_rad)})

    def read(self, timeout_s: float = 1.0) -> GimbalState:
        import json
        poll_ms = int(timeout_s * 1000) if self._last is None else 0
        while self.state.poll(poll_ms):
            _, payload = self.state.recv_multipart()
            self._last = GimbalState(**json.loads(payload))
            poll_ms = 0
        if self._last is None:
            raise TimeoutError("no gimbal readback: is the bridge host (tools/run_scenario.py) running?")
        return self._last


# ---------------------------------------------------------------------------------------------------------
# Config B real hardware (stub)
# ---------------------------------------------------------------------------------------------------------
class Storm32Gimbal:
    """Real STorM32 gimbal over USB-UART using MAVLink 2 Gimbal Protocol v2 (pymavlink).

    Messages it will use:
      * ``GIMBAL_MANAGER_SET_PITCHYAW`` (or ``GIMBAL_MANAGER_SET_ATTITUDE``) to command pan (yaw) and tilt
        (pitch) angle setpoints, with flags ``GIMBAL_MANAGER_FLAGS_YAW_LOCK`` cleared (vehicle-frame yaw).
      * ``GIMBAL_DEVICE_ATTITUDE_STATUS`` (IMU-based attitude quaternion) for :meth:`read`, stamped with host
        time on receipt; ``GIMBAL_MANAGER_STATUS`` / ``GIMBAL_DEVICE_INFORMATION`` at start-up.
      * ``HEARTBEAT`` from the laptop (as a GCS) so STorM32 accepts commands; ``COMMAND_LONG``
        ``MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`` to take primary control.
    Configuration: ``gimbal_storm32.yaml: real_hardware``.
    """

    def __init__(self, cfg_gimbal: dict[str, Any]):
        self.cfg = cfg_gimbal

    def set_target(self, pan_rad: float, tilt_rad: float) -> None:
        raise NotImplementedError("Storm32Gimbal (real hardware) is not implemented yet")

    def read(self) -> GimbalState:
        raise NotImplementedError("Storm32Gimbal (real hardware) is not implemented yet")


# ---------------------------------------------------------------------------------------------------------
def make_gimbal_io(rig: str, *, hardware: bool = False, local: bool = False, world=None,
                   clock: Callable[[], float] | None = None, seed: int = 0,
                   overrides: dict[str, Any] | None = None) -> GimbalIO:
    """Build the GimbalIO for a rig.

    * rig_fixed  -> FixedMount (sim clock from Gazebo, or host clock with ``hardware=True``).
    * rig_gimbal -> by default a :class:`Storm32GimbalRemote` connected to the emulator owned by the bridge
      host (tools/run_scenario.py); ``local=True`` builds an in-process :class:`Storm32GimbalSim` instead
      (tests, or when no bridge host runs); ``hardware=True`` -> :class:`Storm32Gimbal`.
    """
    rigs = cfgmod.load("rigs")
    gim = cfgmod.deep_merge(cfgmod.load("gimbal_storm32"), (overrides or {}).get("gimbal_storm32"))
    cfg_rig = rigs[rig]
    if rig == "rig_fixed":
        if clock is None:
            if hardware:
                import time
                clock = time.monotonic
            else:
                if world is None:
                    from sim_bridge.gz_io import GzWorld
                    world = GzWorld(cfgmod.load("world")["world_name"])
                clock = lambda: world.sim_time  # noqa: E731
        return FixedMount(cfg_rig, clock)
    if rig == "rig_gimbal":
        if hardware:
            return Storm32Gimbal(gim)
        if not local:
            return Storm32GimbalRemote(cfgmod.load("bus"))
        if world is None and clock is None:
            from sim_bridge.gz_io import GzWorld
            world = GzWorld(cfgmod.load("world")["world_name"])
        sim = Storm32GimbalSim(gim, cfg_rig, world=world, clock=clock, seed=seed)
        if world is not None:
            sim.start()
        return sim
    raise ValueError(f"unknown rig {rig!r}")
