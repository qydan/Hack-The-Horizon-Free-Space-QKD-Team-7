"""Simple drone flight model: the default instead of PX4 SITL (``drone.yaml: flight_model: simple``).

Stands in for "pilot parks it, DJI vision positioning holds it" without any autopilot: no arming, no health
checks, nothing that can refuse to fly. The drone is a point mass with
  * a position controller (PID toward the setpoint) whose output is the thrust's horizontal acceleration,
  * the desk-fan jet + turbulence acting as drag  F = 0.5 rho CdA |w - v| (w - v),
  * attitude = the tilt that points the thrust where the controller wants it (lean into the wind), capped at
    the Mini 2 SE's 40 deg.
In steady wind the integral term cancels the drag, so the lean settles at atan(F / m g), the same quasi-static
value as a real multirotor, and the drone is pushed sideways first and then recovers.

The setpoint is the same as with PX4: take-off, then the hover station + Ornstein-Uhlenbeck wander (+ slow
push for T6), from :class:`sim_bridge.hover_station.SetpointGenerator`. The pose is written to Gazebo with
set_pose at ``simple_flight.update_rate_hz`` (the drone model has gravity off in this mode).
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from sim_bridge.fan_wind import FanJet, WindField, drag_force, schedule_level
from sim_bridge.hover_station import SetpointGenerator

log = logging.getLogger("simple_flight")
G = 9.81


def tilt_quaternion(a_thrust_h, yaw_rad: float, g: float = G) -> tuple[float, float, float, float]:
    """Body attitude (w, x, y, z) whose z axis is along the thrust direction (a_h, g), with heading ``yaw``."""
    z = np.array([a_thrust_h[0], a_thrust_h[1], g], float)
    z /= np.linalg.norm(z)
    x_head = np.array([math.cos(yaw_rad), math.sin(yaw_rad), 0.0])
    y = np.cross(z, x_head)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    R = np.column_stack([x, y, z])
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    if w < 1e-6:                                   # not reachable for tilts below 90 deg
        return (1.0, 0.0, 0.0, 0.0)
    return (w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w))


@dataclass
class SimpleFlight:
    cfg_drone: dict[str, Any]
    cfg_fan: dict[str, Any]
    cfg_world: dict[str, Any]
    spawn: np.ndarray
    station: np.ndarray
    fan_schedule: list[dict[str, Any]] = field(default_factory=list)
    seed: int = 0
    wander: bool | None = None
    push_start_s: float | None = None
    gnss: bool = False
    station_schedule: list[tuple[float, Any]] = field(default_factory=list)
    path: list[dict[str, Any]] | None = None       # scripted path (sim_bridge/flight_path.py) instead of wander

    def __post_init__(self) -> None:
        sf = self.cfg_drone["simple_flight"]
        sk = self.cfg_drone["station_keeping"]
        self.mass = float(self.cfg_drone["spec"]["takeoff_mass_kg"]) + float(
            self.cfg_drone["payload"]["variants"][self.cfg_drone["payload"]["variant"]]["mass_kg"])
        self.cda = float(self.cfg_drone["aero"]["body_cd_area_m2"])
        self.rho = float(self.cfg_drone["aero"]["air_density_kgpm3"])
        wn = 2 * math.pi * float(sf["position_bandwidth_hz"])
        z = float(sf["damping_ratio"])
        self.kp, self.kd = wn * wn, 2 * z * wn
        self.ki = float(sf["integral_gain_ratio"]) * wn ** 3
        self.tilt_max = math.radians(float(self.cfg_drone["spec"]["max_pitch_deg"]))
        self.v_max_h = float(self.cfg_drone["spec"]["max_horizontal_speed_mps"])
        self.climb_s = float(sf["takeoff_climb_s"])
        self.yaw = math.radians(float(sk["start_yaw_deg"]))
        self.dt = 1.0 / float(sf["integration_rate_hz"])
        rng = np.random.default_rng(self.seed)
        self.gen = SetpointGenerator.from_cfg(self.cfg_drone, self.station, rng, wander=self.wander,
                                              push_start_s=self.push_start_s, gnss=self.gnss)
        self.wind = WindField(FanJet.from_cfg(self.cfg_fan, self.cfg_world, "off"),
                              float(self.cfg_fan["turbulence"]["intensity"]), float(self.cfg_fan["turbulence"]["tau_s"]),
                              np.random.default_rng(self.seed + 1))
        self.levels = self.cfg_fan["levels_mps"]
        self.level = "off"
        self.p = np.asarray(self.spawn, float).copy()
        self.v = np.zeros(3)
        self.i_err = np.zeros(3)
        self.a_thrust = np.zeros(3)
        self.last_wind = np.zeros(3)
        self.t = 0.0
        self.t0: float | None = None
        self.takeoff_target = np.array([self.spawn[0], self.spawn[1], float(sk["takeoff_alt_m"])])

    # ---------------------------------------------------------------------------------------------------------
    def setpoint(self, t: float) -> np.ndarray:
        el = t - self.t0
        for t_s, xyz in sorted(self.station_schedule, key=lambda it: it[0]):
            if t >= t_s:
                self.gen.station = np.asarray(xyz, float)
        if el < self.climb_s:                                   # vertical take-off
            return self.spawn + (self.takeoff_target - self.spawn) * (el / self.climb_s)
        if el < 2 * self.climb_s:                               # transit to the station
            f = (el - self.climb_s) / self.climb_s
            return self.takeoff_target + (self.gen.station - self.takeoff_target) * f
        if self.path:                                           # controlled tracking test
            from sim_bridge.flight_path import offset_at
            return self.gen.station + offset_at(self.path, el - 2 * self.climb_s)
        return self.gen.at(t)

    def path_windows(self) -> list[dict[str, Any]]:
        """Path segments with absolute sim times (empty without a path or before the first step)."""
        if not self.path or self.t0 is None:
            return []
        from sim_bridge.flight_path import windows
        t_path = self.t0 + 2 * self.climb_s
        return [{**w, "t_start": t_path + w["t_start"], "t_end": t_path + w["t_end"]} for w in windows(self.path)]

    def step(self, t: float) -> None:
        if self.t0 is None:
            self.t0 = t
            self.t = t
        dt = t - self.t
        if dt <= 0:
            return
        level = schedule_level(self.fan_schedule, t - self.t0)
        if level != self.level:
            self.level = level
            self.wind.set_level(float(self.levels[level]))
            log.info("t=%.2f fan level -> %s", t - self.t0, level)
        n = max(1, int(math.ceil(dt / self.dt)))
        h = dt / n
        sp = self.setpoint(t)
        for _ in range(n):
            self.wind.step(h)
            w = self.wind.velocity(self.p)
            f_drag = drag_force(w, self.v, self.cda, self.rho)
            err = sp - self.p
            self.i_err += err * h
            a_cmd = self.kp * err - self.kd * self.v + self.ki * self.i_err
            # the thrust can tilt at most tilt_max: limit the horizontal part
            a_h = a_cmd[:2]
            a_h_max = (G + a_cmd[2]) * math.tan(self.tilt_max)
            nh = np.linalg.norm(a_h)
            if nh > a_h_max > 0:
                a_h = a_h * (a_h_max / nh)
                self.i_err[:2] -= err[:2] * h                   # anti-windup while saturated
            self.a_thrust = np.array([a_h[0], a_h[1], a_cmd[2]])
            a = self.a_thrust + f_drag / self.mass
            self.v += a * h
            vh = np.linalg.norm(self.v[:2])
            if vh > self.v_max_h:
                self.v[:2] *= self.v_max_h / vh
            self.p += self.v * h
            if self.p[2] < self.spawn[2]:                       # floor
                self.p[2], self.v[2] = self.spawn[2], max(0.0, self.v[2])
            self.last_wind = w
        self.t = t

    def attitude(self) -> tuple[float, float, float, float]:
        return tilt_quaternion(self.a_thrust[:2], self.yaw, G + self.a_thrust[2])

    def lean_deg(self) -> float:
        return math.degrees(math.atan2(float(np.linalg.norm(self.a_thrust[:2])), G + self.a_thrust[2]))

    # ---------------------------------------------------------------------------------------------------------
    def run(self, world, model: str, *, until: float | None = None, stop=lambda: False) -> None:  # pragma: no cover
        from sim_bridge.sim_loop import SimTicker
        rate = float(self.cfg_drone["simple_flight"]["update_rate_hz"])
        ticker = SimTicker(world, rate, until=until, stop=stop)
        for t in ticker:
            self.step(t)
            world.set_pose(model, self.p, self.attitude())
        log.info("simple flight: pose updates at %.1f Hz (target %.1f)", ticker.achieved_rate_hz(), rate)
