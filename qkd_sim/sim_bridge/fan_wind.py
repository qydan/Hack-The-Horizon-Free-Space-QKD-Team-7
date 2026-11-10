"""Desk-fan local jet + turbulence -> drag force on the drone (section 7).

Gazebo's built-in wind is global, so the jet is modelled here and applied as a persistent wrench through the
ApplyLinkWrench system (loaded in the world SDF), updated at ``fan.yaml: force.update_rate_hz`` in sim time.

Jet model (fan at p_f, unit axis a, diameter D, outlet speed u0):
  s  = (p - p_f) . a                       distance along the axis (no flow behind the fan, s <= 0)
  r  = |(p - p_f) - s a|                   radial distance
  u_c(s) = u0                 for s <= 3 D (potential core)
         = u0 * 3 D / s       beyond
  b(s) = D/2 + 0.1 s                       half-width (read as half width at half maximum)
  u(s, r) = u_c(s) * exp(-ln2 (r / b)^2)
Turbulence: independent OU gusts on x, y, z with std = 15% of the local mean speed, tau = 0.3 s.
Force: F = 0.5 rho (Cd A) |v_rel| v_rel,  v_rel = v_wind - v_drone.

Run standalone:  python -m sim_bridge.fan_wind --level high --seed 1
"""
from __future__ import annotations

import argparse
import logging
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from sim_bridge import config as cfgmod
from sim_bridge.random_processes import OrnsteinUhlenbeck

log = logging.getLogger("fan_wind")


@dataclass
class FanJet:
    position: np.ndarray
    axis: np.ndarray
    diameter: float
    outlet_speed: float
    core_diameters: float = 3.0
    hw_base_frac: float = 0.5
    hw_growth: float = 0.1

    @classmethod
    def from_cfg(cls, cfg_fan: dict[str, Any], cfg_world: dict[str, Any], level: str) -> "FanJet":
        axis = np.asarray(cfg_world["fan"]["axis"], float)
        return cls(np.asarray(cfg_world["fan"]["position_m"], float), axis / np.linalg.norm(axis),
                   float(cfg_fan["diameter_m"]), float(cfg_fan["levels_mps"][level]),
                   float(cfg_fan["jet"]["potential_core_diameters"]),
                   float(cfg_fan["jet"]["half_width_base_fraction_of_d"]),
                   float(cfg_fan["jet"]["half_width_growth_per_m"]))

    def centreline_speed(self, s: float) -> float:
        if s <= 0.0:
            return 0.0
        core = self.core_diameters * self.diameter
        return self.outlet_speed if s <= core else self.outlet_speed * core / s

    def half_width(self, s: float) -> float:
        return self.hw_base_frac * self.diameter + self.hw_growth * max(s, 0.0)

    def mean_velocity(self, p) -> np.ndarray:
        d = np.asarray(p, float) - self.position
        s = float(d @ self.axis)
        if s <= 0.0 or self.outlet_speed <= 0.0:
            return np.zeros(3)
        r = float(np.linalg.norm(d - s * self.axis))
        b = self.half_width(s)
        u = self.centreline_speed(s) * math.exp(-math.log(2.0) * (r / b) ** 2)
        return u * self.axis


@dataclass
class WindField:
    """Mean jet plus OU turbulence. ``step`` advances the gusts; ``velocity`` is the total wind at p."""
    jet: FanJet
    intensity: float
    tau_s: float
    rng: np.random.Generator
    _ou: OrnsteinUhlenbeck = field(init=False)

    def __post_init__(self) -> None:
        self._ou = OrnsteinUhlenbeck(np.ones(3), self.tau_s, self.rng)   # unit-std gust shape

    def set_level(self, outlet_speed: float) -> None:
        self.jet.outlet_speed = float(outlet_speed)

    def step(self, dt: float) -> None:
        self._ou.step(dt)

    def velocity(self, p) -> np.ndarray:
        mean = self.jet.mean_velocity(p)
        return mean + self.intensity * float(np.linalg.norm(mean)) * self._ou.x


def drag_force(v_wind, v_body, cd_area: float, rho: float) -> np.ndarray:
    v_rel = np.asarray(v_wind, float) - np.asarray(v_body, float)
    return 0.5 * rho * cd_area * float(np.linalg.norm(v_rel)) * v_rel


def steady_lean_deg(wind_speed: float, cd_area: float, mass_kg: float, rho: float = 1.2, g: float = 9.81) -> float:
    """Quasi-static lean angle needed to hold station against a steady wind (no rotor drag)."""
    f = 0.5 * rho * cd_area * wind_speed ** 2
    return math.degrees(math.atan2(f, mass_kg * g))


def schedule_level(schedule: list[dict[str, Any]] | None, t: float, default: str = "off") -> str:
    """Fan schedule: list of {t_s, level}, sorted; returns the level active at t."""
    level = default
    for item in schedule or []:
        if t >= float(item["t_s"]):
            level = str(item["level"])
    return level


class FanWindBridge:  # pragma: no cover - needs Gazebo
    """Applies the jet force to a model link in a running world, and exposes the wind for truth."""

    def __init__(self, world, cfgs: dict[str, dict[str, Any]], *, model: str, link: str, cd_area: float,
                 schedule: list[dict[str, Any]] | None, seed: int, use_step_wrench: bool = False):
        self.world = world
        self.model, self.link = model, link
        self.cd_area = float(cd_area)
        self.rho = float(cfgs["fan"]["force"]["air_density_kgpm3"])
        self.rate = float(cfgs["fan"]["force"]["update_rate_hz"])
        self.schedule = schedule
        self.levels = cfgs["fan"]["levels_mps"]
        self.field = WindField(FanJet.from_cfg(cfgs["fan"], cfgs["world"], "off"),
                               float(cfgs["fan"]["turbulence"]["intensity"]),
                               float(cfgs["fan"]["turbulence"]["tau_s"]), np.random.default_rng(seed))
        self.use_step_wrench = use_step_wrench
        self.level = "off"
        self.last_wind = np.zeros(3)
        self.last_force = np.zeros(3)
        self._prev: tuple[float, np.ndarray] | None = None

    def _body_state(self) -> tuple[np.ndarray, np.ndarray] | None:
        rec = self.world.pose(self.model)
        if rec is None:
            return None
        p = np.asarray(rec.position)
        v = np.zeros(3)
        if self._prev is not None and rec.t_sim > self._prev[0]:
            v = (p - self._prev[1]) / (rec.t_sim - self._prev[0])
        self._prev = (rec.t_sim, p)
        return p, v

    def run(self, until: float | None = None, stop=lambda: False) -> None:
        from sim_bridge.sim_loop import SimTicker
        t_prev = None
        ticker = SimTicker(self.world, self.rate, until=until, stop=stop)
        for t in ticker:
            dt = 0.0 if t_prev is None else t - t_prev
            t_prev = t
            level = schedule_level(self.schedule, t)
            if level != self.level:
                self.level = level
                self.field.set_level(float(self.levels[level]))
                log.info("t=%.2f fan level -> %s", t, level)
            self.field.step(dt)
            st = self._body_state()
            if st is None:
                continue
            p, v = st
            self.last_wind = self.field.velocity(p)
            self.last_force = drag_force(self.last_wind, v, self.cd_area, self.rho)
            if self.use_step_wrench:
                self.world.apply_step_wrench(self.model, self.link, self.last_force)
            else:
                self.world.set_persistent_wrench(self.model, self.link, self.last_force)
        log.info("fan wind achieved %.1f Hz (target %.1f)", ticker.achieved_rate_hz(), self.rate)


def main() -> None:  # pragma: no cover
    from sim_bridge.gz_io import GzWorld
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--level", default="low")
    ap.add_argument("--model", default=None)
    ap.add_argument("--link", default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--until", type=float, default=None)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO)
    cfgs = cfgmod.load_all()
    world = GzWorld(cfgs["world"]["world_name"])
    world.wait_for_clock()
    model = args.model or cfgs["drone"]["model_name"]
    FanWindBridge(world, cfgs, model=model, link=args.link or cfgs["fan"]["force"]["target_link"],
                  cd_area=cfgs["drone"]["aero"]["body_cd_area_m2"],
                  schedule=[{"t_s": 0.0, "level": args.level}], seed=args.seed).run(until=args.until)


if __name__ == "__main__":  # pragma: no cover
    main()
