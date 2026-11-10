"""Station keeping: stands in for "pilot parks it, DJI vision positioning holds it" (section 5.5).

MAVSDK offboard position setpoints: arm, take off to the station altitude, fly to the station, hold.
On top of the station the setpoint follows an optional Ornstein-Uhlenbeck *hover wander* (sigma 0.10 m
horizontal / 0.03 m vertical, tau 5 s, so 3 sigma matches DJI's +-0.3 m / +-0.1 m) and an optional *slow
push* (T6: 0.5 m sideways over 60 s). Everything is paced on Gazebo sim time.

Frames: the world is ENU (x toward the drone, z up). PX4 local NED is centred on the spawn point with
north = world +y, east = world +x, down = -z. Yaw: yaw_ned = 90 deg - yaw_enu.

Run standalone:  python -m sim_bridge.hover_station --duration 120 [--no-wander] [--slow-push-at 30]
"""
from __future__ import annotations

import argparse
import asyncio
import logging
from dataclasses import dataclass
from typing import Any

import numpy as np

from sim_bridge import config as cfgmod
from sim_bridge.random_processes import OrnsteinUhlenbeck

log = logging.getLogger("hover_station")


def enu_to_ned(p_enu, origin_enu) -> tuple[float, float, float]:
    d = np.asarray(p_enu, float) - np.asarray(origin_enu, float)
    return float(d[1]), float(d[0]), float(-d[2])


def yaw_enu_to_ned_deg(yaw_enu_deg: float) -> float:
    y = 90.0 - yaw_enu_deg
    return (y + 180.0) % 360.0 - 180.0


@dataclass
class SetpointGenerator:
    """Station + wander + slow push, as a pure function of sim time (stepped)."""
    station: np.ndarray
    wander_on: bool
    sigma_h: float
    sigma_v: float
    tau_s: float
    rng: np.random.Generator
    push_start_s: float | None = None
    push_distance_m: float = 0.0
    push_duration_s: float = 1.0
    push_axis: np.ndarray | None = None

    def __post_init__(self) -> None:
        self.ou = OrnsteinUhlenbeck([self.sigma_h, self.sigma_h, self.sigma_v], self.tau_s, self.rng)
        self._t = None

    @classmethod
    def from_cfg(cls, drone: dict[str, Any], station, rng, *, wander: bool | None = None,
                 push_start_s: float | None = None, gnss: bool = False) -> "SetpointGenerator":
        sk = drone["station_keeping"]
        w = sk["gnss_wander"] if gnss else sk["wander"]
        sp = sk["slow_push"]
        return cls(np.asarray(station, float), sk["wander"]["enabled"] if wander is None else wander,
                   float(w["sigma_horizontal_m"]), float(w["sigma_vertical_m"]), float(w["tau_s"]), rng,
                   push_start_s, float(sp["distance_m"]), float(sp["duration_s"]),
                   np.asarray(sp["axis"], float))

    def push_offset(self, t: float) -> np.ndarray:
        if self.push_start_s is None or t <= self.push_start_s:
            return np.zeros(3)
        frac = min(1.0, (t - self.push_start_s) / self.push_duration_s)
        return frac * self.push_distance_m * self.push_axis

    def at(self, t: float) -> np.ndarray:
        if self.wander_on:
            if self._t is not None:
                self.ou.step(t - self._t)
            self._t = t
            wander = self.ou.x
        else:
            wander = np.zeros(3)
        return self.station + wander + self.push_offset(t)


async def run_station(drone_cfg: dict[str, Any], world_cfg: dict[str, Any], *, station=None, duration_s=None,
                      wander: bool | None = None, push_start_s: float | None = None, seed: int = 0,
                      gnss: bool = False, world=None, spawn=None,
                      station_schedule=None, stop=lambda: False) -> None:  # pragma: no cover - needs PX4 + Gazebo
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw

    from sim_bridge.gz_io import GzWorld

    world = world or GzWorld(world_cfg["world_name"])
    world.wait_for_clock()
    station = np.asarray(station if station is not None else world_cfg["hover_station_m"], float)
    sk = drone_cfg["station_keeping"]
    if spawn is None:   # must match where tools/world_builder.py placed the model
        spawn = np.asarray(world_cfg["hover_station_m"], float) * np.array([1, 1, 0]) + np.asarray(sk["spawn_offset_m"])
    spawn = np.asarray(spawn, float)
    yaw_ned = yaw_enu_to_ned_deg(float(sk["start_yaw_deg"]))
    gen = SetpointGenerator.from_cfg(drone_cfg, station, np.random.default_rng(seed), wander=wander,
                                     push_start_s=push_start_s, gnss=gnss)
    period = 1.0 / float(sk["setpoint_rate_hz"])

    drone = System()
    await drone.connect(system_address=drone_cfg["mavsdk"]["system_address"])
    async for st in drone.core.connection_state():
        if st.is_connected:
            break
    log.info("connected; waiting for position estimate")
    async for h in drone.telemetry.health():
        if h.is_local_position_ok and h.is_home_position_ok and getattr(h, "is_armable", True):
            break
    log.info("PX4 reports armable")

    # start offboard on the ground at the spawn point, then climb. PX4 can still refuse the first arm right
    # after boot (preflight checks settling), so retry for a while before giving up.
    from mavsdk.action import ActionError
    ground = enu_to_ned(spawn, spawn)
    await drone.offboard.set_position_ned(PositionNedYaw(*ground, yaw_ned))
    for attempt in range(1, 31):
        try:
            await drone.action.arm()
            break
        except ActionError as err:
            if attempt == 30:
                raise
            log.warning("arm attempt %d refused (%s); retrying", attempt, err._result.result)
            await asyncio.sleep(1.0)
    log.info("armed")
    try:
        await drone.offboard.start()
    except OffboardError as err:
        await drone.action.disarm()
        raise RuntimeError(f"offboard start failed: {err._result.result}") from err

    t0 = world.sim_time
    climb_s = 4.0
    takeoff_target = np.array([spawn[0], spawn[1], float(sk["takeoff_alt_m"])])
    end = None if duration_s is None else t0 + duration_s
    t_next = t0
    station_schedule = sorted(station_schedule or [], key=lambda it: it[0])   # [(t_sim, xyz)], T2 trials
    while (end is None or world.sim_time < end) and not stop():
        world.wait_until(t_next)
        t = world.sim_time
        for t_s, xyz in station_schedule:
            if t >= t_s:
                gen.station = np.asarray(xyz, float)
        el = t - t0
        if el < climb_s:                        # vertical take-off
            p = spawn + (takeoff_target - spawn) * (el / climb_s)
        elif el < 2 * climb_s:                  # transit to the station (no-op when spawn is under it)
            f = (el - climb_s) / climb_s
            p = takeoff_target + (station - takeoff_target) * f
        else:
            p = gen.at(t)
        await drone.offboard.set_position_ned(PositionNedYaw(*enu_to_ned(p, spawn), yaw_ned))
        t_next = t + period
    log.info("station keeping finished at t=%.1f", world.sim_time)


def main() -> None:  # pragma: no cover
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--no-wander", action="store_true")
    ap.add_argument("--slow-push-at", type=float, default=None)
    ap.add_argument("--station", type=float, nargs=3, default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--gnss", action="store_true", help="GNSS hover wander (mission scenario)")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO)
    asyncio.run(run_station(cfgmod.load("drone"), cfgmod.load("world"), station=args.station,
                            duration_s=args.duration, wander=False if args.no_wander else None,
                            push_start_s=args.slow_push_at, seed=args.seed, gnss=args.gnss))


if __name__ == "__main__":  # pragma: no cover
    main()
