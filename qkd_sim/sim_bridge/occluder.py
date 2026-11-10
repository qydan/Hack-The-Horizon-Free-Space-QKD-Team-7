"""Scripted occlusion events (T3) and other kinematic props (section 13).

The ``occluder`` model is a hand-sized plate. Each event sweeps it across the line of sight between the lens
centre and the hover station, at ``fraction_of_range`` along that line, from one side to the other.

Event spec (scenario ``occluder.events``): {t_s, duration_s, fraction_of_range, sweep_m}
The same mover also drives scripted target poses (e.g. the calibration checkerboard) via ``pose_track``:
list of {t_s, xyz, rpy_deg}; poses are held (step) between entries.
"""
from __future__ import annotations

import argparse
import logging
import math
from typing import Any

import numpy as np

from sim_bridge import config as cfgmod

log = logging.getLogger("occluder")

PARK = (-3.0, 2.5, -10.0)


def occluder_position(t: float, events: list[dict[str, Any]], lens, station) -> tuple[float, float, float] | None:
    """Occluder position at t, or None when parked."""
    lens = np.asarray(lens, float)
    station = np.asarray(station, float)
    for ev in events or []:
        t0, dur = float(ev["t_s"]), float(ev.get("duration_s", 0.6))
        if t0 <= t <= t0 + dur:
            centre = lens + float(ev.get("fraction_of_range", 0.5)) * (station - lens)
            los = station - lens
            side = np.cross(los, [0.0, 0.0, 1.0])
            side /= np.linalg.norm(side)
            sweep = float(ev.get("sweep_m", 0.4))
            s = -sweep / 2 + sweep * (t - t0) / dur
            p = centre + s * side
            return float(p[0]), float(p[1]), float(p[2])
    return None


def rpy_to_quat_wxyz(r: float, p: float, y: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def pose_track_at(t: float, track: list[dict[str, Any]]):
    cur = None
    for item in track or []:
        if t >= float(item["t_s"]):
            cur = item
    return cur


class OccluderController:  # pragma: no cover - needs Gazebo
    def __init__(self, world, events: list[dict[str, Any]], lens_xyz, station_xyz, *, rate_hz: float = 100.0,
                 pose_tracks: dict[str, list[dict[str, Any]]] | None = None):
        self.world = world
        self.events = events
        self.lens = lens_xyz
        self.station = station_xyz
        self.rate = rate_hz
        self.tracks = pose_tracks or {}
        self._parked = False
        self._last_track_item: dict[str, Any] = {}

    def run(self, until: float | None = None, stop=lambda: False) -> None:
        from sim_bridge.sim_loop import SimTicker
        for t in SimTicker(self.world, self.rate, until=until, stop=stop):
            p = occluder_position(t, self.events, self.lens, self.station)
            if p is None:
                if not self._parked:
                    self.world.set_pose("occluder", PARK)
                    self._parked = True
            else:
                self.world.set_pose("occluder", p)
                self._parked = False
            for model, track in self.tracks.items():
                item = pose_track_at(t, track)
                if item is not None and item is not self._last_track_item.get(model):
                    rpy = [math.radians(v) for v in item.get("rpy_deg", [0, 0, 0])]
                    self.world.set_pose(model, item["xyz"], rpy_to_quat_wxyz(*rpy))
                    self._last_track_item[model] = item


def main() -> None:  # pragma: no cover
    from sim_bridge.gz_io import GzWorld
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--at", type=float, nargs="+", default=[30.0, 70.0, 100.0])
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO)
    w = cfgmod.load("world")
    world = GzWorld(w["world_name"])
    world.wait_for_clock()
    lens = [0.0, 0.0, cfgmod.load("rigs")["rig_fixed"]["lens_height_m"]]
    OccluderController(world, [{"t_s": t} for t in args.at], lens, w["hover_station_m"]).run()


if __name__ == "__main__":  # pragma: no cover
    main()
