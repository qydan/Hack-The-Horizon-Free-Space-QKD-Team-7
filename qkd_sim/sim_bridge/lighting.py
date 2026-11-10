"""Lighting scenarios and mains flicker (section 9), applied at sim time through /world/<w>/light_config.

The world generator (tools/world_builder.py) creates every light and prop any scenario can need, initially
off or parked off-scene. This module switches between scenarios at run time:

  * ``scene ambient`` is fixed at the ``dark`` level (0.02) in the world SDF, because Gazebo Harmonic has no
    runtime service for it. Higher ambient levels are emulated by two shadowless directional *fill* lights
    (``ambient_fill_down``, ``ambient_fill_front``) whose diffuse level equals ``ambient - 0.02``.
  * Ceiling panels are point lights ``panel_0..3``; flicker modulates their intensity every physics step:
        I(t) = I0 * (1 - depth * (0.5 - 0.5 cos(2 pi f t)))      (depth = peak-to-peak fraction)
  * ``lamp_spot``, ``sun``, the window plane and the distractor props are switched per scenario.
  * The blinking red indicator is driven through visual_config at ``blink_hz``.

If light_config cannot keep up with one update per physics step, the achieved rate is logged and written to
the run directory (``lighting_rate.txt``) as the spec requires.

Run standalone:  python -m sim_bridge.lighting --scenario office_50hz --until 60
"""
from __future__ import annotations

import argparse
import logging
import math
from typing import Any

from sim_bridge import config as cfgmod

log = logging.getLogger("lighting")

SCENE_AMBIENT = 0.02          # matches scenarios.dark.ambient; baked into the world SDF
PARK_Z = -20.0                # props are parked below the floor when not in use


def flicker_factor(t: float, flicker_hz: float | None, depth: float) -> float:
    if not flicker_hz:
        return 1.0
    return 1.0 - depth * (0.5 - 0.5 * math.cos(2.0 * math.pi * flicker_hz * t))


def schedule_scenario(schedule: list[dict[str, Any]] | None, t: float, default: str) -> str:
    name = default
    for item in schedule or []:
        if t >= float(item["t_s"]):
            name = str(item["scenario"])
    return name


def blink_on(t: float, hz: float) -> bool:
    return ((t * hz) % 1.0) < 0.5


def light_specs(cfg_light: dict[str, Any], scenario_name: str, *, indoor: bool = True, shadows: bool = False,
                flicker: float = 1.0) -> list[dict[str, Any]]:
    """Complete definition of every light for one lighting scenario.

    Used both to write the world SDF (tools/world_builder.py) and for run-time updates: a light_config request
    replaces the whole light, so every update must carry every field (a partial message zeroes the range and
    attenuation and the light goes dark). ``flicker`` scales the ceiling-panel intensity (mains flicker).
    """
    import numpy as np
    s = cfg_light["scenarios"][scenario_name]
    fill = max(0.0, float(s["ambient"]) - SCENE_AMBIENT)
    out: list[dict[str, Any]] = []
    for name, d in zip(("ambient_fill_down", "ambient_fill_front"), cfg_light["ambient_fill"]["directions"]):
        out.append({"name": name, "type": "directional", "direction": list(d), "diffuse": [fill] * 3,
                    "specular": [0.0, 0.0, 0.0], "intensity": 1.0, "cast_shadows": False})
    p = cfg_light["ceiling_panels"]
    on = s.get("panels") == "on" and indoor
    for k, xyz in enumerate(p["positions_m"]):
        out.append({"name": f"panel_{k}", "type": "point", "position": list(xyz),
                    "diffuse": list(p["diffuse"]) if on else [0.0] * 3, "specular": list(p["specular"]),
                    "intensity": float(p["intensity"]) * flicker if on else 0.0,
                    "range": float(p["attenuation_range_m"]), "attenuation": dict(p["attenuation"]),
                    "cast_shadows": False})
    sd = cfg_light["spot_defaults"]
    spot = cfg_light["scenarios"]["bright_lamp"]["spot"]
    d = np.asarray(spot["aim_at_m"], float) - np.asarray(spot["position_m"], float)
    d = d / np.linalg.norm(d)
    son = scenario_name == "bright_lamp"
    out.append({"name": "lamp_spot", "type": "spot", "position": list(spot["position_m"]), "direction": d.tolist(),
                "diffuse": list(spot["diffuse"]) if son else [0.0] * 3, "specular": list(sd["specular"]),
                "intensity": float(spot["intensity"]) if son else 0.0, "range": float(sd["range_m"]),
                "attenuation": dict(sd["attenuation"]), "cast_shadows": shadows,
                "spot": {"inner": math.radians(spot["inner_angle_deg"]), "outer": math.radians(spot["outer_angle_deg"]),
                         "falloff": float(sd["falloff"])}})
    sun = cfg_light["scenarios"]["window"]["sun"]
    sun_on = scenario_name == "window" or not indoor
    out.append({"name": "sun", "type": "directional", "direction": list(sun["direction"]),
                "diffuse": list(sun["diffuse"]) if sun_on else [0.0] * 3, "specular": [0.3, 0.3, 0.3],
                "intensity": float(sun["intensity"]) if sun_on else 0.0, "cast_shadows": shadows})
    return out


class LightingController:  # pragma: no cover - needs Gazebo
    def __init__(self, world, cfg_light: dict[str, Any], *, schedule: list[dict[str, Any]] | None = None,
                 default: str | None = None, shadows: bool = False):
        self.world = world
        self.cfg = cfg_light
        self.schedule = schedule
        self.default = default or cfg_light["default"]
        self.current: str | None = None
        self.depth = float(cfg_light["mains"]["depth"])
        self.rate = float(cfg_light["mains"]["update_rate_hz"])
        self.shadows = shadows
        self._blink_vid: int | None = None
        self._blink_state: bool | None = None
        self.achieved_rate_hz = float("nan")

    # --------------------------------------------------------------------------------------------
    def _scn(self, name: str) -> dict[str, Any]:
        return self.cfg["scenarios"][name]

    def _send(self, name: str, flicker: float = 1.0, only_panels: bool = False) -> None:
        for spec in light_specs(self.cfg, name, shadows=self.shadows, flicker=flicker):
            if not only_panels or spec["name"].startswith("panel_"):
                self.world.set_light_spec(spec)

    def _apply_static(self, name: str) -> None:
        s = self._scn(name)
        self._send(name)
        sun = s.get("sun")
        wp = (s.get("window_plane") or self.cfg["scenarios"]["window"]["window_plane"])
        c = wp["centre_m"]
        self.world.set_pose("window_plane", c if sun else (c[0], c[1], PARK_Z))
        self._place_distractors(bool(s.get("distractor_set")))

    def _place_distractors(self, show: bool) -> None:
        d = self.cfg["distractors"]
        props = [("steady_red_lamp", d["steady_red_lamp"]["position_m"]),
                 ("blinking_red_indicator", d["blinking_red_indicator"]["position_m"])]
        props += [(f"glint_{i}", g["position_m"]) for i, g in enumerate(d["glints"])]
        for name, pos in props:
            self.world.set_pose(name, pos if show else (pos[0], pos[1], PARK_Z))

    def _blink(self, t: float) -> None:
        d = self.cfg["distractors"]["blinking_red_indicator"]
        on = blink_on(t, float(d["blink_hz"]))
        if on == self._blink_state:
            return
        if self._blink_vid is None:
            self._blink_vid = self.world.find_visual_id("blinking_red_indicator", "visual")
            if self._blink_vid is None:
                return
        rgb = d["emissive_rgb"] if on else (0.0, 0.0, 0.0)
        self.world.set_visual_emissive(self._blink_vid, (*rgb, 1.0))
        self._blink_state = on

    # --------------------------------------------------------------------------------------------
    def run(self, until: float | None = None, stop=lambda: False) -> None:
        from sim_bridge.sim_loop import SimTicker
        ticker = SimTicker(self.world, self.rate, until=until, stop=stop)
        for t in ticker:
            name = schedule_scenario(self.schedule, t, self.default)
            if name != self.current:
                log.info("t=%.2f lighting -> %s", t, name)
                self._apply_static(name)
                self.current = name
            s = self._scn(name)
            if s.get("flicker_hz") and s.get("panels") == "on":
                self._send(name, flicker_factor(t, float(s["flicker_hz"]), self.depth), only_panels=True)
            if s.get("distractor_set"):
                self._blink(t)
        self.achieved_rate_hz = ticker.achieved_rate_hz()
        log.info("lighting update rate achieved %.1f Hz (target %.1f Hz)", self.achieved_rate_hz, self.rate)


def main() -> None:  # pragma: no cover
    from sim_bridge.gz_io import GzWorld
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default=None)
    ap.add_argument("--until", type=float, default=None)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO)
    cfg = cfgmod.load("lighting")
    world = GzWorld(cfgmod.load("world")["world_name"])
    world.wait_for_clock()
    LightingController(world, cfg, default=args.scenario).run(until=args.until)


if __name__ == "__main__":  # pragma: no cover
    main()
