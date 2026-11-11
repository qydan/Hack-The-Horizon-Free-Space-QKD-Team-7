"""Bare red LED driver, locked to Gazebo sim time (section 6.1). Steady by default (constantly lit, as on the
real rig); ``beacon.yaml: red_led.modulation.enabled: true`` switches to the 7 Hz square wave (100% / 40%, never
off). Either way the level is multiplied by the LED's beam pattern as the drone's attitude turns it away from
the camera.

Implementation methods, tried in the order listed in ``beacon.yaml: red_led.methods``:
  1. ``visual_config``: runtime material change through the /world/<w>/visual_config user-command service.
  2. ``plugin``: a C++ gz-sim system that updates the emissive colour every step. NOT shipped in this build
     (see README "Open issues"); selecting it raises with a clear message.
  3. ``shell_toggle``: two co-located spheres (100% and 40%); the inactive one is pulled down into an opaque
     shell by a prismatic joint driven by JointPositionController.
Which one worked is printed at start-up and written into the run directory (``beacon_method.txt``).

Only the level *edges* are sent (14 updates/s at 7 Hz), because the level is constant between edges; a
refresh is re-sent every ``refresh_s`` in case an update was dropped.

Run standalone:  python -m sim_bridge.beacon_modulator --world qkd_lab --model beacon
"""
from __future__ import annotations

import argparse
import logging
import math
from dataclasses import dataclass
from typing import Any

from sim_bridge import config as cfgmod

log = logging.getLogger("beacon_modulator")


@dataclass(frozen=True)
class Modulation:
    frequency_hz: float
    high_level: float
    low_level: float
    duty: float = 0.5
    phase_s: float = 0.0
    enabled: bool = True

    @classmethod
    def from_cfg(cls, m: dict[str, Any]) -> "Modulation":
        return cls(float(m["frequency_hz"]), float(m["high_level"]), float(m["low_level"]),
                   float(m.get("duty", 0.5)), float(m.get("phase_s", 0.0)), bool(m.get("enabled", True)))

    def level(self, t_sim: float) -> float:
        """Intensity level at sim time t (1.0 = full). High during the first ``duty`` of each period."""
        if not self.enabled:
            return self.high_level
        frac = ((t_sim - self.phase_s) * self.frequency_hz) % 1.0
        return self.high_level if frac < self.duty else self.low_level

    def next_edge(self, t_sim: float) -> float:
        """Sim time of the next level change strictly after t."""
        if not self.enabled:
            return math.inf
        period = 1.0 / self.frequency_hz
        k = math.floor((t_sim - self.phase_s) / period)
        start = self.phase_s + k * period
        for edge in (start + self.duty * period, start + period, start + (1 + self.duty) * period):
            if edge > t_sim + 1e-12:
                return edge
        return start + 2 * period


@dataclass(frozen=True)
class BeamPattern:
    """Bare LED radiation pattern I(theta) = cos(theta)^m, I = 0.5 at the half-intensity angle, 0 behind."""
    half_angle_rad: float

    @property
    def exponent(self) -> float:
        return math.log(0.5) / math.log(math.cos(self.half_angle_rad))

    def factor(self, off_axis_rad: float) -> float:
        if off_axis_rad >= math.pi / 2:
            return 0.0
        return max(0.0, math.cos(off_axis_rad)) ** self.exponent


def off_axis_angle(body_pos, body_quat_wxyz, led_offset_body, led_axis_body, camera_pos) -> float:
    """Angle between the LED axis (body frame) and the LED -> camera direction, with the body's current attitude."""
    import numpy as np

    from sim_bridge.truth_publisher import quat_to_rot
    R = quat_to_rot(body_quat_wxyz)
    led = np.asarray(body_pos, float) + R @ np.asarray(led_offset_body, float)
    to_cam = np.asarray(camera_pos, float) - led
    axis = R @ np.asarray(led_axis_body, float)
    c = float(axis @ to_cam / (np.linalg.norm(axis) * np.linalg.norm(to_cam)))
    return math.acos(max(-1.0, min(1.0, c)))


def emissive_for_level(rgb_full, level: float) -> tuple[float, float, float, float]:
    r, g, b = rgb_full
    return (r * level, g * level, b * level, 1.0)


class BeaconModulator:
    """Drives one emissive visual (or the shell-toggle pair) from sim time."""

    SHELL_HIDE_M = -0.045       # slider position that pulls a sphere fully inside the shell

    def __init__(self, world, model_name: str, cfg_beacon: dict[str, Any], *, visual_name: str = "led_visual",
                 target: str = "red_led", refresh_s: float = 0.5, beam_geometry: dict[str, Any] | None = None):
        """``beam_geometry`` (drone only): {"pose_fn", "led_offset_body", "led_axis_body", "camera_pos"}. With it,
        the LED is dimmed by its beam pattern as the drone's attitude turns the LED away from the camera."""
        self.world = world
        self.model = model_name
        self.target = target
        if target == "red_led":
            self.mod = Modulation.from_cfg(cfg_beacon["red_led"]["modulation"])
            self.rgb_full = tuple(cfg_beacon["red_led"]["emissive_rgb_full"])
            self.methods = list(cfg_beacon["red_led"]["methods"])
        else:  # tag lit by the ring light
            m = dict(cfg_beacon["red_led"]["modulation"])
            m["enabled"] = bool(cfg_beacon["tag"]["modulate_with_ring_light"])
            self.mod = Modulation.from_cfg(m)
            w = float(cfg_beacon["tag"]["emissive_white_full"])
            self.rgb_full = (w, w, w)
            self.methods = ["visual_config"]
        self.visual_name = visual_name
        self.refresh_s = refresh_s
        self.geom = beam_geometry if target == "red_led" else None
        led = cfg_beacon["red_led"]
        self.beam = BeamPattern(math.radians(float(led["viewing_half_angle_deg"])))
        self.beam_period = 1.0 / float(led["beam_update_rate_hz"])
        self._last_sent: float | None = None
        self.method: str | None = None
        self._visual_id: int | None = None
        self.edges_sent = 0

    # ----------------------------------------------------------- method setup
    def select_method(self) -> str:
        for method in self.methods:
            if method == "visual_config":
                vid = self.world.find_visual_id(self.model, self.visual_name)
                if vid is not None and self.world.set_visual_emissive(vid, emissive_for_level(self.rgb_full, 1.0)):
                    self._visual_id = vid
                    self.method = method
                    break
                log.warning("visual_config unavailable for %s/%s (visual id %s)", self.model, self.visual_name, vid)
            elif method == "plugin":
                log.warning("C++ emissive plugin is not part of this build; skipping")
            elif method == "shell_toggle":
                self.method = method
                break
        if self.method is None:
            raise RuntimeError(f"No beacon modulation method worked (tried {self.methods})")
        log.info("beacon modulation method: %s", self.method)
        return self.method

    def apply(self, level: float) -> None:
        if self.method == "visual_config":
            self.world.set_visual_emissive(self._visual_id, emissive_for_level(self.rgb_full, level))
        elif self.method == "shell_toggle":
            high = level >= 0.5 * (self.mod.high_level + self.mod.low_level)
            self.world.publish_double(f"/model/{self.model}/slider_hi/cmd_pos", 0.0 if high else self.SHELL_HIDE_M)
            self.world.publish_double(f"/model/{self.model}/slider_lo/cmd_pos", self.SHELL_HIDE_M if high else 0.0)
        self.edges_sent += 1

    # -------------------------------------------------------------------- run
    def beam_factor(self) -> float:
        if self.geom is None:
            return 1.0
        rec = self.geom["pose_fn"]()
        if rec is None:
            return 1.0
        return self.beam.factor(off_axis_angle(rec.position, rec.orientation_wxyz, self.geom["led_offset_body"],
                                               self.geom["led_axis_body"], self.geom["camera_pos"]))

    def _send(self, t: float, force: bool) -> None:
        level = self.mod.level(t) * self.beam_factor()
        if force or self._last_sent is None or abs(level - self._last_sent) > 0.01 * max(self._last_sent, 1e-3):
            self.apply(level)
            self._last_sent = level

    def run(self, until: float | None = None, stop=lambda: False) -> None:
        if self.method is None:
            self.select_method()
        t = self.world.sim_time
        self._send(t, force=True)
        last_refresh = t
        while not stop():
            t_edge = self.mod.next_edge(t)
            t_next = min(t_edge, last_refresh + self.refresh_s)
            if self.geom is not None:
                t_next = min(t_next, t + self.beam_period)
            if until is not None and t_next > until:
                return
            self.world.wait_until(t_next)
            t = self.world.sim_time
            refresh = t >= last_refresh + self.refresh_s
            self._send(t, force=refresh or t >= t_edge)
            if refresh:
                last_refresh = t


def main() -> None:  # pragma: no cover - needs Gazebo
    from sim_bridge.gz_io import GzWorld

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default=cfgmod.load("world")["world_name"])
    ap.add_argument("--model", default="beacon")
    ap.add_argument("--visual", default="led_visual")
    ap.add_argument("--target", default="red_led", choices=["red_led", "tag"])
    ap.add_argument("--until", type=float, default=None)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO)
    world = GzWorld(args.world)
    world.wait_for_clock()
    mod = BeaconModulator(world, args.model, cfgmod.load("beacon"), visual_name=args.visual, target=args.target)
    print(f"beacon modulation method: {mod.select_method()}")
    mod.run(until=args.until)


if __name__ == "__main__":  # pragma: no cover
    main()
