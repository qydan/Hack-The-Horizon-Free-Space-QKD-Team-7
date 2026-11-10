"""Scenario loading and validation (section 13).

A scenario YAML sets: world, rigs, payload, target, fan schedule, lighting, hover wander, fidelity, seed,
duration, recorders, plus optional props, occluder events, pose tracks, gimbal schedules, beacon events,
trials and config overrides. ``load_scenario`` fills defaults and validates; invalid scenarios raise
:class:`ScenarioError` with a clear message (e.g. test A10: an 850 nm beacon with the Action 5 Pro).
"""
from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parent.parent
SCENARIO_DIR = ROOT / "scenarios"

DEFAULTS: dict[str, Any] = {
    "description": "",
    "test": None,
    "world": "qkd_lab",
    "rigs": ["rig_fixed", "rig_gimbal"],
    "payload": "red_led_beacon",
    "target": "red_led",
    "beacon_wavelength_nm": None,          # defaults to beacon.yaml red_led.wavelength_nm
    "fidelity": None,                      # defaults to camera_action5.yaml fidelity.default
    "seed": 0,
    "duration_s": 60.0,
    "drone": {"enabled": True, "wander": True, "slow_push_at_s": None, "station": None,
              "randomise_station_m": None, "gnss_wander": False},
    "trials": {"count": 1, "duration_s": None},
    "fan": {"schedule": [{"t_s": 0.0, "level": "off"}]},
    "lighting": {"initial": "office_50hz", "schedule": []},
    "occluder": {"events": []},
    "props": {"checkerboard": False, "static_beacons": False, "spinning_arm": False, "slider_beacon": False},
    "pose_tracks": {},
    "gimbal": {"command_schedule": [], "active_schedule": []},
    "beacon_events": [],
    "photodiode": {"enabled": False},
    "recorders": {"telemetry": True, "truth": True, "frames_meta": True, "gz_log": False},
    "overrides": {},
}

VALID_RIGS = {"rig_fixed", "rig_gimbal"}
VALID_WORLDS = {"qkd_lab", "qkd_pendulum", "qkd_field"}
VALID_PAYLOADS = {"red_led_beacon", "tag", "none"}


class ScenarioError(ValueError):
    pass


def _merge(base: dict[str, Any], over: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


RIG_ALIASES = {
    "rig_fixed": "rig_fixed", "fixed": "rig_fixed", "a": "rig_fixed", "tripod": "rig_fixed",
    "rig_gimbal": "rig_gimbal", "gimbal": "rig_gimbal", "b": "rig_gimbal", "storm32": "rig_gimbal",
}


def normalize_rig(name: str) -> str:
    """'fixed' / 'A' / 'rig_fixed' -> 'rig_fixed'; 'gimbal' / 'B' / 'rig_gimbal' -> 'rig_gimbal'."""
    try:
        return RIG_ALIASES[name.strip().lower()]
    except KeyError:
        raise ScenarioError(f"unknown rig {name!r}; use 'fixed' (config A) or 'gimbal' (config B)") from None


def resolve_path(name_or_path: str) -> Path:
    """A path, an exact scenario name, or a unique prefix ('t3' -> t3_turbulence, 'calib_gim' -> calib_gimbal).
    For a prefix that matches several scenarios, an exact '<prefix>_...' stem match is not guessed: the error
    lists the candidates."""
    p = Path(name_or_path)
    if p.exists():
        return p
    for cand in (SCENARIO_DIR / name_or_path, SCENARIO_DIR / f"{name_or_path}.yaml"):
        if cand.exists():
            return cand
    stems = [q.stem for q in list_scenarios()]
    key = name_or_path.strip().lower()
    hits = [s for s in stems if s.startswith(key + "_")] or [s for s in stems if s.startswith(key)]
    if len(hits) > 1:                                    # 't3' -> prefer the plain one over variants
        shortest = min(hits, key=len)
        if all(h.startswith(shortest) for h in hits):
            hits = [shortest]
    if len(hits) == 1:
        return SCENARIO_DIR / f"{hits[0]}.yaml"
    if hits:
        raise ScenarioError(f"{name_or_path!r} is ambiguous: {', '.join(hits)}")
    raise ScenarioError(f"scenario not found: {name_or_path!r}. Available: {', '.join(stems)}")


def validate(scn: dict[str, Any], cfgs: dict[str, dict[str, Any]]) -> None:
    if scn["world"] not in VALID_WORLDS:
        raise ScenarioError(f"unknown world {scn['world']!r}; expected one of {sorted(VALID_WORLDS)}")
    bad = set(scn["rigs"]) - VALID_RIGS
    if bad or not scn["rigs"]:
        raise ScenarioError(f"invalid rigs {scn['rigs']!r}; expected a non-empty subset of {sorted(VALID_RIGS)}")
    if scn["payload"] not in VALID_PAYLOADS:
        raise ScenarioError(f"unknown payload {scn['payload']!r}")
    if scn["target"] not in ("red_led", "tag"):
        raise ScenarioError(f"unknown target {scn['target']!r}")
    if scn["target"] == "tag" and scn["payload"] not in ("tag",) and scn["drone"]["enabled"]:
        raise ScenarioError("target 'tag' needs payload 'tag'")
    # IR-cut validation (A10): the Action 5 Pro cannot see near-IR targets.
    wl = scn["beacon_wavelength_nm"] or cfgs["beacon"]["red_led"]["wavelength_nm"]
    lo, hi = cfgs["camera_action5"]["sensor_spec"]["passband_nm"]
    if cfgs["camera_action5"]["sensor_spec"].get("ir_cut_filter") and not (lo <= float(wl) <= hi):
        raise ScenarioError(
            f"beacon wavelength {wl} nm is outside the Osmo Action 5 Pro passband {lo}-{hi} nm: the camera has a "
            f"built-in IR-cut filter, so an {wl} nm beacon is invisible to it. Use the ~630 nm red LED beacon or the "
            f"ArUco tag with the ring light.")
    levels = cfgs["fan"]["levels_mps"]
    for item in scn["fan"]["schedule"]:
        if item["level"] not in levels:
            raise ScenarioError(f"unknown fan level {item['level']!r}; expected one of {list(levels)}")
    lights = cfgs["lighting"]["scenarios"]
    for name in [scn["lighting"]["initial"]] + [i["scenario"] for i in scn["lighting"].get("schedule", [])]:
        if name not in lights:
            raise ScenarioError(f"unknown lighting scenario {name!r}; expected one of {list(lights)}")
    fid = scn["fidelity"]
    if fid not in (None, "full", "fast"):
        raise ScenarioError(f"fidelity must be 'full' or 'fast', got {fid!r}")
    if scn["gimbal"]["command_schedule"] and scn["rigs"] != ["rig_gimbal"]:
        raise ScenarioError("a gimbal command_schedule needs rigs: [rig_gimbal]")
    if scn["world"] == "qkd_pendulum" and scn["drone"]["enabled"]:
        raise ScenarioError("the pendulum world has no drone: set drone.enabled: false")
    if float(scn["duration_s"]) <= 0:
        raise ScenarioError("duration_s must be positive")


def load_scenario(name_or_path: str, cfgs: dict[str, dict[str, Any]] | None = None) -> dict[str, Any]:
    from sim_bridge import config as cfgmod
    path = resolve_path(name_or_path)
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    scn = _merge(DEFAULTS, raw)
    scn["name"] = raw.get("name", path.stem)
    scn["path"] = str(path)
    cfgs = cfgs or cfgmod.load_all(scn.get("overrides"))
    validate(scn, cfgs)
    return scn


def list_scenarios() -> list[Path]:
    return sorted(SCENARIO_DIR.glob("*.yaml"))
