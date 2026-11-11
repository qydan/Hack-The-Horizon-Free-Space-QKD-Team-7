"""Configuration for tracking: config/tracking.yaml, the calibration outputs (config/calibration.yaml) and the
camera's datasheet values (config/camera_action5.yaml ``stream`` and ``lens``: what a team knows before
calibrating). tracking/ may not import sim_bridge.config, so it reads the YAML files itself.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parent.parent


def load_yaml(name: str, config_dir: Path | None = None) -> dict[str, Any]:
    with open((config_dir or ROOT / "config") / f"{name}.yaml", "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def load_tracking_cfg(config_dir: Path | None = None) -> dict[str, Any]:
    """tracking.yaml plus ``calibration`` (calibration.yaml) and ``camera_nominal`` (datasheet values)."""
    cfg = load_yaml("tracking", config_dir)
    cfg["calibration"] = load_yaml("calibration", config_dir)
    cam = load_yaml("camera_action5", config_dir)
    cfg["camera_nominal"] = {
        "width_px": int(cam["stream"]["width_px"]), "height_px": int(cam["stream"]["height_px"]),
        "fps": float(cam["stream"]["fps"]), "projection": cam["lens"]["projection"],
        "hfov_deg": float(cam["lens"]["hfov_deg"])}
    if str(cfg["acquire"].get("beacon", "auto")) == "auto":       # the team's own beacon: blinking or steady
        mod = (load_yaml("beacon", config_dir).get("red_led") or {}).get("modulation") or {}
        cfg["acquire"]["beacon"] = "blink" if mod.get("enabled", True) else "steady"
        if mod.get("frequency_hz"):
            cfg["acquire"]["lockin_frequency_hz"] = float(mod["frequency_hz"])
    return cfg


def calib(cfg: dict[str, Any], section: str, key: str, default=None):
    """A calibration.yaml value, or ``default`` while it is still null."""
    v = ((cfg.get("calibration") or {}).get(section) or {}).get(key)
    return default if v is None else v
