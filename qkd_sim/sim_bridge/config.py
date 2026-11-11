"""YAML config loading. Every number lives in config/*.yaml; code only reads it."""
from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = REPO_ROOT / "config"


def load(name: str, config_dir: Path | None = None) -> dict[str, Any]:
    """Load ``config/<name>.yaml`` (name without extension)."""
    path = (config_dir or CONFIG_DIR) / f"{name}.yaml"
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    return data or {}


def deep_merge(base: dict[str, Any], override: dict[str, Any] | None) -> dict[str, Any]:
    """Return a copy of ``base`` with ``override`` merged in recursively (override wins)."""
    out = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def load_all(overrides: dict[str, dict[str, Any]] | None = None) -> dict[str, dict[str, Any]]:
    """Load every simulator config file, applying per-file overrides (as a scenario does)."""
    names = ["world", "drone", "beacon", "fan", "camera_action5", "rigs", "gimbal_storm32",
             "lighting", "bus"]
    overrides = overrides or {}
    return {n: deep_merge(load(n), overrides.get(n)) for n in names}
