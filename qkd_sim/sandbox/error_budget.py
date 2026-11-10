"""[OPEN] Turns I2 telemetry (residual, jitter, intensity, range, lean) into a per-block optical error budget.

Config: ``config/sandbox.yaml``.
"""
from __future__ import annotations

from typing import Any


def geometric_efficiency(record: dict[str, Any], cfg: dict[str, Any]) -> float:
    """Collection efficiency for one telemetry record (eta_geo_mode, theta_fov_virtual_mrad)."""
    raise NotImplementedError


def block_budget(records: list[dict[str, Any]], cfg: dict[str, Any]) -> dict[str, float]:
    """Aggregate one block (block_s seconds) of records into channel parameters."""
    raise NotImplementedError
