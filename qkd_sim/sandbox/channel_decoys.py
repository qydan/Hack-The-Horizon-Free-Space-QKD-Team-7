"""[OPEN] Decoy-state channel model: photon statistics (mu), detection efficiency (eta_d), dark counts (p_dark).

Config: ``config/sandbox.yaml``.
"""
from __future__ import annotations

from typing import Any


def detection_probabilities(budget: dict[str, float], cfg: dict[str, Any]) -> dict[str, float]:
    """Signal / decoy / vacuum detection probabilities for one block."""
    raise NotImplementedError


def simulate_detections(n_pulses: int, budget: dict[str, float], cfg: dict[str, Any], rng: Any) -> dict[str, Any]:
    """Draw detection events for one block."""
    raise NotImplementedError
