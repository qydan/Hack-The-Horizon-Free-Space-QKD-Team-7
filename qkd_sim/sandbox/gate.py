"""[OPEN] Security gate: QBER < 11% plus decoy check; publishes aborts / re-acquire requests on port 5557.

Config: ``config/sandbox.yaml``.
"""
from __future__ import annotations

from typing import Any


class Gate:
    """Decides whether a block may produce key."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        """Store configuration (abort_qber) and open the abort publisher."""
        raise NotImplementedError

    def check(self, qber: float, decoy_stats: dict[str, Any]) -> bool:
        """Return True if the block passes; publish an abort otherwise."""
        raise NotImplementedError
