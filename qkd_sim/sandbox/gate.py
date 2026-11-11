"""Security gate: pooled block QBER < abort_qber (11%) plus the decoy-state check; publishes aborts /
re-acquire requests on port 5557 (I8).

Ported from the gate in src/sandbox.py. The gate decides on the *pooled* QBER of a whole key block, because
single frames often spike above 11% even when the block is fine. Outcomes:

    abort         pooled QBER >= abort_qber (intercept-resend, gust, bad pointing)
    decoy_abort   per-class gains inconsistent with the Poisson model (photon-number splitting)
    no_key        decoy bounds certify no single-photon yield
    pass          the block may be distilled into key

Abort message (topic "abort"): ``{"t", "cmd": "reacquire", "reason", "qber"}``; the tracker drops its lock and
re-acquires. Config: ``config/sandbox.yaml: abort_qber``.
"""
from __future__ import annotations

from typing import Any


class Gate:
    """Decides whether a block may produce key."""

    def __init__(self, cfg: dict[str, Any], publisher: Any = None) -> None:
        """``publisher``: a ``telemetry.bus.Publisher("abort")`` (or anything with ``send(dict)``); None
        = decide only (replay and tests)."""
        self.abort_qber = float(cfg["abort_qber"])
        self.publisher = publisher
        self.aborts = 0

    def decide(self, qber: float, decoy_stats: dict[str, Any] | None) -> tuple[str, str]:
        """Return (outcome, note): outcome is pass | abort | decoy_abort | no_key."""
        if qber >= self.abort_qber:
            return "abort", f"QBER {qber * 100:.1f}% >= {self.abort_qber * 100:.0f}%"
        if decoy_stats is not None:
            if not decoy_stats.get("consistent", True):
                return "decoy_abort", decoy_stats.get("note") or "decoy statistics inconsistent"
            if not (decoy_stats.get("single_photon_fraction", 1.0) > 0 and decoy_stats.get("y1_lower", 1.0) > 0):
                return "no_key", "no certified single-photon yield"
        return "pass", ""

    def abort(self, t: float, reason: str, qber: float) -> None:
        """Tell the tracker to drop its lock and re-acquire."""
        self.aborts += 1
        if self.publisher is not None:
            self.publisher.send({"t": float(t), "cmd": "reacquire", "reason": reason, "qber": float(qber)})

    def check(self, qber: float, decoy_stats: dict[str, Any], t: float = 0.0) -> bool:
        """Return True if the block passes; publish an abort if it fails the QBER or decoy check."""
        outcome, note = self.decide(qber, decoy_stats)
        if outcome in ("abort", "decoy_abort"):
            self.abort(t, f"{outcome}: {note}", qber)
        return outcome == "pass"
