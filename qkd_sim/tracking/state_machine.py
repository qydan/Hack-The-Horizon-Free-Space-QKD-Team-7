"""[OPEN] Tracker state machine: SEARCH -> ACQUIRE -> TRACK -> COAST (single camera, no handover).

Inputs:  events (candidate found, measurement accepted/rejected, timeout, abort from the sandbox on 5557).
Outputs: current :class:`State` (published in telemetry ``state``).
Config:  ``tracking.yaml: state_machine.*`` (acquire_frames, coast_timeout_s, lost_timeout_s).
Rig behaviour: same in both rigs.
"""
from __future__ import annotations

import enum
from typing import Any


class State(str, enum.Enum):
    """Tracker states (must match telemetry/schema.json)."""
    SEARCH = "SEARCH"
    ACQUIRE = "ACQUIRE"
    TRACK = "TRACK"
    COAST = "COAST"


class StateMachine:
    """Event-driven state machine."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        """Store configuration; start in SEARCH."""
        raise NotImplementedError

    def on_event(self, t: float, event: str, data: dict[str, Any] | None = None) -> State:
        """Handle an event and return the new state."""
        raise NotImplementedError

    @property
    def state(self) -> State:
        """Current state."""
        raise NotImplementedError
