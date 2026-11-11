"""Tracker state machine: SEARCH -> ACQUIRE -> TRACK <-> COAST (single camera, no handover).

Ported from the state logic of src/tracker.py (SEARCH / TRACK / COAST), with the hit-counting phase made the
explicit ACQUIRE state of telemetry/schema.json:

    SEARCH  --candidate-->            ACQUIRE (hits = 1)
    ACQUIRE --candidate x acquire_frames--> TRACK   (lock: the filter starts)
    ACQUIRE --no candidate-->         SEARCH
    TRACK   --miss-->                 COAST   (follow the prediction)
    COAST   --measurement-->          TRACK
    COAST   --miss for coast_timeout_s--> SEARCH (lock lost)
    any     --abort (gate, port 5557)--> SEARCH

Config: ``tracking.yaml: state_machine.*``. Rig behaviour: same in both rigs.
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
    """Event-driven state machine. Events: candidate, no_candidate, measurement, miss, abort."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        s = cfg.get("state_machine", cfg)
        self.acquire_frames = int(s["acquire_frames"])
        self.coast_timeout = float(s["coast_timeout_s"])
        self.lost_timeout = float(s["lost_timeout_s"])
        self._state = State.SEARCH
        self.hits = 0
        self.last_seen: float | None = None
        self.search_since: float | None = None
        self.locked_now = False      # this event produced a lock
        self.lost_now = False        # this event lost the lock (COAST timeout or abort)

    @property
    def state(self) -> State:
        return self._state

    def _search(self, t: float) -> None:
        self._state, self.hits, self.search_since = State.SEARCH, 0, t

    def on_event(self, t: float, event: str, data: dict[str, Any] | None = None) -> State:
        """Handle an event and return the new state."""
        self.locked_now = self.lost_now = False
        if self.search_since is None:
            self.search_since = t
        s = self._state
        if event == "abort":
            if s in (State.TRACK, State.COAST):
                self.lost_now = True
            self._search(t)
        elif s in (State.SEARCH, State.ACQUIRE):
            if event == "candidate":
                self.hits += 1
                if self.hits >= self.acquire_frames:
                    self._state, self.last_seen, self.locked_now = State.TRACK, t, True
                else:
                    self._state = State.ACQUIRE
            elif event == "no_candidate":
                self._state, self.hits = State.SEARCH, 0
        elif event == "measurement":
            self._state, self.last_seen = State.TRACK, t
        elif event == "miss":
            if t - (self.last_seen if self.last_seen is not None else t) >= self.coast_timeout:
                self.lost_now = True
                self._search(t)
            else:
                self._state = State.COAST
        else:
            raise ValueError(f"unknown event {event!r} in state {s.value}")
        return self._state

    def searching_for(self, t: float) -> float:
        """Seconds spent in SEARCH / ACQUIRE so far (0 while locked)."""
        if self._state in (State.TRACK, State.COAST) or self.search_since is None:
            return 0.0
        return t - self.search_since
