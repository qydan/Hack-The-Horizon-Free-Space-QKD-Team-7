"""Sim-time paced loops.

Gazebo runs lockstep only with PX4. The Python bridges are asynchronous clients, so they pace themselves
on the sim clock: each loop iteration waits for sim time to reach the next tick and reports the actual
sim time it ran at. If the bridge falls behind (heavy rendering), ticks are skipped rather than queued,
and the achieved rate is reported.
"""
from __future__ import annotations

import time
from typing import Callable, Iterator, Protocol


class Clock(Protocol):
    @property
    def sim_time(self) -> float: ...

    def wait_until(self, t_sim: float, timeout_wall_s: float = 30.0) -> bool: ...


class SimTicker:
    """Iterate at a fixed sim-time rate: ``for t in SimTicker(world, 100.0, until=60.0): ...``."""

    def __init__(self, clock: Clock, rate_hz: float, until: float | None = None,
                 stop: Callable[[], bool] | None = None, start: float | None = None):
        self.clock = clock
        self.period = 1.0 / float(rate_hz)
        self.until = until
        self.stop = stop or (lambda: False)
        self.start = start
        self.ticks = 0
        self.skipped = 0
        self._t0_wall = time.monotonic()
        self._t_first: float | None = None
        self._t_last: float | None = None

    def __iter__(self) -> Iterator[float]:
        t_next = self.clock.sim_time if self.start is None else self.start
        while not self.stop():
            if self.until is not None and t_next > self.until:
                return
            if not self.clock.wait_until(t_next, timeout_wall_s=30.0):
                raise TimeoutError(f"sim clock did not reach t={t_next:.3f} s within 30 s wall time")
            t = self.clock.sim_time
            if self._t_first is None:
                self._t_first = t
            self._t_last = t
            self.ticks += 1
            # skip ticks we are already past (do not queue a burst)
            n_behind = int((t - t_next) / self.period)
            self.skipped += max(0, n_behind)
            t_next += (n_behind + 1) * self.period
            yield t

    def achieved_rate_hz(self) -> float:
        if self._t_first is None or self._t_last is None or self._t_last <= self._t_first:
            return float("nan")
        return (self.ticks - 1) / (self._t_last - self._t_first)
