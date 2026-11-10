"""Gimbal interface. The ground team's pan-tilt gimbal points the camera and reports its own angles.

``Gimbal`` is what the tracker depends on; wrap the real driver in a subclass (command in,
encoder angles out). ``SimGimbal`` stands in for it during development and tests, with the
error sources the design lists: a control-loop latency (tracking lag), step quantisation
(one step of a 4096-step servo is 1.53 mrad) and a slew limit.

Angles are milliradians: pan positive to the right, tilt positive downwards.
"""
import math
from collections import deque
from typing import Deque, Tuple


class Gimbal:
    def angles(self) -> Tuple[float, float]:
        """Encoder angles (pan, tilt) in mrad. Timestamp and interpolate to the frame time in
        the real driver, or a fast slew shows up as false drone jitter."""
        raise NotImplementedError

    def command(self, pan: float, tilt: float) -> None:
        """Point at absolute angles (mrad)."""
        raise NotImplementedError

    @property
    def commanded(self) -> Tuple[float, float]:
        raise NotImplementedError


class FixedGimbal(Gimbal):
    """No gimbal: the camera is mounted still. Encoder angles are always 0, so the drone's
    direction is simply its pixel offset from the image centre, and commands do nothing."""

    def angles(self) -> Tuple[float, float]:
        return 0.0, 0.0

    def command(self, pan: float, tilt: float) -> None:
        pass

    @property
    def commanded(self) -> Tuple[float, float]:
        return 0.0, 0.0


class SimGimbal(Gimbal):
    def __init__(self, pan: float = 0.0, tilt: float = 0.0, step_mrad: float = 2 * math.pi * 1000 / 4096,
                 latency_s: float = 0.03, gain: float = 25.0, max_rate: float = 2500.0):
        self.step = step_mrad
        self.latency = latency_s
        self.gain = gain              # servo bandwidth, 1/s
        self.max_rate = max_rate      # mrad/s
        self._a = [pan, tilt]         # continuous servo angle
        self._cmd = (pan, tilt)
        self._queue: Deque = deque()  # (apply_time, pan, tilt)
        self._applied = (pan, tilt)
        self._t = 0.0

    def angles(self) -> Tuple[float, float]:
        return tuple(round(a / self.step) * self.step for a in self._a)   # servo stops on steps

    @property
    def commanded(self) -> Tuple[float, float]:
        return self._cmd

    def command(self, pan: float, tilt: float) -> None:
        self._cmd = (pan, tilt)
        self._queue.append((self._t + self.latency, pan, tilt))

    def place(self, pan: float, tilt: float) -> None:
        """Teleport (test/sim set-up only)."""
        self._a = [pan, tilt]
        self._cmd = self._applied = (pan, tilt)
        self._queue.clear()

    def step_time(self, dt: float) -> None:
        """Advance the simulation by ``dt`` seconds."""
        self._t += dt
        while self._queue and self._queue[0][0] <= self._t:
            _, p, t = self._queue.popleft()
            self._applied = (p, t)
        for i in range(2):
            err = self._applied[i] - self._a[i]
            v = max(-self.max_rate, min(self.max_rate, self.gain * err))
            self._a[i] += v * dt
