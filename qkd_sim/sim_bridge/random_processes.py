"""Seeded random processes used by the wind, hover wander and gimbal models (pure numpy)."""
from __future__ import annotations

import math

import numpy as np


class OrnsteinUhlenbeck:
    """Vector Ornstein-Uhlenbeck process with stationary std ``sigma`` and correlation time ``tau``.

    Uses the exact discretisation, so it is stable for any ``dt``:
    x' = x * a + sigma * sqrt(1 - a^2) * n,  a = exp(-dt / tau).
    """

    def __init__(self, sigma, tau_s: float, rng: np.random.Generator, x0=None):
        self.sigma = np.atleast_1d(np.asarray(sigma, dtype=float))
        self.tau = float(tau_s)
        self.rng = rng
        self.x = (np.zeros_like(self.sigma) if x0 is None
                  else np.atleast_1d(np.asarray(x0, dtype=float)).copy())

    def set_sigma(self, sigma) -> None:
        self.sigma = np.atleast_1d(np.asarray(sigma, dtype=float))

    def step(self, dt: float) -> np.ndarray:
        if dt <= 0:
            return self.x
        a = math.exp(-dt / self.tau)
        self.x = self.x * a + self.sigma * math.sqrt(max(0.0, 1.0 - a * a)) * self.rng.standard_normal(self.x.shape)
        return self.x


class RandomWalk:
    """Scalar random walk with std growth ``sigma_per_sqrt_s`` * sqrt(t)."""

    def __init__(self, sigma_per_sqrt_s: float, rng: np.random.Generator):
        self.k = float(sigma_per_sqrt_s)
        self.rng = rng
        self.x = 0.0

    def step(self, dt: float) -> float:
        if dt > 0:
            self.x += self.k * math.sqrt(dt) * float(self.rng.standard_normal())
        return self.x


class BandLimitedNoise:
    """Gaussian noise band-limited to [f_lo, f_hi] with a given RMS, generated sample by sample.

    Implemented as a 2nd-order band-pass (RBJ biquad) on white noise, scaled so the stationary output
    RMS matches ``rms``. The scale is computed numerically from the filter's noise gain.
    """

    def __init__(self, rms: float, f_lo: float, f_hi: float, fs: float, rng: np.random.Generator):
        self.rng = rng
        f0 = math.sqrt(f_lo * f_hi)
        bw_oct = math.log2(f_hi / f_lo)
        w0 = 2 * math.pi * f0 / fs
        alpha = math.sin(w0) * math.sinh(math.log(2) / 2 * bw_oct * w0 / math.sin(w0))
        b0, b1, b2 = alpha, 0.0, -alpha
        a0, a1, a2 = 1 + alpha, -2 * math.cos(w0), 1 - alpha
        self.b = np.array([b0, b1, b2]) / a0
        self.a = np.array([1.0, a1 / a0, a2 / a0])
        self.z = [0.0, 0.0]
        # noise gain = sum of squared impulse response
        h = self._impulse(4096)
        self.scale = rms / math.sqrt(float(np.sum(h * h))) if rms > 0 else 0.0

    def _impulse(self, n: int) -> np.ndarray:
        z1 = z2 = 0.0
        out = np.empty(n)
        for i in range(n):
            x = 1.0 if i == 0 else 0.0
            y = self.b[0] * x + z1
            z1 = self.b[1] * x - self.a[1] * y + z2
            z2 = self.b[2] * x - self.a[2] * y
            out[i] = y
        return out

    def sample(self) -> float:
        x = float(self.rng.standard_normal())
        y = self.b[0] * x + self.z[0]
        self.z[0] = self.b[1] * x - self.a[1] * y + self.z[1]
        self.z[1] = self.b[2] * x - self.a[2] * y
        return y * self.scale
