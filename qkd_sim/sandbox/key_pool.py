"""Optional pooling of accepted blocks before distilling (ported from src/key_pool.py).

Finite-size effects make a single 1 s block yield no key well below the 11% gate (about 5% QBER). Pooling several
accepted windows shrinks the statistical margin and amortises the fixed leakage, so moderately noisy conditions
still produce keys, just less often. Enable with ``config/sandbox.yaml: pool.enabled`` or ``--pool``.
"""
from __future__ import annotations

import numpy as np

from sandbox.bb84 import QuantumCryptoEngine, binary_entropy


class KeyPool:
    def __init__(self, engine: QuantumCryptoEngine, max_bits: int = 400_000):
        self.engine = engine
        self.max_bits = max_bits
        self._windows: list[tuple[np.ndarray, np.ndarray, int, int]] = []   # (alice, bob, sample errors, sample size)

    @property
    def size(self) -> int:
        return int(sum(len(w[0]) for w in self._windows))

    @property
    def qber(self) -> float:
        samples = sum(w[3] for w in self._windows)
        return sum(w[2] for w in self._windows) / samples if samples else 1.0

    def add(self, a_bits, b_bits, qber: float, sample_size: int) -> None:
        self._windows.append((np.asarray(a_bits), np.asarray(b_bits), int(round(qber * sample_size)), int(sample_size)))
        while self.size > self.max_bits and len(self._windows) > 1:
            self._windows.pop(0)        # drop the oldest window together with its QBER statistics

    def clear(self) -> None:
        self._windows = []

    def try_distill(self, decoy=None) -> str | None:
        """Cascade + privacy amplification over the whole pool. Returns a key (and empties the pool) or None."""
        if not self._windows:
            return None
        n = self.size
        samples = sum(w[3] for w in self._windows)
        q = self.qber
        est_leak = int(1.25 * n * binary_entropy(q)) + self.engine.TAG_BITS      # cheap pessimistic pre-check
        delta = 1.0 if decoy is None else decoy.single_photon_fraction
        e1 = None if decoy is None else decoy.e1_upper
        if self.engine.secret_key_length(n, q, samples, est_leak, delta, e1) < self.engine.KEY_BITS:
            return None
        a = np.concatenate([w[0] for w in self._windows])
        b = np.concatenate([w[1] for w in self._windows])
        try:
            key = self.engine.reconcile_and_amplify(a, b, qber=q, sample_size=samples, decoy=decoy)
        except ValueError:
            return None
        self.clear()
        return key
