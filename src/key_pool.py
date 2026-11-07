from typing import List, Optional, Tuple

import numpy as np

from qkd_core import QuantumCryptoEngine, binary_entropy


class KeyPool:
    """Accumulates sifted bits from accepted (QBER < gate) windows and distills a key
    from the pool once enough secret bits are available.

    Finite-size effects make a single 32k-pulse block yield no key above roughly 6-7% QBER.
    Pooling several windows shrinks the statistical margin and amortises the fixed
    leakage, so moderately noisy conditions still produce keys (just less often).
    """

    def __init__(self, engine: QuantumCryptoEngine, max_bits: int = 400_000):
        self.engine = engine
        self.max_bits = max_bits
        # per window: (alice_bits, bob_bits, sample_errors, sample_size)
        self._windows: List[Tuple[np.ndarray, np.ndarray, int, int]] = []

    @property
    def size(self) -> int:
        return int(sum(len(w[0]) for w in self._windows))

    @property
    def qber(self) -> float:
        samples = sum(w[3] for w in self._windows)
        return sum(w[2] for w in self._windows) / samples if samples else 1.0

    def add(self, a_bits: np.ndarray, b_bits: np.ndarray, qber: float, sample_size: int) -> None:
        self._windows.append((np.asarray(a_bits), np.asarray(b_bits),
                              int(round(qber * sample_size)), int(sample_size)))
        while self.size > self.max_bits and len(self._windows) > 1:
            self._windows.pop(0)  # drop the oldest window together with its QBER statistics

    def clear(self) -> None:
        self._windows = []

    def try_distill(self, decoy=None) -> Optional[str]:
        """Attempt Cascade + privacy amplification over the whole pool.
        Returns a key (and empties the pool), or None if not enough secret bits yet."""
        if not self._windows:
            return None
        n = self.size
        samples = sum(w[3] for w in self._windows)
        q = self.qber

        # Cheap pre-check with a pessimistic leakage estimate, to avoid running Cascade on
        # a pool that cannot possibly yield a key yet.
        est_leak = int(1.25 * n * binary_entropy(q)) + self.engine.TAG_BITS
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
