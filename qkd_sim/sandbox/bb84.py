"""BB84 post-processing: sifting, QBER sample, Cascade error correction, Toeplitz privacy amplification.

Ported from src/qkd_core.py (``QuantumCryptoEngine``). The error rate fed into ``transmit_detected`` comes from
the channel model (``channel_decoys``), which is driven by the measured tracking telemetry through the error
budget. Basis and parity exchanges are in-process calls standing in for the authenticated classical channel.

Randomness: Alice's bits and bases come from ``os.urandom``; simulation randomness (Bob's bases, channel noise,
sample choice, public Cascade permutations, Toeplitz seed) from a NumPy generator seeded from ``os.urandom``.
Passing ``seed`` makes everything deterministic: tests only, never for real keys.

Config: ``config/sandbox.yaml: bb84``.
"""
from __future__ import annotations

import hashlib
import math
import os
from typing import Any

import numpy as np


def binary_entropy(x: float) -> float:
    """h2(x) in bits, with h2(0) = h2(1) = 0."""
    if x <= 0.0 or x >= 1.0:
        return 0.0
    return -x * math.log2(x) - (1.0 - x) * math.log2(1.0 - x)


class QuantumCryptoEngine:
    """transmit -> sift -> estimate QBER -> Cascade -> privacy amplification."""

    default_seed: int | None = None     # tests set this for reproducibility

    def __init__(self, cfg: dict[str, Any] | None = None, seed: int | None = None):
        c = cfg or {}
        self.KEY_BITS = int(c.get("key_bits", 256))
        self.CASCADE_PASSES = int(c.get("cascade_passes", 4))
        self.TAG_BITS = int(c.get("tag_bits", 64))
        self.sample_fraction = float(c.get("sample_fraction", 0.2))
        self.eps_est = float(c.get("eps_est", 1e-6))
        self.eps_sec = float(c.get("eps_sec", 1e-9))
        if seed is None:
            seed = self.default_seed
        self._seeded = seed is not None
        self.rng = np.random.default_rng(seed if self._seeded else int.from_bytes(os.urandom(16), "big"))
        self.last_report: dict[str, float] = {}

    # ------------------------------------------------------------------ randomness
    def _secure_bits(self, n: int) -> np.ndarray:
        if self._seeded:
            return self.rng.integers(0, 2, size=n, dtype=np.int64)
        raw = np.frombuffer(os.urandom((n + 7) // 8), dtype=np.uint8)
        return np.unpackbits(raw)[:n].astype(np.int64)

    # ------------------------------------------------------------------ channel
    def transmit_detected(self, n_detected: int, error_rate: float):
        """BB84 for the pulses Bob detected. The channel model decides how many clicks and what fraction of
        them are wrong on matched bases. Mismatched bases give a random result (discarded by sifting)."""
        n = int(n_detected)
        a_bits = self._secure_bits(n)
        a_bases = self._secure_bits(n)
        b_bases = self.rng.integers(0, 2, size=n, dtype=np.int64)
        flips = self.rng.random(n) < min(max(error_rate, 0.0), 0.5)
        random_bits = self.rng.integers(0, 2, size=n, dtype=np.int64)
        b_bits = np.where(a_bases == b_bases, a_bits ^ flips, random_bits).astype(np.int64)
        return a_bits, a_bases, b_bits, b_bases

    # ------------------------------------------------------------------ sifting / parameter estimation
    def sift_and_estimate(self, a_bits, a_bases, b_bits, b_bases, sample_ratio: float | None = None):
        """Discard basis mismatches, then destructively sample ``sample_ratio`` of the sifted bits for QBER.
        Returns (qber, alice_remaining, bob_remaining)."""
        ratio = self.sample_fraction if sample_ratio is None else sample_ratio
        matches = a_bases == b_bases
        a_s, b_s = a_bits[matches], b_bits[matches]
        if len(a_s) == 0:
            return 1.0, np.array([], dtype=np.int64), np.array([], dtype=np.int64)
        m = int(len(a_s) * ratio)
        idx = self.rng.choice(len(a_s), size=m, replace=False)
        qber = float(np.sum(a_s[idx] != b_s[idx]) / m) if m > 0 else 1.0
        keep = np.setdiff1d(np.arange(len(a_s)), idx)
        return qber, a_s[keep], b_s[keep]

    # ------------------------------------------------------------------ error correction
    def cascade_reconcile(self, a_key, b_key, qber: float):
        """Cascade (Brassard & Salvail). Bob corrects his key; every parity Alice announces counts as leaked.
        Returns (bob_corrected, leaked_bits, errors_corrected); ValueError if the confirmation hash differs."""
        n = len(a_key)
        if n == 0:
            raise ValueError("Cannot reconcile an empty key.")
        a = np.asarray(a_key, dtype=np.int64)
        b = np.array(b_key, dtype=np.int64)
        leaked = 0
        corrected = 0

        def alice_parity(pos) -> int:
            nonlocal leaked
            leaked += 1
            return int(a[pos].sum() & 1)

        def bob_parity(pos) -> int:
            return int(b[pos].sum() & 1)

        def bisect(block) -> int:
            while len(block) > 1:
                half = block[: len(block) // 2]
                block = half if alice_parity(half) != bob_parity(half) else block[len(block) // 2:]
            return int(block[0])

        k1 = int(min(n, max(4, math.ceil(0.73 / max(qber, 1e-3)))))
        passes = []
        for i in range(self.CASCADE_PASSES):
            k = min(n, k1 * (2 ** i))
            perm = np.arange(n) if i == 0 else self.rng.permutation(n)      # permutations are public
            blocks = [perm[s:s + k] for s in range(0, n, k)]
            block_of = np.empty(n, dtype=np.int64)
            for idx, blk in enumerate(blocks):
                block_of[blk] = idx
            passes.append((blocks, block_of))
            for blk in blocks:
                if alice_parity(blk) == bob_parity(blk):
                    continue
                pos = bisect(blk)
                b[pos] ^= 1
                corrected += 1
                stack = [(pos, i)]                    # cascade back through the earlier passes
                while stack:
                    q, found_in = stack.pop()
                    for j in range(i + 1):
                        if j == found_in:
                            continue
                        pb = passes[j][0][passes[j][1][q]]
                        if alice_parity(pb) != bob_parity(pb):
                            r = bisect(pb)
                            b[r] ^= 1
                            corrected += 1
                            stack.append((r, j))
        leaked += self.TAG_BITS                       # key confirmation
        tag_a = hashlib.sha256(np.packbits(a.astype(np.uint8)).tobytes()).digest()[: self.TAG_BITS // 8]
        tag_b = hashlib.sha256(np.packbits(b.astype(np.uint8)).tobytes()).digest()[: self.TAG_BITS // 8]
        if tag_a != tag_b:
            raise ValueError("Reconciliation failed to match keys.")
        return b, leaked, corrected

    # ------------------------------------------------------------------ privacy amplification
    def finite_size_margin(self, n: int, m: int) -> float:
        """Hoeffding-type margin between the sampled QBER (m bits) and the QBER of the n key bits."""
        if m <= 0 or n <= 0:
            return 0.5
        return math.sqrt(((n + m) / (n * m)) * ((m + 1) / m) * math.log(1.0 / self.eps_est) / 2.0)

    def secret_key_length(self, n: int, qber: float, sample_size: int, leaked_bits: int,
                          single_photon_fraction: float = 1.0, e1_upper: float | None = None) -> int:
        """n * Delta * (1 - h2(e_upper)) - leakage - 2 log2(1/eps_sec)."""
        base = qber if e1_upper is None else e1_upper
        e_upper = min(0.5, base + self.finite_size_margin(n, sample_size))
        loss = 2.0 * math.log2(1.0 / self.eps_sec)
        usable = n * max(0.0, single_photon_fraction) * (1.0 - binary_entropy(e_upper))
        return int(math.floor(usable - leaked_bits - loss))

    def privacy_amplify(self, key, secret_length: int) -> str:
        """Toeplitz hash of the reconciled key down to KEY_BITS bits (hex); refuses below KEY_BITS secret bits."""
        if secret_length < self.KEY_BITS:
            raise ValueError(f"Insufficient secret key length ({secret_length} < {self.KEY_BITS} bits); "
                             "key discarded.")
        seed = self.rng.integers(0, 2, size=len(key) + self.KEY_BITS - 1, dtype=np.int32)   # public
        return np.packbits(toeplitz_amplify(key, self.KEY_BITS, seed).astype(np.uint8)).tobytes().hex()

    def reconcile_and_amplify(self, a_keys, b_keys, qber: float | None = None, sample_size: int | None = None,
                              decoy=None) -> str:
        """Cascade then Toeplitz. ``decoy`` (a ``DecoyResult``) limits the secret part to the certified
        single-photon fraction. Returns a KEY_BITS-bit key as hex; ``last_report`` holds diagnostics."""
        n = len(a_keys)
        b_fixed, leaked, corrected = self.cascade_reconcile(a_keys, b_keys, 0.05 if qber is None else qber)
        qber_used = qber if qber is not None else corrected / n
        m = sample_size if sample_size is not None else max(1, n // 4)
        delta = 1.0 if decoy is None else decoy.single_photon_fraction
        e1 = None if decoy is None else decoy.e1_upper
        secret_len = self.secret_key_length(n, qber_used, m, leaked, delta, e1)
        self.last_report = {"reconciled_bits": n, "errors_corrected": corrected, "leaked_bits": leaked,
                            "secret_length": secret_len, "qber_used": float(qber_used),
                            "single_photon_fraction": float(delta)}
        return self.privacy_amplify(b_fixed, secret_len)


# ----------------------------------------------------------------------------- interface of the original stub
def sift(alice: Any, bob: Any) -> tuple[Any, Any]:
    """Keep matching-basis bits. ``alice`` and ``bob`` are (bits, bases) pairs."""
    (a_bits, a_bases), (b_bits, b_bases) = alice, bob
    m = np.asarray(a_bases) == np.asarray(b_bases)
    return np.asarray(a_bits)[m], np.asarray(b_bits)[m]


def estimate_qber(alice_sifted: Any, bob_sifted: Any, sample_fraction: float, rng=None) -> float:
    """QBER of a random disclosed sample (non-destructive; ``QuantumCryptoEngine.sift_and_estimate`` also
    removes the sample from the key)."""
    a, b = np.asarray(alice_sifted), np.asarray(bob_sifted)
    m = int(len(a) * sample_fraction)
    if m <= 0:
        return 1.0
    idx = (rng or np.random.default_rng()).choice(len(a), size=m, replace=False)
    return float(np.mean(a[idx] != b[idx]))


def cascade(alice_key: Any, bob_key: Any, qber: float) -> tuple[Any, int]:
    """Cascade error correction; return the corrected key and the number of bits leaked."""
    b, leaked, _ = QuantumCryptoEngine().cascade_reconcile(alice_key, bob_key, qber)
    return b, leaked


def toeplitz_amplify(key: Any, out_len: int, seed: Any) -> Any:
    """Toeplitz-hash privacy amplification. ``seed`` = len(key) + out_len - 1 public random bits.

    out[i] = sum_j T[i, j] key[j] mod 2 with T[i, j] = seed[n - 1 + i - j]: a slice of the full convolution
    seed * key, computed by FFT (exact after rounding) instead of building the out_len x n matrix
    (src/ did, which costs ~150 MB per 1 s block at 1 GHz)."""
    k = np.asarray(key, dtype=np.float64)
    n = len(k)
    s = np.asarray(seed, dtype=np.float64)[: n + out_len - 1]
    if len(s) < n + out_len - 1:
        raise ValueError("Toeplitz seed too short")
    nfft = 1 << int(len(s) + n - 1).bit_length()
    conv = np.fft.irfft(np.fft.rfft(s, nfft) * np.fft.rfft(k, nfft), nfft)
    return np.rint(conv[n - 1:n - 1 + out_len]).astype(np.int64) & 1
