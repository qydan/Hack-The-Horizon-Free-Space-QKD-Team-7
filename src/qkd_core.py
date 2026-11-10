import hashlib
import math
import os
from typing import Dict, Optional, Tuple

import numpy as np


def binary_entropy(x: float) -> float:
    """h2(x) in bits, with h2(0) = h2(1) = 0."""
    if x <= 0.0 or x >= 1.0:
        return 0.0
    return -x * math.log2(x) - (1.0 - x) * math.log2(1.0 - x)


class QuantumCryptoEngine:
    """Software BB84 pipeline: transmit -> sift -> estimate QBER -> Cascade -> privacy amplification.

    The error rate fed into ``transmit_detected`` / ``generate_and_transmit`` comes from the
    sandbox channel (``channel.py``), which is driven by the measured tracking telemetry
    through the error budget (``error_budget.py``).

    Randomness:
      * Alice's key material (bits and bases) comes from ``os.urandom``.
      * Simulation randomness (Bob's bases, channel noise, QBER sample choice, public
        Cascade permutations, Toeplitz seed) comes from a NumPy generator seeded from
        ``os.urandom``. None of that is secret key material.
      * Passing ``seed`` (or setting ``QuantumCryptoEngine.default_seed``) makes *everything*
        deterministic. That is for tests only; never use a seed for real keys.
    """

    default_seed: Optional[int] = None  # tests set this for reproducibility

    KEY_BITS = 256          # length of the final symmetric key
    CASCADE_PASSES = 4
    TAG_BITS = 64           # leaked by the post-Cascade key confirmation hash

    def __init__(self, block_size: int = 32768, seed: Optional[int] = None):
        self.block_size = block_size

        # Security parameters
        self.eps_est = 1e-6   # failure probability of the finite-size QBER bound
        self.eps_sec = 1e-9   # secrecy parameter used in the privacy-amplification loss term

        if seed is None:
            seed = self.default_seed
        self._seeded = seed is not None
        self.rng = np.random.default_rng(
            seed if self._seeded else int.from_bytes(os.urandom(16), "big"))

        # Diagnostics from the most recent reconcile_and_amplify() call
        self.last_report: Dict[str, float] = {}

    # ------------------------------------------------------------------ randomness

    def _secure_bits(self, n: int) -> np.ndarray:
        """Key-material bits: OS CSPRNG (or the seeded generator in test mode)."""
        if self._seeded:
            return self.rng.integers(0, 2, size=n, dtype=np.int64)
        raw = np.frombuffer(os.urandom((n + 7) // 8), dtype=np.uint8)
        return np.unpackbits(raw)[:n].astype(np.int64)

    # ------------------------------------------------------------------ channel

    def transmit_detected(self, n_detected: int, error_rate: float
                          ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """BB84 over the sandbox channel, for the pulses Bob actually detected.

        The channel model (``channel.py``) decides how many signal pulses produced a click
        and what fraction of those clicks are wrong when the bases match (optical error,
        dark counts, background, Eve). Alice's bits and bases are fresh key material;
        Bob's bases are random. Mismatched bases give a random result; the sifting step
        later throws those away.
        """
        n = int(n_detected)
        a_bits = self._secure_bits(n)
        a_bases = self._secure_bits(n)
        b_bases = self.rng.integers(0, 2, size=n, dtype=np.int64)
        flips = self.rng.random(n) < min(max(error_rate, 0.0), 0.5)
        random_bits = self.rng.integers(0, 2, size=n, dtype=np.int64)
        b_bits = np.where(a_bases == b_bases, a_bits ^ flips, random_bits).astype(np.int64)
        return a_bits, a_bases, b_bits, b_bases

    def generate_and_transmit(self, p_error: float, eve_fraction: float = 0.0
                              ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Standalone BB84 round over ``block_size`` pulses with a fixed error probability
        ``p_error`` on matched bases. Used by the engine-level tests; the sandbox uses
        ``transmit_detected`` with the click counts and error rate from ``channel.py``.

        ``eve_fraction`` is the fraction of pulses an intercept-resend eavesdropper
        measures in a random basis and resends. Full interception (1.0) raises the
        sifted-key QBER by about 25 percentage points.
        """
        n = self.block_size
        # Alice: random bits and bases (0: rectilinear, 1: diagonal)
        a_bits = self._secure_bits(n)
        a_bases = self._secure_bits(n)

        # State actually arriving at Bob: Alice's, or Eve's re-prepared one
        state_bits, state_bases = a_bits, a_bases
        if eve_fraction > 0.0:
            hit = self.rng.random(n) < eve_fraction
            e_bases = self.rng.integers(0, 2, size=n, dtype=np.int64)
            e_rand = self.rng.integers(0, 2, size=n, dtype=np.int64)
            e_bits = np.where(e_bases == a_bases, a_bits, e_rand)  # wrong basis -> random result
            state_bits = np.where(hit, e_bits, a_bits)
            state_bases = np.where(hit, e_bases, a_bases)

        # Bob: random measurement bases
        b_bases = self.rng.integers(0, 2, size=n, dtype=np.int64)

        # True where the environment corrupts the photon
        noise_mask = self.rng.random(n) < p_error

        b_bits = state_bits.copy()
        mismatched = state_bases != b_bases
        # Mismatched bases yield a completely random bit (quantum collapse)
        random_bits = self.rng.integers(0, 2, size=n, dtype=np.int64)
        b_bits[mismatched] = random_bits[mismatched]
        # Environmental flip on matched bases
        b_bits[~mismatched & noise_mask] ^= 1

        return a_bits, a_bases, b_bits, b_bases

    # ------------------------------------------------------------------ sifting / PE

    def sift_and_estimate(self, a_bits: np.ndarray, a_bases: np.ndarray,
                          b_bits: np.ndarray, b_bases: np.ndarray,
                          sample_ratio: float = 0.2) -> Tuple[float, np.ndarray, np.ndarray]:
        """Discards basis mismatches and destructively samples for QBER."""
        # 1. Sifting (in the real protocol only the bases cross the classical channel)
        matches = (a_bases == b_bases)
        a_sifted = a_bits[matches]
        b_sifted = b_bits[matches]

        if len(a_sifted) == 0:
            return 1.0, np.array([]), np.array([])

        # 2. Parameter Estimation (The 11% Gate)
        sample_size = int(len(a_sifted) * sample_ratio)
        sample_indices = self.rng.choice(len(a_sifted), size=sample_size, replace=False)

        mismatches = np.sum(a_sifted[sample_indices] != b_sifted[sample_indices])
        qber = mismatches / sample_size if sample_size > 0 else 1.0

        # Remove the revealed sample bits from the secure key pool
        keep_indices = np.setdiff1d(np.arange(len(a_sifted)), sample_indices)
        return qber, a_sifted[keep_indices], b_sifted[keep_indices]

    # ------------------------------------------------------------------ error correction

    def cascade_reconcile(self, a_key: np.ndarray, b_key: np.ndarray,
                          qber: float) -> Tuple[np.ndarray, int, int]:
        """Cascade error correction (Brassard & Salvail).

        Bob corrects his key. Alice's key is only touched through ``alice_parity``, which
        models a public-channel parity announcement and counts every bit it leaks.

        Returns (bob_corrected_key, leaked_bits, errors_corrected). Raises ValueError if
        the final confirmation hash shows the keys still differ.
        """
        n = len(a_key)
        if n == 0:
            raise ValueError("Cannot reconcile an empty key.")

        a = np.asarray(a_key, dtype=np.int64)
        b = np.array(b_key, dtype=np.int64)  # copy
        leaked = 0
        corrected = 0

        def alice_parity(pos: np.ndarray) -> int:
            nonlocal leaked
            leaked += 1
            return int(a[pos].sum() & 1)

        def bob_parity(pos: np.ndarray) -> int:
            return int(b[pos].sum() & 1)

        def bisect(block: np.ndarray) -> int:
            """Locate one error in a block with odd error count via binary search."""
            while len(block) > 1:
                half = block[: len(block) // 2]
                if alice_parity(half) != bob_parity(half):
                    block = half
                else:
                    block = block[len(block) // 2:]
            return int(block[0])

        k1 = int(min(n, max(4, math.ceil(0.73 / max(qber, 1e-3)))))
        passes = []  # per pass: (list of blocks, position -> block index)

        for i in range(self.CASCADE_PASSES):
            k = min(n, k1 * (2 ** i))
            perm = np.arange(n) if i == 0 else self.rng.permutation(n)  # permutations are public
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

                # Cascade back: the flip changes parity of that bit's block in every other pass.
                stack = [(pos, i)]
                while stack:
                    q, found_in = stack.pop()
                    for j in range(i + 1):
                        if j == found_in:
                            continue
                        prev_blocks, prev_block_of = passes[j]
                        pb = prev_blocks[prev_block_of[q]]
                        if alice_parity(pb) != bob_parity(pb):
                            r = bisect(pb)
                            b[r] ^= 1
                            corrected += 1
                            stack.append((r, j))

        # Key confirmation: compare short hashes. Leaks TAG_BITS more bits.
        leaked += self.TAG_BITS
        tag_a = hashlib.sha256(np.packbits(a.astype(np.uint8)).tobytes()).digest()[: self.TAG_BITS // 8]
        tag_b = hashlib.sha256(np.packbits(b.astype(np.uint8)).tobytes()).digest()[: self.TAG_BITS // 8]
        if tag_a != tag_b:
            raise ValueError("Reconciliation failed to match keys.")

        return b, leaked, corrected

    # ------------------------------------------------------------------ privacy amplification

    def finite_size_margin(self, n: int, m: int) -> float:
        """Hoeffding-type statistical margin between the sampled QBER (m bits) and the
        QBER of the remaining n key bits. A conservative approximation, not a full
        composable security proof."""
        if m <= 0 or n <= 0:
            return 0.5
        return math.sqrt(((n + m) / (n * m)) * ((m + 1) / m) * math.log(1.0 / self.eps_est) / 2.0)

    def secret_key_length(self, n: int, qber: float, sample_size: int, leaked_bits: int,
                          single_photon_fraction: float = 1.0,
                          e1_upper: Optional[float] = None) -> int:
        """Bits of secret key extractable.

        Ideal single-photon source (default):
            n(1 - h2(e_upper)) - leakage - 2 log2(1/eps_sec)
        With decoy-state analysis (weak coherent pulses):
            n * Delta * (1 - h2(e1_upper + margin)) - leakage - 2 log2(1/eps_sec)
        where Delta is the certified single-photon fraction of the sifted key.
        """
        base = qber if e1_upper is None else e1_upper
        e_upper = min(0.5, base + self.finite_size_margin(n, sample_size))
        loss = 2.0 * math.log2(1.0 / self.eps_sec)
        usable = n * max(0.0, single_photon_fraction) * (1.0 - binary_entropy(e_upper))
        return int(math.floor(usable - leaked_bits - loss))

    def privacy_amplify(self, key: np.ndarray, secret_length: int) -> str:
        """Toeplitz universal hashing of the reconciled key down to KEY_BITS bits (hex).

        Refuses to run if fewer than KEY_BITS secret bits can be extracted.
        """
        if secret_length < self.KEY_BITS:
            raise ValueError(
                f"Insufficient secret key length ({secret_length} < {self.KEY_BITS} bits); "
                "key discarded.")
        n = len(key)
        seed_bits = self.rng.integers(0, 2, size=n + self.KEY_BITS - 1, dtype=np.int32)  # public
        idx = (n - 1) + np.arange(self.KEY_BITS)[:, None] - np.arange(n)[None, :]
        toeplitz = seed_bits[idx]
        out = (toeplitz @ np.asarray(key, dtype=np.int32)) & 1
        return np.packbits(out.astype(np.uint8)).tobytes().hex()

    def reconcile_and_amplify(self, a_keys: np.ndarray, b_keys: np.ndarray,
                              qber: Optional[float] = None,
                              sample_size: Optional[int] = None,
                              decoy=None) -> str:
        """Cascade error correction followed by Toeplitz privacy amplification.

        ``qber`` / ``sample_size`` are the parameter-estimation results from
        ``sift_and_estimate`` (sample size is the number of sampled bits). If omitted, the
        error rate is taken from the number of errors Cascade corrected and the sample size
        defaults to n/4 (the default 20% sample ratio).

        ``decoy`` is an optional decoy-state result (see ``decoy.py``) exposing
        ``single_photon_fraction`` and ``e1_upper``; if given, only the certified
        single-photon part of the key is counted as secret.

        Returns a 256-bit key as 64 hex chars. ``self.last_report`` holds diagnostics.
        """
        n = len(a_keys)
        qber_hint = 0.05 if qber is None else qber
        b_fixed, leaked, corrected = self.cascade_reconcile(a_keys, b_keys, qber_hint)

        qber_used = qber if qber is not None else corrected / n
        m = sample_size if sample_size is not None else max(1, n // 4)
        delta = 1.0 if decoy is None else decoy.single_photon_fraction
        e1 = None if decoy is None else decoy.e1_upper
        secret_len = self.secret_key_length(n, qber_used, m, leaked, delta, e1)

        self.last_report = {
            "reconciled_bits": n,
            "errors_corrected": corrected,
            "leaked_bits": leaked,
            "secret_length": secret_len,
            "qber_used": float(qber_used),
            "single_photon_fraction": float(delta),
        }
        return self.privacy_amplify(b_fixed, secret_len)
