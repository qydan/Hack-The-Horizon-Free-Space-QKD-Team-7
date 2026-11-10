"""[OPEN] BB84 post-processing: sifting, Cascade error correction, Toeplitz privacy amplification.

Config: ``config/sandbox.yaml``.
"""
from __future__ import annotations

from typing import Any


def sift(alice: Any, bob: Any) -> tuple[Any, Any]:
    """Keep matching-basis bits."""
    raise NotImplementedError


def estimate_qber(alice_sifted: Any, bob_sifted: Any, sample_fraction: float) -> float:
    """Estimate the QBER from a disclosed sample."""
    raise NotImplementedError


def cascade(alice_key: Any, bob_key: Any, qber: float) -> tuple[Any, int]:
    """Cascade error correction; return the corrected key and the number of bits leaked."""
    raise NotImplementedError


def toeplitz_amplify(key: Any, out_len: int, seed: Any) -> Any:
    """Toeplitz-hash privacy amplification."""
    raise NotImplementedError
