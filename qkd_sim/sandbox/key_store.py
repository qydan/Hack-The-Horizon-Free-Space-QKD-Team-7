"""[OPEN] Final key storage and the I4 key publisher (port 5559) for the operator display.

Config: ``config/sandbox.yaml``.
"""
from __future__ import annotations

from typing import Any


class KeyStore:
    """Accumulates final key material."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        """Open the key publisher."""
        raise NotImplementedError

    def add(self, key_bits: Any, meta: dict[str, Any]) -> None:
        """Store and publish one key block."""
        raise NotImplementedError
