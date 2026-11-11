"""Final key storage and the I4 key publisher (port 5559) for the operator display.

Every key block decision is published on topic "keys" (``kind: "block"``: outcome, QBER, secret / leaked bits),
so the operator sees aborts as well as keys; only ``outcome == "key"`` records carry the key (64 hex chars).
``telemetry/recorder.py`` writes them to ``keys.jsonl`` in the run directory.
Config: ``config/sandbox.yaml``.
"""
from __future__ import annotations

import json
from typing import Any


class KeyStore:
    """Accumulates final key material and publishes block decisions."""

    def __init__(self, cfg: dict[str, Any] | None = None, publisher: Any = None, path: str | None = None) -> None:
        """``publisher``: a ``telemetry.bus.Publisher("keys", validate=False)`` or None; ``path``: optional
        JSONL file the keys are appended to (the operator's key store)."""
        self.publisher = publisher
        self.path = path
        self.keys: list[dict[str, Any]] = []

    def add(self, key_bits: Any, meta: dict[str, Any]) -> None:
        """Store one key (hex string) with its block metadata."""
        rec = {"t": meta.get("t"), "key": key_bits, "secret_bits": meta.get("secret_bits")}
        self.keys.append(rec)
        if self.path:
            with open(self.path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(rec) + "\n")

    def publish(self, block_record: dict[str, Any]) -> None:
        """Publish one block decision (with the key when the outcome is "key")."""
        if self.publisher is not None:
            self.publisher.send(block_record)

    def __len__(self) -> int:
        return len(self.keys)
