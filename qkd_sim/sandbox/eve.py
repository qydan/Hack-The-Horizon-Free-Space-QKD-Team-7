"""[OPEN] Eavesdropper models toggled from the operator UI (port 5560): intercept-resend, beam splitting.

Config: ``config/sandbox.yaml``.
"""
from __future__ import annotations

from typing import Any


class Eve:
    """Eavesdropper attached to the channel."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        """Store configuration."""
        raise NotImplementedError

    def set_mode(self, mode: str | None) -> None:
        """Enable or disable an attack mode."""
        raise NotImplementedError

    def apply(self, detections: dict[str, Any]) -> dict[str, Any]:
        """Modify detections according to the active attack."""
        raise NotImplementedError
