"""[OPEN] Builds the I2 telemetry record (telemetry/schema.json) and publishes it on port 5556; listens for
abort / re-acquire commands from the sandbox gate on port 5557.

Inputs:  per-frame fields from the other tracking modules.
Outputs: one validated record per camera frame (``telemetry.bus.Publisher("telemetry")``).
Config:  ``tracking.yaml: telemetry_out.*`` and ``config/bus.yaml``.
Rig behaviour: ``rig`` field set from the running rig; gimbal_* fields stay constant in rig_fixed.
"""
from __future__ import annotations

from typing import Any


class TelemetryOut:
    """Publisher for I2 telemetry and subscriber for I8 aborts."""

    def __init__(self, cfg: dict[str, Any], rig: str) -> None:
        """Open the bus sockets."""
        raise NotImplementedError

    def publish(self, fields: dict[str, Any]) -> None:
        """Build, validate and publish one record."""
        raise NotImplementedError

    def poll_abort(self) -> dict[str, Any] | None:
        """Return a pending abort / re-acquire command, if any."""
        raise NotImplementedError
