"""Builds the I2 telemetry record (telemetry/schema.json) and publishes it on port 5556; listens for abort /
re-acquire commands from the sandbox gate on port 5557.

Inputs:  per-frame fields from ``tracking.tracker.Tracker``.
Outputs: one validated record per camera frame (``telemetry.bus.Publisher("telemetry")`` validates it).
Config:  ``tracking.yaml: telemetry_out.*`` and ``config/bus.yaml``.
Rig behaviour: ``rig`` field set from the running rig; gimbal_* fields stay constant in rig_fixed.
"""
from __future__ import annotations

import math
from typing import Any

from telemetry.bus import schema

def _nullable() -> set[str]:
    props = schema()["properties"]
    return {k for k, v in props.items() if "null" in (v.get("type") if isinstance(v.get("type"), list) else [])}


def build_record(fields: dict[str, Any], rig: str) -> dict[str, Any]:
    """A complete schema record: every required field (read from telemetry/schema.json, so it follows schema
    changes), plus any optional schema field the tracker supplied (e.g. spot_u_px / spot_v_px for the accuracy
    overlay); finite numbers, nulls only where the schema allows them. Fields not in the schema are dropped."""
    rec: dict[str, Any] = {}
    NULLABLE = _nullable()
    required = list(schema()["required"])
    optional = [k for k in schema()["properties"] if k not in required and k in fields]
    for k in required + optional:
        v = fields.get(k)
        if k == "rig":
            v = rig
        elif k == "state":
            v = str(getattr(v, "value", v))
        elif k == "saturated":
            v = bool(v)
        elif v is None:
            v = None if k in NULLABLE else 0.0
        else:
            v = float(v)
            if not math.isfinite(v):
                v = None if k in NULLABLE else 0.0
        rec[k] = v
    return rec


class TelemetryOut:
    """Publisher for I2 telemetry and subscriber for I8 aborts."""

    def __init__(self, cfg: dict[str, Any], rig: str, publisher: Any = None, abort_subscriber: Any = None) -> None:
        """Open the bus sockets (or use the given ones: tests)."""
        from telemetry.bus import Publisher, Subscriber
        self.rig = rig
        self.pub = publisher if publisher is not None else Publisher("telemetry")
        self.sub = abort_subscriber if abort_subscriber is not None else Subscriber("abort")
        self.sent = 0

    def publish(self, fields: dict[str, Any]) -> dict[str, Any]:
        """Build, validate and publish one record; returns it."""
        rec = build_record(fields, self.rig)
        self.pub.send(rec)
        self.sent += 1
        return rec

    def poll_abort(self) -> dict[str, Any] | None:
        """Return a pending abort / re-acquire command, if any (the newest wins)."""
        cmd = None
        while True:
            got = self.sub.recv(timeout_s=0.0)
            if got is None:
                return cmd
            cmd = got[1]

    def close(self) -> None:
        for s in (self.pub, self.sub):
            close = getattr(s, "close", None)
            if close:
                close()
