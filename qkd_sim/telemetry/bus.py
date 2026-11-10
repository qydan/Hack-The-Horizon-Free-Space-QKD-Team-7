"""Typed ZeroMQ PUB/SUB helpers on 127.0.0.1 (section 11.1), with JSON-schema validation of telemetry.

Messages are two-frame multipart: [topic bytes, JSON payload]. Ports come from config/bus.yaml.

    pub = Publisher("telemetry")          # binds 127.0.0.1:5556
    pub.send(record)                      # validated against telemetry/schema.json
    sub = Subscriber("telemetry")         # connects to 127.0.0.1:5556
    topic, rec = sub.recv(timeout_s=1.0)

This module deliberately knows nothing about the truth port: ``Publisher``/``Subscriber`` refuse the name
"truth" unless ``allow_truth=True`` is passed, which only evaluation/ and tests/ do.
"""
from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml

_ROOT = Path(__file__).resolve().parent.parent
SCHEMA_PATH = Path(__file__).resolve().parent / "schema.json"


@lru_cache(maxsize=1)
def bus_config() -> dict[str, Any]:
    with open(_ROOT / "config" / "bus.yaml", "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


@lru_cache(maxsize=1)
def schema() -> dict[str, Any]:
    with open(SCHEMA_PATH, "r", encoding="utf-8") as fh:
        return json.load(fh)


class TelemetryValidationError(ValueError):
    pass


@lru_cache(maxsize=1)
def _validator():
    import jsonschema
    cls = jsonschema.validators.validator_for(schema())
    cls.check_schema(schema())
    return cls(schema())


def validate_telemetry(record: dict[str, Any]) -> None:
    errors = sorted(_validator().iter_errors(record), key=lambda e: list(e.path))
    if errors:
        msg = "; ".join(f"{'/'.join(map(str, e.path)) or '<root>'}: {e.message}" for e in errors)
        raise TelemetryValidationError(msg)


def endpoint(name: str, *, allow_truth: bool = False, internal: bool = False) -> str:
    cfg = bus_config()
    if name == "truth" and not allow_truth:
        raise PermissionError("the truth port is evaluation-only")
    port = cfg["internal_ports"][name] if internal else cfg["ports"][name]
    return f"tcp://{cfg['host']}:{port}"


class Publisher:
    def __init__(self, name: str, *, bind: bool = True, validate: bool | None = None, allow_truth: bool = False,
                 internal: bool = False):
        import zmq
        self.name = name
        self.topic = (bus_config()["topics"].get(name, name)).encode()
        self.sock = zmq.Context.instance().socket(zmq.PUB)
        ep = endpoint(name, allow_truth=allow_truth, internal=internal)
        (self.sock.bind if bind else self.sock.connect)(ep)
        self.validate = (name == "telemetry") if validate is None else validate

    def send(self, record: dict[str, Any]) -> None:
        if self.validate:
            validate_telemetry(record)
        self.sock.send_multipart([self.topic, json.dumps(record).encode()])

    def close(self) -> None:
        self.sock.close(linger=0)


class Subscriber:
    def __init__(self, name: str, *, bind: bool = False, allow_truth: bool = False, internal: bool = False,
                 topic: str | None = None):
        import zmq
        self.sock = zmq.Context.instance().socket(zmq.SUB)
        ep = endpoint(name, allow_truth=allow_truth, internal=internal)
        (self.sock.bind if bind else self.sock.connect)(ep)
        self.sock.setsockopt(zmq.SUBSCRIBE, (topic or "").encode())

    def recv(self, timeout_s: float | None = None) -> tuple[str, dict[str, Any]] | None:
        if timeout_s is not None and not self.sock.poll(int(timeout_s * 1000)):
            return None
        topic, payload = self.sock.recv_multipart()
        return topic.decode(), json.loads(payload)

    def close(self) -> None:
        self.sock.close(linger=0)
