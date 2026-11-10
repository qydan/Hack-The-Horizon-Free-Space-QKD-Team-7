"""Telemetry: the only thing that crosses from the physical world into the key software (I2).

One small JSON message per camera frame (about 120 Hz), sent over ZeroMQ on localhost
from the tracker program to the sandbox program. No radio, no hardcoded values.

Example message::

    {"t": 12.508, "state": "TRACK", "drift_mrad": 3.1, "jitter_mrad": 0.8,
     "residual_mrad": 0.4, "gimbal_lag_mrad": 0.3, "lean_deg": 4.2, "range_m": 3.05,
     "intensity": 0.92, "rx_signal": 0.88, "rx_background": 0.04}

``rx_signal`` / ``rx_background`` exist only when the IR data link (I9) is built; they are
``None`` otherwise. Both are normalised to their preflight calibration values
(S / S_cal and B / B_cal).

A second ZeroMQ channel carries the gate's abort / re-acquire command back to the
tracker (I8).
"""
import csv
import json
from dataclasses import MISSING, asdict, dataclass, fields
from typing import Dict, Iterator, List, Optional

TELEMETRY_ENDPOINT = "tcp://127.0.0.1:5556"   # tracker -> sandbox (I2)
CONTROL_ENDPOINT = "tcp://127.0.0.1:5557"     # sandbox -> tracker (I8)

SEARCH, FINE_TRACK, COAST = "SEARCH", "TRACK", "COAST"
STATES = (SEARCH, FINE_TRACK, COAST)


class TelemetryExhausted(Exception):
    """Raised by finite sources (e.g. a CSV replay) when there is no more data."""


@dataclass
class TelemetryMessage:
    """One frame of measurements, all taken from the ground (no drone telemetry).

    t               seconds since the tracker started
    state           SEARCH | TRACK | COAST (beacon briefly lost, following the prediction)
    drift_mrad      slow (< ~0.5 Hz) wander of the drone's direction from the calibrated
                    boresight. At range L this is a lateral offset of drift * L.
    jitter_mrad     RMS of the fast (> ~0.5 Hz) part of the drone's motion over about 1 s
    residual_mrad   distance of the beacon from the image centre: what the gimbal and
                    tracker failed to remove (ground receiver pointing error)
    gimbal_lag_mrad commanded minus encoder angle, the lag part of the residual
    lean_deg        drone tilt (angle between the beacon axis and the line of sight to the
                    ground station), from the AprilTag pose
    range_m         distance to the drone, from the tag size in pixels
    intensity       beacon brightness / preflight calibration, 0..1 (about 0 = lost)
    rx_signal       optional: photodiode signal S / S_cal (IR data link only)
    rx_background   optional: photodiode background B / B_cal (IR data link only)
    """
    t: float
    state: str
    drift_mrad: float
    jitter_mrad: float
    residual_mrad: float
    gimbal_lag_mrad: float
    lean_deg: float
    range_m: float
    intensity: float
    rx_signal: Optional[float] = None
    rx_background: Optional[float] = None

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(",", ":"))

    @classmethod
    def from_dict(cls, d: Dict) -> "TelemetryMessage":
        names = {f.name for f in fields(cls)}
        unknown = set(d) - names
        if unknown:
            raise ValueError(f"unknown telemetry field(s): {sorted(unknown)}")
        required = [f.name for f in fields(cls) if f.default is MISSING]
        missing = [k for k in required if k not in d]
        if missing:
            raise ValueError(f"telemetry message missing field(s): {missing}")
        if d["state"] not in STATES:
            raise ValueError(f"bad tracking state {d['state']!r}")
        return cls(**d)

    @classmethod
    def from_json(cls, s: str) -> "TelemetryMessage":
        return cls.from_dict(json.loads(s))

    @property
    def locked(self) -> bool:
        """True while the link can carry signal (the tracker has the beacon or is coasting
        for a moment). SEARCH means the beacon is lost."""
        return self.state in (FINE_TRACK, COAST)


FIELDS = [f.name for f in fields(TelemetryMessage)]


# --------------------------------------------------------------------------- sources

class TelemetrySource:
    """Anything that yields TelemetryMessages in time order. The sandbox only depends on this."""

    def __iter__(self) -> Iterator[TelemetryMessage]:
        raise NotImplementedError

    def close(self) -> None:
        pass


class CSVReplaySource(TelemetrySource):
    """Replays recorded or synthetic telemetry (one row per message; columns = FIELDS).
    ``rx_signal`` and ``rx_background`` may be empty."""

    def __init__(self, path: str):
        self.messages: List[TelemetryMessage] = self._load(path)

    @staticmethod
    def _load(path: str) -> List[TelemetryMessage]:
        out: List[TelemetryMessage] = []
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            required = FIELDS[:9]
            missing = [c for c in required if c not in (reader.fieldnames or [])]
            if missing:
                raise ValueError(f"{path}: missing column(s) {missing}; need {FIELDS}")
            for lineno, rec in enumerate(reader, start=2):
                d: Dict = {"state": rec["state"]}
                try:
                    for k in FIELDS:
                        if k == "state":
                            continue
                        v = rec.get(k)
                        if v in (None, ""):
                            if k in required:
                                raise ValueError(k)
                            d[k] = None
                        else:
                            d[k] = float(v)
                    msg = TelemetryMessage.from_dict(d)
                except ValueError as e:
                    raise ValueError(f"{path}:{lineno}: bad row {rec} ({e})")
                if out and msg.t <= out[-1].t:
                    raise ValueError(f"{path}:{lineno}: timestamps must be strictly increasing")
                out.append(msg)
        if not out:
            raise ValueError(f"{path}: no data rows")
        return out

    @property
    def duration(self) -> float:
        return self.messages[-1].t - self.messages[0].t

    def __iter__(self) -> Iterator[TelemetryMessage]:
        return iter(self.messages)


def write_csv(path: str, messages) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(FIELDS)
        for m in messages:
            d = asdict(m)
            w.writerow(["" if d[k] is None else d[k] if k == "state" else f"{d[k]:.5f}" if k == "t" else f"{d[k]:.5g}"
                        for k in FIELDS])


# --------------------------------------------------------------------------- ZeroMQ

class ZmqPublisher:
    """Tracker side of I2: publishes one JSON message per frame."""

    def __init__(self, endpoint: str = TELEMETRY_ENDPOINT):
        import zmq
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.bind(endpoint)

    def send(self, msg: TelemetryMessage) -> None:
        self._sock.send_string(msg.to_json())

    def close(self) -> None:
        self._sock.close(0)


class ZmqSubscriber(TelemetrySource):
    """Sandbox side of I2. Iterating blocks until a message arrives; stops after
    ``timeout_s`` of silence (None = wait forever)."""

    def __init__(self, endpoint: str = TELEMETRY_ENDPOINT, timeout_s: Optional[float] = 5.0):
        import zmq
        self._zmq = zmq
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self._sock.connect(endpoint)
        self._timeout_ms = -1 if timeout_s is None else int(timeout_s * 1000)

    def __iter__(self) -> Iterator[TelemetryMessage]:
        while True:
            if not self._sock.poll(self._timeout_ms):
                return
            yield TelemetryMessage.from_json(self._sock.recv_string())

    def close(self) -> None:
        self._sock.close(0)


class ControlPublisher:
    """Sandbox side of I8: tells the tracker to drop lock and re-acquire after an abort."""

    def __init__(self, endpoint: str = CONTROL_ENDPOINT):
        import zmq
        self._sock = zmq.Context.instance().socket(zmq.PUB)
        self._sock.bind(endpoint)

    def send_reacquire(self, reason: str = "abort") -> None:
        self._sock.send_string(json.dumps({"cmd": "reacquire", "reason": reason}))

    def close(self) -> None:
        self._sock.close(0)


class ControlSubscriber:
    """Tracker side of I8. ``poll()`` is non-blocking and returns the commands received."""

    def __init__(self, endpoint: str = CONTROL_ENDPOINT):
        import zmq
        self._zmq = zmq
        self._sock = zmq.Context.instance().socket(zmq.SUB)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self._sock.connect(endpoint)

    def poll(self) -> List[Dict]:
        cmds = []
        while self._sock.poll(0):
            cmds.append(json.loads(self._sock.recv_string()))
        return cmds

    def close(self) -> None:
        self._sock.close(0)
