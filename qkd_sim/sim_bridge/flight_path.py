"""Scripted drone paths for controlled tracking tests (scenario ``drone.path``, simple flight model only).

A path is a list of segments, flown one after the other. Offsets are metres relative to the hover station
(ENU: +x away from the camera, +y left as seen from the camera, +z up). Each segment starts where the previous
one ended:

  {name, type: hold,  duration_s}                         stay put
  {name, type: step,  offset_m: [dx,dy,dz], duration_s}   jump the setpoint (the flight model makes the transient)
  {name, type: move,  to_m: [dx,dy,dz], speed_mps}        straight line at constant speed
  {name, type: sine,  amplitude_m: [ax,ay,az], freq_hz: [fx,fy,fz], duration_s}
                                                          start + A sin(2 pi f t) per axis (sweeps, figure-8s)
The setpoint goes to the flight model, which flies it with its own dynamics; ground truth records what the
drone actually did, so scoring never depends on the path being followed exactly.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np


def _duration(seg: dict[str, Any], start: np.ndarray) -> float:
    if seg["type"] == "move":
        dist = float(np.linalg.norm(np.asarray(seg["to_m"], float) - start))
        return dist / float(seg["speed_mps"]) if dist > 0 else 0.0
    return float(seg["duration_s"])


def _value(seg: dict[str, Any], start: np.ndarray, tau: float, dur: float) -> np.ndarray:
    kind = seg["type"]
    if kind == "hold":
        return start
    if kind == "step":
        return np.asarray(seg["offset_m"], float)
    if kind == "move":
        end = np.asarray(seg["to_m"], float)
        return start + (end - start) * (min(tau, dur) / dur if dur > 0 else 1.0)
    if kind == "sine":
        a = np.asarray(seg["amplitude_m"], float)
        f = np.asarray(seg["freq_hz"], float)
        return start + a * np.sin(2 * math.pi * f * tau)
    raise ValueError(f"unknown path segment type {kind!r}")


def windows(path: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """[{name, type, t_start, t_end}] relative to the start of the path."""
    out, t, start = [], 0.0, np.zeros(3)
    for k, seg in enumerate(path):
        dur = _duration(seg, start)
        out.append({"name": seg.get("name", f"{k}_{seg['type']}"), "type": seg["type"], "t_start": t, "t_end": t + dur})
        start = _value(seg, start, dur, dur)
        t += dur
    return out


def offset_at(path: list[dict[str, Any]], tau: float) -> np.ndarray:
    """Setpoint offset (m, relative to the station) at time ``tau`` after the path starts. Holds the last point."""
    start, t = np.zeros(3), 0.0
    for seg in path:
        dur = _duration(seg, start)
        if tau < t + dur:
            return _value(seg, start, tau - t, dur)
        start = _value(seg, start, dur, dur)
        t += dur
    return start


def validate(path: list[dict[str, Any]]) -> None:
    for k, seg in enumerate(path):
        kind = seg.get("type")
        need = {"hold": ["duration_s"], "step": ["offset_m", "duration_s"], "move": ["to_m", "speed_mps"],
                "sine": ["amplitude_m", "freq_hz", "duration_s"]}.get(kind)
        if need is None:
            raise ValueError(f"path segment {k}: unknown type {kind!r} (hold | step | move | sine)")
        missing = [n for n in need if n not in seg]
        if missing:
            raise ValueError(f"path segment {k} ({kind}): missing {missing}")
