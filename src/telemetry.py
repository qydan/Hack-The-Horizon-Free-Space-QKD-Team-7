import csv
import math
from abc import ABC, abstractmethod
from typing import Dict, List


class TelemetryExhausted(Exception):
    """Raised by finite providers (e.g. a CSV replay) when there is no more data."""


class TelemetryProvider(ABC):
    """
    Contract for physical link telemetry. The crypto engine only depends on this.
    Any source (OpenCV camera, CSV replay, photodiode, ...) must implement it.

    ``get_metrics()`` returns ONE aggregate over the time window since the previous call
    (typically one BB84 block), not a single instantaneous sample:

    - "drift":     float >= 0. Distance from the beacon centroid to the calibrated
                   boresight, averaged over the window. Pixels.
    - "jitter":    float >= 0. RMS frame-to-frame centroid motion over the window.
                   Pixels per second.
    - "intensity": float in [0, 1]. Mean beacon brightness divided by the calibrated
                   clear-lock reference (exposure must be locked). 1.0 = full signal,
                   ~0 = lost beacon.

    Providers should raise TelemetryExhausted when a finite source runs out.
    """
    @abstractmethod
    def get_metrics(self) -> Dict[str, float]:
        pass


class CSVReplayProvider(TelemetryProvider):
    """Replays recorded or synthetic telemetry from a CSV file.

    Columns: ``timestamp`` (seconds), ``drift``, ``jitter``, ``intensity`` (per-frame
    values, same units as the TelemetryProvider contract). Each ``get_metrics()`` call
    consumes the next ``window_s`` seconds of rows and aggregates them:
    drift = mean, jitter = RMS, intensity = mean.
    """
    COLUMNS = ("timestamp", "drift", "jitter", "intensity")

    def __init__(self, path: str, window_s: float = 1.0, loop: bool = False):
        if window_s <= 0:
            raise ValueError("window_s must be positive")
        self.window_s = window_s
        self.loop = loop
        self._rows: List[Dict[str, float]] = self._load(path)
        self._cursor = 0

    @classmethod
    def _load(cls, path: str) -> List[Dict[str, float]]:
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            missing = [c for c in cls.COLUMNS if c not in (reader.fieldnames or [])]
            if missing:
                raise ValueError(f"{path}: missing column(s) {missing}; need {list(cls.COLUMNS)}")
            rows = []
            for lineno, rec in enumerate(reader, start=2):
                try:
                    row = {c: float(rec[c]) for c in cls.COLUMNS}
                except (TypeError, ValueError):
                    raise ValueError(f"{path}:{lineno}: non-numeric value in {rec}")
                if not all(math.isfinite(v) for v in row.values()):
                    raise ValueError(f"{path}:{lineno}: non-finite value in {rec}")
                if rows and row["timestamp"] <= rows[-1]["timestamp"]:
                    raise ValueError(f"{path}:{lineno}: timestamps must be strictly increasing")
                rows.append(row)
        if not rows:
            raise ValueError(f"{path}: no data rows")
        return rows

    @property
    def duration(self) -> float:
        return self._rows[-1]["timestamp"] - self._rows[0]["timestamp"]

    def reset(self) -> None:
        self._cursor = 0

    def get_metrics(self) -> Dict[str, float]:
        if self._cursor >= len(self._rows):
            if not self.loop:
                raise TelemetryExhausted("CSV telemetry replay finished")
            self._cursor = 0

        t_end = self._rows[self._cursor]["timestamp"] + self.window_s
        start = self._cursor
        while self._cursor < len(self._rows) and self._rows[self._cursor]["timestamp"] < t_end:
            self._cursor += 1
        window = self._rows[start:self._cursor]

        n = len(window)
        return {
            "drift": sum(r["drift"] for r in window) / n,
            "jitter": math.sqrt(sum(r["jitter"] ** 2 for r in window) / n),
            "intensity": sum(r["intensity"] for r in window) / n,
        }
