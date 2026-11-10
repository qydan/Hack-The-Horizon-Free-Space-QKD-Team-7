"""Records bus topics to JSONL (every record) and CSV (flat records) in a run directory.

Records every I-interface port except truth (5556-5560), plus the simulator-internal ports (frames_meta,
sim_events) which this recorder *binds* (publishers connect to it). Truth is recorded separately by
evaluation/truth_recorder.py.

    python -m telemetry.recorder --out runs/<id> [--duration 60]

Files: <name>.jsonl for every stream, telemetry.csv, photodiode.csv and frames_meta.csv (seq, t_capture,
t_delivered, exposure_gain, saturated).
"""
from __future__ import annotations

import argparse
import csv
import json
import signal
import time
from pathlib import Path
from typing import Any

from telemetry.bus import bus_config

CSV_STREAMS = {"telemetry", "photodiode", "frames_meta"}


class Recorder:
    def __init__(self, out_dir: Path, *, ports: list[str] | None = None, internal: list[str] | None = None):
        import zmq
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        cfg = bus_config()
        ctx = zmq.Context.instance()
        self.poller = zmq.Poller()
        self.socks: dict[Any, str] = {}
        for name in ports if ports is not None else ["telemetry", "abort", "photodiode", "keys", "eve"]:
            s = ctx.socket(zmq.SUB)
            s.connect(f"tcp://{cfg['host']}:{cfg['ports'][name]}")
            s.setsockopt(zmq.SUBSCRIBE, b"")
            self.poller.register(s, zmq.POLLIN)
            self.socks[s] = name
        for name in internal if internal is not None else ["frames_meta", "sim_events"]:
            s = ctx.socket(zmq.SUB)
            s.bind(f"tcp://{cfg['host']}:{cfg['internal_ports'][name]}")
            s.setsockopt(zmq.SUBSCRIBE, b"")
            self.poller.register(s, zmq.POLLIN)
            self.socks[s] = name
        self._jsonl: dict[str, Any] = {}
        self._csv: dict[str, tuple[Any, Any]] = {}
        self.counts: dict[str, int] = {}

    def _write(self, name: str, rec: dict[str, Any]) -> None:
        if name not in self._jsonl:
            self._jsonl[name] = open(self.out / f"{name}.jsonl", "w", encoding="utf-8")
        self._jsonl[name].write(json.dumps(rec) + "\n")
        self.counts[name] = self.counts.get(name, 0) + 1
        if name in CSV_STREAMS and isinstance(rec, dict):
            if name not in self._csv:
                fh = open(self.out / f"{name}.csv", "w", newline="", encoding="utf-8")
                w = csv.DictWriter(fh, fieldnames=list(rec.keys()), extrasaction="ignore")
                w.writeheader()
                self._csv[name] = (fh, w)
            self._csv[name][1].writerow({k: (json.dumps(v) if isinstance(v, (list, dict)) else v)
                                         for k, v in rec.items()})

    def poll_once(self, timeout_ms: int = 100) -> int:
        n = 0
        for sock, _ in self.poller.poll(timeout_ms):
            name = self.socks[sock]
            parts = sock.recv_multipart()
            try:
                rec = json.loads(parts[-1])
            except json.JSONDecodeError:
                rec = {"raw": parts[-1].decode(errors="replace")}
            self._write(name, rec)
            n += 1
        return n

    def run(self, duration_s: float | None = None, stop=lambda: False) -> None:
        t_end = None if duration_s is None else time.monotonic() + duration_s
        while not stop() and (t_end is None or time.monotonic() < t_end):
            self.poll_once()
        self.close()

    def close(self) -> None:
        for sock in list(self.socks):
            sock.close(linger=0)
        for fh in self._jsonl.values():
            fh.close()
        for fh, _ in self._csv.values():
            fh.close()
        with open(self.out / "recorder_counts.json", "w", encoding="utf-8") as fh:
            json.dump(self.counts, fh, indent=2)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=None, help="wall-clock seconds")
    args = ap.parse_args()
    stop = {"v": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("v", True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("v", True))
    Recorder(Path(args.out)).run(args.duration, stop=lambda: stop["v"])


if __name__ == "__main__":
    main()
