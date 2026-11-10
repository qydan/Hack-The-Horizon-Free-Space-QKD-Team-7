"""Records the ground-truth stream (port 5590) to truth.jsonl and truth.csv in a run directory.

    python -m evaluation.truth_recorder --out runs/<id> [--duration 60]
"""
from __future__ import annotations

import argparse
import signal
from pathlib import Path

from telemetry.recorder import Recorder


class TruthRecorder(Recorder):
    def __init__(self, out_dir: Path):
        import zmq

        from telemetry.bus import endpoint
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        ctx = zmq.Context.instance()
        self.poller = zmq.Poller()
        s = ctx.socket(zmq.SUB)
        s.connect(endpoint("truth", allow_truth=True))
        s.setsockopt(zmq.SUBSCRIBE, b"")
        self.poller.register(s, zmq.POLLIN)
        self.socks = {s: "truth"}
        self._jsonl, self._csv, self.counts = {}, {}, {}

    def _write(self, name, rec):  # truth gets a CSV too
        super()._write(name, rec)
        if "truth_csv" not in self._csv:
            import csv
            fh = open(self.out / "truth.csv", "w", newline="", encoding="utf-8")
            w = csv.DictWriter(fh, fieldnames=list(rec.keys()), extrasaction="ignore")
            w.writeheader()
            self._csv["truth_csv"] = (fh, w)
        import json
        self._csv["truth_csv"][1].writerow({k: (json.dumps(v) if isinstance(v, (list, dict)) else v)
                                            for k, v in rec.items()})


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()
    stop = {"v": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("v", True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("v", True))
    TruthRecorder(Path(args.out)).run(args.duration, stop=lambda: stop["v"])


if __name__ == "__main__":
    main()
