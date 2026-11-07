"""Summarise recorded telemetry CSVs into numbers for tools/tune.py.

    python tools/summarize_telemetry.py [--skip SECONDS] calm.csv fan.csv [obstruct.csv]

Prints the mean drift / RMS jitter / mean intensity of each recording and a ready-to-run
tune.py command. Record each CSV with `demo_runner.py --source camera --record NAME.csv`.
"""
import csv
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from telemetry import CSVReplayProvider, aggregate_window  # noqa: E402


def summarize(path, skip=0.0):
    prov = CSVReplayProvider(path, window_s=1e9)
    rows = [r for r in prov._rows if r["timestamp"] >= skip]   # drop the acquisition phase
    return aggregate_window(rows)


def main():
    args = sys.argv[1:]
    skip = 0.0
    if "--skip" in args:
        i = args.index("--skip")
        skip = float(args[i + 1])
        del args[i:i + 2]
    paths = args
    if len(paths) < 2:
        sys.exit(__doc__)
    names = ["calm", "fan", "obstruct"]
    flags = []
    for name, path in zip(names, paths):
        m = summarize(path, skip)
        print(f"{name:<9} {path}: drift={m['drift']:.2f}px jitter={m['jitter']:.2f}px/s intensity={m['intensity']:.3f}")
        flags.append(f"--{name} {m['drift']:.2f},{m['jitter']:.2f},{m['intensity']:.3f}")
    print("\npython tools/tune.py " + " ".join(flags))


if __name__ == "__main__":
    main()
