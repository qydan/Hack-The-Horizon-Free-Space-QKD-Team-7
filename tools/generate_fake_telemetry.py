"""Generate synthetic telemetry CSV for CSVReplayProvider (stand-in for the real camera).

Usage (from repo root):
    python tools/generate_fake_telemetry.py                 # writes data/fake_telemetry.csv
    python tools/generate_fake_telemetry.py --seed 7 --out /tmp/t.csv

Scenario (120 s @ 60 Hz), tuned to the current transfer-function constants:

    0-8 s     acquisition: beacon found, drift collapses, intensity ramps up (expect aborts)
    8-45 s    calm hover                     (P_error ~3%)
    50-78 s   moderate fan, lock held        (P_error ~5.5%, keys still issued)
    82-90 s   strong gust, lock degraded     (P_error >11%, aborts)
    94-120 s  recovery to calm hover, with a brief lens obstruction at ~105 s
"""
import argparse
import csv
import os

import numpy as np

# (time_s, drift_px, jitter_px_per_s, intensity)
KEYFRAMES = [
    (0.0, 120.0, 25.0, 0.15),
    (4.0, 25.0, 8.0, 0.50),
    (8.0, 3.0, 1.0, 0.90),
    (45.0, 3.0, 1.0, 0.90),
    (50.0, 7.0, 12.0, 0.80),
    (78.0, 7.0, 12.0, 0.80),
    (82.0, 35.0, 50.0, 0.25),
    (90.0, 35.0, 50.0, 0.25),
    (94.0, 7.0, 12.0, 0.80),
    (100.0, 3.0, 1.0, 0.90),
    (104.5, 3.0, 1.0, 0.90),
    (104.6, 3.0, 1.0, 0.20),   # hand over the lens
    (106.0, 3.0, 1.0, 0.20),
    (106.2, 3.0, 1.0, 0.90),
    (120.0, 3.0, 1.0, 0.90),
]


def ar1(rng, n, phi=0.9):
    """Unit-variance AR(1) noise (slowly varying, like real gimbal/wind wander)."""
    x = np.zeros(n)
    e = rng.standard_normal(n) * np.sqrt(1 - phi ** 2)
    for i in range(1, n):
        x[i] = phi * x[i - 1] + e[i]
    return x


def generate(duration=120.0, hz=60, seed=1234, rel_noise=0.10):
    rng = np.random.default_rng(seed)
    t = np.arange(0, duration, 1.0 / hz)
    kt, kd, kj, ki = zip(*KEYFRAMES)
    drift = np.interp(t, kt, kd)
    jitter = np.interp(t, kt, kj)
    inten = np.interp(t, kt, ki)

    drift = np.clip(drift * (1 + rel_noise * ar1(rng, len(t))) + 0.3 * np.abs(rng.standard_normal(len(t))), 0, None)
    jitter = np.clip(jitter * (1 + 2 * rel_noise * ar1(rng, len(t))) + 0.1 * np.abs(rng.standard_normal(len(t))), 0, None)
    inten = np.clip(inten * (1 + 0.5 * rel_noise * ar1(rng, len(t))), 0.01, 1.0)
    return t, drift, jitter, inten


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "data", "fake_telemetry.csv"))
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--hz", type=int, default=60)
    ap.add_argument("--seed", type=int, default=1234)
    a = ap.parse_args()

    t, drift, jitter, inten = generate(a.duration, a.hz, a.seed)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "drift", "jitter", "intensity"])
        for row in zip(t, drift, jitter, inten):
            w.writerow([f"{row[0]:.4f}", f"{row[1]:.3f}", f"{row[2]:.3f}", f"{row[3]:.4f}"])
    print(f"wrote {len(t)} rows ({a.duration:.0f}s @ {a.hz} Hz) to {os.path.normpath(a.out)}")


if __name__ == "__main__":
    main()
