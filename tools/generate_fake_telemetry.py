"""Generate synthetic telemetry CSV (same schema the tracker publishes) without any rig.

    python tools/generate_fake_telemetry.py                 # writes data/fake_telemetry.csv
    python tools/generate_fake_telemetry.py --seed 7 --out my.csv

Scenario (120 s @ 120 Hz), mirroring the simulated rig's flight:

    0-2 s     searching for the drone
    2-30 s    fan off:  small drift, low residual           (keys)
    30-60 s   fan low                                        (keys)
    60-90 s   fan high: lean near the beam edge, residual up (keys get scarce)
    90-97 s   gust:  tracked but wild                        (gate aborts)
    97-100 s  beacon lost, tracker re-acquires               (no link)
    100-120 s recovery                                       (keys again)
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from telemetry import SEARCH, FINE_TRACK, COAST, TelemetryMessage, write_csv  # noqa: E402

# (t_start, t_end, drift_mrad, jitter_mrad, residual_mrad, lean_deg, intensity)
PHASES = [
    (0, 30, 2.0, 0.4, 0.7, 0.8, 0.99),
    (30, 60, 12.0, 1.5, 2.2, 2.5, 0.98),
    (60, 90, 24.0, 3.0, 3.5, 3.6, 0.95),
]


def ar1(rng, n, phi=0.97):
    x = np.zeros(n)
    e = rng.standard_normal(n) * np.sqrt(1 - phi ** 2)
    for i in range(1, n):
        x[i] = phi * x[i - 1] + e[i]
    return x


def generate(duration=120.0, hz=120, seed=1234):
    rng = np.random.default_rng(seed)
    t = np.arange(0, duration, 1.0 / hz)
    n = len(t)
    slow, mid = ar1(rng, n), ar1(rng, n, 0.9)
    msgs = []
    for i, ti in enumerate(t):
        if ti < 2.0:
            msgs.append(TelemetryMessage(round(ti, 5), SEARCH, 0, 0, 0, 0, 0, 3.0, 0.0))
            continue
        ph = next((p for p in PHASES if p[0] <= ti < p[1]), None)
        if ph is None and 90 <= ti < 97:          # gust: still tracked, but wild (gate aborts)
            msgs.append(TelemetryMessage(round(ti, 5), FINE_TRACK, 60.0, 12.0, 15.0, 8.0, 6.0, 3.0, 0.5))
            continue
        if ph is None and 97 <= ti < 100:         # beacon lost, tracker re-acquiring
            state = COAST if ti < 97.3 else SEARCH
            msgs.append(TelemetryMessage(round(ti, 5), state, 60.0, 12.0, 0.0, 0.0, 6.0, 3.0, 0.0))
            continue
        if ph is None:
            ph = (100, 120, 2.0, 0.4, 0.7, 0.8, 0.99)
        _, _, dr, jit, res, lean, inten = ph
        msgs.append(TelemetryMessage(
            t=round(ti, 5), state=FINE_TRACK,
            drift_mrad=max(0.0, dr * (1 + 0.15 * slow[i])),
            jitter_mrad=max(0.0, jit * (1 + 0.2 * mid[i])),
            residual_mrad=abs(res * (1 + 0.4 * mid[i])),
            gimbal_lag_mrad=abs(0.3 * res * (1 + 0.3 * mid[i])),
            lean_deg=max(0.0, lean * (1 + 0.1 * slow[i])), range_m=3.0 + 0.03 * slow[i],
            intensity=float(np.clip(inten * (1 + 0.01 * mid[i]), 0.0, 1.0))))
    return msgs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "data", "fake_telemetry.csv"))
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--hz", type=int, default=120)
    ap.add_argument("--seed", type=int, default=1234)
    a = ap.parse_args()
    msgs = generate(a.duration, a.hz, a.seed)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    write_csv(a.out, msgs)
    print(f"wrote {len(msgs)} rows ({a.duration:.0f}s @ {a.hz} Hz) to {os.path.normpath(a.out)}")


if __name__ == "__main__":
    main()
