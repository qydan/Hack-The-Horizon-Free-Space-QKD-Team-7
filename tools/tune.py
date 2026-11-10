"""Dose-response sweep (test T6) and error-budget explorer.

Sweeps measured ground residual and drone lean through the error budget and prints the
expected QBER and loss, so you can see where the 11% gate sits, and compare the two error
bars the demo puts on screen: ground residual (what the gimbal and tracker failed to
remove) vs drone lean (what only a drone-side steering mirror could fix).

    python tools/tune.py                         # default budget
    python tools/tune.py --snr 40 --fov 10       # change the operating point
    python tools/tune.py --run                   # also push each point through the real pipeline
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from error_budget import (QBER_GATE, default_params, lin_to_db, pointing_loss_to_gate_db, qber_estimate,  # noqa: E402
                          transmittance)
from telemetry import FINE_TRACK, TelemetryMessage  # noqa: E402


def msg(res, lean, drift=2.0, rng=3.0, inten=1.0):
    return TelemetryMessage(0.0, FINE_TRACK, drift, 0.5, res, 0.3, lean, rng, inten)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snr", type=float, default=50.0, help="on-axis signal/noise at the demo range")
    ap.add_argument("--fov", type=float, default=10.0, help="virtual receiver half-width, mrad")
    ap.add_argument("--range", type=float, default=3.0)
    ap.add_argument("--run", action="store_true", help="run each point through the sandbox (slower)")
    a = ap.parse_args()

    p = default_params(a.snr)
    p.theta_fov_mrad = a.fov
    print(f"on-axis loss {lin_to_db(transmittance(p, msg(0, 0, 0, a.range)).eta):.1f} dB, "
          f"p_dark={p.p_dark:.2e}, extra loss to reach {QBER_GATE:.0%}: {pointing_loss_to_gate_db(p):.1f} dB\n")

    print("Ground residual (drone lean 0):")
    print(f"{'residual mrad':>14} {'loss dB':>9} {'QBER':>7}")
    for res in (0, 1, 2, 3, 4, 5, 6, 8, 10, 15):
        b = transmittance(p, msg(res, 0, rng=a.range))
        print(f"{res:>14} {lin_to_db(b.eta / b.eta_geo):>9.1f} {qber_estimate(p, b):>7.1%}"
              + ("  <- gate" if qber_estimate(p, b) >= QBER_GATE else ""))

    print("\nDrone lean (ground residual 0.7 mrad):")
    print(f"{'lean deg':>14} {'loss dB':>9} {'QBER':>7}")
    for lean in (0, 1, 2, 3, 4, 4.5, 5, 5.5, 6, 8):
        b = transmittance(p, msg(0.7, lean, rng=a.range))
        print(f"{lean:>14} {lin_to_db(b.eta / b.eta_geo):>9.1f} {qber_estimate(p, b):>7.1%}"
              + ("  <- gate" if qber_estimate(p, b) >= QBER_GATE else ""))

    if a.run:
        from sandbox import Sandbox
        print("\nThrough the pipeline (4 s per point):")
        for res in (0.7, 3, 5, 8, 12):
            sb = Sandbox(budget=p)
            recs = [r for r in sb.run(msg_stream(res, a.range)) if r.kind == "block"]
            print(f"residual {res:>5} mrad -> " + ", ".join(f"{r.outcome} {r.qber:.1%}" for r in recs))


def msg_stream(res, rng, seconds=4.0, hz=120):
    for i in range(int(seconds * hz) + 1):
        m = msg(res, 1.0, rng=rng)
        m.t = i / hz
        yield m


if __name__ == "__main__":
    main()
