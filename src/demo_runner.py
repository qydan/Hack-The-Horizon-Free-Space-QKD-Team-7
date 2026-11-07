import argparse
import os
import time

import numpy as np

from telemetry import CSVReplayProvider, TelemetryExhausted, TelemetryProvider
from qkd_core import QuantumCryptoEngine

QBER_GATE = 0.11
DEFAULT_CSV = os.path.join(os.path.dirname(__file__), "..", "data", "fake_telemetry.csv")


# Simple synthetic source for a quick smoke test without any data file
class MockHardware(TelemetryProvider):
    def __init__(self):
        self.turbulence = False

    def get_metrics(self):
        if self.turbulence:
            return {"drift": np.random.uniform(20, 50),
                    "jitter": np.random.uniform(30, 70),
                    "intensity": np.random.uniform(0.1, 0.4)}
        else:
            return {"drift": np.random.uniform(0, 5),
                    "jitter": np.random.uniform(0, 2),
                    "intensity": np.random.uniform(0.8, 1.0)}


def run_cycle(engine, metrics, cycle_label):
    """One BB84 block driven by one telemetry window. Returns the key or None."""
    p_error = engine.calculate_channel_error(**metrics)
    print(f"\n{cycle_label} | drift={metrics['drift']:.1f}px jitter={metrics['jitter']:.1f}px/s "
          f"intensity={metrics['intensity']:.2f} | Channel Error Prob: {p_error*100:.2f}%")

    a_bits, a_bases, b_bits, b_bases = engine.generate_and_transmit(p_error)
    qber, a_retained, b_retained = engine.sift_and_estimate(a_bits, a_bases, b_bits, b_bases)
    print(f"Sampled QBER: {qber*100:.2f}%")

    if qber >= QBER_GATE:
        print("🚨 SECURITY ABORT: QBER exceeds 11%. Key discarded.")
        return None

    sample_size = int(round(len(a_retained) * 0.25))  # 20% of sifted bits were sampled
    try:
        key = engine.reconcile_and_amplify(a_retained, b_retained, qber=qber, sample_size=sample_size)
    except ValueError as e:
        print(f"⚠️ {e}")
        return None

    r = engine.last_report
    print(f"✅ QBER < 11%. Reconciled {r['reconciled_bits']} bits "
          f"({r['errors_corrected']} errors fixed, {r['leaked_bits']} bits leaked, "
          f"{r['secret_length']} secret bits available)")
    print(f"🔑 AES-256 KEY: {key}")
    return key


def main():
    ap = argparse.ArgumentParser(description="QKD cyber-sandbox demo")
    ap.add_argument("--source", choices=["mock", "csv"], default="mock",
                    help="telemetry source (a camera provider will be added later)")
    ap.add_argument("--csv", default=DEFAULT_CSV, help="telemetry CSV for --source csv")
    ap.add_argument("--window", type=float, default=1.0, help="seconds of telemetry per BB84 block")
    ap.add_argument("--cycles", type=int, default=5, help="blocks to run for --source mock")
    ap.add_argument("--sleep", type=float, default=0.0, help="pause between blocks (seconds)")
    args = ap.parse_args()

    engine = QuantumCryptoEngine()
    print("--- QKD CYBER SANDBOX INITIALIZED ---")

    keys = aborts = 0
    if args.source == "csv":
        provider = CSVReplayProvider(args.csv, window_s=args.window)
        print(f"Replaying {provider.duration:.0f}s of telemetry from {os.path.normpath(args.csv)}")
        cycle = 0
        while True:
            try:
                metrics = provider.get_metrics()
            except TelemetryExhausted:
                break
            cycle += 1
            if run_cycle(engine, metrics, f"Window {cycle}"):
                keys += 1
            else:
                aborts += 1
            time.sleep(args.sleep)
    else:
        hardware = MockHardware()
        for cycle in range(args.cycles):
            # Simulate turning on the desk fan on cycle 3
            if cycle == 3:
                print("\n💨 [ENVIRONMENT] Desk fan activated. High turbulence expected.")
                hardware.turbulence = True
            if run_cycle(engine, hardware.get_metrics(), f"Cycle {cycle+1}"):
                keys += 1
            else:
                aborts += 1
            time.sleep(args.sleep)

    print(f"\n--- DONE: {keys} keys issued, {aborts} blocks without a key ---")


if __name__ == "__main__":
    main()
