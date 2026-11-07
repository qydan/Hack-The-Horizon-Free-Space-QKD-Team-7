"""QKD cyber-sandbox demo: telemetry source -> channel error -> BB84 -> key.

Examples (from the repo root):
    python src/demo_runner.py                                   # quick mock run
    python src/demo_runner.py --source csv                      # replay data/fake_telemetry.csv
    python src/demo_runner.py --source synthetic --log run.jsonl   # synthetic camera frames, 120 s
    python src/demo_runner.py --source camera --calibrate 10    # real USB camera (run on the OS that owns it)
    python src/demo_runner.py --source csv --eve 1.0            # intercept-resend attacker
    python src/demo_runner.py --source csv --decoy --pns        # decoy states vs PNS attacker
    python src/demo_runner.py --source csv --pool               # pool windows into larger blocks
"""
import argparse
import json
import os
import time
from typing import Dict, Optional

import numpy as np

from decoy import DecoyParams, run_decoy_analysis
from key_pool import KeyPool
from qkd_core import QuantumCryptoEngine
from telemetry import CSVReplayProvider, TelemetryExhausted, TelemetryProvider

QBER_GATE = 0.11
ROOT = os.path.join(os.path.dirname(__file__), "..")
DEFAULT_CSV = os.path.join(ROOT, "data", "fake_telemetry.csv")
SYNTHETIC_NOMINAL = (400.0, 260.0)


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


def process_window(engine: QuantumCryptoEngine, metrics: Dict[str, float], *,
                   eve: float = 0.0, decoy_params: Optional[DecoyParams] = None,
                   pns: bool = False, pool: Optional[KeyPool] = None) -> Dict:
    """Run one telemetry window through the full pipeline. Returns a result record."""
    p_error = engine.calculate_channel_error(**metrics)
    a_bits, a_bases, b_bits, b_bases = engine.generate_and_transmit(p_error, eve_fraction=eve)
    qber, a_ret, b_ret = engine.sift_and_estimate(a_bits, a_bases, b_bits, b_bases)
    rec = {"p_error": float(p_error), "qber": float(qber), "outcome": None, "key": None,
           "secret_bits": None, "leaked_bits": None, "errors_corrected": None,
           "pool_bits": None, "decoy": None, "note": ""}

    if qber >= QBER_GATE:
        rec["outcome"] = "abort"
        return rec

    decoy = None
    if decoy_params is not None:
        decoy = run_decoy_analysis(decoy_params, metrics["intensity"], p_error, pns_attack=pns)
        rec["decoy"] = {"consistent": decoy.consistent, "y1_lower": decoy.y1_lower,
                        "e1_upper": decoy.e1_upper, "single_photon_fraction": decoy.single_photon_fraction}
        if not decoy.usable:
            rec["outcome"] = "decoy_reject"
            rec["note"] = decoy.note or "no certified single-photon yield"
            return rec

    sample_size = int(round(len(a_ret) * 0.25))  # 20% of sifted bits were sampled
    if pool is not None:
        pool.add(a_ret, b_ret, qber, sample_size)
        key = pool.try_distill(decoy=decoy)
        rec["pool_bits"] = pool.size
        if key is None:
            rec["outcome"] = "pooling"
            return rec
    else:
        try:
            key = engine.reconcile_and_amplify(a_ret, b_ret, qber=qber, sample_size=sample_size, decoy=decoy)
        except ValueError as e:
            rec["outcome"] = "no_key"
            rec["note"] = str(e)
            return rec

    r = engine.last_report
    rec.update(outcome="key", key=key, secret_bits=r["secret_length"], leaked_bits=r["leaked_bits"],
               errors_corrected=r["errors_corrected"])
    return rec


def describe(label: str, metrics: Dict[str, float], rec: Dict, state: Optional[str], quiet: bool) -> None:
    head = (f"{label} | drift={metrics['drift']:.1f}px jitter={metrics['jitter']:.1f}px/s "
            f"intensity={metrics['intensity']:.2f}" + (f" | {state}" if state else ""))
    out = rec["outcome"]
    if quiet:
        tail = {"key": "KEY " + (rec["key"] or "")[:16] + "...", "abort": "ABORT",
                "pooling": f"pooling ({rec['pool_bits']} bits)", "no_key": "no key",
                "decoy_reject": "DECOY REJECT"}.get(out, out)
        print(f"{head} | P={rec['p_error']*100:.1f}% QBER={rec['qber']*100:.1f}% -> {tail}")
        return
    print(f"\n{head} | Channel Error Prob: {rec['p_error']*100:.2f}%")
    print(f"Sampled QBER: {rec['qber']*100:.2f}%")
    if out == "abort":
        print("🚨 SECURITY ABORT: QBER exceeds 11%. Key discarded.")
    elif out == "decoy_reject":
        print(f"🛡️  DECOY-STATE REJECT: {rec['note']}")
    elif out == "pooling":
        print(f"⏳ Not enough secret bits yet; pooled {rec['pool_bits']} sifted bits.")
    elif out == "no_key":
        print(f"⚠️ {rec['note']}")
    else:
        print(f"✅ QBER < 11%. {rec['errors_corrected']} errors corrected, {rec['leaked_bits']} bits leaked, "
              f"{rec['secret_bits']} secret bits available")
        print(f"🔑 AES-256 KEY: {rec['key']}")


def build_camera_provider(args):
    from camera_provider import (Calibration, LEDDetector, OpenCVCameraProvider, PATTracker,
                                 SyntheticLEDSource, VideoCaptureSource, demo_flight_scene)
    detector = LEDDetector()
    if args.source == "synthetic":
        ref = SyntheticLEDSource(lambda t: (*SYNTHETIC_NOMINAL, 1.0), duration=0.1, noise=3).read()[0]
        cal = Calibration(*SYNTHETIC_NOMINAL, i_ref=detector.detect(ref).intensity)
        source = SyntheticLEDSource(demo_flight_scene(SYNTHETIC_NOMINAL), fps=30, duration=args.duration)
        provider = OpenCVCameraProvider(source, PATTracker(detector, cal), args.window, args.record)
        return provider
    source = VideoCaptureSource(args.device, args.width, args.height, args.fps, args.exposure)
    if args.calibration and os.path.exists(args.calibration) and not args.calibrate:
        cal = Calibration.load(args.calibration)
    else:
        cal = Calibration(args.width / 2, args.height / 2, 100.0)   # placeholder until calibrated
    provider = OpenCVCameraProvider(source, PATTracker(detector, cal), args.window, args.record)
    if args.calibrate:
        print(f"Calibrating for {args.calibrate:.0f}s: hold the drone in calm hover...")
        cal = provider.calibrate(args.calibrate)
        if args.calibration:
            cal.save(args.calibration)
        print(f"Calibration: boresight=({cal.boresight_x:.1f},{cal.boresight_y:.1f}) i_ref={cal.i_ref:.1f}")
    return provider


def main():
    ap = argparse.ArgumentParser(description="QKD cyber-sandbox demo",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--source", choices=["mock", "csv", "synthetic", "camera"], default="mock")
    ap.add_argument("--csv", default=DEFAULT_CSV, help="telemetry CSV for --source csv")
    ap.add_argument("--window", type=float, default=1.0, help="seconds of telemetry per BB84 block")
    ap.add_argument("--cycles", type=int, default=5, help="blocks for --source mock")
    ap.add_argument("--duration", type=float, default=120.0, help="seconds to run for synthetic/camera")
    ap.add_argument("--sleep", type=float, default=0.0, help="pause between blocks (seconds)")
    ap.add_argument("--quiet", action="store_true", help="one line per window")
    ap.add_argument("--log", help="write a JSONL run log (for tools/make_dashboard.py)")
    # camera options
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=60)
    ap.add_argument("--exposure", type=float, default=-6, help="manual exposure value (backend specific)")
    ap.add_argument("--calibrate", type=float, default=0.0, help="seconds of calm hover to calibrate on")
    ap.add_argument("--calibration", default="calibration.json", help="calibration file to save/load")
    ap.add_argument("--record", help="record per-frame telemetry to this CSV (replayable)")
    # threat model / key options
    ap.add_argument("--eve", type=float, default=0.0, help="intercept-resend fraction 0..1")
    ap.add_argument("--decoy", action="store_true", help="enable decoy-state analysis")
    ap.add_argument("--pns", action="store_true", help="with --decoy: add a PNS attacker to the channel")
    ap.add_argument("--pool", action="store_true", help="pool windows into larger blocks before distilling")
    args = ap.parse_args()

    engine = QuantumCryptoEngine()
    pool = KeyPool(engine) if args.pool else None
    decoy_params = DecoyParams() if args.decoy else None
    print("--- QKD CYBER SANDBOX INITIALIZED ---")
    if args.eve:
        print(f"⚠️  Intercept-resend attacker active on {args.eve*100:.0f}% of pulses")
    if args.pns and not args.decoy:
        print("Note: --pns only has an effect together with --decoy")

    provider = None
    if args.source == "csv":
        provider = CSVReplayProvider(args.csv, window_s=args.window)
        print(f"Replaying {provider.duration:.0f}s of telemetry from {os.path.normpath(args.csv)}")
    elif args.source in ("synthetic", "camera"):
        provider = build_camera_provider(args)
    mock = MockHardware() if args.source == "mock" else None

    log = open(args.log, "w") if args.log else None
    counts: Dict[str, int] = {}
    cycle = 0
    try:
        while True:
            if mock:
                if cycle >= args.cycles:
                    break
                if cycle == 3:
                    print("\n💨 [ENVIRONMENT] Desk fan activated. High turbulence expected.")
                    mock.turbulence = True
                metrics = mock.get_metrics()
            else:
                try:
                    metrics = provider.get_metrics()
                except TelemetryExhausted:
                    break
                if args.source == "camera" and cycle * args.window >= args.duration:
                    break
            cycle += 1
            rec = process_window(engine, metrics, eve=args.eve, decoy_params=decoy_params,
                                 pns=args.pns, pool=pool)
            state = getattr(provider, "state", None)
            describe(f"Window {cycle}", metrics, rec, state, args.quiet)
            counts[rec["outcome"]] = counts.get(rec["outcome"], 0) + 1
            if log:
                log.write(json.dumps({"window": cycle, "t": cycle * args.window, **metrics,
                                      "tracking_state": state, "eve": args.eve, **rec}) + "\n")
            time.sleep(args.sleep)
    finally:
        if log:
            log.close()
        if provider is not None and hasattr(provider, "close"):
            provider.close()

    print("\n--- DONE: " + ", ".join(f"{v} {k}" for k, v in sorted(counts.items())) + " ---")
    tracker = getattr(provider, "tracker", None)
    if tracker is not None:
        acq = tracker.acquisition_time
        print(f"Tracking: first lock after {acq:.2f}s" if acq is not None else "Tracking: never locked",
              f"| lock losses: {tracker.lock_losses}")
    if args.log:
        print(f"Run log written to {args.log} (render with tools/make_dashboard.py)")


if __name__ == "__main__":
    main()
