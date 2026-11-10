"""Drone QKD demo: ground tracker -> telemetry -> cyber sandbox -> keys.

The tracker and the sandbox are two separate programs joined only by the telemetry stream.
This runner can run them in one process for convenience, or as the two real programs:

    python src/demo_runner.py                         # simulated rig, in one process (120 s)
    python src/demo_runner.py --source csv            # replay data/fake_telemetry.csv
    python src/demo_runner.py --source csv --eve 0.5  # intercept-resend on half the pulses -> aborts
    python src/demo_runner.py --source csv --pns      # PNS attacker -> caught by the decoy check
    python src/demo_runner.py --source sim --log run.jsonl && python tools/make_dashboard.py run.jsonl dash.html

    # two programs over ZeroMQ (start the sandbox first, then the tracker):
    python src/demo_runner.py --source zmq
    python src/tracker.py --source sim --realtime      # or --source camera for the real rig
"""
import argparse
import json
import os
import sys
from argparse import Namespace
from typing import List, Optional

from channel import EveConfig
from error_budget import default_params
from key_pool import KeyPool
from qkd_core import QuantumCryptoEngine
from sandbox import BlockRecord, KeyStore, Sandbox
from telemetry import (CSVReplaySource, ControlPublisher, TelemetryMessage, ZmqSubscriber, write_csv)

ROOT = os.path.join(os.path.dirname(__file__), "..")
DEFAULT_CSV = os.path.join(ROOT, "data", "fake_telemetry.csv")


def describe_block(rec: BlockRecord) -> str:
    q = f"QBER={rec.qber * 100:.1f}% (budget {rec.qber_expected * 100:.1f}%)"
    if rec.outcome == "key":
        tail = f"KEY {rec.key[:16]}... ({rec.secret_bits} secret bits, {rec.leaked_bits} leaked)"
    elif rec.outcome == "abort":
        tail = f"ABORT: {rec.note}; tracker re-acquires"
    elif rec.outcome == "decoy_abort":
        tail = f"DECOY ABORT: {rec.note}; tracker re-acquires"
    elif rec.outcome == "pooling":
        tail = f"pooling ({rec.pool_bits} sifted bits)"
    elif rec.outcome == "no_link":
        tail = f"no link ({rec.note})"
    else:
        tail = f"no key ({rec.note})"
    return f"t={rec.t:6.1f}s | {q} | {tail}"


def main(argv: Optional[List[str]] = None) -> int:
    # show output as it happens (even when piped) and never crash on a console that can't encode a character
    sys.stdout.reconfigure(line_buffering=True, errors="replace")
    ap = argparse.ArgumentParser(description="Drone QKD demo", formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("--source", choices=["sim", "csv", "zmq"], default="sim")
    ap.add_argument("--csv", default=DEFAULT_CSV, help="telemetry CSV for --source csv")
    ap.add_argument("--duration", type=float, default=120.0, help="seconds of simulated flight (--source sim)")
    ap.add_argument("--block", type=float, default=1.0, help="seconds of telemetry per key block")
    ap.add_argument("--interval", type=float, default=0.1, help="seconds per live-trace step")
    ap.add_argument("--pulse-rate", type=float, default=1e9, help="emulated source pulses per second")
    ap.add_argument("--snr", type=float, default=50.0, help="on-axis signal / noise at the demo range")
    ap.add_argument("--eve", type=float, default=0.0, help="intercept-resend fraction of pulses, 0..1")
    ap.add_argument("--pns", action="store_true", help="photon-number-splitting attacker")
    ap.add_argument("--pool", action="store_true", help="pool accepted blocks before distilling")
    ap.add_argument("--quiet", action="store_true", help="only print aborts and keys")
    ap.add_argument("--log", help="write a JSONL run log (for tools/make_dashboard.py)")
    ap.add_argument("--keys-out", help="append finished keys to this file (the operator's key store)")
    ap.add_argument("--record", help="record the telemetry stream to a replayable CSV")
    ap.add_argument("--timeout", type=float, default=5.0, help="--source zmq: stop after this much silence")
    args = ap.parse_args(argv)

    engine = QuantumCryptoEngine()
    control = ControlPublisher() if args.source == "zmq" else None
    tracker_ref = {}

    def on_abort(rec: BlockRecord) -> None:       # I8: gate -> tracker
        if control is not None:
            control.send_reacquire(rec.outcome)
        elif tracker_ref.get("t") is not None:
            tracker_ref["t"].reacquire()

    sb = Sandbox(engine=engine, budget=default_params(args.snr), eve=EveConfig(args.eve, args.pns),
                 pulse_rate_hz=args.pulse_rate, interval_s=args.interval, block_s=args.block,
                 pool=KeyPool(engine) if args.pool else None, key_store=KeyStore(args.keys_out),
                 on_abort=on_abort)

    tracker = None
    if args.source == "csv":
        source = CSVReplaySource(args.csv)
        print(f"Replaying {source.duration:.0f}s of telemetry from {os.path.normpath(args.csv)}")
    elif args.source == "zmq":
        source = ZmqSubscriber(timeout_s=args.timeout)
        print(f"Waiting for telemetry from tracker.py (stops after {args.timeout:.0f}s of silence)...")
    else:
        from tracker import build_sim
        print(f"Calibrating the simulated rig, then flying a {args.duration:.0f} s scripted flight "
              "(fan off 0-30 s, low 30-60, high 60-90, gust 90-100, then recovery)...")
        tracker, rig = build_sim(Namespace(duration=args.duration, fps=120.0))
        tracker_ref["t"] = tracker
        source = tracker.run(rig)

    print("--- QKD CYBER SANDBOX ---")
    if args.eve:
        print(f"Eve: intercept-resend on {args.eve * 100:.0f}% of pulses")
    if args.pns:
        print("Eve: photon-number-splitting attack")

    log = open(args.log, "w") if args.log else None
    recorded: List[TelemetryMessage] = []

    def tap(it):
        for m in it:
            if args.record:
                recorded.append(m)
            yield m

    try:
        for rec in sb.run(tap(source)):
            if log:
                log.write(json.dumps(rec.__dict__) + "\n")
                log.flush()                              # so tools/live_dashboard.py can follow the run
            if isinstance(rec, BlockRecord) and (not args.quiet or rec.outcome in ("key", "abort", "decoy_abort")):
                print(describe_block(rec))
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if log:
            log.close()
        if hasattr(source, "close"):
            source.close()
        if control is not None:
            control.close()
        if args.record and recorded:
            write_csv(args.record, recorded)
            print(f"Telemetry recorded to {args.record}")

    print("\n--- DONE: " + ", ".join(f"{v} {k}" for k, v in sorted(sb.counts.items())) + " ---")
    print(f"{len(sb.keys)} keys in the key store" + (f" (written to {args.keys_out})" if args.keys_out else ""))
    if tracker is not None:
        acq = tracker.acquisition_time
        print("Tracking: first lock after " + (f"{acq:.2f}s" if acq is not None else "never"),
              f"| lock losses: {tracker.lock_losses}")
    if args.log:
        print(f"Run log written to {args.log} (render with tools/make_dashboard.py)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
