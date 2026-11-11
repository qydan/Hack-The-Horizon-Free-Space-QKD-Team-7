"""Sandbox entry point: subscribes to 5556 (telemetry) + 5560 (Eve toggle), runs blocks through the pipeline,
publishes aborts on 5557 and key-block decisions on 5559.

    python -m sandbox.main [--log runs/<id>/sandbox_log.jsonl] [--keys-out keys.jsonl] [--duration 60]
    python -m sandbox.main --replay runs/<id>/telemetry.jsonl [--eve 0.5 | --pns] [--pool] [--log out.jsonl]

``--replay`` re-runs the key segment on a recorded run's telemetry with no bus at all (fast, repeatable; try
different attacks or operating points on the same flight). ``--log`` writes the interval / block records that
``tools/make_dashboard.py`` and ``tools/live_dashboard.py`` draw. tools/run_scenario.py starts this module
under tools/port_guard.py with ``--log <run>/sandbox_log.jsonl``.

Config: ``config/sandbox.yaml``.
"""
from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Iterable

import yaml

ROOT = Path(__file__).resolve().parent.parent


def load_cfg(path: Path | None = None) -> dict[str, Any]:
    with open(path or ROOT / "config" / "sandbox.yaml", "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _apply_overrides(cfg: dict[str, Any], *, snr: float | None = None, pool: bool | None = None,
                     eve_fraction: float | None = None, pns: bool = False) -> dict[str, Any]:
    cfg = json.loads(json.dumps(cfg))
    if snr is not None:
        cfg["on_axis_snr"], cfg["p_dark"] = float(snr), None
    if pool is not None:
        cfg.setdefault("pool", {})["enabled"] = bool(pool)
    if pns:
        cfg["eve"] = {**cfg.get("eve", {}), "mode": "pns"}
    elif eve_fraction:
        cfg["eve"] = {**cfg.get("eve", {}), "mode": "intercept_resend", "intercept_fraction": float(eve_fraction)}
    return cfg


class _Output:
    """Console lines and the optional JSONL run log."""

    def __init__(self, log_path: str | None, quiet: bool):
        self.fh = open(log_path, "w", encoding="utf-8") if log_path else None
        self.quiet = quiet

    def emit(self, rec) -> None:
        from sandbox.pipeline import BlockRecord, describe_block
        if self.fh:
            self.fh.write(json.dumps(asdict(rec)) + "\n")
            self.fh.flush()                       # so tools/live_dashboard.py can follow the run
        if isinstance(rec, BlockRecord) and (not self.quiet or rec.outcome in ("key", "abort", "decoy_abort")):
            print(describe_block(rec), flush=True)

    def close(self) -> None:
        if self.fh:
            self.fh.close()


def _summary(sb) -> None:
    print("--- DONE: " + ", ".join(f"{v} {k}" for k, v in sorted(sb.counts.items())) + " ---")
    print(f"{len(sb.keys)} keys in the key store", flush=True)


def replay(cfg: dict[str, Any], records: Iterable[dict[str, Any]], *, log_path: str | None = None,
           keys_out: str | None = None, quiet: bool = False):
    """Run the key segment over recorded telemetry records (no sockets)."""
    from sandbox.key_store import KeyStore
    from sandbox.pipeline import Sandbox
    sb = Sandbox(cfg, key_store=KeyStore(cfg, path=keys_out))
    out = _Output(log_path, quiet)
    print(f"--- QKD CYBER SANDBOX (replay) --- Eve: {sb.eve.describe()}", flush=True)
    try:
        for rec in sb.run(records):
            out.emit(rec)
    finally:
        out.close()
    _summary(sb)
    return sb


def load_records(path: Path) -> list[dict[str, Any]]:
    """telemetry.jsonl (one schema record per line) or telemetry.csv from a run directory."""
    path = Path(path)
    if path.is_dir():
        path = path / "telemetry.jsonl"
    if path.suffix == ".csv":
        import csv
        recs = []
        with open(path, newline="", encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                rec: dict[str, Any] = {}
                for k, v in row.items():
                    if v in ("", None):
                        rec[k] = None
                    elif k in ("state", "rig"):
                        rec[k] = v
                    elif k == "saturated":
                        rec[k] = v in ("True", "true", "1")
                    else:
                        rec[k] = float(v)
                recs.append(rec)
        return recs
    with open(path, "r", encoding="utf-8") as fh:
        return [json.loads(line) for line in fh if line.strip()]


def run(cfg: dict[str, Any], *, duration_s: float | None = None, log_path: str | None = None,
        keys_out: str | None = None, quiet: bool = False) -> None:
    """Run the sandbox on the bus until interrupted (or for ``duration_s`` wall seconds)."""
    import zmq

    from sandbox.gate import Gate
    from sandbox.key_store import KeyStore
    from sandbox.pipeline import Sandbox
    from telemetry.bus import Publisher, Subscriber, TelemetryValidationError, validate_telemetry

    stop = {"v": False}

    def _stop(*_):
        stop["v"] = True
    for sig in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGBREAK", None)):
        if sig is not None:
            try:
                signal.signal(sig, _stop)
            except (ValueError, OSError):        # not in the main thread (tests)
                pass

    abort_pub = Publisher("abort")
    keys_pub = Publisher("keys", validate=False)
    tel = Subscriber("telemetry")
    eve_sub = Subscriber("eve")
    sb = Sandbox(cfg, gate=Gate(cfg, abort_pub), key_store=KeyStore(cfg, keys_pub, keys_out))
    out = _Output(log_path, quiet)
    poller = zmq.Poller()
    poller.register(tel.sock, zmq.POLLIN)
    poller.register(eve_sub.sock, zmq.POLLIN)
    print(f"--- QKD CYBER SANDBOX --- waiting for telemetry on the bus; Eve: {sb.eve.describe()}", flush=True)
    t_end = None if duration_s is None else time.monotonic() + duration_s
    bad = 0
    try:
        while not stop["v"] and (t_end is None or time.monotonic() < t_end):
            ready = dict(poller.poll(100))

            def check_eve() -> None:
                while eve_sub.sock.poll(0):
                    _, msg = eve_sub.recv()
                    try:
                        before = sb.eve.describe()
                        sb.eve.handle(msg)
                        if sb.eve.describe() != before:      # the operator UI repeats messages for late joiners
                            print(f"Eve: {sb.eve.describe()}", flush=True)
                    except ValueError as e:
                        print(f"ignored Eve toggle {msg}: {e}", flush=True)
            check_eve()
            if tel.sock in ready:
                while tel.sock.poll(0):
                    check_eve()                              # a toggle takes effect between telemetry records
                    _, rec = tel.recv()
                    try:
                        validate_telemetry(rec)
                    except TelemetryValidationError as e:
                        bad += 1
                        if bad <= 5:
                            print(f"rejected telemetry record: {e}", flush=True)
                        continue
                    for r in sb.feed(rec):
                        out.emit(r)
    except KeyboardInterrupt:
        pass
    finally:
        out.close()
        for s in (abort_pub, keys_pub, tel, eve_sub):
            s.close()
    _summary(sb)


def main(argv: list[str] | None = None) -> None:
    sys.stdout.reconfigure(line_buffering=True, errors="replace")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path, default=None, help="sandbox config (default config/sandbox.yaml)")
    ap.add_argument("--replay", type=Path, help="telemetry.jsonl / .csv or a run directory: offline, no bus")
    ap.add_argument("--duration", type=float, default=None, help="stop after this many wall seconds (bus mode)")
    ap.add_argument("--log", help="write the interval/block JSONL run log (for tools/make_dashboard.py)")
    ap.add_argument("--keys-out", help="append finished keys to this file")
    ap.add_argument("--quiet", action="store_true", help="only print keys and aborts")
    ap.add_argument("--pool", action=argparse.BooleanOptionalAction, default=None, help="pool accepted blocks")
    ap.add_argument("--snr", type=float, help="on-axis signal/noise (sets p_dark; 50 -> ~3%% QBER locked)")
    ap.add_argument("--eve", type=float, help="intercept-resend on this fraction of pulses from the start")
    ap.add_argument("--pns", action="store_true", help="photon-number-splitting attacker from the start")
    a = ap.parse_args(argv)
    cfg = _apply_overrides(load_cfg(a.config), snr=a.snr, pool=a.pool, eve_fraction=a.eve, pns=a.pns)
    if a.replay:
        replay(cfg, load_records(a.replay), log_path=a.log, keys_out=a.keys_out, quiet=a.quiet)
    else:
        run(cfg, duration_s=a.duration, log_path=a.log, keys_out=a.keys_out, quiet=a.quiet)


if __name__ == "__main__":
    main()
