"""Operator UI for the eavesdropper (I6, port 5560): switch Eve on or off in a running sandbox.

    python -m tools.eve_toggle intercept 0.5     # intercept-resend on half the pulses -> QBER ~14.5%, gate aborts
    python -m tools.eve_toggle pns               # photon-number splitting -> QBER stays low, decoy check aborts
    python -m tools.eve_toggle off

Run it where the sandbox runs (inside the simulator container: `qkd shell`). Binds port 5560 (the sandbox and
the recorder connect to it), so it can't run while tools/run_scenario.py plays a scenario's Eve schedule.
Scenarios can schedule the same messages: ``eve: {schedule: [{t_s: 60, mode: intercept_resend, fraction: 0.5}]}``.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

MODES = {"intercept": "intercept_resend", "intercept_resend": "intercept_resend", "pns": "pns", "off": "off"}


def message(mode: str, fraction: float | None = None) -> dict:
    m = {"mode": MODES[mode]}
    if m["mode"] == "intercept_resend" and fraction is not None:
        m["fraction"] = float(fraction)
    return m


def main() -> None:
    from telemetry.bus import Publisher
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=sorted(MODES))
    ap.add_argument("fraction", nargs="?", type=float, default=None, help="intercept-resend fraction 0..1")
    ap.add_argument("--repeat-s", type=float, default=2.0, help="keep re-sending this long (PUB/SUB late joiners)")
    a = ap.parse_args()
    pub = Publisher("eve", validate=False)
    msg = message(a.mode, a.fraction)
    t_end = time.monotonic() + a.repeat_s
    while time.monotonic() < t_end:
        pub.send(msg)
        time.sleep(0.25)
    pub.close()
    print(f"sent {msg}")


if __name__ == "__main__":
    main()
