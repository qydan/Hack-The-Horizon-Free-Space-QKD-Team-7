"""Monte-Carlo sweep to tune alpha/beta/gamma/e_optical against targets.

Usage (from repo root, venv active):
    python tools/tune.py                         # evaluate current constants
    python tools/tune.py --gamma 0.02 --beta 0.0008
    python tools/tune.py --calm 3,1,0.9 --fan 40,60,0.2   # use your measured telemetry
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from qkd_core import QuantumCryptoEngine  # noqa: E402

GATE = 0.11


def parse(s):
    d, j, i = (float(x) for x in s.split(","))
    return {"drift": d, "jitter": j, "intensity": i}


def evaluate(engine, metrics, trials):
    qbers, outcomes = [], {"abort": 0, "key": 0, "reconcile_fail": 0}
    p = engine.calculate_channel_error(**metrics)
    for _ in range(trials):
        q, ar, br = engine.sift_and_estimate(*engine.generate_and_transmit(p))
        qbers.append(q)
        if q >= GATE:
            outcomes["abort"] += 1
            continue
        try:
            engine.reconcile_and_amplify(ar, br)
            outcomes["key"] += 1
        except ValueError:
            outcomes["reconcile_fail"] += 1
    return p, np.mean(qbers), np.std(qbers), {k: v / trials for k, v in outcomes.items()}


def main():
    ap = argparse.ArgumentParser()
    for name in ("e_optical", "alpha", "beta", "gamma"):
        ap.add_argument(f"--{name}", type=float)
    ap.add_argument("--block", type=int, default=32768)
    ap.add_argument("--trials", type=int, default=300)
    ap.add_argument("--calm", type=parse, default="2.5,1,0.9")
    ap.add_argument("--fan", type=parse, default="35,50,0.25")
    ap.add_argument("--obstruct", type=parse, default="2.5,1,0.2")
    a = ap.parse_args()
    if isinstance(a.calm, str): a.calm = parse(a.calm)
    if isinstance(a.fan, str): a.fan = parse(a.fan)

    e = QuantumCryptoEngine(block_size=a.block)
    for name in ("e_optical", "alpha", "beta", "gamma"):
        if getattr(a, name) is not None:
            setattr(e, name, getattr(a, name))
    print(f"constants: e_optical={e.e_optical} alpha={e.alpha} beta={e.beta} gamma={e.gamma}  block={a.block}\n")
    print(f"{'scenario':<10}{'P_error':>9}{'QBER':>8}{'std':>7}   abort  key  reconcile_fail   target")
    targets = {"calm": "QBER ~3%, no aborts", "fan": "QBER >11%, all abort", "obstruct": "all abort"}
    for label, m in (("calm", a.calm), ("fan", a.fan), ("obstruct", a.obstruct)):
        p, q, s, o = evaluate(e, m, a.trials)
        print(f"{label:<10}{p*100:8.2f}%{q*100:7.2f}%{s*100:6.2f}%  {o['abort']:5.0%} {o['key']:4.0%} {o['reconcile_fail']:10.0%}      {targets[label]}")


if __name__ == "__main__":
    main()
