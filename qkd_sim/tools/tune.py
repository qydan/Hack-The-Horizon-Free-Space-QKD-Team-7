"""Dose-response sweep (test T6) and error-budget explorer (ported from the team's tools/tune.py).

Sweeps the ground residual and the drone lean through the sandbox error budget (config/sandbox.yaml) and prints
the expected QBER and loss, so you can see where the abort gate sits, and compare the two error bars: ground
residual (what the gimbal / virtual receiver and tracker failed to remove) vs drone lean (what only a drone-side
steering mirror could fix).

    python tools/tune.py                         # config/sandbox.yaml
    python tools/tune.py --snr 10 --fov 10       # change the operating point
    python tools/tune.py --run                   # also push each point through the real pipeline
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sandbox.error_budget import BudgetParams, lin_to_db, pointing_loss_to_gate_db, qber_estimate, transmittance  # noqa: E402
from sandbox.main import _apply_overrides, load_cfg  # noqa: E402


def record(res_mrad: float, lean_deg: float, drift: float = 2.0, range_m: float | None = None, t: float = 0.0) -> dict:
    """A TRACK telemetry record (telemetry/schema.json) with the given motion."""
    return {"t": t, "state": "TRACK", "rig": "rig_fixed", "err_x_urad": 0.0, "err_y_urad": 0.0,
            "residual_mrad": res_mrad, "drift_mrad": drift, "jitter_mrad": 0.5, "gimbal_lag_mrad": 0.3,
            "lean_deg": lean_deg, "range_m": range_m, "intensity": 1.0, "exposure_gain": 1.0, "spot_fwhm_px": 5.0,
            "snr": 20.0, "saturated": False, "latency_ms": 150.0, "gimbal_pan_deg": 0.0, "gimbal_tilt_deg": 3.8}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--snr", type=float, help="on-axis signal/noise at cal_range_m (default: sandbox.yaml)")
    ap.add_argument("--fov", type=float, help="virtual receiver half-width, mrad (default: sandbox.yaml)")
    ap.add_argument("--range", type=float, default=None)
    ap.add_argument("--run", action="store_true", help="run each point through the sandbox (slower)")
    a = ap.parse_args()
    cfg = _apply_overrides(load_cfg(), snr=a.snr)
    if a.fov:
        cfg["theta_fov_virtual_mrad"] = a.fov
    p = BudgetParams.from_cfg(cfg)
    gate = p.abort_qber
    print(f"on-axis loss {lin_to_db(transmittance(p, record(0, 0, 0, a.range)).eta):.1f} dB, p_dark={p.p_dark:.2e}, "
          f"extra loss to reach {gate:.0%}: {pointing_loss_to_gate_db(p):.1f} dB\n")
    for title, key, values in (("Ground residual (drone lean 0):", "res", (0, 1, 2, 3, 4, 5, 6, 8, 10, 15)),
                               ("Drone lean (ground residual 0.7 mrad):", "lean", (0, 1, 2, 3, 4, 4.5, 5, 5.5, 6, 8))):
        print(title)
        print(f"{'residual mrad' if key == 'res' else 'lean deg':>14} {'loss dB':>9} {'QBER':>7}")
        for v in values:
            b = transmittance(p, record(v, 0, range_m=a.range) if key == "res" else record(0.7, v, range_m=a.range))
            q = qber_estimate(p, b)
            print(f"{v:>14} {lin_to_db(b.eta / b.eta_geo) + 0.0:>9.1f} {q:>7.1%}" + ("  <- gate" if q >= gate else ""))
        print()
    if a.run:
        from sandbox.pipeline import Sandbox
        print("Through the pipeline (4 s per point):")
        for res in (0.7, 3, 5, 8, 12):
            sb = Sandbox(cfg)
            recs = [r for r in sb.run(record(res, 1.0, range_m=a.range, t=i / 30) for i in range(121)) if r.kind == "block"]
            print(f"residual {res:>5} mrad -> " + ", ".join(f"{r.outcome} {r.qber:.1%}" for r in recs))


if __name__ == "__main__":
    main()
