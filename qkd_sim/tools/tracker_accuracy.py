"""Score a recorded run: tracker LED position vs ground truth, frame by frame (evaluation side: uses truth).

    python tools/tracker_accuracy.py                 # latest run
    python tools/tracker_accuracy.py runs/<dir>      # a specific run

Uses telemetry.jsonl (spot_u_px / spot_v_px, state) and truth.jsonl, projecting the truth LED through the
exact simulated camera (the same maths as the live overlay, tools/accuracy_overlay.py).
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sim_bridge import config as cfgmod  # noqa: E402
from sim_bridge.camera_model import make_lens  # noqa: E402
from tools.accuracy_overlay import angle_between_px, truth_pixel  # noqa: E402


def load(path: Path) -> list[dict]:
    return [json.loads(x) for x in path.read_text().splitlines() if x.strip()] if path.exists() else []


def score(run: Path) -> dict:
    scn = yaml.safe_load((run / "scenario.yaml").read_text())
    rig = json.loads((run / "summary.json").read_text())["rig"]
    cfgs = cfgmod.load_all(scn.get("overrides"))
    lens = make_lens(cfgs["camera_action5"], cfgs["rigs"][rig]["principal_point_offset_px"])
    tel = load(run / "telemetry.jsonl")
    truth = load(run / "truth.jsonl")
    if not tel or not truth:
        return {"error": "run has no telemetry.jsonl or truth.jsonl (was tracking running?)"}
    tt = [r["t_sim"] for r in truth]
    states = [r["state"] for r in tel]
    errs_px, errs_mrad = [], []
    per_frame = []                     # (t, state, err_mrad or None, truth pixel or None)
    first_lock = next((r["t"] for r in tel if r["state"] == "TRACK"), None)
    for r in tel:
        i = bisect.bisect_left(tt, r["t"])
        cand = [j for j in (i - 1, i) if 0 <= j < len(tt)]
        j = min(cand, key=lambda k: abs(tt[k] - r["t"]))
        tp = truth_pixel(lens, truth[j]) if abs(tt[j] - r["t"]) <= 0.02 else None
        if r.get("spot_u_px") is None or r.get("state") != "TRACK" or tp is None:
            per_frame.append((r["t"], r["state"], None, tp))
            continue
        sp = (float(r["spot_u_px"]), float(r["spot_v_px"]))
        errs_px.append(math.hypot(sp[0] - tp[0], sp[1] - tp[1]))
        errs_mrad.append(1e3 * angle_between_px(lens, sp, tp))
        per_frame.append((r["t"], r["state"], errs_mrad[-1], tp))
    e = np.asarray(errs_mrad)
    p = np.asarray(errs_px)
    out = {
        "frames": len(tel),
        "track_fraction": states.count("TRACK") / len(tel),
        "states": {s: states.count(s) for s in ("SEARCH", "ACQUIRE", "TRACK", "COAST")},
        "first_lock_s": None if first_lock is None else first_lock - tel[0]["t"],
        "scored_track_frames": int(e.size),
    }
    if e.size:
        out.update({
            "error_mrad": {"median": float(np.median(e)), "rms": float(np.sqrt(np.mean(e ** 2))),
                           "p95": float(np.percentile(e, 95)), "max": float(e.max())},
            "error_px": {"median": float(np.median(p)), "rms": float(np.sqrt(np.mean(p ** 2))),
                         "p95": float(np.percentile(p, 95)), "max": float(p.max())},
            "frames_over_1mrad": int((e > 1.0).sum()),
        })
    segs = json.loads((run / "summary.json").read_text()).get("path_segments") or []
    if segs:
        out["segments"] = [_segment_stats(seg, per_frame, lens) for seg in segs]
    return out


def _segment_stats(seg: dict, per_frame: list, lens) -> dict:
    rows = [f for f in per_frame if seg["t_start"] <= f[0] < seg["t_end"]]
    errs = np.array([f[2] for f in rows if f[2] is not None])
    # how fast the LED moved across the image (truth), in mrad/s
    pts = [(f[0], f[3]) for f in rows if f[3] is not None]
    rates = []
    for (t0, a), (t1, b) in zip(pts, pts[1:]):
        if t1 > t0:
            rates.append(1e3 * angle_between_px(lens, a, b) / (t1 - t0))
    return {
        "name": seg["name"], "t": [round(seg["t_start"], 1), round(seg["t_end"], 1)], "frames": len(rows),
        "track_pct": round(100.0 * sum(f[1] == "TRACK" for f in rows) / max(1, len(rows)), 1),
        "led_speed_mrad_s": {"mean": round(float(np.mean(rates)), 1) if rates else None,
                             "max": round(float(np.max(rates)), 1) if rates else None},
        "err_mrad": None if errs.size == 0 else {"median": round(float(np.median(errs)), 3),
                                                 "p95": round(float(np.percentile(errs, 95)), 3),
                                                 "max": round(float(errs.max()), 3)},
    }


def print_table(res: dict) -> None:
    print(f"{'segment':18s} {'TRACK %':>8s} {'LED speed mrad/s':>17s} {'err median':>11s} {'err p95':>8s} {'err max':>8s}  (mrad)")
    for s in res["segments"]:
        e = s["err_mrad"] or {}
        sp = s["led_speed_mrad_s"]
        print(f"{s['name']:18s} {s['track_pct']:8.1f} {str(sp['mean']) + ' / ' + str(sp['max']):>17s} "
              f"{e.get('median', float('nan')):11.3f} {e.get('p95', float('nan')):8.3f} {e.get('max', float('nan')):8.3f}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", nargs="?")
    a = ap.parse_args()
    run = Path(a.run_dir) if a.run_dir else max((ROOT / "runs").glob("*/summary.json"), key=lambda q: q.stat().st_mtime).parent
    print(run.name)
    res = score(run)
    segs = res.pop("segments", None)
    print(json.dumps(res, indent=2))
    if segs:
        print()
        print_table({"segments": segs})


if __name__ == "__main__":
    main()
