"""Compare telemetry (and calibration outputs) with ground truth once the OPEN blocks exist.

    python -m evaluation.score runs/<id>

Telemetry/truth alignment: each telemetry record (time ``t`` = frame capture time) is paired with the truth
record nearest in sim time. Sign convention expected from tracking: err_x positive = target right of the
optical axis in the image, err_y positive = below (same as truth boresight_err_x/y).

Calibration: config/calibration.yaml values are compared with evaluation/camera_truth.yaml (written by the
world builder) and with the configured emulation parameters. Null (not yet calibrated) entries are skipped.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    with open(path, "r", encoding="utf-8") as fh:
        return [json.loads(line) for line in fh if line.strip()]


def align(telemetry: list[dict], truth: list[dict], max_dt: float = 0.02) -> list[tuple[dict, dict]]:
    ts = [r["t_sim"] for r in truth]
    pairs = []
    for rec in telemetry:
        i = bisect.bisect_left(ts, rec["t"])
        cands = [j for j in (i - 1, i) if 0 <= j < len(ts)]
        if not cands:
            continue
        j = min(cands, key=lambda k: abs(ts[k] - rec["t"]))
        if abs(ts[j] - rec["t"]) <= max_dt:
            pairs.append((rec, truth[j]))
    return pairs


def rms(x) -> float:
    x = np.asarray(list(x), float)
    return float(np.sqrt(np.mean(x * x))) if x.size else float("nan")


def score_tracking(pairs: list[tuple[dict, dict]]) -> dict[str, Any]:
    if not pairs:
        return {"n": 0, "note": "no aligned telemetry (OPEN tracking block not implemented or not running)"}
    tracked = [(a, b) for a, b in pairs if a["state"] in ("TRACK", "COAST")]
    out: dict[str, Any] = {
        "n": len(pairs),
        "fraction_track": sum(a["state"] == "TRACK" for a, _ in pairs) / len(pairs),
        "state_counts": {s: sum(a["state"] == s for a, _ in pairs) for s in ("SEARCH", "ACQUIRE", "TRACK", "COAST")},
    }
    if tracked:
        ex = [a["err_x_urad"] * 1e-3 - b["boresight_err_x_mrad"] for a, b in tracked]
        ey = [a["err_y_urad"] * 1e-3 - b["boresight_err_y_mrad"] for a, b in tracked]
        out["err_x_rms_mrad"] = rms(ex)
        out["err_y_rms_mrad"] = rms(ey)
        out["err_x_bias_mrad"] = float(np.mean(ex))
        out["err_y_bias_mrad"] = float(np.mean(ey))
        lean = [(a["lean_deg"] - b["lean_deg"]) for a, b in tracked if a.get("lean_deg") is not None]
        if lean:
            out["lean_rms_deg"] = rms(lean)
        rng = [(a["range_m"] - b["range_m"]) for a, b in tracked if a.get("range_m") is not None]
        if rng:
            out["range_rms_m"] = rms(rng)
        out["latency_ms_mean"] = float(np.mean([a["latency_ms"] for a, _ in tracked]))
    return out


def score_calibration(calib: dict[str, Any], cam_truth: dict[str, Any], emu: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    intr = (calib or {}).get("intrinsics") or {}
    if intr.get("K") is not None and cam_truth:
        K, Kt = np.asarray(intr["K"], float), np.asarray(cam_truth["K"], float)
        out["f_err_px"] = float(K[0, 0] - Kt[0, 0])
        out["cx_err_px"] = float(K[0, 2] - Kt[0, 2])
        out["cy_err_px"] = float(K[1, 2] - Kt[1, 2])
    ma = (calib or {}).get("mount_alignment") or {}
    if ma.get("yaw_deg") is not None and cam_truth.get("mount_error_deg"):
        te = cam_truth["mount_error_deg"]
        out["mount_err_deg"] = {k: float(ma[f"{k}_deg"]) - float(te[k]) for k in ("yaw", "pitch", "roll")
                                if ma.get(f"{k}_deg") is not None}
    for key, path in (("readout_s", ("rolling_shutter", "readout_s")),
                      ("capture_to_frame_s", ("latency", "capture_to_frame_s"))):
        v = ((calib or {}).get(path[0]) or {}).get(path[1])
        if v is not None:
            out[f"{key}_err"] = float(v) - float(emu[key])
    gr = (calib or {}).get("gimbal_response") or {}
    for k in ("natural_frequency_hz", "damping_ratio", "rate_limit_dps"):
        if gr.get(k) is not None and k in emu:
            out[f"gimbal_{k}_err"] = float(gr[k]) - float(emu[k])
    return out or {"note": "config/calibration.yaml has no values yet (OPEN calibration block)"}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    args = ap.parse_args()
    run = Path(args.run_dir)
    tel = load_jsonl(run / "telemetry.jsonl")
    truth = load_jsonl(run / "truth.jsonl")
    result = {"tracking": score_tracking(align(tel, truth))}
    cam_truth_path = ROOT / "evaluation" / "camera_truth.yaml"
    cam_truth = yaml.safe_load(cam_truth_path.read_text()) if cam_truth_path.exists() else {}
    calib = yaml.safe_load((ROOT / "config" / "calibration.yaml").read_text())
    cam = yaml.safe_load((ROOT / "config" / "camera_action5.yaml").read_text())
    gim = yaml.safe_load((ROOT / "config" / "gimbal_storm32.yaml").read_text())
    emu = {"readout_s": cam["rolling_shutter"]["readout_s"], "capture_to_frame_s": cam["latency"]["capture_to_frame_s"],
           **gim["response"]}
    result["calibration"] = score_calibration(calib, cam_truth or {}, emu)
    (run / "score.json").write_text(json.dumps(result, indent=2, default=lambda o: None if
                                               (isinstance(o, float) and math.isnan(o)) else str(o)))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
