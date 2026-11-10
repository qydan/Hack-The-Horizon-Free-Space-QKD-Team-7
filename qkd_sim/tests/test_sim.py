"""End-to-end acceptance tests in Gazebo Harmonic (+ PX4 SITL for the flying ones). Section 14.

Run inside the docker image:   QKD_RUN_SIM=1 pytest -m sim -s
Each test launches tools/run_scenario.run(...) headless and analyses what it recorded. Evaluation-side code
(these tests) may use ground truth and evaluation/camera_truth.yaml.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

pytestmark = pytest.mark.sim

ROOT = Path(__file__).resolve().parent.parent


def _run(scenario, rig, tmp_path, **kw):
    from tools.run_scenario import run
    return run(scenario, rig, out=tmp_path / f"{scenario}_{rig}", **kw)


def _truth(run_dir: Path) -> list[dict]:
    p = run_dir / "truth.jsonl"
    return [json.loads(x) for x in p.read_text().splitlines()] if p.exists() else []


def _lens_and_pose(rig: str):
    from sim_bridge import config as cfgmod
    from sim_bridge.camera_model import EquidistantLens
    from sim_bridge.truth_publisher import camera_rotation, lens_position
    cfgs = cfgmod.load_all()
    lens = EquidistantLens.from_cfg(cfgs["camera_action5"], cfgs["rigs"][rig]["principal_point_offset_px"])
    R = camera_rotation(rig, cfgs["rigs"][rig], None)
    return lens, lens_position(rig, cfgs["rigs"][rig], R), R


# ------------------------------------------------------------------------------------------------ A1
@pytest.mark.parametrize("rig", ["rig_fixed", "rig_gimbal"])
@pytest.mark.parametrize("fidelity", ["full", "fast"])
def test_a1_world_loads_headless(rig, fidelity, tmp_path):
    summary, out = _run("calib_brightness", rig, tmp_path, duration=10.0, fidelity=fidelity)
    print(f"\nA1 {rig} {fidelity}: RTF mean {summary.get('rtf_mean'):.3f}, last reported {summary.get('rtf_last_reported')}")
    assert summary.get("frames_read", 0) > 0
    assert not [w for w in summary["warnings"] if "failed" in w]


# ------------------------------------------------------------------------------------------------ A2, A3
@pytest.mark.px4
def test_a2_a3_station_keeping_and_hover_throttle(tmp_path):
    """Wander off, fan off: horizontal RMS < 0.05 m, vertical RMS < 0.03 m over 60 s. A3: hover thrust from
    the commanded motor speeds (THR_MDL_FAC = 1, so normalised thrust = (omega / omega_max)^2) in 0.40-0.60."""
    from gz.msgs10.actuators_pb2 import Actuators  # type: ignore

    speeds = []

    def cb(frame, world):
        if not getattr(cb, "subscribed", False):
            world.subscribe(Actuators, "/mini2se/command/motor_speed", lambda m: speeds.append(list(m.velocity)))
            cb.subscribed = True

    summary, out = _run("t7_eve", "rig_fixed", tmp_path, duration=100.0, fidelity="fast",
                        patch={"drone": {"wander": False}, "fan": {"schedule": [{"t_s": 0, "level": "off"}]}},
                        frame_callback=cb)
    tr = [r for r in _truth(out) if r["t_sim"] - _truth(out)[0]["t_sim"] > 35.0]
    p = np.array([r["drone_pos_m"] for r in tr])
    st = np.array(yaml.safe_load((ROOT / "config" / "world.yaml").read_text())["hover_station_m"])
    h_rms = float(np.sqrt(np.mean(np.sum((p[:, :2] - st[:2]) ** 2, axis=1))))
    v_rms = float(np.sqrt(np.mean((p[:, 2] - st[2]) ** 2)))
    print(f"\nA2 horizontal RMS {h_rms:.3f} m, vertical RMS {v_rms:.3f} m")
    assert h_rms < 0.05 and v_rms < 0.03
    w = np.array(speeds[len(speeds) // 2:])
    thr = float(np.mean((w / 1000.0) ** 2))
    print(f"A3 hover thrust (normalised) {thr:.3f}")
    assert 0.40 <= thr <= 0.60


# ------------------------------------------------------------------------------------------------ A4
@pytest.mark.px4
def test_a4_fan_high_lean(tmp_path):
    summary, out = _run("t7_eve", "rig_fixed", tmp_path, duration=90.0, fidelity="fast",
                        patch={"drone": {"wander": False},
                               "fan": {"schedule": [{"t_s": 0, "level": "off"}, {"t_s": 40, "level": "high"}]}})
    tr = _truth(out)
    t0 = tr[0]["t_sim"]
    on = [r for r in tr if r["t_sim"] - t0 > 40]
    steady = [r for r in on if r["t_sim"] - t0 > 70]
    lean = float(np.mean([r["lean_deg"] for r in steady]))
    y = np.array([r["drone_pos_m"][1] for r in on])
    print(f"\nA4 steady lean {lean:.2f} deg, peak +y displacement {y.max():.3f} m, final y {y[-50:].mean():.3f} m")
    assert 3.0 <= lean <= 6.0, "see README: spec values predict ~6.6 deg at 5 m/s"
    assert y.max() > 0.02 and abs(y[-50:].mean()) < 0.5 * y.max()


# ------------------------------------------------------------------------------------------------ A5
@pytest.mark.parametrize("rig", ["rig_fixed", "rig_gimbal"])
def test_a5_camera(rig, tmp_path):
    frames = []

    def cb(fr, world):
        frames.append((fr.t_capture_sim, fr.t_delivered_sim, fr.image.shape, fr.image.dtype))

    from sim_bridge import config as cfgmod
    cam = cfgmod.load("camera_action5")
    _run("calib_brightness", rig, tmp_path, duration=8.0, fidelity="full", frame_callback=cb)
    caps = np.array([f[0] for f in frames])
    fps = 1.0 / np.mean(np.diff(caps))
    assert fps == pytest.approx(cam["stream"]["fps"], rel=0.05)
    assert all(f[2] == (1080, 1920, 3) and f[3] == np.uint8 for f in frames)
    lat = np.array([f[1] - f[0] for f in frames])
    assert np.all(np.abs(lat - cam["latency"]["capture_to_frame_s"]) <= 1.0 / cam["stream"]["fps"])


# ------------------------------------------------------------------------------------------------ A6
def test_a6_beacon_modulation(tmp_path):
    from sim_bridge.camera_model import project_world_point
    lens, lens_pos, R = _lens_and_pose("rig_fixed")
    uv = project_world_point(lens, lens_pos, R, np.array([1.0, 0.0, 1.0]))     # static near beacon (truth)
    u, v = int(round(uv[0])), int(round(uv[1]))
    reds, ts = [], []

    def cb(fr, world):
        reds.append(float(fr.image[v - 2:v + 3, u - 2:u + 3, 2].mean()))
        ts.append(fr.t_capture_sim)

    _run("t1_noise_floor", "rig_fixed", tmp_path, duration=12.0, fidelity="full", frame_callback=cb,
         patch={"lighting": {"initial": "dark"}})
    r = np.array(reds[30:])
    spec = np.abs(np.fft.rfft(r - r.mean()))
    f = np.fft.rfftfreq(len(r), np.mean(np.diff(ts)))
    hi, lo = np.percentile(r, 90), np.percentile(r, 10)
    print(f"\nA6 FFT peak {f[np.argmax(spec)]:.2f} Hz, low/high {lo / hi:.3f}, high level {hi:.0f}")
    assert f[np.argmax(spec)] == pytest.approx(7.0, abs=0.2)
    assert hi < 250, "beacon saturates: lower beacon.yaml red_led.emissive_rgb_full"
    assert lo / hi == pytest.approx(0.40, abs=0.05)


# ------------------------------------------------------------------------------------------------ A7
def test_a7_rolling_shutter_skew(tmp_path):
    """Beacon sliding at 1 m/s at 3 m: its image position lags/leads the truth position at t_capture by
    speed_px * readout * (v / H). Averaged over frames the implied readout must match the config +-20%."""
    from sim_bridge import config as cfgmod
    from sim_bridge.camera_model import project_world_point
    cam = cfgmod.load("camera_action5")
    lens, lens_pos, R = _lens_and_pose("rig_fixed")
    s = cfgmod.load("world")["props"]["slider_beacon"]
    est = []

    def cb(fr, world):
        t_rel = fr.t_capture_sim               # the slider joint starts moving at sim time 0 (JointController)
        if t_rel > 0.95 * s["travel_m"] / s["speed_mps"]:
            return                             # at the end stop
        red = fr.image[:, :, 2].astype(float) - fr.image[:, :, 1].astype(float)
        if red.max() < 60:
            return
        vv, uu = np.nonzero(red > 0.5 * red.max())
        u_meas, v_meas = uu.mean(), vv.mean()
        p0 = np.array(s["position_m"]) + np.array([0, s["speed_mps"] * t_rel, 0])
        p1 = p0 + np.array([0, s["speed_mps"] * 0.01, 0])
        uv0 = project_world_point(lens, lens_pos, R, p0)
        uv1 = project_world_point(lens, lens_pos, R, p1)
        speed_px = (uv1[0] - uv0[0]) / 0.01
        if abs(speed_px) < 1 or not (0.05 < v_meas / cam["stream"]["height_px"] < 0.95):
            return
        dt_row = (u_meas - uv0[0]) / speed_px
        est.append(dt_row / (v_meas / cam["stream"]["height_px"]))

    _run("test_a7_slider", "rig_fixed", tmp_path, duration=3.5, fidelity="full", frame_callback=cb)
    readout = float(np.median(est))
    print(f"\nA7 implied readout {1e3 * readout:.1f} ms from {len(est)} frames")
    assert readout == pytest.approx(cam["rolling_shutter"]["readout_s"], rel=0.2)


# ------------------------------------------------------------------------------------------------ A8
def test_a8_flicker_and_banding(tmp_path):
    wall, profiles, ts = [], [], []

    def cb(fr, world):
        g = fr.image[:, :, 1].astype(float)
        wall.append(float(g[100:140, 1700:1760].mean()))          # upper-right wall region
        profiles.append(g[:, 1700:1760].mean(axis=1))
        ts.append(fr.t_capture_sim)

    _run("calib_brightness", "rig_fixed", tmp_path, duration=12.0, fidelity="full", frame_callback=cb,
         patch={"lighting": {"initial": "office_50hz"}, "overrides": {"camera_action5": {"exposure": {"mode": "locked"}}}})
    w = np.array(wall[15:])
    spec = np.abs(np.fft.rfft(w - w.mean()))
    f = np.fft.rfftfreq(len(w), np.mean(np.diff(ts)))
    k10 = np.argmin(np.abs(f - 10.0))
    print(f"\nA8 10 Hz amplitude {spec[k10]:.2f} vs median {np.median(spec):.2f}")
    assert spec[k10] > 5 * np.median(spec[1:])
    # banding: frame-to-frame differences vary along rows (band structure) with rolling shutter on
    d = np.array(profiles[16:]) - np.array(profiles[15:-1])
    band_var = float(np.mean(np.std(d, axis=1)))
    assert band_var > 0.2


# ------------------------------------------------------------------------------------------------ A9
def test_a9_auto_exposure_dark_to_bright(tmp_path):
    from sim_bridge import config as cfgmod
    tau = cfgmod.load("camera_action5")["exposure"]["tau_s"]
    summary, out = _run("t4_lighting", "rig_fixed", tmp_path, duration=12.0, fidelity="full",
                        patch={"drone": {"enabled": False}, "props": {"static_beacons": True},
                               "lighting": {"initial": "dark", "schedule": [{"t_s": 0, "scenario": "dark"},
                                                                            {"t_s": 5, "scenario": "bright_lamp"}]}})
    import csv
    rows = list(csv.DictReader(open(out / "frames_meta.csv")))
    t = np.array([float(r["t_capture"]) for r in rows])
    g = np.array([float(r["exposure_gain"]) for r in rows])
    t_sw = t[0] + 5.0
    g_end = g[-1]
    g_dark = g[t < t_sw][-1]
    g3 = g[t >= t_sw + 3 * tau][0]
    assert abs(math.log(g3 / g_end)) < 0.10 * abs(math.log(g_dark / g_end))
    sat = [r["saturated"] == "True" for r in rows]
    assert any(sat)          # the lamp washes out


# ------------------------------------------------------------------------------------------------ A14
SCENARIOS = [p.stem for p in sorted((ROOT / "scenarios").glob("*.yaml")) if p.stem != "test_a10_ir_beacon"]


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_a14_every_scenario_runs_10s(scenario, tmp_path):
    from tools.scenario import load_scenario
    scn = load_scenario(scenario)
    if scn["drone"]["enabled"]:
        from tests.conftest import _px4_ready
        ok, why = _px4_ready()
        if not ok:
            pytest.skip(why)
    for rig in scn["rigs"]:
        summary, out = _run(scenario, rig, tmp_path, duration=10.0)
        assert (out / "summary.json").exists()
        assert (out / "truth.jsonl").exists() or not scn["recorders"]["truth"]
        assert (out / "frames_meta.csv").exists()
        assert not summary.get("sandbox_port_violations")
