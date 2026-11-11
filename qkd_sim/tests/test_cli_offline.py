"""The convenience layer: short scenario / rig names, the browser camera viewer, and the qkd launcher."""
from __future__ import annotations

import subprocess
import sys
import urllib.request
from pathlib import Path

import numpy as np
import pytest

from tools.scenario import ScenarioError, normalize_rig, resolve_path

ROOT = Path(__file__).resolve().parent.parent


@pytest.mark.parametrize("name,stem", [("t3", "t3_turbulence"), ("t3_turbulence_tag", "t3_turbulence_tag"),
                                       ("t1", "t1_noise_floor"), ("calib_gim", "calib_gimbal"),
                                       ("mission", "mission_300m"), ("pendulum", "pendulum")])
def test_scenario_short_names(name, stem):
    assert resolve_path(name).stem == stem


def test_scenario_ambiguous_and_unknown():
    with pytest.raises(ScenarioError, match="ambiguous"):
        resolve_path("calib")
    with pytest.raises(ScenarioError, match="Available"):
        resolve_path("t9")


@pytest.mark.parametrize("alias,rig", [("fixed", "rig_fixed"), ("A", "rig_fixed"), ("rig_fixed", "rig_fixed"),
                                       ("gimbal", "rig_gimbal"), ("b", "rig_gimbal"), ("Gimbal", "rig_gimbal")])
def test_rig_aliases(alias, rig):
    assert normalize_rig(alias) == rig


def test_rig_alias_unknown():
    with pytest.raises(ScenarioError):
        normalize_rig("C")


def test_mjpeg_server_serves_frames():
    from tools.mjpeg_server import MjpegServer
    srv = MjpegServer(port=0, host="127.0.0.1", max_fps=1000).start()
    try:
        base = f"http://127.0.0.1:{srv.port}"
        assert b"/stream" in urllib.request.urlopen(base + "/", timeout=5).read()
        img = np.zeros((1080, 1920, 3), np.uint8)
        img[500:580, 900:1020, 2] = 255
        assert srv.update(img, ["t = 1.00 s"])
        jpg = urllib.request.urlopen(base + "/frame.jpg", timeout=5).read()
        import cv2
        dec = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        assert dec.shape[1] == 960                     # downscaled for the browser
        assert dec[260:285, 460:500, 2].mean() > 200   # red patch survives
        with urllib.request.urlopen(base + "/stream", timeout=5) as r:
            assert r.headers["Content-Type"].startswith("multipart/x-mixed-replace")
            assert r.read(7) == b"--frame"
    finally:
        srv.stop()


def _qkd(*args) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(ROOT / "qkd.py"), *args], capture_output=True, text=True,
                          timeout=120, cwd=ROOT)


def test_qkd_help_and_doctor():
    r = _qkd()
    assert r.returncode == 0 and "doctor" in r.stdout
    r = _qkd("doctor")
    assert r.returncode == 0 and "Next step" in r.stdout


def test_qkd_docker_args(monkeypatch):
    import importlib.util
    spec = importlib.util.spec_from_file_location("qkd_cli", ROOT / "qkd.py")
    qkd = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(qkd)
    monkeypatch.setattr(qkd, "has_nvidia", lambda: True)
    monkeypatch.setattr(qkd, "IS_LINUX", False)
    args = qkd.docker_run_args(["python3", "tools/run_scenario.py", "t3"], ports=(8080,))
    assert args[:2] == ["docker", "run"]
    assert "--gpus" in args and "8080:8080" in args
    assert f"{qkd.ROOT}:/qkd_sim" in args                       # repo mounted, paths with spaces stay one arg
    assert args[-3:] == ["python3", "tools/run_scenario.py", "t3"]
    monkeypatch.setattr(qkd, "IS_LINUX", True)
    args = qkd.docker_run_args(["bash"], ports=(8080,))
    assert "--net" in args and "8080:8080" not in args


def test_accuracy_overlay_marks_truth_and_tracker(cfgs):
    """The overlay projects the truth LED through the exact camera and measures the tracker's pixel error."""
    import math
    from collections import deque
    from types import SimpleNamespace

    from sim_bridge.truth_publisher import camera_rotation, compute_truth, lens_position, tx_axis_body
    from tools.accuracy_overlay import AccuracyOverlay, truth_pixel

    rig = cfgs["rigs"]["rig_fixed"]
    st = cfgs["world"]["hover_station_m"]
    off = cfgs["drone"]["payload"]["variants"]["red_led_beacon"]["offset_m"]
    lens0 = lens_position("rig_fixed", rig, camera_rotation("rig_fixed", rig, None))
    truth = compute_truth(t_sim=5.0, rig="rig_fixed", rig_cfg=rig, drone_pos=st, drone_quat=(1, 0, 0, 0),
                          drone_vel=(0, 0, 0), payload_offset=off, tx_axis_b=tx_axis_body(st, off, lens0, 0.0))

    class Tap:
        def __init__(self, rec):
            self.rec = rec

        def find(self, t, tol=0.002):
            return self.rec

    ov = AccuracyOverlay(cfgs, "rig_fixed", deque([truth]), None)
    u, v = truth_pixel(ov.lens, truth)
    assert 0 < u < 1920 and 0 < v < 1080
    assert abs(u - 960) < 60 and abs(v - 540) < 60          # aimed at the station: LED near the image centre
    frame = SimpleNamespace(image=np.zeros((1080, 1920, 3), np.uint8), t_capture_sim=5.0)
    rec = {"t": 5.0, "state": "TRACK", "spot_u_px": u + 3.0, "spot_v_px": v - 4.0}
    ov.tel = Tap(rec)
    img, lines = ov.annotate(frame, rec)
    assert "error 5.0 px" in lines[0]
    mrad = float(lines[0].split("= ")[1].split(" mrad")[0])
    assert mrad == pytest.approx(5.0 / ov.lens.f_px * 1e3, rel=0.05)
    b, g, r = img[int(round(v)), int(round(u))].tolist()
    assert g > 150 and g > 2 * max(b, r)                                      # green truth dot drawn
    _, lines = ov.annotate(frame, {"t": 5.0, "state": "SEARCH", "spot_u_px": None, "spot_v_px": None})
    assert "no spot" in lines[0]
    _ = math


def test_flight_path_segments():
    """Scripted paths: continuous between segments, move speed honoured, windows add up."""
    from sim_bridge.flight_path import offset_at, validate, windows
    path = [{"name": "h", "type": "hold", "duration_s": 2},
            {"name": "s", "type": "step", "offset_m": [0, 0.3, 0], "duration_s": 1},
            {"name": "m", "type": "move", "to_m": [1.0, 0.3, 0], "speed_mps": 0.5},
            {"name": "w", "type": "sine", "amplitude_m": [0, 0.5, 0], "freq_hz": [0, 0.25, 0], "duration_s": 4}]
    validate(path)
    w = windows(path)
    assert [x["name"] for x in w] == ["h", "s", "m", "w"]
    assert w[2]["t_end"] - w[2]["t_start"] == pytest.approx(2.0)          # 1 m at 0.5 m/s
    assert np.allclose(offset_at(path, 1.0), [0, 0, 0])
    assert np.allclose(offset_at(path, 2.5), [0, 0.3, 0])
    assert np.allclose(offset_at(path, 4.0), [0.5, 0.3, 0])               # half-way along the move
    assert np.allclose(offset_at(path, 5.0 + 1.0), [1.0, 0.8, 0])         # sine peak (quarter period)
    assert np.allclose(offset_at(path, 100.0), offset_at(path, w[-1]["t_end"]))
    with pytest.raises(ValueError):
        validate([{"type": "zigzag"}])


def test_tracking_sweep_scenario_loads():
    from tools.scenario import load_scenario
    scn = load_scenario("tracking_sweep")
    assert scn["drone"]["wander"] is False and len(scn["drone"]["path"]) >= 10
