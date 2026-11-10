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
