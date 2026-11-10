"""Test configuration.

Two kinds of tests:
  * offline  - pure Python (models, pipeline, schema, no-bypass, OPEN stubs). Run anywhere: ``pytest``.
  * sim      - need Gazebo Harmonic (and PX4 SITL for the flying ones). Marked ``@pytest.mark.sim`` /
               ``@pytest.mark.px4``; skipped unless QKD_RUN_SIM=1 and the gz bindings + ``gz`` are present
               (PX4 tests also need PX4_DIR with a built px4_sitl). Run inside docker/run.sh:
                   QKD_RUN_SIM=1 pytest -m sim -s
"""
from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def _gz_ready() -> tuple[bool, str]:
    if os.environ.get("QKD_RUN_SIM") != "1":
        return False, "simulator tests disabled (set QKD_RUN_SIM=1 inside the docker image)"
    from sim_bridge.gz_io import gz_available
    if not gz_available():
        return False, "gz-transport13 / gz-msgs10 Python bindings not importable"
    if shutil.which("gz") is None:
        return False, "gz executable not on PATH"
    return True, ""


def _px4_ready() -> tuple[bool, str]:
    px4 = Path(os.environ.get("PX4_DIR", Path.home() / "PX4-Autopilot")) / "build" / "px4_sitl_default" / "bin" / "px4"
    return (px4.exists(), f"PX4 SITL not built at {px4}")


def pytest_configure(config):
    config.addinivalue_line("markers", "sim: needs a running Gazebo Harmonic (QKD_RUN_SIM=1)")
    config.addinivalue_line("markers", "px4: needs PX4 SITL (PX4_DIR)")


def pytest_collection_modifyitems(config, items):
    gz_ok, gz_why = _gz_ready()
    px4_ok, px4_why = _px4_ready()
    for item in items:
        if "sim" in item.keywords and not gz_ok:
            item.add_marker(pytest.mark.skip(reason=gz_why))
        if "px4" in item.keywords and not px4_ok:
            item.add_marker(pytest.mark.skip(reason=px4_why))


@pytest.fixture(scope="session")
def cfgs():
    from sim_bridge import config as cfgmod
    return cfgmod.load_all()


@pytest.fixture
def run_dir(tmp_path):
    return tmp_path / "run"
