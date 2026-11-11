"""Offline checks of the generated assets: every scenario builds a valid world for each of its rigs (A14
build half), the drone model matches the spec mass split, A10 visible-beacon rejection, config tagging."""
from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from tools.scenario import ScenarioError, list_scenarios, load_scenario
from tools.world_builder import build_world

ROOT = Path(__file__).resolve().parent.parent
VALID = [p for p in list_scenarios() if p.stem != "test_a10_ir_beacon"]


@pytest.mark.parametrize("path", VALID, ids=[p.stem for p in VALID])
def test_a14_scenarios_build_for_each_rig(path):
    from sim_bridge import config as cfgmod
    scn = load_scenario(str(path))
    cfgs = cfgmod.load_all(scn.get("overrides"))
    for rig in scn["rigs"]:
        sdf, info = build_world(cfgs, scn, rig)
        root = ET.fromstring(sdf.split("?>", 1)[1])
        world = root.find("world")
        names = [m.get("name") for m in world.findall("model")]
        assert ("mount_fixed" if rig == "rig_fixed" else "gimbal_storm32_3axis") in names
        sensors = root.findall(".//sensor[@name='action5_camera']")
        assert len(sensors) == 1
        plugins = [p.get("name") for p in world.findall("plugin")]
        assert "gz::sim::systems::ApplyLinkWrench" in plugins
        assert "gz::sim::systems::UserCommands" in plugins
        if scn["drone"]["enabled"]:
            assert cfgs["drone"]["model_name"] in names


def test_a10_ir_cut_rejected():
    with pytest.raises(ScenarioError, match="RGB camera"):
        load_scenario("test_a10_ir_beacon")


def test_drone_mass_and_inertia(cfgs):
    from tools.compute_inertia import inertia
    r = inertia(cfgs["drone"], "none")
    assert r["mass_kg"] == pytest.approx(cfgs["drone"]["spec"]["takeoff_mass_kg"], abs=1e-6)
    I = r["I"]
    assert I[0][0] == pytest.approx(5.9e-4, rel=0.10)
    assert I[1][1] == pytest.approx(7.6e-4, rel=0.10)
    assert I[2][2] == pytest.approx(1.26e-3, rel=0.05)
    r2 = inertia(cfgs["drone"], "red_led_beacon")
    assert r2["mass_kg"] == pytest.approx(0.264, abs=1e-6)
    # motor constant gives 1.2 N max thrust per motor, thrust/weight ~2
    m = cfgs["drone"]["motor"]
    tmax = m["motor_constant"] * m["max_rot_velocity"] ** 2
    assert tmax == pytest.approx(1.2, rel=1e-6)
    assert 4 * tmax / (0.246 * 9.81) == pytest.approx(2.0, rel=0.03)


def test_airframe_matches_config(cfgs):
    af = (ROOT / "px4" / "airframes" / f"{cfgs['drone']['px4']['airframe_id']}_gz_mini2se").read_text()
    from tools.compute_inertia import rotor_positions
    a = rotor_positions(cfgs["drone"])[0][0]
    assert f"CA_ROTOR0_PX {a:.4f}" in af
    assert "MPC_TILTMAX_AIR 40" in af and "MPC_XY_VEL_MAX 16" in af


def test_config_values_are_tagged():
    """Every numeric leaf in the simulator configs carries a SPEC / ESTIMATE / MEASURE tag, either on its own
    line or on its parent key's line (lists and grouped dicts)."""
    untagged = []
    for name in ("world", "drone", "beacon", "fan", "camera_action5", "rigs", "gimbal_storm32", "lighting",
                 "bus"):
        lines = (ROOT / "config" / f"{name}.yaml").read_text().splitlines()
        for i, line in enumerate(lines):
            code = line.split("#", 1)[0]
            if not re.search(r":\s*\[?-?\d|^\s*-\s*\[?-?\d|^\s*-\s*\{", code):
                continue
            if re.search(r"#\s*(SPEC|ESTIMATE|MEASURE)", line):
                continue
            # tag inherited from an enclosing key (same or lower indent, within the block)
            indent = len(line) - len(line.lstrip())
            j, inherited = i - 1, False
            while j >= 0:
                prev = lines[j]
                pind = len(prev) - len(prev.lstrip())
                if prev.strip() and pind < indent:
                    inherited = bool(re.search(r"#\s*(SPEC|ESTIMATE|MEASURE)", prev))
                    if inherited or pind == 0:
                        break
                    indent = pind
                j -= 1
            if not inherited:
                untagged.append(f"{name}.yaml:{i + 1}: {line.strip()}")
    assert not untagged, "\n".join(untagged[:40])
