import json
import os
import subprocess
import sys

import pytest

import demo_runner
from conftest import CALM, FAN_ON
from decoy import DecoyParams
from key_pool import KeyPool
from qkd_core import QuantumCryptoEngine

ROOT = os.path.join(os.path.dirname(__file__), "..")


def test_calm_window_issues_key(engine):
    rec = demo_runner.process_window(engine, CALM)
    assert rec["outcome"] == "key" and len(rec["key"]) == 64 and rec["qber"] < 0.11


def test_fan_window_aborts(engine):
    assert demo_runner.process_window(engine, FAN_ON)["outcome"] == "abort"


def test_eve_forces_abort(engine):
    assert demo_runner.process_window(engine, CALM, eve=1.0)["outcome"] == "abort"


def test_pns_rejected_by_decoy_layer_even_when_qber_looks_fine():
    e = QuantumCryptoEngine(block_size=65536)
    rec = demo_runner.process_window(e, CALM, decoy_params=DecoyParams(), pns=True)
    assert rec["outcome"] == "decoy_reject" and rec["qber"] < 0.11


def test_decoy_without_attack_can_issue_key():
    e = QuantumCryptoEngine(block_size=65536)
    rec = demo_runner.process_window(e, CALM, decoy_params=DecoyParams(), pns=False)
    assert rec["outcome"] == "key" and rec["decoy"]["consistent"]


def test_pooling_outcome():
    e = QuantumCryptoEngine(block_size=32768)
    pool = KeyPool(e)
    metrics = {"drift": 10.0, "jitter": 20.0, "intensity": 0.7}   # ~8% error: single blocks fail
    outcomes = [demo_runner.process_window(e, metrics, pool=pool)["outcome"] for _ in range(14)]
    assert "pooling" in outcomes and "key" in outcomes


def run_cli(*args):
    return subprocess.run([sys.executable, os.path.join(ROOT, "src", "demo_runner.py"), *args],
                          capture_output=True, text=True, cwd=ROOT, timeout=120)


def test_cli_csv_run_and_dashboard(tmp_path):
    log = tmp_path / "run.jsonl"
    r = run_cli("--source", "csv", "--quiet", "--log", str(log))
    assert r.returncode == 0 and "DONE" in r.stdout
    rows = [json.loads(l) for l in log.read_text().splitlines()]
    assert len(rows) == 120 and {"p_error", "qber", "outcome", "drift"} <= set(rows[0])

    out = tmp_path / "dash.html"
    d = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "make_dashboard.py"), str(log), str(out)],
                       capture_output=True, text=True)
    assert d.returncode == 0
    html = out.read_text()
    assert html.startswith("<!doctype html>") and "<svg" in html and "Keys issued" in html


def test_cli_mock_default_runs():
    r = run_cli()
    assert r.returncode == 0 and "DONE" in r.stdout


def test_summarize_telemetry_tool(tmp_path):
    p = tmp_path / "t.csv"
    p.write_text("timestamp,drift,jitter,intensity\n0,100,0,0.1\n1,2,1,0.9\n2,4,1,0.9\n")
    r = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "summarize_telemetry.py"),
                        "--skip", "1", str(p), str(p)], capture_output=True, text=True)
    assert r.returncode == 0 and "drift=3.00px" in r.stdout and "tune.py --calm 3.00" in r.stdout
