import json
import os
import subprocess
import sys
import time

import pytest

ROOT = os.path.join(os.path.dirname(__file__), "..")
PY = sys.executable


def run(*args, timeout=300):
    return subprocess.run([PY, os.path.join(ROOT, "src", "demo_runner.py"), *args],
                          capture_output=True, text=True, cwd=ROOT, timeout=timeout)


@pytest.mark.slow
def test_csv_run_log_and_dashboard(tmp_path):
    log, keys = tmp_path / "run.jsonl", tmp_path / "keys.jsonl"
    r = run("--source", "csv", "--quiet", "--log", str(log), "--keys-out", str(keys))
    assert r.returncode == 0 and "DONE" in r.stdout
    rows = [json.loads(l) for l in log.read_text().splitlines()]
    blocks = [x for x in rows if x["kind"] == "block"]
    assert len(blocks) >= 110 and any(x["kind"] == "interval" for x in rows)
    outcomes = {b["outcome"] for b in blocks}
    assert "key" in outcomes and "abort" in outcomes          # fan phases give keys, the gust aborts
    assert len(keys.read_text().splitlines()) == sum(b["outcome"] == "key" for b in blocks)

    out = tmp_path / "dash.html"
    d = subprocess.run([PY, os.path.join(ROOT, "tools", "make_dashboard.py"), str(log), str(out)],
                       capture_output=True, text=True)
    assert d.returncode == 0, d.stderr
    html = out.read_text()
    assert html.startswith("<!doctype html>") and "<svg" in html and "Two error bars" in html


@pytest.mark.slow
def test_eve_aborts_every_block():
    r = run("--source", "csv", "--eve", "0.5", "--quiet")
    assert r.returncode == 0 and "0 keys in the key store" in r.stdout and " key" not in r.stdout.split("DONE")[1].split("\n")[0]


@pytest.mark.slow
def test_pns_is_caught_by_decoy_states():
    r = run("--source", "csv", "--pns", "--quiet")
    assert "decoy_abort" in r.stdout and "0 keys in the key store" in r.stdout


@pytest.mark.slow
def test_record_and_replay(tmp_path):
    rec = tmp_path / "rec.csv"
    r = run("--source", "csv", "--quiet", "--record", str(rec))
    assert r.returncode == 0 and rec.exists()
    r2 = run("--source", "csv", "--csv", str(rec), "--quiet")
    assert r2.returncode == 0 and "DONE" in r2.stdout


def test_generate_fake_telemetry_tool(tmp_path):
    out = tmp_path / "t.csv"
    r = subprocess.run([PY, os.path.join(ROOT, "tools", "generate_fake_telemetry.py"), "--out", str(out),
                        "--duration", "10"], capture_output=True, text=True)
    assert r.returncode == 0 and len(out.read_text().splitlines()) == 1201


def test_dose_response_tool_prints_both_error_bars():
    r = subprocess.run([PY, os.path.join(ROOT, "tools", "tune.py")], capture_output=True, text=True)
    assert r.returncode == 0 and "Ground residual" in r.stdout and "Drone lean" in r.stdout and "<- gate" in r.stdout


@pytest.mark.slow
def test_two_programs_over_zeromq():
    """Tracker and sandbox as separate processes, joined only by the telemetry stream (and the
    abort / re-acquire command back)."""
    sb = subprocess.Popen([PY, os.path.join(ROOT, "src", "demo_runner.py"), "--source", "zmq", "--quiet",
                           "--timeout", "30"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, cwd=ROOT)
    time.sleep(2.0)
    tr = subprocess.run([PY, os.path.join(ROOT, "src", "tracker.py"), "--source", "sim", "--duration", "8"],
                        capture_output=True, text=True, cwd=ROOT, timeout=200)
    out, err = sb.communicate(timeout=120)
    assert tr.returncode == 0 and "first lock after" in tr.stdout, tr.stderr
    assert sb.returncode == 0, err
    assert "DONE" in out and "key" in out


@pytest.mark.slow
def test_live_dashboard_follows_a_partial_log_and_hides_keys(tmp_path):
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import live_dashboard
    import make_dashboard
    log = tmp_path / "run.jsonl"
    r = run("--source", "csv", "--quiet", "--log", str(log))
    assert r.returncode == 0, r.stderr
    lines = log.read_text().splitlines()
    log.write_text("\n".join(lines) + "\n" + lines[-1][:20])        # a half-written last line
    rows = live_dashboard.read_log(str(log))
    assert len(rows) == len(lines) and all("key" not in x for x in rows)
    body = make_dashboard.body_html(rows, window_s=20.0, suffix=" (live)")
    assert body and "(live)" in body and "Gate decision per key block" in body
    assert make_dashboard.body_html([]) is None
