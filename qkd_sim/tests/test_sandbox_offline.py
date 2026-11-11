"""Offline tests of sandbox/ (ported from the team's tests/test_sandbox.py, test_qkd_core.py, test_error_budget.py).

Records follow telemetry/schema.json. Engines are seeded (test mode only; real runs use os.urandom).
"""
from __future__ import annotations

import json
import math
import threading
import time

import numpy as np
import pytest

from sandbox.bb84 import QuantumCryptoEngine, toeplitz_amplify
from sandbox.error_budget import BudgetParams, qber_estimate, transmittance
from sandbox.eve import Eve, intercept_resend_error
from sandbox.main import _apply_overrides, load_cfg, load_records, main as sandbox_main
from sandbox.pipeline import Sandbox


def rec(t: float, residual: float = 0.7, state: str = "TRACK", lean=None, rng=None, drift=2.0, intensity=1.0) -> dict:
    return {"t": t, "state": state, "rig": "rig_fixed", "err_x_urad": 0.0, "err_y_urad": 0.0,
            "residual_mrad": residual, "drift_mrad": drift, "jitter_mrad": 0.5, "gimbal_lag_mrad": None,
            "lean_deg": lean, "range_m": rng, "intensity": intensity, "exposure_gain": 1.0, "spot_fwhm_px": 5.0,
            "snr": 20.0, "saturated": False, "latency_ms": 150.0, "gimbal_pan_deg": 0.0, "gimbal_tilt_deg": 3.8}


def stream(seconds: float = 4.0, hz: float = 30.0, **kw):
    return [rec(i / hz, **kw) for i in range(int(seconds * hz) + 1)]


def blocks(cfg, records, seed=1, **kw):
    sb = Sandbox(cfg, engine=QuantumCryptoEngine(cfg["bb84"], seed=seed), **kw)
    return sb, [r for r in sb.run(records) if r.kind == "block"]


@pytest.fixture
def cfg():
    return load_cfg()


def test_records_follow_the_telemetry_schema():
    from telemetry.bus import validate_telemetry
    validate_telemetry(rec(1.0))


def test_calm_hover_issues_keys_near_3_percent(cfg):
    sb, b = blocks(cfg, stream())
    assert [x.outcome for x in b] == ["key"] * 4
    assert all(0.015 < x.qber < 0.045 for x in b)
    assert all(len(x.key) == 64 and x.secret_bits >= 256 for x in b)
    assert len({x.key for x in b}) == 4 and len(sb.keys) == 4


def test_more_residual_means_higher_qber_dose_response(cfg):
    q = [np.mean([x.qber for x in blocks(cfg, stream(2.0, residual=r))[1]]) for r in (0.7, 4.0, 8.0)]
    assert q[0] < q[1] < q[2]


def test_searching_is_no_link_not_a_key(cfg):
    _, b = blocks(cfg, stream(2.0, state="SEARCH"))
    assert all(x.outcome == "no_link" for x in b)
    _, b = blocks(cfg, stream(2.0, state="ACQUIRE"))
    assert all(x.outcome == "no_link" for x in b)


def test_null_lean_and_range_use_the_calibration_defaults(cfg):
    p = BudgetParams.from_cfg(cfg)
    assert transmittance(p, rec(0, lean=None, rng=None)).eta == pytest.approx(
        transmittance(p, rec(0, lean=0.0, rng=p.cal_range_m)).eta)


def test_intercept_resend_on_half_the_pulses_aborts_at_about_14_5_percent(cfg):
    aborts = []
    gate_pub = type("P", (), {"send": lambda self, m: aborts.append(m)})()
    from sandbox.gate import Gate
    _, b = blocks(_apply_overrides(cfg, eve_fraction=0.5), stream(3.0), gate=Gate(cfg, gate_pub))
    assert all(x.outcome == "abort" and 0.13 < x.qber < 0.165 for x in b)
    assert len(aborts) == len(b) and aborts[0]["cmd"] == "reacquire"
    assert intercept_resend_error(0.02, 0.5) == pytest.approx(0.02 * 0.875 + 0.98 * 0.125)


def test_pns_keeps_qber_low_but_the_decoy_check_aborts(cfg):
    _, b = blocks(_apply_overrides(cfg, pns=True), stream(3.0))
    assert all(x.outcome == "decoy_abort" and x.qber < 0.05 for x in b)


def test_no_false_decoy_alarms_when_transmittance_varies_within_a_block(cfg):
    recs = [rec(i / 30, residual=0.7 + 4.0 * (0.5 + 0.5 * math.sin(i / 3))) for i in range(121)]
    _, b = blocks(cfg, recs)
    assert not any(x.outcome == "decoy_abort" for x in b)


def test_eve_toggle_messages(cfg):
    e = Eve(cfg)
    assert e.mode == "off" and not e.config.active
    e.handle({"mode": "intercept_resend", "fraction": 0.3})
    assert e.config.intercept_fraction == 0.3
    e.handle({"mode": "pns"})
    assert e.config.pns and e.config.intercept_fraction == 0.0
    e.handle({"mode": "off"})
    assert not e.config.active
    with pytest.raises(ValueError):
        e.handle({"mode": "bogus"})


def test_pooling_yields_more_keys_in_the_marginal_band(cfg):
    """At 6 mrad residual (~3.9% QBER) most single 1 s blocks are too small for a key after the finite-size and
    decoy terms; pooling recovers them. (At ~4.6% QBER neither does: the decoy-certified single-photon share no
    longer covers the Cascade leakage, so pooling only accumulates.)"""
    _, single = blocks(cfg, stream(8.0, residual=6.0))
    _, pooled = blocks(_apply_overrides(cfg, pool=True), stream(8.0, residual=6.0))
    n_single = sum(x.outcome == "key" for x in single)
    n_pooled = sum(x.outcome == "key" for x in pooled)
    assert n_pooled > n_single and any(x.outcome == "pooling" for x in pooled)
    _, worse = blocks(_apply_overrides(cfg, pool=True), stream(4.0, residual=7.0))
    assert {x.outcome for x in worse} == {"pooling"}


def test_toeplitz_fft_matches_the_matrix_product():
    rng = np.random.default_rng(0)
    key = rng.integers(0, 2, 5000)
    seed = rng.integers(0, 2, 5000 + 256 - 1)
    idx = (5000 - 1) + np.arange(256)[:, None] - np.arange(5000)[None, :]
    assert np.array_equal(toeplitz_amplify(key, 256, seed), (seed[idx] @ key) & 1)


def test_cascade_corrects_every_error_and_counts_leakage():
    eng = QuantumCryptoEngine(seed=3)
    a = eng.rng.integers(0, 2, 20000)
    b = a ^ (eng.rng.random(20000) < 0.03)
    fixed, leaked, corrected = eng.cascade_reconcile(a, b, 0.03)
    assert np.array_equal(fixed, a) and corrected == int((a != b).sum())
    assert leaked > 64


def test_qber_formula_matches_the_simulated_clicks(cfg):
    p = BudgetParams.from_cfg(cfg)
    q_formula = qber_estimate(p, transmittance(p, rec(0)))
    _, b = blocks(cfg, stream(2.0))
    assert np.mean([x.qber for x in b]) == pytest.approx(q_formula, abs=0.006)


def test_replay_writes_a_log_the_dashboard_draws(tmp_path):
    tel = tmp_path / "telemetry.jsonl"
    tel.write_text("\n".join(json.dumps(r) for r in stream(3.0)) + "\n")
    log = tmp_path / "sandbox_log.jsonl"
    sandbox_main(["--replay", str(tmp_path), "--log", str(log), "--quiet"])
    rows = [json.loads(x) for x in log.read_text().splitlines()]
    assert {r["kind"] for r in rows} == {"interval", "block"}
    from tools.make_dashboard import body_html
    assert "QKD Run Dashboard" in body_html(rows)
    assert len(load_records(tel)) == 91


def test_sandbox_on_the_bus_publishes_keys_and_aborts():
    """The real program: telemetry in on 5556, an Eve toggle on 5560, keys out on 5559, aborts on 5557."""
    from sandbox.main import run
    from telemetry.bus import Publisher, Subscriber
    tel, eve = Publisher("telemetry"), Publisher("eve", validate=False)
    keys, abort = Subscriber("keys"), Subscriber("abort")
    th = threading.Thread(target=run, args=(load_cfg(),), kwargs={"duration_s": 12.0, "quiet": True}, daemon=True)
    th.start()
    try:
        time.sleep(1.0)                                      # slow joiners
        got_keys, got_aborts = [], []
        for i in range(181):                                 # 3 s calm, then Eve on half the pulses for 3 s
            if i == 90:
                for _ in range(3):
                    eve.send({"mode": "intercept_resend", "fraction": 0.5})
                time.sleep(0.5)
            tel.send(rec(i / 30))
            time.sleep(0.01)
            while (m := keys.recv(timeout_s=0)) is not None:
                got_keys.append(m[1])
            while (m := abort.recv(timeout_s=0)) is not None:
                got_aborts.append(m[1])
        t_end = time.monotonic() + 8
        while time.monotonic() < t_end and not (got_aborts and len(got_keys) >= 5):
            if (m := keys.recv(timeout_s=0.1)) is not None:
                got_keys.append(m[1])
            if (m := abort.recv(timeout_s=0)) is not None:
                got_aborts.append(m[1])
        outcomes = [k["outcome"] for k in got_keys]
        assert "key" in outcomes and "abort" in outcomes, outcomes
        assert got_aborts and got_aborts[0]["cmd"] == "reacquire"
        assert all(("key" in k) == (k["outcome"] == "key") or k["key"] is None for k in got_keys)
    finally:
        th.join(timeout=15)
        for s in (tel, eve, keys, abort):
            s.close()
