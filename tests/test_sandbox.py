"""The sandbox: telemetry in, keys out, with Eve and the gate."""
import numpy as np
import pytest

from channel import EveConfig, intercept_resend_error
from error_budget import default_params
from key_pool import KeyPool
from qkd_core import QuantumCryptoEngine
from sandbox import Sandbox
from telemetry import FINE_TRACK, SEARCH, TelemetryMessage


def stream(res=0.5, lean=1.0, drift=2.0, inten=0.99, seconds=3.0, state=FINE_TRACK, hz=120):
    return [TelemetryMessage(i / hz, state, drift, 0.5, res, 0.3, lean, 3.0, inten)
            for i in range(int(seconds * hz) + 1)]


def blocks(sb, msgs):
    return [r for r in sb.run(msgs) if r.kind == "block"]


def test_calm_telemetry_issues_keys():
    sb = Sandbox()
    recs = blocks(sb, stream())
    assert len(recs) == 3 and all(r.outcome == "key" for r in recs)
    assert all(len(r.key) == 64 and r.qber < 0.11 for r in recs)
    assert len(sb.keys) == 3


def test_live_trace_runs_at_100ms():
    sb = Sandbox()
    ints = [r for r in sb.run(stream(seconds=2.0)) if r.kind == "interval"]
    assert 18 <= len(ints) <= 21
    assert all(r.qber_measured < 0.11 and r.state == FINE_TRACK for r in ints)


def test_more_motion_means_higher_qber_dose_response():
    """Fan off < low < high: QBER follows the measured motion (test T6)."""
    qs = []
    for res in (0.5, 4.0, 7.0, 9.0):
        recs = blocks(Sandbox(), stream(res=res, seconds=3.0))
        qs.append(np.mean([r.qber for r in recs]))
    assert qs == sorted(qs) and qs[-1] > qs[0] + 0.02


def test_ground_tracking_matters():
    """A few mrad of ground residual visibly costs signal (otherwise the tracker looks pointless)."""
    calm = Sandbox()
    blocks(calm, stream(res=0.5))
    worse = Sandbox()
    blocks(worse, stream(res=6.0))
    assert worse._blk.mean_eta == 0.0  # blocks reset; use records instead
    # compare interval transmittance loss
    a = [r.eta_db for r in Sandbox().run(stream(res=0.5)) if r.kind == "interval"]
    b = [r.eta_db for r in Sandbox().run(stream(res=6.0)) if r.kind == "interval"]
    assert np.mean(b) > np.mean(a) + 2.0


def test_gust_aborts_and_tells_the_tracker_to_reacquire():
    aborted = []
    sb = Sandbox(on_abort=aborted.append)
    recs = blocks(sb, stream(res=15.0, lean=2.0))
    assert all(r.outcome == "abort" for r in recs)
    assert len(aborted) == len(recs) and all(r.aborted for r in aborted)
    assert len(sb.keys) == 0


def test_losing_the_beacon_is_no_link_not_a_key():
    sb = Sandbox()
    recs = blocks(sb, stream(state=SEARCH, inten=0.0))
    assert all(r.outcome == "no_link" and r.key is None for r in recs)


def test_intercept_resend_on_half_the_pulses_aborts_at_about_14_5_percent():
    sb = Sandbox(eve=EveConfig(intercept_fraction=0.5))
    recs = blocks(sb, stream())
    assert all(r.outcome == "abort" for r in recs)
    assert np.mean([r.qber for r in recs]) == pytest.approx(0.145, abs=0.015)


def test_small_eve_is_not_caught_by_qber_alone():
    """Documenting the limit: 10% interception adds only ~2.5 points and stays under the gate."""
    sb = Sandbox(eve=EveConfig(intercept_fraction=0.1))
    assert all(r.qber < 0.11 for r in blocks(sb, stream()))


def test_pns_keeps_qber_low_but_the_decoy_check_aborts():
    aborted = []
    sb = Sandbox(eve=EveConfig(pns=True), on_abort=aborted.append)
    recs = blocks(sb, stream())
    assert all(r.qber < 0.05 for r in recs)                       # the naive check passes...
    assert all(r.outcome == "decoy_abort" for r in recs)           # ...the decoy states do not
    assert len(aborted) == len(recs) and len(sb.keys) == 0


def test_no_false_decoy_alarms_when_transmittance_varies_within_a_block():
    """The mean of 1 - exp(-eta*lam) over a varying eta is not 1 - exp(-mean(eta)*lam)."""
    msgs = [TelemetryMessage(i / 120, FINE_TRACK, 2.0, 0.5, 0.5 + 6.0 * (i // 12 % 2), 0.3, 1.0, 3.0, 0.99)
            for i in range(121 * 6)]
    recs = blocks(Sandbox(), msgs)
    assert not any(r.outcome == "decoy_abort" for r in recs)


def test_gate_uses_pooled_block_qber_not_single_frames():
    """A short dip in the middle of a block must not abort when the pooled QBER is fine."""
    msgs = stream(seconds=2.0)
    for i in range(40, 50):                    # 80 ms of lost beacon inside the first block
        msgs[i] = TelemetryMessage(msgs[i].t, SEARCH, 0, 0, 0, 0, 0, 3.0, 0.0)
    recs = blocks(Sandbox(), msgs)
    assert recs[0].outcome in ("key", "no_key") and recs[0].qber < 0.11


def test_pooling_blocks_still_works():
    eng = QuantumCryptoEngine()
    sb = Sandbox(engine=eng, pool=KeyPool(eng))
    recs = blocks(sb, stream(res=8.0, seconds=6.0))
    assert {"pooling", "key"} & {r.outcome for r in recs}


def test_ir_link_measured_signal_drives_the_channel():
    """With the photodiode, the measured S/S_cal sets the transmittance (and a lamp adds noise)."""
    def with_rx(sig, bg):
        return [TelemetryMessage(i / 120, FINE_TRACK, 2.0, 0.5, 0.5, 0.3, 1.0, 3.0, 0.99, sig, bg)
                for i in range(361)]
    good = blocks(Sandbox(), with_rx(1.0, 0.0))
    dim = [r for r in Sandbox().run(with_rx(0.05, 0.0)) if r.kind == "interval"]
    lamp = [r for r in Sandbox().run(with_rx(1.0, 30.0)) if r.kind == "interval"]
    assert all(r.outcome == "key" for r in good)
    assert np.mean([r.qber_expected for r in dim]) > 0.05
    assert np.mean([r.qber_expected for r in lamp]) > 0.05


def test_intercept_resend_error_formula():
    assert intercept_resend_error(0.02, 0.0) == pytest.approx(0.02)
    assert intercept_resend_error(0.0, 1.0) == pytest.approx(0.25)


def test_key_material_is_secret_and_distinct():
    sb = Sandbox()
    keys = {r.key for r in blocks(sb, stream(seconds=4.0))}
    assert len(keys) == 4
