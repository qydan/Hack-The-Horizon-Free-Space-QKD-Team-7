import json
import time

import pytest

from telemetry import (FIELDS, ControlPublisher, ControlSubscriber, CSVReplaySource, TelemetryMessage,
                       ZmqPublisher, ZmqSubscriber, write_csv)

EXAMPLE = {"t": 12.508, "state": "TRACK", "drift_mrad": 3.1, "jitter_mrad": 0.8, "residual_mrad": 0.4,
           "gimbal_lag_mrad": 0.3, "lean_deg": 4.2, "range_m": 3.05, "intensity": 0.92,
           "rx_signal": 0.88, "rx_background": 0.04}


def msg(t=0.0, **kw):
    d = {**EXAMPLE, "t": t, **kw}
    return TelemetryMessage.from_dict(d)


def test_example_message_from_the_design_doc_round_trips():
    m = TelemetryMessage.from_dict(EXAMPLE)
    assert json.loads(m.to_json()) == EXAMPLE
    assert TelemetryMessage.from_json(m.to_json()) == m


def test_ir_link_fields_are_optional():
    d = {k: v for k, v in EXAMPLE.items() if not k.startswith("rx_")}
    m = TelemetryMessage.from_dict(d)
    assert m.rx_signal is None and m.rx_background is None


@pytest.mark.parametrize("bad", [{"state": "FLYING"}, {"bogus": 1}])
def test_bad_messages_rejected(bad):
    with pytest.raises(ValueError):
        TelemetryMessage.from_dict({**EXAMPLE, **bad})


def test_missing_field_rejected():
    d = dict(EXAMPLE)
    del d["residual_mrad"]
    with pytest.raises(ValueError, match="missing"):
        TelemetryMessage.from_dict(d)


def test_locked_property():
    assert msg(state="TRACK").locked and msg(state="COAST").locked and not msg(state="SEARCH").locked


def test_csv_round_trip(tmp_path):
    msgs = [msg(i / 120, rx_signal=None, rx_background=None) for i in range(5)]
    msgs.append(msg(5 / 120, rx_signal=0.5, rx_background=0.1))
    p = tmp_path / "t.csv"
    write_csv(str(p), msgs)
    back = CSVReplaySource(str(p))
    assert len(list(back)) == 6 and back.duration == pytest.approx(5 / 120, abs=1e-4)
    assert list(back)[0].rx_signal is None and list(back)[-1].rx_signal == pytest.approx(0.5)


def test_csv_rejects_bad_files(tmp_path):
    p = tmp_path / "bad.csv"
    p.write_text("t,state\n0,TRACK\n")
    with pytest.raises(ValueError, match="missing column"):
        CSVReplaySource(str(p))
    write_csv(str(p), [msg(1.0), msg(0.5)])
    with pytest.raises(ValueError, match="increasing"):
        CSVReplaySource(str(p))


def test_default_data_file_is_valid():
    import os
    src = CSVReplaySource(os.path.join(os.path.dirname(__file__), "..", "data", "fake_telemetry.csv"))
    assert src.duration > 100


def test_zmq_telemetry_and_control_round_trip():
    ep, cep = "tcp://127.0.0.1:35561", "tcp://127.0.0.1:35562"
    pub = ZmqPublisher(ep)
    sub = ZmqSubscriber(ep, timeout_s=1.0)
    cpub = ControlPublisher(cep)
    csub = ControlSubscriber(cep)
    try:
        time.sleep(0.4)  # PUB/SUB slow joiner
        sent = [msg(i / 120) for i in range(3)]
        for m in sent:
            pub.send(m)
        got = []
        for m in sub:
            got.append(m)
            if len(got) == 3:
                break
        assert got == sent
        cpub.send_reacquire("abort")
        time.sleep(0.2)
        assert csub.poll() == [{"cmd": "reacquire", "reason": "abort"}]
        assert csub.poll() == []
    finally:
        for o in (pub, sub, cpub, csub):
            o.close()
