import math
import os

import pytest

from telemetry import CSVReplayProvider, TelemetryExhausted

HEADER = "timestamp,drift,jitter,intensity\n"
FAKE = os.path.join(os.path.dirname(__file__), "..", "data", "fake_telemetry.csv")


def write(tmp_path, body, header=HEADER):
    p = tmp_path / "t.csv"
    p.write_text(header + body)
    return str(p)


def test_window_aggregation(tmp_path):
    path = write(tmp_path, "0.0,2,3,1.0\n0.5,4,4,0.5\n1.0,10,0,0.2\n")
    prov = CSVReplayProvider(path, window_s=1.0)
    m = prov.get_metrics()  # rows at t=0.0 and 0.5
    assert m["drift"] == pytest.approx(3.0)
    assert m["jitter"] == pytest.approx(math.sqrt((9 + 16) / 2))  # RMS
    assert m["intensity"] == pytest.approx(0.75)
    m2 = prov.get_metrics()  # row at t=1.0
    assert m2["drift"] == 10
    with pytest.raises(TelemetryExhausted):
        prov.get_metrics()


def test_loop_restarts(tmp_path):
    prov = CSVReplayProvider(write(tmp_path, "0,1,1,1\n"), window_s=1.0, loop=True)
    assert prov.get_metrics() == prov.get_metrics()


def test_reset(tmp_path):
    prov = CSVReplayProvider(write(tmp_path, "0,1,1,1\n"), window_s=1.0)
    prov.get_metrics()
    prov.reset()
    prov.get_metrics()


@pytest.mark.parametrize("body,header,msg", [
    ("0,1,1,1\n", "timestamp,drift,jitter\n", "missing column"),
    ("", HEADER, "no data rows"),
    ("0,1,1,1\n0,1,1,1\n", HEADER, "strictly increasing"),
    ("0,abc,1,1\n", HEADER, "non-numeric"),
    ("0,nan,1,1\n", HEADER, "non-finite"),
])
def test_bad_files_are_rejected(tmp_path, body, header, msg):
    with pytest.raises(ValueError, match=msg):
        CSVReplayProvider(write(tmp_path, body, header))


def test_bad_window(tmp_path):
    with pytest.raises(ValueError):
        CSVReplayProvider(write(tmp_path, "0,1,1,1\n"), window_s=0)


def test_fake_data_matches_contract_and_drives_both_outcomes():
    from qkd_core import QuantumCryptoEngine
    prov = CSVReplayProvider(FAKE, window_s=1.0)
    assert prov.duration == pytest.approx(120, abs=0.1)
    e = QuantumCryptoEngine()
    ps = []
    while True:
        try:
            m = prov.get_metrics()
        except TelemetryExhausted:
            break
        assert m["drift"] >= 0 and m["jitter"] >= 0 and 0 < m["intensity"] <= 1
        ps.append(e.calculate_channel_error(**m))
    assert len(ps) == 120
    assert min(ps) < 0.04 and max(ps) > 0.11  # calm and abort phases both present
