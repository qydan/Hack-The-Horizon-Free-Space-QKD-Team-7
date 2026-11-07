import csv

import numpy as np
import pytest

from camera_provider import (ACQUIRE, SEARCH, TRACK, Calibration, LEDDetector, OpenCVCameraProvider,
                             PATTracker, SyntheticLEDSource, demo_flight_scene)
from telemetry import CSVReplayProvider, TelemetryExhausted

NOMINAL = (400.0, 260.0)


def static_scene(x=NOMINAL[0], y=NOMINAL[1], b=1.0, appear=0.0, vanish=1e9):
    return lambda t: (x, y, b) if appear <= t < vanish else None


def make_tracker(cal=None, **kw):
    cal = cal or Calibration(NOMINAL[0], NOMINAL[1], i_ref=1.0)
    return PATTracker(LEDDetector(), cal, **kw)


def run(tracker, source):
    samples = []
    while (item := source.read()) is not None:
        samples.append(tracker.update(*item))
    return samples


class TestDetector:
    def test_finds_centroid_subpixel(self):
        src = SyntheticLEDSource(static_scene(300.4, 200.7), duration=0.1, noise=3)
        frame, _ = src.read()
        d = LEDDetector().detect(frame)
        assert d.x == pytest.approx(300.4, abs=0.3) and d.y == pytest.approx(200.7, abs=0.3)

    def test_no_led_returns_none(self):
        frame, _ = SyntheticLEDSource(lambda t: None, duration=0.1).read()
        assert LEDDetector().detect(frame) is None

    def test_ignores_red_blob(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        frame[200:206, 300:306, 2] = 255  # red in BGR
        assert LEDDetector().detect(frame) is None

    def test_roi_limits_search(self):
        frame, _ = SyntheticLEDSource(static_scene(100, 100), duration=0.1).read()
        assert LEDDetector().detect(frame, roi=(300, 300, 500, 450)) is None
        assert LEDDetector().detect(frame, roi=(50, 50, 150, 150)) is not None

    def test_coordinates_are_resolution_independent(self):
        small = SyntheticLEDSource(static_scene(320, 240), 640, 480, duration=0.1).read()[0]
        big = SyntheticLEDSource(static_scene(640, 480), 1280, 960, duration=0.1, sigma=5).read()[0]
        a, b = LEDDetector().detect(small), LEDDetector().detect(big)
        assert a.x == pytest.approx(b.x, abs=0.5) and a.y == pytest.approx(b.y, abs=0.5)

    def test_intensity_scales_with_brightness(self):
        bright = LEDDetector().detect(SyntheticLEDSource(static_scene(b=1.0), duration=0.1, noise=2).read()[0])
        dim = LEDDetector().detect(SyntheticLEDSource(static_scene(b=0.5), duration=0.1, noise=2).read()[0])
        assert dim.intensity < bright.intensity


class TestTracker:
    def test_acquires_quickly_once_visible(self):
        tr = make_tracker()
        run(tr, SyntheticLEDSource(static_scene(appear=2.0), duration=5.0))
        assert tr.state == TRACK
        assert tr.acquisition_time == pytest.approx(2.0, abs=0.2)   # appears at 2 s, locks in ~5 frames
        assert tr.acquisition_time < 60

    def test_state_progression(self):
        tr = make_tracker(acquire_frames=5)
        src = SyntheticLEDSource(static_scene(appear=0.1), duration=1.0)
        states = [s.state for s in run(tr, src)]
        assert states[0] == SEARCH and ACQUIRE in states and states[-1] == TRACK

    def test_metrics_at_boresight_are_small(self):
        tr = make_tracker()
        samples = run(tr, SyntheticLEDSource(static_scene(), duration=2.0, noise=3))
        locked = [s for s in samples if s.locked]
        assert np.mean([s.drift for s in locked]) < 0.5
        assert np.mean([s.jitter for s in locked]) < 5

    def test_drift_matches_offset(self):
        tr = make_tracker()
        samples = run(tr, SyntheticLEDSource(static_scene(NOMINAL[0] + 30, NOMINAL[1] + 40), duration=2.0))
        assert samples[-1].drift == pytest.approx(50.0, abs=0.7)   # 3-4-5 triangle

    def test_jitter_matches_known_motion(self):
        # LED alternates +-2 px each frame at 60 fps -> speed 4 px * 60 = 240 px/s
        flip = lambda t: (NOMINAL[0] + (2 if int(round(t * 60)) % 2 else -2), NOMINAL[1], 1.0)
        tr = make_tracker()
        samples = run(tr, SyntheticLEDSource(flip, duration=1.5))
        j = np.mean([s.jitter for s in samples[-30:]])
        assert j == pytest.approx(240, rel=0.1)

    def test_lost_signal_reports_zero_intensity_then_reacquires(self):
        tr = make_tracker()
        samples = run(tr, SyntheticLEDSource(static_scene(vanish=2.0) if False else
                                             (lambda t: None if 2.0 <= t < 3.0 else (NOMINAL[0], NOMINAL[1], 1.0)),
                                             duration=5.0))
        gap = [s for s in samples if 2.5 <= s.t < 3.0]
        assert all(s.intensity == 0.0 and not s.locked for s in gap)
        assert tr.lock_losses == 1 and len(tr.acquisition_times) == 2
        assert samples[-1].state == TRACK and samples[-1].locked

    def test_short_dropout_keeps_lock(self):
        tr = make_tracker(lost_frames=15)
        run(tr, SyntheticLEDSource(lambda t: None if 2.0 <= t < 2.1 else (NOMINAL[0], NOMINAL[1], 1.0), duration=4.0))
        assert tr.lock_losses == 0

    def test_follows_moving_led_within_roi(self):
        scene = lambda t: (200 + 40 * t, 240.0, 1.0)   # 40 px/s drift
        tr = make_tracker(cal=Calibration(320, 240, 1.0))
        samples = run(tr, SyntheticLEDSource(scene, duration=5.0))
        assert samples[-1].locked and tr.lock_losses == 0

    def test_intensity_is_normalised_to_reference(self):
        d = LEDDetector().detect(SyntheticLEDSource(static_scene(), duration=0.1).read()[0])
        tr = make_tracker(cal=Calibration(*NOMINAL, i_ref=d.intensity))
        samples = run(tr, SyntheticLEDSource(static_scene(b=1.0), duration=1.0))
        assert samples[-1].intensity == pytest.approx(1.0, abs=0.05)
        tr2 = make_tracker(cal=Calibration(*NOMINAL, i_ref=d.intensity))
        samples2 = run(tr2, SyntheticLEDSource(static_scene(b=0.5), duration=1.0))
        assert samples2[-1].intensity < 0.75


class TestProviderAndCalibration:
    def test_calibration_recovers_boresight(self):
        tr = make_tracker(cal=Calibration(0, 0, 1.0))
        prov = OpenCVCameraProvider(SyntheticLEDSource(static_scene(), duration=20.0), tr)
        cal = prov.calibrate(seconds=3.0)
        assert cal.boresight_x == pytest.approx(NOMINAL[0], abs=0.5)
        assert cal.boresight_y == pytest.approx(NOMINAL[1], abs=0.5)
        assert cal.i_ref > 0

    def test_calibration_roundtrip(self, tmp_path):
        cal = Calibration(1.5, 2.5, 3.5)
        cal.save(str(tmp_path / "c.json"))
        assert Calibration.load(str(tmp_path / "c.json")) == cal

    def test_calibration_without_detections_raises(self):
        prov = OpenCVCameraProvider(SyntheticLEDSource(lambda t: None, duration=1.0), make_tracker())
        with pytest.raises(ValueError):
            prov.calibrate(1.0)

    def test_windows_and_exhaustion(self):
        prov = OpenCVCameraProvider(SyntheticLEDSource(static_scene(), duration=3.0), make_tracker(), window_s=1.0)
        windows = []
        while True:
            try:
                windows.append(prov.get_metrics())
            except TelemetryExhausted:
                break
        assert len(windows) == 3
        for w in windows:
            assert set(w) == {"drift", "jitter", "intensity"}

    def test_record_csv_is_replayable(self, tmp_path):
        path = str(tmp_path / "rec.csv")
        prov = OpenCVCameraProvider(SyntheticLEDSource(static_scene(), duration=3.0), make_tracker(),
                                    window_s=1.0, record_path=path)
        while True:
            try:
                prov.get_metrics()
            except TelemetryExhausted:
                break
        prov.close()
        replay = CSVReplayProvider(path, window_s=1.0)
        assert replay.duration == pytest.approx(3.0, abs=0.1)
        m = replay.get_metrics()
        assert 0 <= m["intensity"] <= 1

    def test_demo_flight_full_run_acquires_and_degrades_in_gust(self):
        from qkd_core import QuantumCryptoEngine
        cal = Calibration(NOMINAL[0], NOMINAL[1], i_ref=LEDDetector().detect(
            SyntheticLEDSource(static_scene(), duration=0.1, noise=3).read()[0]).intensity)
        tr = PATTracker(LEDDetector(), cal)
        prov = OpenCVCameraProvider(SyntheticLEDSource(demo_flight_scene(NOMINAL), duration=120.0,
                                                       fps=30), tr, window_s=1.0)
        e = QuantumCryptoEngine()
        ps = []
        while True:
            try:
                ps.append(e.calculate_channel_error(**prov.get_metrics()))
            except TelemetryExhausted:
                break
        assert tr.acquisition_time is not None and tr.acquisition_time < 60
        assert len(ps) == 120
        calm = np.mean(ps[10:40])
        gust = np.max(ps[82:90])
        assert calm < 0.06 and gust > 0.11
