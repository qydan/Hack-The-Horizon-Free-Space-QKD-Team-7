"""The ground tracker on the simulated rig (no hardware)."""
import numpy as np
import pytest

from gimbal import SimGimbal
from sim import Phase, SimRig
from telemetry import COAST, FINE_TRACK, SEARCH
from tracker import Calibration, GroundTracker
from vision import CameraModel

CAM = CameraModel(480, 270, 70.0)


def make(phases, duration, appear_at=2.0, seed=0):
    rig = SimRig(CAM, SimGimbal(), phases=phases, duration=duration, appear_at=appear_at, seed=seed)
    tr = GroundTracker(CAM, rig.gimbal, pose_fn=rig.pose)
    crig = SimRig(CAM, SimGimbal(), phases=[Phase(0, 1e9, 2, 0.2, 0.5)], duration=15, appear_at=0.0, seed=5)
    tr.cal = GroundTracker(CAM, crig.gimbal).calibrate(crig, 12.0)
    return tr, rig


@pytest.fixture(scope="module")
def calm_run():
    tr, rig = make([Phase(0, 1e9, 3, 0.3, 0.5)], 20.0)
    return tr, rig, list(tr.run(rig))


@pytest.mark.slow
def test_acquires_the_drone_quickly_after_it_appears(calm_run):
    tr, _, msgs = calm_run
    assert tr.acquisition_time is not None and tr.acquisition_time < 60      # D1
    first_lock = next(m.t for m in msgs if m.state == FINE_TRACK)
    assert 2.0 <= first_lock < 5.0                      # not fooled by the lamps before the drone appears
    assert all(m.state == SEARCH for m in msgs if m.t < 2.0)


def test_holds_lock_in_calm_hover(calm_run):
    tr, _, msgs = calm_run
    assert all(m.state == FINE_TRACK for m in msgs if m.t > 5) and tr.lock_losses == 0


def test_calm_measurements_are_small_and_bright(calm_run):
    _, _, msgs = calm_run
    after = [m for m in msgs if m.t > 6]
    assert np.mean([m.residual_mrad for m in after]) < 2.0
    assert np.mean([m.intensity for m in after]) > 0.9
    assert np.mean([m.jitter_mrad for m in after]) < 2.0
    assert all(m.range_m == pytest.approx(3.0, rel=0.1) for m in after)


def test_drift_is_measured_from_the_ground_only(calm_run):
    _, _, msgs = calm_run
    assert 0.2 < np.mean([m.drift_mrad for m in msgs if m.t > 6]) < 12


@pytest.mark.slow
def test_measured_motion_grows_with_wind():
    phases = [Phase(0, 15, 3, 0.3, 0.5), Phase(15, 1e9, 25, 3.0, 3.5)]
    tr, rig = make(phases, 30.0)
    msgs = list(tr.run(rig))
    calm = [m for m in msgs if 6 < m.t < 15]
    windy = [m for m in msgs if 20 < m.t < 30 and m.state == FINE_TRACK]
    assert np.mean([m.drift_mrad for m in windy]) > 2 * np.mean([m.drift_mrad for m in calm])
    assert np.mean([m.jitter_mrad for m in windy]) > 2 * np.mean([m.jitter_mrad for m in calm])
    assert np.mean([m.residual_mrad for m in windy]) > np.mean([m.residual_mrad for m in calm])
    assert np.mean([m.lean_deg for m in windy]) > np.mean([m.lean_deg for m in calm]) + 1.5


@pytest.mark.slow
def test_survives_a_short_beacon_dropout_by_coasting():
    phases = [Phase(0, 8, 3, 0.3, 0.5), Phase(8, 8.15, 3, 0.3, 0.5, brightness=0.0), Phase(8.15, 1e9, 3, 0.3, 0.5)]
    tr, rig = make(phases, 12.0)
    msgs = list(tr.run(rig))
    assert COAST in [m.state for m in msgs if 8.0 < m.t < 8.4] and tr.lock_losses == 0
    assert all(m.state == FINE_TRACK for m in msgs if m.t > 10)


@pytest.mark.slow
def test_long_loss_returns_to_search_then_reacquires():
    phases = [Phase(0, 8, 3, 0.3, 0.5), Phase(8, 10, 3, 0.3, 0.5, brightness=0.0), Phase(10, 1e9, 3, 0.3, 0.5)]
    tr, rig = make(phases, 16.0)
    msgs = list(tr.run(rig))
    assert any(m.state == SEARCH and 8 < m.t < 10.5 for m in msgs)
    assert tr.lock_losses >= 1 and len(tr.acquisition_times) >= 2
    assert all(m.state == FINE_TRACK for m in msgs if m.t > 14)


@pytest.mark.slow
def test_reacquire_command_drops_the_lock_and_finds_it_again():
    tr, rig = make([Phase(0, 1e9, 3, 0.3, 0.5)], 12.0)
    out = []
    for m in tr.run(rig):
        out.append(m)
        if abs(m.t - 6.0) < 1e-3:
            tr.reacquire()
    assert tr.lock_losses == 1 and len(tr.acquisition_times) == 2
    assert all(m.state == FINE_TRACK for m in out if m.t > 9)


def test_calibration_round_trip(tmp_path):
    cal = Calibration(1.5, -2.0, 400.0)
    cal.save(str(tmp_path / "c.json"))
    assert Calibration.load(str(tmp_path / "c.json")) == cal


def test_calibration_fails_loudly_without_a_beacon():
    rig = SimRig(CAM, SimGimbal(), duration=3.0, appear_at=99.0)
    with pytest.raises(RuntimeError, match="never tracked"):
        GroundTracker(CAM, rig.gimbal).calibrate(rig, 2.0)


def test_sim_gimbal_has_lag_and_steps():
    g = SimGimbal(latency_s=0.03)
    g.command(100.0, 0.0)
    for _ in range(2):
        g.step_time(0.01)
    assert g.angles()[0] == 0.0                         # still inside the control latency
    for _ in range(100):
        g.step_time(0.01)
    assert g.angles()[0] == pytest.approx(100.0, abs=g.step)
    assert g.angles()[0] / g.step == pytest.approx(round(g.angles()[0] / g.step))   # servo stops on steps


# ---------------------------------------------------------------- fixed camera (no gimbal)

class _Still(SimGimbal):
    """A camera that never moves: ignores every command (stands in for FixedGimbal in the sim)."""

    def command(self, pan, tilt):
        pass


def _fixed_run(fps, blink_hz, phases, duration, virtual_mount=False):
    from tracker import TrackerConfig
    cam = CameraModel(480, 270, 70.0)
    s = fps / 120.0
    cfg = TrackerConfig(fps=fps, blink_hz=blink_hz, fixed_camera=True, acquire_frames=max(3, round(6 * s)),
                        coast_frames=max(12, round(72 * s)), virtual_mount=virtual_mount)
    rig = SimRig(cam, _Still(), phases=phases, duration=duration, fps=fps, blink_hz=blink_hz,
                 appear_at=2.0, start_offset=(0.0, 0.0), seed=3)
    tr = GroundTracker(cam, rig.gimbal, config=cfg)
    msgs = list(tr.run(rig))
    return tr, msgs


@pytest.mark.parametrize("fps,blink", [(120, 13.0), (60, 13.0), (30, 7.5)])
def test_fixed_camera_acquires_and_holds_lock(fps, blink):
    tr, msgs = _fixed_run(fps, blink, [Phase(0, 1e9, 6, 0.4, 0.5)], 14.0)
    assert tr.acquisition_time is not None and tr.acquisition_time < 6.0, tr.acquisition_time
    late = [m for m in msgs if m.t > 8.0]
    assert sum(m.state == FINE_TRACK for m in late) > 0.9 * len(late)


def test_virtual_mount_leaves_only_the_fast_motion_as_residual():
    phases = [Phase(0, 1e9, 20, 0.4, 0.5)]
    _, bare = _fixed_run(30, 7.5, phases, 20.0)
    tr, mounted = _fixed_run(30, 7.5, phases, 20.0, virtual_mount=True)
    mean = lambda ms: np.mean([m.residual_mrad for m in ms if m.t > 8.0 and m.state == FINE_TRACK])
    assert mean(mounted) < 0.6 * mean(bare)         # the mount follows the slow wander away
    assert mean(mounted) > 0.2                      # but its lag and steps leave a real residual
    assert mounted[-1].drift_mrad == pytest.approx(bare[-1].drift_mrad, abs=2.0)   # drift is measured the same way


# ---------------------------------------------------------------- steady (non-blinking) beacon

def _steady_run(distractors, fps=30, duration=14.0):
    from tracker import TrackerConfig
    cam = CameraModel(480, 270, 70.0)
    s = fps / 120.0
    cfg = TrackerConfig(fps=fps, fixed_camera=True, steady_beacon=True, min_amplitude=40.0,
                        acquire_frames=max(3, round(6 * s)), coast_frames=max(12, round(72 * s)))
    rig = SimRig(cam, _Still(), phases=[Phase(0, 1e9, 6, 0.4, 0.5)], duration=duration, fps=fps,
                 appear_at=2.0, start_offset=(0.0, 0.0), seed=3, steady_beacon=True, distractors=distractors)
    tr = GroundTracker(cam, rig.gimbal, config=cfg)
    msgs = list(tr.run(rig))
    return tr, rig, msgs


@pytest.mark.parametrize("fps", [30, 60])
def test_steady_beacon_in_a_dark_scene_is_acquired_and_followed(fps):
    tr, rig, msgs = _steady_run(distractors=False, fps=fps)
    assert tr.acquisition_time is not None and tr.acquisition_time < 5.0, tr.acquisition_time
    late = [m for m in msgs if m.t > 8.0]
    assert sum(m.state == FINE_TRACK for m in late) > 0.9 * len(late)
    assert np.mean([m.intensity for m in late]) > 0.0


def test_steady_beacon_with_the_virtual_mount_matches_the_rig_setup():
    """The real rig: 30 fps, constantly-lit LED, virtual mount. Residual stays small while drift is measured."""
    from tracker import TrackerConfig
    cam = CameraModel(480, 270, 70.0)
    rig = SimRig(cam, _Still(), phases=[Phase(0, 1e9, 20, 0.4, 0.5)], duration=20.0, fps=30, appear_at=2.0,
                 start_offset=(0.0, 0.0), seed=3, steady_beacon=True)
    cfg = TrackerConfig(fps=30, fixed_camera=True, steady_beacon=True, min_amplitude=40.0, acquire_frames=3,
                        coast_frames=18, virtual_mount=True)
    tr = GroundTracker(cam, rig.gimbal, config=cfg)
    msgs = [m for m in tr.run(rig) if m.t > 8.0 and m.state == FINE_TRACK]
    assert len(msgs) > 200
    assert np.mean([m.residual_mrad for m in msgs]) < 3.0
    assert np.mean([m.drift_mrad for m in msgs]) > 3.0


def test_steady_beacon_does_not_lock_before_the_drone_appears():
    tr, rig, msgs = _steady_run(distractors=False)
    assert all(m.state == SEARCH for m in msgs if m.t < 2.0)


def test_search_zone_keeps_a_window_light_from_stealing_the_lock():
    from tracker import TrackerConfig
    cam = CameraModel(480, 270, 70.0)
    rig = SimRig(cam, _Still(), phases=[Phase(0, 1e9, 6, 0.4, 0.5)], duration=14.0, fps=30, appear_at=2.0,
                 start_offset=(0.0, 0.0), seed=3, steady_beacon=True, distractors=True)
    # drone hovers near the middle; the window/lamp are off to the sides
    cfg = TrackerConfig(fps=30, fixed_camera=True, steady_beacon=True, min_amplitude=40.0, acquire_frames=3,
                        coast_frames=18, search_zone=(0.3, 0.1, 0.7, 0.9))
    tr = GroundTracker(cam, rig.gimbal, config=cfg)
    msgs = list(tr.run(rig))
    assert all(m.state == SEARCH for m in msgs if m.t < 2.0)
    assert tr.acquisition_time is not None and 1.9 < tr.acquisition_time + 0.0 < 5.0
    late = [m for m in msgs if m.t > 8.0]
    assert sum(m.state == FINE_TRACK for m in late) > 0.9 * len(late)


def test_search_zone_keeps_a_window_light_from_stealing_the_lock():
    from tracker import TrackerConfig
    cam = CameraModel(480, 270, 70.0)
    rig = SimRig(cam, _Still(), phases=[Phase(0, 1e9, 6, 0.4, 0.5)], duration=14.0, fps=30, appear_at=2.0,
                 start_offset=(0.0, 0.0), seed=3, steady_beacon=True, distractors=True)
    # the drone hovers near the middle; the window light (x = 0.62 of the frame) and the lamp
    # (x = 0.34) are off to the sides, so the zone has to be narrower than 0.34..0.62
    cfg = TrackerConfig(fps=30, fixed_camera=True, steady_beacon=True, min_amplitude=40.0, acquire_frames=3,
                        coast_frames=18, search_zone=(0.4, 0.1, 0.6, 0.9))
    tr = GroundTracker(cam, rig.gimbal, config=cfg)
    msgs = list(tr.run(rig))
    assert all(m.state == SEARCH for m in msgs if m.t < 2.0)
    assert tr.acquisition_time is not None and 1.9 < tr.acquisition_time < 5.0
    late = [m for m in msgs if m.t > 8.0]
    assert sum(m.state == FINE_TRACK for m in late) > 0.9 * len(late)


# ---------------------------------------------------------------- steady beacon in a lit room

def _bright_run(spot_color, duration=12.0, **rig_kw):
    """Fixed camera, lit colour scene (ambient light, blown-out window, a white lamp brighter than the LED
    20 px from the drone, a red poster). Returns (tracker, rig, per-frame tracking error in mrad)."""
    from tracker import TrackerConfig
    cam = CameraModel(480, 270, 70.0)
    kw = dict(phases=[Phase(0, 1e9, 6, 0.4, 0.5)], duration=duration, fps=30, appear_at=2.0,
              start_offset=(0.0, 0.0), seed=3, steady_beacon=True)
    kw.update(rig_kw)
    rig = SimRig(cam, _Still(), **kw)
    cfg = TrackerConfig(fps=30, fixed_camera=True, steady_beacon=True, min_amplitude=40.0, acquire_frames=3,
                        coast_frames=18, spot_color=spot_color)
    tr = GroundTracker(cam, rig.gimbal, config=cfg)
    errs, states = [], []
    while (item := rig.read()) is not None:
        msg = tr.update(*item)
        states.append((msg.t, msg.state))
        if msg.state == FINE_TRACK:
            errs.append(float(np.hypot(*(np.array(tr.kf.position) - np.array(rig.drone)))))
    return tr, states, errs


def test_red_steady_detector_tracks_the_led_in_a_lit_room_without_a_zone():
    tr, states, errs = _bright_run("red", bright_scene=True)
    assert all(s == SEARCH for t, s in states if t < 2.0)            # nothing LED-like before the drone appears
    assert tr.acquisition_time is not None and tr.acquisition_time < 3.0
    late = [s for t, s in states if t > 5.0]
    assert sum(s == FINE_TRACK for s in late) > 0.95 * len(late)
    assert np.median(errs) < 1.0 and max(errs) < 5.0                  # on the LED, never the lamp (~60 mrad off)


def test_brightness_only_detector_is_stolen_by_the_lamp_in_a_lit_room():
    """The reason for the colour mode: on brightness alone the clipped white lamp wins."""
    tr, states, errs = _bright_run("gray", bright_scene=True)
    assert errs and np.median(errs) > 20.0


def test_clutter_learning_keeps_room_lights_from_stealing_the_lock_without_a_zone():
    """Grey frames (no colour to help): learn the window and the mains lamp with the beacon not yet in view."""
    from tracker import TrackerConfig
    cam = CameraModel(480, 270, 70.0)
    rig = SimRig(cam, _Still(), phases=[Phase(0, 1e9, 6, 0.4, 0.5)], duration=12.0, fps=30, appear_at=2.0,
                 start_offset=(0.0, 0.0), seed=3, steady_beacon=True, distractors=True)
    cfg = TrackerConfig(fps=30, fixed_camera=True, steady_beacon=True, min_amplitude=40.0, acquire_frames=3,
                        coast_frames=18)
    tr = GroundTracker(cam, rig.gimbal, config=cfg)
    assert tr.learn_clutter(rig, 1.5) == 2                           # the window light and the mains lamp
    errs = []
    while (item := rig.read()) is not None:
        if tr.update(*item).state == FINE_TRACK:
            errs.append(float(np.hypot(*(np.array(tr.kf.position) - np.array(rig.drone)))))
    assert tr.acquisition_time is not None
    assert len(errs) > 200 and np.median(errs) < 1.0


def test_red_steady_detector_tracks_the_led_in_a_squash_court():
    """Red court lines on a white wall (one running right behind the hover point), plus the lit-room lamp,
    window and red poster."""
    tr, states, errs = _bright_run("red", bright_scene=True, court_lines=True)
    assert all(s == SEARCH for t, s in states if t < 2.0)
    assert tr.acquisition_time is not None and tr.acquisition_time < 3.0
    late = [s for t, s in states if t > 5.0]
    assert sum(s == FINE_TRACK for s in late) > 0.95 * len(late)
    assert np.median(errs) < 1.0 and max(errs) < 5.0
