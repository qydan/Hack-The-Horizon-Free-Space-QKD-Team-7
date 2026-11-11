"""Offline tests of tracking/ (ported from the team's tests/test_tracker.py and tests/test_vision.py).

The tracker runs closed loop against tests/synthetic_rig.py: synthetic hover scenes pushed through the real
Action 5 Pro emulation (auto-exposure, 150 ms latency, principal-point offset) and, for rig_gimbal, the real
STorM32 dynamics. Ground truth from the rig scores the measurements, as evaluation/score.py does in Gazebo.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from synthetic_rig import Phase, SceneOptions, SyntheticRig  # noqa: E402
from telemetry.bus import validate_telemetry  # noqa: E402
from tracking.config import load_tracking_cfg  # noqa: E402
from tracking.main import run  # noqa: E402
from tracking.telemetry_out import build_record  # noqa: E402


class FakeOut:
    """TelemetryOut stand-in: validates every record against telemetry/schema.json and can inject a gate abort."""

    def __init__(self, rig: str, abort_at: float | None = None):
        self.rig, self.recs, self.sent, self.abort_at = rig, [], 0, abort_at
        self.last_t = 0.0

    def publish(self, fields):
        rec = build_record(fields, self.rig)
        validate_telemetry(rec)
        self.recs.append(rec)
        self.sent += 1
        self.last_t = rec["t"]
        return rec

    def poll_abort(self):
        if self.abort_at is not None and self.last_t >= self.abort_at:
            self.abort_at = None
            return {"cmd": "reacquire", "reason": "test abort"}
        return None


def fly(rig_name: str, duration: float, scene: SceneOptions, abort_at: float | None = None, seed: int = 0,
        cfg: dict | None = None):
    """Fly the synthetic scene. The tracker's beacon mode is set from the scene explicitly (beacon.yaml now
    defaults to a steady LED; these scenes blink unless ``scene.steady``)."""
    cfg = cfg or load_tracking_cfg()
    cfg["acquire"]["beacon"] = "steady" if scene.steady else "blink"
    rig = SyntheticRig(rig_name, duration_s=duration, scene=scene, seed=seed)
    out = FakeOut(rig_name, abort_at)
    tr = run(rig_name, cfg, source=rig, gimbal=rig.gimbal, out=out, stop=lambda: False)
    return tr, rig, out.recs


def err_vs_truth(rig, recs, axis: str) -> np.ndarray:
    return np.array([r[f"err_{axis}_urad"] * 1e-3 - 1e3 * rig.truth_at(r["t"])[f"err_{axis}_rad"] for r in recs])


def window(recs, t0, t1, state=None):
    return [r for r in recs if t0 <= r["t"] < t1 and (state is None or r["state"] == state)]


# --------------------------------------------------------------------------- the main fixed-rig flight
@pytest.fixture(scope="module")
def fixed_flight():
    """14 s: calm hover 0-7 s, windy 7-14 s, a gate abort at 10.5 s. Drone starts 0.6 m off the station;
    the steady red lamp, 2 Hz indicator and 100 Hz flickering panel are in view."""
    scene = SceneOptions(phases=[Phase(0, 7, 0.02, 0.001), Phase(7, 1e9, 0.05, 0.006)], start_offset_m=(0.6, 0.2))
    return fly("rig_fixed", 14.0, scene, abort_at=10.5)


def test_acquires_within_a_second_despite_distractors(fixed_flight):
    tr, rig, recs = fixed_flight
    assert tr.acquisition_time is not None and tr.acquisition_time < 1.5     # lock-in window 0.67 s + 4 frames
    assert any(r["state"] == "ACQUIRE" for r in recs)                         # schema state between SEARCH and TRACK


def test_holds_lock_and_measures_the_beacon_not_a_distractor(fixed_flight):
    tr, rig, recs = fixed_flight
    tracked = window(recs, 1.5, 10.5)
    assert sum(r["state"] == "TRACK" for r in tracked) / len(tracked) > 0.95
    t = window(recs, 1.5, 10.5, "TRACK")
    for axis in ("x", "y"):
        e = err_vs_truth(rig, t, axis)
        # constant bias = the deliberate mount error + principal-point offset the nominal lens can't know
        assert abs(e.mean()) < 20.0
        assert e.std() < 1.5, f"err_{axis} scatter {e.std():.2f} mrad vs truth"


def test_every_record_is_schema_valid_with_sim_timing(fixed_flight):
    _, _, recs = fixed_flight
    assert recs and all(r["rig"] == "rig_fixed" for r in recs)
    lat = [r["latency_ms"] for r in recs]
    assert 140 <= np.median(lat) <= 200                    # camera_action5.yaml latency 150 ms (+ frame quantisation)
    assert np.all(np.diff([r["t"] for r in recs]) > 0)


def test_spot_pixel_is_published_for_the_accuracy_overlay(fixed_flight):
    """spot_u_px / spot_v_px (optional schema fields): the centroid used each TRACK frame, in full-resolution
    pixels of the delivered frame (it includes the camera's principal-point offset, unlike err_x / err_y); null
    while searching. tools/accuracy_overlay.py matches records to frames on t = t_capture_sim."""
    _, rig, recs = fixed_flight
    assert all(r["spot_u_px"] is None for r in recs if r["state"] == "SEARCH")
    t = window(recs, 3, 7, "TRACK")
    assert all(r["spot_u_px"] is not None for r in t)
    pp = rig.cfg_rig["principal_point_offset_px"]
    du = []
    for r in t:
        tr = rig.truth_at(r["t"])                      # true pixel = lens projection + the pipeline's pp shift
        u_true = rig.lens.project(rig._optical(tr["drone"], rig.camera_rotation()))[0][0] + pp[0]
        du.append(r["spot_u_px"] - u_true)
    assert abs(np.median(du)) < 1.0 and np.std(du) < 1.0                # sub-pixel against truth


def test_virtual_receiver_residual_and_lag(fixed_flight):
    _, _, recs = fixed_flight
    calm = window(recs, 3, 7, "TRACK")
    assert np.median([r["residual_mrad"] for r in calm]) < 6.0
    assert all(r["gimbal_lag_mrad"] is not None for r in calm)          # the virtual mount's lag
    assert all(r["gimbal_pan_deg"] == recs[0]["gimbal_pan_deg"] for r in recs)   # rig_fixed: constant


def test_measured_motion_grows_with_wind(fixed_flight):
    _, _, recs = fixed_flight
    calm, windy = window(recs, 3, 7, "TRACK"), window(recs, 8, 10.5, "TRACK")
    assert np.median([r["jitter_mrad"] for r in windy]) > 1.5 * np.median([r["jitter_mrad"] for r in calm])
    assert np.median([r["residual_mrad"] for r in windy]) > np.median([r["residual_mrad"] for r in calm])


def test_gate_abort_drops_the_lock_and_it_is_found_again(fixed_flight):
    tr, _, recs = fixed_flight
    after = window(recs, 10.5, 14)
    assert after[0]["state"] in ("SEARCH", "ACQUIRE") or after[1]["state"] in ("SEARCH", "ACQUIRE")
    assert tr.aborts == 1 and len(tr.acquisition_times) == 2
    assert tr.acquisition_times[1] < 1.0                   # the lock-in window is still full: quick re-lock
    assert window(recs, 12, 14, "TRACK")


def test_intensity_is_referenced_and_exposure_gain_estimated(fixed_flight):
    _, rig, recs = fixed_flight
    calm = window(recs, 5, 7, "TRACK")
    assert 0.7 < np.median([r["intensity"] for r in calm]) <= 1.0
    est = np.array([r["exposure_gain"] for r in recs[5:]])
    true = np.array([rig.true_gain[round(r["t"], 6)] for r in recs[5:]])
    assert np.max(np.abs(est / true - 1.0)) < 0.05         # replaying the AE law recovers the gain


# --------------------------------------------------------------------------- dropouts
@pytest.fixture(scope="module")
def dropout_flight():
    """Beacon appears at 2 s; hand-sized occluder for 0.3 s at 5 s; beacon hidden 2.5 s from 8 s."""
    scene = SceneOptions(appear_at_s=2.0, occlusions=[(5.0, 5.3), (8.0, 10.5)])
    return fly("rig_fixed", 13.0, scene)


def test_no_lock_on_distractors_before_the_beacon_appears(dropout_flight):
    _, _, recs = dropout_flight
    assert not window(recs, 0, 2.0, "TRACK")


def test_short_occlusion_coasts_without_losing_lock(dropout_flight):
    _, _, recs = dropout_flight
    during = window(recs, 5.0, 5.4)
    assert any(r["state"] == "COAST" for r in during)
    assert all(r["state"] in ("TRACK", "COAST") for r in window(recs, 3.5, 7.9))
    assert all(r["intensity"] == 0.0 for r in during if r["state"] == "COAST")     # no beacon light: no photons


def test_long_loss_returns_to_search_then_reacquires(dropout_flight):
    tr, _, recs = dropout_flight
    assert any(r["state"] == "SEARCH" for r in window(recs, 8.6, 10.5))
    assert tr.lock_losses == 1
    assert window(recs, 11.5, 13, "TRACK")


# --------------------------------------------------------------------------- rig_gimbal
@pytest.fixture(scope="module")
def gimbal_flight():
    scene = SceneOptions(phases=[Phase(0, 1e9, 0.02, 0.001)], start_offset_m=(0.6, 0.2))
    return fly("rig_gimbal", 9.0, scene, seed=3)


def test_gimbal_slews_onto_the_drone_and_keeps_it_centred(gimbal_flight):
    tr, rig, recs = gimbal_flight
    assert tr.acquisition_time is not None and tr.acquisition_time < 1.5
    target_pan = math.degrees(math.atan2(0.6, 3.0))
    assert abs(recs[-1]["gimbal_pan_deg"] - target_pan) < 3.0             # it turned ~11 deg toward the drone
    late = window(recs, 4, 9, "TRACK")
    assert len(late) > 0.9 * len(window(recs, 4, 9))
    assert np.median([r["residual_mrad"] for r in late]) < 8.0
    for axis in ("x", "y"):
        assert err_vs_truth(rig, late, axis).std() < 1.5


def test_gimbal_lag_is_reported(gimbal_flight):
    _, _, recs = gimbal_flight
    lags = [r["gimbal_lag_mrad"] for r in window(recs, 4, 9, "TRACK")]
    assert all(v is not None and v >= 0 for v in lags) and max(lags) > 0


# --------------------------------------------------------------------------- steady beacon
@pytest.mark.parametrize("zone", [None, [0.43, 0.25, 0.58, 0.75]])
def test_steady_beacon_ignores_the_steady_red_lamp(zone):
    """The default beacon (beacon.yaml modulation off): the bright-spot detector (src/ --beacon steady) locks on
    the small pale-pink LED, not the larger steady red lamp (core size), the redder 2 Hz indicator (it never stays
    lit for steady_persist_frames) or the white glints (redness channel); also with a search_zone."""
    cfg = load_tracking_cfg()
    cfg["acquire"]["search_zone"] = zone
    tr, rig, recs = fly("rig_fixed", 6.0, SceneOptions(steady=True), cfg=cfg)
    assert tr.acquisition_time is not None and tr.acquisition_time < 1.0     # 0.5 s persistence + 4 frames
    t = window(recs, 1.0, 6.0, "TRACK")
    assert len(t) > 0.9 * len(window(recs, 1.0, 6.0))
    for axis in ("x", "y"):
        e = err_vs_truth(rig, t, axis)
        assert abs(e.mean()) < 20.0 and e.std() < 1.5


def test_steady_relock_after_a_gate_abort_is_quick():
    """Gazebo tracking_sweep: 25 of 26 lock losses were gate aborts (I8: drop the lock, re-acquire). The spot we
    were tracking has already proved it stays lit, so re-acquisition re-checks it with acquire_frames only
    (~0.13 s) instead of the full 0.5 s steady persistence."""
    tr, _, recs = fly("rig_fixed", 6.0, SceneOptions(steady=True), abort_at=3.0)
    assert tr.aborts == 1 and len(tr.acquisition_times) == 2
    assert tr.acquisition_times[0] > 0.5                       # the first lock needs the full persistence
    assert tr.acquisition_times[1] < 0.25
    assert sum(r["state"] != "TRACK" for r in window(recs, 3.0, 6.0)) <= 8


def test_virtual_receiver_leads_its_own_servo_lag():
    """A 0.45 m/s sideways sweep: the lead covers frame latency, command lag and the servo's 1/gain velocity lag
    (without the last term the residual was 7.2 mrad; above ~1 m/s the 210 ms prediction of a sine dominates)."""
    tr, _, recs = fly("rig_fixed", 8.0, SceneOptions(steady=True, sweep_mps=0.45, phases=[Phase(0, 1e9, 0.0, 0.0)]))
    t = window(recs, 2, 8, "TRACK")
    assert len(t) > 0.95 * len(window(recs, 2, 8))
    assert np.median([r["residual_mrad"] for r in t]) < 6.0


def test_takeoff_does_not_poison_the_reference():
    """Regression from Gazebo t5: the drone (simple_flight) takes off at t = 0 and the tracker locks while it is
    still on the floor. A reference taken during the climb read as 100-180 mrad of drift for the whole run, the
    error budget saw the transmitter pointing ~8 deg away and the gate aborted every block (37 of 44). The
    boresight is now fixed only on a trend-free hover window and then follows the hover mean: no aborts, keys
    flow, and drift stays near the drone's real wander."""
    from sandbox.bb84 import QuantumCryptoEngine
    from sandbox.gate import Gate
    from sandbox.main import load_cfg
    from sandbox.pipeline import Sandbox

    class Loop(FakeOut):
        """The sandbox in-line on the telemetry, with its aborts fed back to the tracker (ports 5556 / 5557)."""

        def __init__(self):
            super().__init__("rig_fixed")
            cfg = load_cfg()
            self.pending, self.blocks = [], []
            self.sb = Sandbox(cfg, engine=QuantumCryptoEngine(cfg["bb84"], seed=1),
                              gate=Gate(cfg, type("P", (), {"send": lambda _, m: self.pending.append(m)})()))

        def publish(self, fields):
            rec = super().publish(fields)
            self.blocks += [b for b in self.sb.feed(rec) if b.kind == "block"]
            return rec

        def poll_abort(self):
            return self.pending.pop(0) if self.pending else None

    cfg = load_tracking_cfg()
    cfg["acquire"]["beacon"] = "steady"
    rig = SyntheticRig("rig_fixed", duration_s=16.0, scene=SceneOptions(steady=True, takeoff_s=4.0,
                                                                         phases=[Phase(0, 1e9, 0.03, 0.002)]))
    out = Loop()
    tr = run("rig_fixed", cfg, source=rig, gimbal=rig.gimbal, out=out, stop=lambda: False)
    assert tr.acquisition_time < 2.0                                  # it does lock during the climb
    assert tr.motion.reference.t_fixed > 5.0                          # but references only the hover
    assert tr.aborts == 0
    assert sum(b.outcome == "key" for b in out.blocks if b.t > 9) >= 5
    late = window(out.recs, 10, 16, "TRACK")
    hover = [rig.truth_at(r["t"]) for r in late]
    ex = np.array([1e3 * h["err_x_rad"] for h in hover])
    ey = np.array([1e3 * h["err_y_rad"] for h in hover])
    true_wander = np.median(np.hypot(ex - ex.mean(), ey - ey.mean()))
    assert np.median([r["drift_mrad"] for r in late]) < true_wander + 15.0


@pytest.mark.parametrize("steady", [True, False])
def test_bright_office_auto_exposure(steady):
    """A bright room pulls the auto-exposure gain to ~0.5: the pale-pink LED's red then falls below the room's on
    the dark drone body. Both modes still lock and hold (redness channel; lock-in-confirmed low contrast)."""
    tr, rig, recs = fly("rig_fixed", 5.0, SceneOptions(steady=steady, background=150))
    late = window(recs, 2, 5)
    assert np.median([r["exposure_gain"] for r in late]) < 0.7
    assert sum(r["state"] == "TRACK" for r in late) > 0.9 * len(late)
    assert err_vs_truth(rig, [r for r in late if r["state"] == "TRACK"], "x").std() < 1.5


# --------------------------------------------------------------------------- units
@pytest.mark.parametrize("projection,hfov,mrad_px", [("rectilinear", 104.0, 1000 * math.tan(math.radians(52)) / 960),
                                                      ("equidistant", 140.0, 1000 * math.radians(70) / 960)])
def test_camera_angles_round_trip_nominal_and_distorted(projection, hfov, mrad_px):
    """Both lenses camera_action5.yaml offers (rectilinear 104 deg: 1.33 mrad/px; fisheye 140 deg: 1.27)."""
    from tracking.camera import CameraModel
    nom = CameraModel.nominal({"width_px": 1920, "height_px": 1080, "projection": projection, "hfov_deg": hfov})
    assert nom.mrad_per_px == pytest.approx(mrad_px, rel=1e-6)
    D = [0.02, -0.01, 0.003, 0.0] if projection == "equidistant" else [-0.05, 0.01, 0.0, 0.0]
    dist = CameraModel(nom.K, np.array(D), nom.width, nom.height, projection)
    for cam in (nom, dist):
        for u, v in ((960, 540), (100, 900), (1500, 200)):
            ax, ay = cam.pixel_to_angles(u, v)
            assert cam.angles_to_pixel(ax, ay) == pytest.approx((u, v), abs=0.05)
    # a known direction: 10 deg right of the axis lands where the projection says
    u, _ = nom.angles_to_pixel(math.radians(10), 0.0)
    r = math.tan(math.radians(10)) if projection == "rectilinear" else math.radians(10)
    assert u - nom.K[0, 2] == pytest.approx(nom.f_px * r, rel=1e-9)


def test_tracking_uses_the_active_lens():
    from tracking.camera import CameraModel
    cfg = load_tracking_cfg()
    import yaml
    from tracking.config import ROOT
    lens = yaml.safe_load((ROOT / "config" / "camera_action5.yaml").read_text())["lens"]
    cam = CameraModel.from_config(cfg)
    assert cam.projection == lens["projection"] or cam.source == "calibration.yaml"


def test_lockin_rejects_steady_and_mains_flicker():
    """A steady spot and a 100 Hz flicker (10 Hz alias at 30 fps) produce no lock-in candidate; 7 Hz does."""
    from tracking.acquire import LockInDetector
    cfg = load_tracking_cfg()
    cfg["acquire"]["lockin_frequency_hz"] = 7.0
    det = LockInDetector(cfg)
    for k in range(40):
        t = k / 30.0
        img = np.full((270, 480, 3), 60, np.uint8)
        img[50:56, 50:56, 2] = 250                                                 # steady red
        img[100:106, 300:306, 2] = int(150 + 80 * math.cos(2 * math.pi * 100 * t))  # mains lamp
        if math.sin(2 * math.pi * 7 * t) >= 0:
            img[200:206, 200:206, 2] = 250                                         # beacon high
        else:
            img[200:206, 200:206, 2] = 170                                         # beacon low (40% linear)
        det.push((img, t))
    c = det.candidates()
    assert len(c) == 1 and abs(c[0].u_px - 202.5) < 2 and abs(c[0].v_px - 202.5) < 2


def test_exposure_estimator_follows_an_auto_exposure_step():
    """The room light drops at 3 s; the emulated AE raises its gain ~3x. The replayed AE law follows it."""
    from sim_bridge import config as cfgmod
    from sim_bridge.image_pipeline import Action5Pipeline
    from tracking.track import ExposureEstimator
    pipe = Action5Pipeline(cfgmod.load("camera_action5"), "fast")
    est = ExposureEstimator(load_tracking_cfg())
    rng = np.random.default_rng(0)
    pairs = []
    for k in range(300):
        t = k / 30.0
        level = 125 if t < 3.0 else 75
        img = np.clip(level + rng.normal(0, 3, (270, 480, 3)), 0, 255).astype(np.uint8)
        for pf in pipe.push_render(img, t):
            pairs.append((est.update(pf.image_bgr, t), pf.exposure_gain))
    e, g = np.array(pairs).T
    assert g[-1] / g[60] > 2.0                                  # the camera's AE really moved
    assert np.max(np.abs(e / g - 1.0)) < 0.10


def test_tag_pose_recovers_lean_and_range():
    from tracking.camera import CameraModel
    from tracking.pose import pose_from_corners
    cam = CameraModel.nominal(load_tracking_cfg()["camera_nominal"])
    for lean in (0.0, 10.0, 30.0):
        h, rng = 0.04, 1.5
        a = math.radians(lean)
        pts = []
        for x, y in ((-h, h), (h, h), (h, -h), (-h, -h)):          # tag rotated about its vertical axis by `lean`
            p = np.array([x * math.cos(a), -y, rng + x * math.sin(a)])
            th = math.atan2(math.hypot(p[0], p[1]), p[2])
            r = math.hypot(p[0], p[1])
            pts.append(cam.angles_to_pixel(th * p[0] / r, th * p[1] / r))
        pose = pose_from_corners(np.array(pts), 0.08, cam)
        assert pose.range_m == pytest.approx(rng, rel=0.02)
        assert pose.lean_deg == pytest.approx(lean, abs=1.0)


def test_state_machine_transitions():
    from tracking.state_machine import State, StateMachine
    sm = StateMachine({"acquire_frames": 3, "coast_timeout_s": 0.5, "lost_timeout_s": 5.0})
    assert sm.on_event(0.0, "candidate") == State.ACQUIRE
    assert sm.on_event(0.1, "no_candidate") == State.SEARCH
    for k in range(3):
        s = sm.on_event(0.2 + k / 30, "candidate")
    assert s == State.TRACK and sm.locked_now
    assert sm.on_event(0.4, "miss") == State.COAST
    assert sm.on_event(0.5, "measurement") == State.TRACK
    assert sm.on_event(0.8, "miss") == State.COAST
    assert sm.on_event(1.1, "miss") == State.SEARCH and sm.lost_now
    sm.on_event(1.2, "candidate")
    assert sm.on_event(1.3, "abort") == State.SEARCH


def test_tracker_telemetry_feeds_the_sandbox(fixed_flight):
    """End to end without the bus: the tracker's calm-hover records produce keys in the sandbox."""
    from sandbox.bb84 import QuantumCryptoEngine
    from sandbox.main import load_cfg
    from sandbox.pipeline import Sandbox
    cfg = load_cfg()
    _, _, recs = fixed_flight
    sb = Sandbox(cfg, engine=QuantumCryptoEngine(cfg["bb84"], seed=7))
    blocks = [r for r in sb.run(window(recs, 0, 7)) if r.kind == "block"]
    assert blocks[0].outcome == "no_link"                  # searching at the start
    assert sum(b.outcome == "key" for b in blocks) >= 3
