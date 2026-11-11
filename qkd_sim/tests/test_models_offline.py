"""Offline acceptance checks on the pure models (no Gazebo needed).

Covers the model side of A4 (jet + drag -> lean), A6 (modulation through the camera pipeline), A7 (rolling-shutter
skew), A9 (auto-exposure), A10 (visible-beacon validation), A11 (FixedMount), A12 (Storm32 emulation),
A13 (schema). The IR data link (A15) was removed: the rig is a red LED and an RGB camera. The same items are also checked end-to-end in Gazebo by test_sim.py.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from sim_bridge.beacon_modulator import Modulation
from sim_bridge.fan_wind import FanJet, WindField, drag_force, schedule_level, steady_lean_deg
from sim_bridge.gimbal_io import FixedMount, GimbalIO, Storm32GimbalSim
from sim_bridge.image_pipeline import Action5Pipeline, AutoExposure, RollingShutterAssembler, SubFrame

DEG = math.pi / 180


# ------------------------------------------------------------------------------------------- beacon
def test_steady_is_default(cfgs):
    """The real rig's LED is constantly lit: steady unless blink is switched on in beacon.yaml."""
    m = Modulation.from_cfg(cfgs["beacon"]["red_led"]["modulation"])
    assert cfgs["beacon"]["red_led"]["modulation"]["enabled"] is False
    assert {m.level(t) for t in np.arange(0, 2, 0.003)} == {m.high_level}
    assert m.next_edge(0.5) == math.inf


def _blink(cfgs):
    return {**cfgs["beacon"]["red_led"]["modulation"], "enabled": True}


def test_modulation_square_wave(cfgs):
    m = Modulation.from_cfg(_blink(cfgs))
    t = np.arange(0, 10, 1e-4)
    lv = np.array([m.level(x) for x in t])
    assert set(np.round(lv, 6)) == {0.4, 1.0}
    assert abs(lv.mean() - 0.7) < 0.01
    spec = np.abs(np.fft.rfft(lv - lv.mean()))
    f = np.fft.rfftfreq(len(lv), 1e-4)
    assert abs(f[np.argmax(spec)] - 7.0) < 0.05
    # next_edge is consistent with level()
    t0 = 0.0123
    e = m.next_edge(t0)
    assert m.level(e - 1e-6) != m.level(e + 1e-6)
    assert abs(e - 1 / 14) < 1e-9


def test_bare_led_beam_pattern(cfgs):
    """Bare LED (no diffuser): half brightness at the half-intensity angle, nothing from behind, and the
    drone-attitude geometry turns a lean into dimming."""
    from sim_bridge.beacon_modulator import BeamPattern, off_axis_angle
    half = math.radians(cfgs["beacon"]["red_led"]["viewing_half_angle_deg"])
    beam = BeamPattern(half)
    assert beam.factor(0.0) == pytest.approx(1.0)
    assert beam.factor(half) == pytest.approx(0.5, abs=1e-9)
    assert beam.factor(math.radians(95)) == 0.0
    # LED axis along body -x, camera straight behind: on axis; pitch the body 20 deg -> 20 deg off axis
    cam = np.array([-3.0, 0.0, 0.0])
    q0 = (1.0, 0.0, 0.0, 0.0)
    assert off_axis_angle([0, 0, 0], q0, [0, 0, 0], [-1, 0, 0], cam) == pytest.approx(0.0, abs=1e-9)
    a = math.radians(20) / 2
    q_pitch = (math.cos(a), 0.0, math.sin(a), 0.0)
    assert math.degrees(off_axis_angle([0, 0, 0], q_pitch, [0, 0, 0], [-1, 0, 0], cam)) == pytest.approx(20.0, abs=1e-6)
    d = cfgs["beacon"]["red_led"]["drive"]
    assert d["current_a"] * d["forward_voltage_red_v"] == pytest.approx(0.040)


def test_7hz_clear_of_mains_aliases(cfgs):
    from sim_bridge.image_pipeline import nyquist_alias_hz
    fps = cfgs["camera_action5"]["stream"]["fps"]
    f_mod = cfgs["beacon"]["red_led"]["modulation"]["frequency_hz"]
    assert f_mod < fps / 2
    for flick in (100.0, 120.0):
        assert abs(nyquist_alias_hz(flick, fps) - f_mod) > 2.0
    assert nyquist_alias_hz(100.0, 30.0) == pytest.approx(10.0)
    assert nyquist_alias_hz(120.0, 30.0) == pytest.approx(0.0)


# ------------------------------------------------------------------------------------------- fan
def test_jet_profile(cfgs):
    jet = FanJet.from_cfg(cfgs["fan"], cfgs["world"], "high")
    D, u0 = jet.diameter, jet.outlet_speed
    fan = jet.position
    assert np.allclose(jet.mean_velocity(fan + [0, 1.0, 0]), [0, u0, 0])               # inside the core
    assert jet.mean_velocity(fan + [0, 6 * D, 0])[1] == pytest.approx(u0 * 0.5)          # 3D/s decay
    assert np.allclose(jet.mean_velocity(fan + [0, -0.5, 0]), 0)                         # behind the fan
    s = 1.2
    b = D / 2 + 0.1 * s
    assert jet.mean_velocity(fan + [b, s, 0])[1] == pytest.approx(u0 * 0.5)              # half width
    # the drone station sits on the jet axis at s = 1.2 m = 3 D
    st = np.asarray(cfgs["world"]["hover_station_m"])
    assert jet.mean_velocity(st)[1] == pytest.approx(u0)


def test_turbulence_intensity(cfgs):
    jet = FanJet.from_cfg(cfgs["fan"], cfgs["world"], "low")
    wf = WindField(jet, 0.15, 0.3, np.random.default_rng(0))
    p = np.asarray(cfgs["world"]["hover_station_m"])
    v = []
    for _ in range(60000):
        wf.step(0.01)
        v.append(wf.velocity(p))
    v = np.array(v)
    assert v[:, 1].mean() == pytest.approx(jet.outlet_speed, rel=0.05)
    assert v[:, 0].std() / jet.outlet_speed == pytest.approx(0.15, rel=0.1)


def test_drag_and_lean_prediction(cfgs):
    cda = cfgs["drone"]["aero"]["body_cd_area_m2"]
    f = drag_force([4, 0, 0], [0, 0, 0], cda, 1.2)
    assert f[0] == pytest.approx(0.5 * 1.2 * cda * 16)
    lean4 = steady_lean_deg(4.0, cda, 0.246)
    assert lean4 == pytest.approx(4.5, abs=0.15)             # design-doc anchor: 4 m/s -> ~4.5 deg
    assert schedule_level([{"t_s": 0, "level": "off"}, {"t_s": 5, "level": "high"}], 6.0) == "high"


def test_a4_predicted_lean_at_fan_high(cfgs):
    """A4 asks for 3-6 deg at fan high. With the spec's own numbers (5 m/s outlet, drone at 3 D so the full
    outlet speed reaches it, Cd*A 0.02 m^2, 264 g with the beacon) the quasi-static lean is ~6.6 deg, i.e.
    just above the band. Reported in the README; this test documents the predicted value."""
    jet = FanJet.from_cfg(cfgs["fan"], cfgs["world"], "high")
    u = jet.mean_velocity(cfgs["world"]["hover_station_m"])[1]
    lean = steady_lean_deg(u, cfgs["drone"]["aero"]["body_cd_area_m2"], 0.264)
    assert 6.0 < lean < 7.0
    lean_med = steady_lean_deg(4.0, cfgs["drone"]["aero"]["body_cd_area_m2"], 0.264)
    assert 3.0 < lean_med < 6.0


def _fly(cfgs, fan, wander, seconds=90.0):
    from sim_bridge.simple_flight import SimpleFlight
    st = np.array(cfgs["world"]["hover_station_m"])
    fl = SimpleFlight(cfgs["drone"], cfgs["fan"], cfgs["world"], np.array([st[0], st[1], 0.03]), st,
                      fan_schedule=[{"t_s": 0, "level": "off"}, {"t_s": 20, "level": fan}], seed=1, wander=wander)
    out = []
    for k in range(int(seconds / 0.01) + 1):
        fl.step(k * 0.01)
        out.append((k * 0.01, fl.p.copy(), fl.lean_deg(), fl.last_wind.copy()))
    return fl, out


def test_simple_flight_takes_off_and_holds(cfgs):
    """Default flight model (no PX4): climbs to the station and holds it."""
    assert cfgs["drone"]["flight_model"] == "simple"
    _, out = _fly(cfgs, "off", wander=False)
    p = np.array([o[1] for o in out if o[0] > 30])
    assert np.allclose(p.mean(0), cfgs["world"]["hover_station_m"], atol=0.01)
    assert p.std(0).max() < 0.005


def test_simple_flight_leans_into_fan(cfgs):
    """Steady lean = atan(drag / weight) (within 15%, turbulence adds a little); pushed downwind, then recovers."""
    fl, out = _fly(cfgs, "high", wander=False)
    late = [o for o in out if o[0] > 60]
    lean = np.mean([o[2] for o in late])
    wind = abs(np.mean([o[3][1] for o in late]))
    assert lean == pytest.approx(steady_lean_deg(wind, cfgs["drone"]["aero"]["body_cd_area_m2"], fl.mass), rel=0.15)
    y = np.array([o[1][1] for o in out if 20 < o[0] < 40])
    assert y.max() > 0.02                                        # blown +y when the fan comes on
    assert abs(np.mean([o[1][1] for o in late])) < 0.5 * y.max()  # and recovers


def test_hover_wander_stays_in_dji_envelope(cfgs):
    """The wander setpoint never leaves DJI's +-0.3 m / +-0.1 m and keeps roughly the specified spread."""
    from sim_bridge.hover_station import SetpointGenerator
    mx_h, sd = [], []
    for seed in range(40):
        g = SetpointGenerator.from_cfg(cfgs["drone"], np.zeros(3), np.random.default_rng(seed), wander=True)
        w = np.array([g.at(t) for t in np.arange(0, 75, 0.05)])
        mx_h.append(np.hypot(w[:, 0], w[:, 1]).max())
        assert np.abs(w[:, 2]).max() < 0.1
        sd.append(w[:, 0].std())
    assert max(mx_h) < 0.3
    assert 0.05 < np.mean(sd) < 0.12


def test_tilt_quaternion_matches_lean():
    from sim_bridge.simple_flight import tilt_quaternion
    from sim_bridge.truth_publisher import quat_to_rot
    a = 9.81 * math.tan(math.radians(10))
    R = quat_to_rot(tilt_quaternion([0.0, a], 0.0))
    assert math.degrees(math.acos(R[2, 2])) == pytest.approx(10.0, abs=1e-6)
    assert R[1, 2] > 0                                           # thrust tilted toward +y


# ------------------------------------------------------------------------------------------- gimbal
def test_a11_fixed_mount(cfgs, caplog):
    t = {"v": 1.0}
    fm = FixedMount(cfgs["rigs"]["rig_fixed"], lambda: t["v"])
    assert isinstance(fm, GimbalIO)
    a = fm.read()
    t["v"] = 5.0
    with caplog.at_level("WARNING"):
        fm.set_target(0.3, 0.2)
    b = fm.read()
    assert (a.pan_rad, a.tilt_rad, a.roll_rad) == (b.pan_rad, b.tilt_rad, b.roll_rad)
    assert b.t_sim == 5.0
    assert b.tilt_rad == pytest.approx(cfgs["rigs"]["rig_fixed"]["nominal_tilt_deg"] * DEG)
    assert any("no pan-tilt" in r.message for r in caplog.records)


def _gimbal(cfgs, seed=0, **over):
    from sim_bridge.config import deep_merge
    g = deep_merge(cfgs["gimbal_storm32"], over)
    clock = {"t": 0.0}
    sim = Storm32GimbalSim(g, {**cfgs["rigs"]["rig_gimbal"], "initial_pan_deg": 0.0, "initial_tilt_deg": 0.0},
                           clock=lambda: clock["t"], seed=seed)
    return sim


def test_a12_step_response(cfgs):
    """Small step (rate limit not active): fit wn and zeta from overshoot and peak time; +-15%."""
    sim = _gimbal(cfgs, holding_jitter={"rms_deg": 0.0})
    step = 1.0 * DEG
    sim.set_target(step, 0.0)
    ts, ys = [], []
    for k in range(1, 1501):
        sim.advance_to(k * 1e-3)
        ts.append(k * 1e-3)
        ys.append(sim.dyn.pan.theta)
    ts, ys = np.array(ts), np.array(ys)
    lat = cfgs["gimbal_storm32"]["command"]["latency_s"]
    os_frac = ys.max() / step - 1.0
    zeta = -math.log(os_frac) / math.sqrt(math.pi ** 2 + math.log(os_frac) ** 2)
    t_peak = ts[np.argmax(ys)] - lat - 1e-3
    wn = math.pi / (t_peak * math.sqrt(1 - zeta ** 2))
    assert zeta == pytest.approx(0.7, rel=0.15)
    assert wn / (2 * math.pi) == pytest.approx(6.0, rel=0.15)
    assert abs(ys[-1] - step) < 1e-3 * step


def test_a12_rate_limit(cfgs):
    sim = _gimbal(cfgs, holding_jitter={"rms_deg": 0.0})
    sim.set_target(90 * DEG, 0.0)
    prev, rates = 0.0, []
    for k in range(1, 2001):
        sim.advance_to(k * 1e-3)
        rates.append((sim.dyn.pan.theta - prev) / 1e-3)
        prev = sim.dyn.pan.theta
    assert max(rates) / DEG == pytest.approx(120.0, rel=0.05)


def test_a12_command_latency_and_rate(cfgs):
    sim = _gimbal(cfgs, holding_jitter={"rms_deg": 0.0})
    sim.set_target(1 * DEG, 0.0)
    sim.advance_to(0.0195)
    assert sim.dyn.pan.theta == 0.0
    sim.advance_to(0.0225)
    assert sim.dyn.pan.theta > 0.0


def test_a12_readback_noise_and_drift(cfgs):
    g = cfgs["gimbal_storm32"]
    noise, drift_final = [], []
    for seed in range(8):
        sim = _gimbal(cfgs, seed=seed, holding_jitter={"rms_deg": 0.0})
        errs_tilt = []
        pans = []
        for k in range(1, 121):                       # 120 s, read at 1 Hz
            sim.advance_to(float(k))
            rb = sim.read()
            true = sim.true_attitude()
            errs_tilt.append(rb.tilt_rad - true[1])
            pans.append(rb.pan_rad - true[0])
        noise.append(np.std(errs_tilt))
        drift_final.append(pans[-1])
    assert np.mean(noise) / DEG == pytest.approx(g["readback"]["noise_rms_deg"], rel=0.15)
    # random walk: std after 2 min = rate * sqrt(2); readback noise adds in quadrature
    expect = math.hypot(g["readback"]["pan_drift_deg_per_min"] * math.sqrt(2), g["readback"]["noise_rms_deg"])
    assert np.std(drift_final) / DEG == pytest.approx(expect, rel=0.6)    # 8 seeds: loose statistical bound


def test_a12_readback_rate_and_jitter(cfgs):
    sim = _gimbal(cfgs)
    stamps, tilt = [], []
    for k in range(1, 4001):
        sim.advance_to(k * 1e-3)
        stamps.append(sim.read().t_sim)
        tilt.append(sim.true_attitude()[1])
    uniq = sorted(set(stamps))
    assert np.median(np.diff(uniq)) == pytest.approx(1 / cfgs["gimbal_storm32"]["readback"]["rate_hz"], rel=0.05)
    jit = np.std(np.array(tilt[500:]))
    assert jit / DEG == pytest.approx(cfgs["gimbal_storm32"]["holding_jitter"]["rms_deg"], rel=0.25)


def test_a12_offset_schedule_and_inactive(cfgs):
    sim = _gimbal(cfgs, holding_jitter={"rms_deg": 0.0},
                  offset_schedule=[{"t_s": 0.5, "pan_mrad": 8.0, "tilt_mrad": 0.0}])
    sim.advance_to(0.45)
    assert abs(sim.dyn.pan.theta) < 1e-9
    sim.advance_to(1.5)
    assert sim.dyn.pan.theta == pytest.approx(8e-3, rel=0.01)
    sim.dyn.set_active(False)
    sim.set_target(0.2, 0.0)
    sim.advance_to(3.0)
    assert sim.dyn.pan.theta == pytest.approx(8e-3, rel=0.01)


# ------------------------------------------------------------------------------------------- camera pipeline
def _render_beacon(level: float, rng, H=1080, W=1920, base=30, peak=180, x=960, y=540, r=6):
    img = np.full((H, W, 3), base, np.uint8)
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    disk = (xx * xx + yy * yy) <= r * r
    patch = img[y - r:y + r + 1, x - r:x + r + 1]
    patch[disk] = [int(peak * level * 0.05), int(peak * level * 0.1), int(peak * level)]   # RGB, red dominant
    return img


def test_a5_pipeline_timing(cfgs):
    cam = cfgs["camera_action5"]
    for fidelity in ("fast", "full"):
        pipe = Action5Pipeline(cam, fidelity)
        rate = Action5Pipeline.required_render_rate_hz(cam, fidelity)
        img = np.full((cam["stream"]["height_px"], cam["stream"]["width_px"], 3), 100, np.uint8)
        frames = []
        t = 0.0
        while t < 2.0:
            pipe.push_render(img, t)
            frames += pipe.pop_delivered(t)
            t += 1.0 / rate
        caps = np.array([f.t_capture_sim for f in frames])
        fps = 1.0 / np.mean(np.diff(caps))
        assert fps == pytest.approx(cam["stream"]["fps"], rel=0.05), fidelity
        f0 = frames[0]
        assert f0.image_bgr.shape == (1080, 1920, 3) and f0.image_bgr.dtype == np.uint8
        lat = cam["latency"]["capture_to_frame_s"]
        assert all(abs((f.t_delivered_sim - f.t_capture_sim) - lat) <= 1 / cam["stream"]["fps"] for f in frames)


def test_a6_modulation_through_pipeline(cfgs):
    """Beacon brightness follows the 7 Hz modulator; after the full pipeline (rolling shutter, AE, MJPG) the
    red channel at the beacon pixel has its FFT peak at 7.0 +- 0.2 Hz and a low/high ratio of 0.40 +- 0.05."""
    cam = cfgs["camera_action5"]
    m = Modulation.from_cfg(_blink(cfgs))
    pipe = Action5Pipeline(cam, "full")
    rate = Action5Pipeline.required_render_rate_hz(cam, "full")
    rng = np.random.default_rng(0)
    cache = {lv: _render_beacon(lv, rng) for lv in (0.4, 1.0)}
    reds, times = [], []
    t = 0.0
    while t < 8.0:
        pipe.push_render(cache[round(m.level(t), 1)], t)
        for f in pipe.pop_delivered(t):
            reds.append(float(f.image_bgr[536:545, 956:965, 2].mean()))
            times.append(f.t_capture_sim)
        t += 1.0 / rate
    reds = np.array(reds)
    spec = np.abs(np.fft.rfft(reds - reds.mean()))
    freqs = np.fft.rfftfreq(len(reds), np.mean(np.diff(times)))
    assert freqs[np.argmax(spec)] == pytest.approx(7.0, abs=0.2)
    hi, lo = np.percentile(reds, 90), np.percentile(reds, 10)
    assert hi < 255                                           # "before saturation"
    assert lo / hi == pytest.approx(0.40, abs=0.05)


def test_a7_rolling_shutter_skew(cfgs):
    """Vertical bar moving sideways at 1 m/s at 3 m: centroid x vs row gives readout +-20%."""
    cam = cfgs["camera_action5"]
    from sim_bridge.camera_model import make_lens
    lens = make_lens(cam)
    v_px = 1.0 / 3.0 * lens.f_px                         # px/s near the image centre
    readout = cam["rolling_shutter"]["readout_s"]
    H, W = cam["stream"]["height_px"], cam["stream"]["width_px"]
    asm = RollingShutterAssembler(cam["stream"]["fps"], readout, cam["rolling_shutter"]["bands"], H)
    xs = np.arange(W)

    def render(t):
        x0 = 900 + v_px * t
        prof = np.clip(1.0 - np.abs(xs - x0) / 6.0, 0, 1) * 200
        return np.repeat(prof[None, :, None].astype(np.uint8), H, axis=0).repeat(3, axis=2)

    rate = cam["rolling_shutter"]["bands"] / readout
    frames = []
    t = 0.0
    while t < 0.2:
        frames += asm.push(SubFrame(render(t), t))
        t += 1.0 / rate
    slopes = []
    for img, _ in frames:
        rows = [int((b + 0.5) * H / cam["rolling_shutter"]["bands"]) for b in range(cam["rolling_shutter"]["bands"])]
        cx = [float((img[r, :, 0] * xs).sum() / img[r, :, 0].sum()) for r in rows]
        slopes.append(np.polyfit(rows, cx, 1)[0])      # px per row
    readout_est = np.mean(slopes) * H / v_px
    assert readout_est == pytest.approx(readout, rel=0.2)


def test_a9_auto_exposure(cfgs):
    e = cfgs["camera_action5"]["exposure"]
    ae = AutoExposure.from_cfg(e)
    dark = np.full((90, 160, 3), 12, np.uint8)
    bright = np.full((90, 160, 3), 200, np.uint8)
    bright[:10, :10] = 255
    t, dt = 0.0, 1 / 30
    for _ in range(int(5 / dt)):
        out, g, sat = ae.apply(dark, t)
        t += dt
    g_dark = ae.gain
    assert g_dark > 4.0
    # switch to the bright lamp: settle within 3 tau
    gains = []
    t_switch = t
    while t < t_switch + 10 * e["tau_s"]:
        out, g, sat = ae.apply(bright, t)
        gains.append((t - t_switch, g, sat, int(out.max())))
        t += dt
    g_final = gains[-1][1]
    g3 = [g for tt, g, _, _ in gains if tt >= 3 * e["tau_s"]][0]
    # first-order loop in log-gain: after 3 tau ~5% of the log step remains (a little more here because the
    # first bright frames saturate, which caps the measured mean). Settled = < 10% of the log step left.
    step = abs(math.log(g_dark / g_final))
    assert abs(math.log(g3 / g_final)) < 0.10 * step
    assert abs(math.log(gains[-5][1] / g_final)) < 0.01        # converged at the end
    assert gains[0][2] is True                                   # first bright frame at the dark gain saturates
    # saturation flagged only above 255: a frame at gain <= 1 whose max is 255 is not "above"
    ae2 = AutoExposure.from_cfg({**e, "mode": "locked", "locked_gain": 1.0})
    _, _, sat = ae2.apply(bright, 0.0)
    assert sat is False


def test_mjpg_roundtrip_shape(cfgs):
    from sim_bridge.image_pipeline import mjpg_roundtrip
    img = (np.random.default_rng(0).random((1080, 1920, 3)) * 255).astype(np.uint8)
    out = mjpg_roundtrip(img, cfgs["camera_action5"]["compression"]["mjpg_quality"])
    assert out.shape == img.shape and out.dtype == np.uint8


def test_camera_model(cfgs):
    """Default lens = the real rig's dewarped 104 deg rectilinear view; the 140 deg fisheye stays as an option."""
    from sim_bridge.camera_model import (EquidistantLens, RectilinearLens, make_lens, rectilinear_to_equidistant_maps,
                                         render_sensor_type)
    cam = cfgs["camera_action5"]
    lens = make_lens(cam)
    assert isinstance(lens, RectilinearLens) and render_sensor_type(cam) == "camera"
    assert lens.f_px == pytest.approx(960 / math.tan(math.radians(52)), rel=1e-9)        # ~750 px
    assert 1e3 * lens.ifov_rad == pytest.approx(1.333, abs=0.01)
    assert math.degrees(lens.vfov_rad) == pytest.approx(71.5, abs=0.5)
    d = lens.unproject(np.array([[100.0, 200.0], [1800.0, 900.0]]))
    assert np.allclose(lens.project(d), [[100, 200], [1800, 900]], atol=1e-6)
    assert 0.08 / 3.0 * lens.f_px == pytest.approx(20, abs=1)          # 8 cm tag at 3 m ~20 px (README limit)
    assert 2 * math.degrees(math.atan(1 / 3)) < math.degrees(lens.hfov_rad)    # +-1 m at 3 m (37 deg) fits
    # fisheye option
    fish = {**cam, "lens": {**cam["lens"], "projection": "equidistant", "hfov_deg": 140.0}}
    fl = make_lens(fish)
    assert isinstance(fl, EquidistantLens) and render_sensor_type(fish) == "wideanglecamera"
    assert fl.f_px == pytest.approx(786, abs=1)
    mx, _ = rectilinear_to_equidistant_maps(fl, 640, 360, math.radians(120))
    assert (mx[540, :] < 0).any() and (mx[540, 960] > 0)        # 120 deg render leaves black edges at 140 deg


# ------------------------------------------------------------------------------------------- telemetry
def test_a13_schema():
    from telemetry.bus import TelemetryValidationError, schema, validate_telemetry
    ex = dict(schema()["examples"][0])
    validate_telemetry(ex)
    bad = dict(ex)
    del bad["residual_mrad"]
    with pytest.raises(TelemetryValidationError, match="residual_mrad"):
        validate_telemetry(bad)
    with pytest.raises(TelemetryValidationError):
        validate_telemetry({**ex, "state": "LOCKED"})
    validate_telemetry({**ex, "lean_deg": 4.2})
    with pytest.raises(TelemetryValidationError):          # IR-link fields are gone
        validate_telemetry({**ex, "rx_signal": 3e-4})


def test_bus_roundtrip_and_truth_refusal():
    import time
    from telemetry.bus import Publisher, Subscriber, schema
    with pytest.raises(PermissionError):
        Subscriber("truth")
    pub = Publisher("telemetry")
    sub = Subscriber("telemetry")
    time.sleep(0.3)
    ex = dict(schema()["examples"][0])
    got = None
    for _ in range(20):
        pub.send(ex)
        got = sub.recv(timeout_s=0.1)
        if got:
            break
    pub.close()
    sub.close()
    assert got and got[1]["residual_mrad"] == ex["residual_mrad"]


# ------------------------------------------------------------------------------------------- truth geometry
def test_truth_geometry(cfgs):
    from sim_bridge.camera_model import LINK_TO_OPTICAL, make_lens, project_world_point
    from sim_bridge.truth_publisher import camera_rotation, compute_truth, lens_position, tx_axis_body
    rig = cfgs["rigs"]["rig_fixed"]
    st = cfgs["world"]["hover_station_m"]
    off = cfgs["drone"]["payload"]["variants"]["red_led_beacon"]["offset_m"]
    lens0 = lens_position("rig_fixed", rig, camera_rotation("rig_fixed", rig, None))
    txb = tx_axis_body(st, off, lens0, 0.0)
    rec = compute_truth(t_sim=1.0, rig="rig_fixed", rig_cfg=rig, drone_pos=st, drone_quat=(1, 0, 0, 0),
                        drone_vel=(0, 0, 0), payload_offset=off, tx_axis_b=txb)
    assert rec["range_m"] == pytest.approx(math.dist(lens0, np.add(st, off)), rel=1e-9)
    assert rec["tx_pointing_err_mrad"] == pytest.approx(0.0, abs=1e-3)     # acos round-off
    assert rec["lean_deg"] == pytest.approx(0.0, abs=1e-9)
    # nominal aim is at the station, so the boresight error is the mount error plus the beacon height offset
    assert rec["boresight_err_mrad"] < 30.0
    # pixel projection agrees with the boresight error sign convention (x right, y down)
    lens = make_lens(cfgs["camera_action5"])
    uv = project_world_point(lens, lens0, camera_rotation("rig_fixed", rig, None), rec["beacon_pos_m"])
    assert np.sign(uv[0] - lens.cx) == np.sign(rec["boresight_err_x_mrad"])
    assert np.sign(uv[1] - lens.cy) == np.sign(rec["boresight_err_y_mrad"])
    _ = LINK_TO_OPTICAL
