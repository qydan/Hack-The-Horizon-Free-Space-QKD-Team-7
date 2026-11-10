# System architecture

Source of truth: *Drone QKD demo: system diagram explained* (`Drone QKD demo system diagram explained.docx`).
This file records how the code implements it.

The demo proves one thing: **the quality of the quantum key rises and falls with how well the ground station tracks a real, wind-blown drone.** Information flows one way through three segments:

```
 AIR                      GROUND                        KEY (cyber sandbox)
 drone + beacon  --I1-->  camera + gimbal + tracker --I2--> error budget -> channel+decoys
 (fan pushes it)  light   (src/tracker.py)  telemetry       -> Eve -> BB84 -> gate -> key store
                                  ^                                            |
                                  +---------------- I8: abort / re-acquire ----+
```

Two programs, one laptop, joined only by the telemetry stream (ZeroMQ on localhost):

| Program | File | Role |
|---|---|---|
| Ground tracker | `src/tracker.py` | frames in, one telemetry message per frame out |
| Cyber sandbox | `src/sandbox.py` (run via `src/demo_runner.py`) | telemetry in, keys out |

`demo_runner.py` can also run both in one process (`--source sim`) or replay a recording (`--source csv`).

## Air segment

A DJI Mini 2 SE hovers in front of a fan carrying a beacon. No DJI SDK or drone telemetry is used: everything is measured from the ground, so a commercial ground station works with any vendor's drone. Hardware only; the software stand-in is `src/sim.py` (`SimRig`: wind-blown drone, 13 Hz beacon, room lights, camera, gimbal).

Beacon: 100% / 40% blink at 13 Hz (never fully off), IR 850 nm or red, or retroreflective tape with a ring light. An AprilTag on the drone gives its tilt.

## Ground segment (`src/tracker.py`, `src/vision.py`, `src/gimbal.py`)

| Stage | What it does | Code |
|---|---|---|
| Search | A drone-detection model boxes the drone in the wide view; box offset becomes pan/tilt and the gimbal turns. Hands over to fine track when the drone is near the centre and the beacon is seen. | `vision.YoloDnnDetector` (OpenCV DNN, ONNX), `parse_yolo_output`; `BrightBlobDetector` is the synthetic stand-in |
| Fine track | Lock-in detection finds the pixels pulsing at 13 Hz (room lights and mains flicker are ignored); the spot centre is found to a fraction of a pixel; a Kalman filter smooths and predicts; a control loop keeps the spot centred. Beacon gone briefly: COAST on the prediction; gone for good: back to SEARCH. | `vision.LockInDetector`, `refine_centroid`, `KalmanCV`; `GroundTracker` |
| Gimbal | Wrap the real driver in a `gimbal.Gimbal` subclass (absolute command in, encoder angles out). | `gimbal.SimGimbal` models lag, step size and slew limit; `gimbal.FixedGimbal` is a camera mounted still |
| Pose | AprilTag via `cv2.solvePnP`: lean (angle between the beacon axis and the line of sight) and range. | `vision.AprilTagPose`, `tag_pose` |

The drone's direction is the gimbal encoder angle plus where the spot sits in the image. Calibration (`GroundTracker.calibrate`, `tracker.Calibration`) records the calm-hover boresight and the full-signal beacon brightness.

Frame rate and blink rate go together: 13 Hz assumes about 120 fps; at 30 fps use about 7 Hz (`TrackerConfig.blink_hz`, `fps`).

### Real camera, fixed mount (`tracker.py --source camera`)

This is what runs on hardware today. The camera does not move (`FixedGimbal`), so the drone's direction is the spot's offset from the image centre, and the residual is that same offset.

* No search detector and no steering: the beacon is acquired anywhere in view (`TrackerConfig.fixed_camera`), or only inside `--zone x0,y0,x1,y1` so lamps and windows outside it cannot steal the lock.
* `--beacon blink` (default) uses the lock-in detector; `--beacon steady` finds a constantly-lit LED as the brightest small spot (`vision.SteadySpotDetector`), which needs a dark scene.
* `--calibrate N` records the boresight and full-signal brightness over N seconds of calm hover and saves `calibration.json`; later runs load it.
* No pose estimator is wired in, so `lean_deg` is 0 and `range_m` is `nominal_range_m` (3 m). The drone-lean error bar stays empty on the real rig.
* The frame-count settings (`acquire_frames`, `coast_frames`, ROI size) are rescaled from their 120 fps / 480 px defaults.
* `tools/camera_check.py` measures the real fps, dropped frames, exposure drift, the lock-in noise floor and the beacon SNR, and prints the settings to use.

## Telemetry (`src/telemetry.py`), interface I2

One JSON message per camera frame:

```json
{"t": 12.508, "state": "TRACK", "drift_mrad": 3.1, "jitter_mrad": 0.8, "residual_mrad": 0.4,
 "gimbal_lag_mrad": 0.3, "lean_deg": 4.2, "range_m": 3.05, "intensity": 0.92,
 "rx_signal": 0.88, "rx_background": 0.04}
```

| Field | Meaning | How |
|---|---|---|
| drift | slow wander (< 0.5 Hz) | low-pass of the drone direction vs the calibrated boresight |
| jitter | fast shaking | RMS of the > 0.5 Hz part over 1 s |
| residual | ground pointing error | spot distance from the image centre (0 while coasting or searching) |
| gimbal lag | part of the residual from the gimbal falling behind | commanded minus encoder angle |
| intensity | beacon brightness | lock-in amplitude / calibration |
| lean, range | drone tilt, distance | AprilTag pose |
| rx_signal, rx_background | optional, only with the IR data link (I9) | photodiode, normalised to S_cal / B_cal |

`TelemetryMessage` (schema + validation), `ZmqPublisher` / `ZmqSubscriber`, `CSVReplaySource`. No hardcoded values, no radio.

## Key segment (`src/sandbox.py`)

Each telemetry message passes through:

1. **Error budget** (`src/error_budget.py`): `eta = eta_geo(L) * exp(-2 theta_rx^2 / theta_fov^2) * eta_tx(theta_tx) * I / I_cal`.
   - `eta_geo`: receiver lens area / beam spot area (about 31 dB at 5 m, 43 dB at 20 m).
   - `theta_rx` is the measured residual, so gimbal lag, step size and jitter are already inside it. `theta_fov` is a *virtual* 1/e^2 half-width (10 mrad) so a few mrad of ground error visibly costs signal.
   - `theta_tx = lean + drift / range` with a flat-topped `eta_tx` (about 1 inside +-5 deg).
   - With the IR link, measured `rx_signal` replaces the two pointing factors and `rx_background` sets `p_bg`.
2. **Channel + decoys** (`src/channel.py`, `src/decoy.py`): each pulse is signal / decoy / vacuum (70 / 20 / 10%, `mu` = 0.5, `nu` = 0.1), Poisson photon number, survival with `eta`, dark counts and a 2% optical error. Counts are drawn exactly (binomial) so a 1 GHz source is cheap.
3. **Eve emulator** (`channel.EveConfig`): off by default; intercept-resend on a fraction of pulses, or PNS.
4. **BB84 pipeline** (`src/qkd_core.py`): sifting, 20% QBER sample, Cascade error correction (leakage counted), Toeplitz privacy amplification to 256 bits. Pooling across blocks optional (`src/key_pool.py`).
5. **QBER + decoy gate**: keep the key only if the **pooled QBER of the key block** is below 11% and the decoy check passes; otherwise discard and send `reacquire` to the tracker (I8, `ControlPublisher` / `ControlSubscriber`). The 100 ms values are the live trace only. A block where the beacon was unlocked most of the time, or with too few clicks, is `no_link` (no abort, no key); a block that passes the gate but cannot yield 256 secret bits is `no_key`.
6. **Key store** (`sandbox.KeyStore`): finished keys for the operator (I4).

`QBER = (e_opt p_sig + p_noise / 2) / (p_sig + p_noise)`, `p_sig = 1 - exp(-mu eta eta_d)`, `p_noise = p_dark + p_bg`.

Operating point: `default_params(on_axis_snr)` chooses `p_dark` so the on-axis signal is `on_axis_snr` times the noise at the demo range (`--snr`, default 50 gives about 3% QBER when locked).

| Eve | What judges see |
|---|---|
| off | keys flow; QBER follows the fan |
| intercept-resend, half the pulses | QBER about 14.5%, gate aborts |
| PNS | QBER stays near 3%, decoy check fails, gate aborts |

## Interfaces

| ID | From -> to | In the code |
|---|---|---|
| I1 | drone -> ground camera (beacon light) | frames into `GroundTracker.update` |
| I2 | tracker -> sandbox | `TelemetryMessage` over ZeroMQ |
| I3 | field crew -> drone | manual flight, not software |
| I4 | key store -> operator | `KeyStore`, `--keys-out` |
| I5 | fan and room -> drone | physical |
| I6 | Eve -> sandbox | `EveConfig`, `--eve`, `--pns` |
| I7 | regulator -> system | flight rules, eye safety |
| I8 | gate -> tracker | `reacquire` over ZeroMQ |
| I9 | drone LED -> ground photodiode (optional) | `rx_signal`, `rx_background` fields |

## Not implemented (hardware or model files needed)

* The real gimbal driver: `tracker.py --source camera` runs a fixed camera only (see above). A steered run needs a `Gimbal` subclass and a `build_*` function like `build_sim`.
* A trained drone-detection ONNX model for `YoloDnnDetector` (only needed with a gimbal).
* Pose on the real camera: `AprilTagPose` exists but is not passed to the tracker in camera mode, so lean and range are not measured there.
* Exposure lock in `tracker.py --source camera`: it opens the camera with whatever exposure the driver has (`tools/camera_check.py` has `--exposure` and `--dialog`).
* The IR data link hardware and its receiver reader (only the `rx_*` fields and the budget handling exist).
* AprilTag pose is tested against projected tags, not a real camera.
