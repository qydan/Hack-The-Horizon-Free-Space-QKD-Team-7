# TODO

Status as of 2026-11-09. The redesign (lock-in tracker, mrad telemetry over ZeroMQ, error budget,
always-on decoy states) is implemented and tested on the simulated rig and on replayed
telemetry. What remains is mostly hardware validation, calibration and presentation prep.

## 1. Find out about the rig (do first)
- [ ] Ask organizers: is the judges' rig available beforehand? If not, how much time on-site before judging (10-15 min needed for camera check and calibration)? -> asking
- [ ] Ask what camera, distance, lighting, and turbulence source (fan?) the rig uses.
- [x] Confirm whether we bring our own camera/LED/drone or use theirs. -> using our own
- [ ] Check the event rules on drone weight (the Mini 2 SE plus a beacon is over 250 g).

## 2. Real rig bring-up (before the event)
Goal: prove the real-camera path works end to end. Procedure: `docs/hardware_handoff.md`.
- [ ] Build the beacon (red LED, 7.5 Hz, 100% / 40%, never fully off) or the retroreflector + blinking ring light.
- [ ] Run `python tools/camera_check.py --fps 30 --blink-hz 7.5` on Windows Python (not WSL) until it reports READY; note fps, `min_amplitude`, blink rate.
- [ ] Exposure is locked on the Action 5 Pro; confirm intensity is steady at calm hover.
- [ ] Calibrate in calm hover: `python src/tracker.py --source camera --calibrate 15 --duration 1` (saves `calibration.json`).
- [ ] Check: lens cover drops the state to `COAST` / `SEARCH` and tracking returns when removed.
- [ ] Check: first `TRACK` lock under 60 s, tracking held for a full 120 s.
- [ ] Record fallback CSVs (`calm.csv`, `fan.csv`, `obstruct.csv`, `demo.csv`) with `demo_runner.py --source zmq --record ...`.

## 3. Software gaps
- [x] **Fixed-camera residual.** Done in software: the residual is measured against a virtual `SimGimbal` mount (`--virtual-mount`, default on). Still to decide: present it as a modelled mount, and check `--mount-latency` against the real Action 5 Pro latency if you can measure it.
- [ ] Wire the real gimbal: a `gimbal.Gimbal` subclass for the driver and a `build_*` function in `tracker.py` (timestamp encoder reads to the frame time).
- [ ] Pose on the real camera: pass `vision.AprilTagPose` as `pose_fn` in `build_camera` (lean and range are 0 / 3 m today, so the drone-lean error bar is empty).
- [ ] `tracker.py --source camera`: add `--exposure` (it does not lock exposure itself).
- [ ] Drone-detection ONNX model for `YoloDnnDetector` (only needed with a gimbal).
- [ ] Live plots of tracking state, QBER and key rate (the dashboard is post-run only).
- [ ] Optional: IR data link receiver reader feeding `rx_signal` / `rx_background` (I9).

## 4. Calibration targets (on the rig)
- [ ] Calm hover: QBER about 3%, keys issued.
- [ ] Low fan: block QBER stays under about 5% so keys keep flowing (finite-size limit, see README), or use `--pool` / a longer `--block`.
- [ ] High fan / gust / obstruction: QBER at or above 11% or lock lost; no key issued.
- [ ] Decide the operating point: `--snr 50` (about 3% locked, the default) vs `--snr 10` (the design document's "10 times the noise" reading, about 6.4%). See "Operating point" in the README.
- [ ] Fill in `docs/calibration_log.md` with real entries (currently only templates).

## 5. Deliverable checks
- [ ] D1: acquisition < 60 s, measured on the real camera (currently only shown on simulated frames).
- [ ] D1: continuous tracking for 2 minutes under fan turbulence.
- [ ] D2: full run with the live camera showing sift, error correction, privacy amplification, QBER < 11%.
- [ ] D3: review `docs/d3_engineering_design.md` mass/power budget against real COTS part specs (<5 kg payload).
- [ ] Optional threat model: decide whether to build the physical second IR channel (currently software decoy states only).

## 6. Presentation prep
- [ ] Decide wording for the claims: "keys below about 5% QBER per 1 s block, abort gate at 11%" (not "keys up to 11%").
- [ ] Be ready to explain that basis/parity exchange are in-process calls standing in for the classical channel (no wireless bypass).
- [ ] Say clearly when a run is a CSV or simulated replay and not the live link.
- [ ] Present the error budget as a model with a preflight calibration step, not a measured law; `theta_fov` is a deliberately narrow *virtual* receiver.
- [ ] Rehearse `docs/live_demo_runbook.md` end to end, including the fallback (`--source csv --csv demo.csv`).
- [ ] Run `python tools/make_dashboard.py run.jsonl dashboard.html` for a visual to show.

## 7. Housekeeping
- [ ] Commit the redesign (most of `src/`, `tests/`, `tools/` and the docs are uncommitted or untracked).
- [x] Design `.docx` stays in the repo as `original_design.docx`, marked as an early, partly outdated brief.
- [ ] Delete `docs/transfer_function_derivation.md` once nobody needs the old model's reasoning.
- [ ] Re-run `pytest tests/ -v` after any error-budget or tracker change (about 4 minutes; `-m "not slow"` for a quick run).
