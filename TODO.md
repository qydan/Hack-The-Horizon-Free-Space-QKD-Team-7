# TODO

Status as of 2026-11-09. The redesign (lock-in tracker, mrad telemetry over ZeroMQ, error budget,
always-on decoy states) is implemented and tested on the simulated rig and on replayed
telemetry, and the real-camera path (30 fps Action 5 Pro, steady red LED, virtual mount) is written
but has never run on hardware. What remains is hardware validation, calibration and presentation prep.
Do section 2 first: it will show what the software guesses got wrong.

## 1. Find out about the rig (do first)
- [ ] Ask organizers: is the judges' rig available beforehand? If not, how much time on-site before judging (10-15 min needed for camera check and calibration)? -> asking
- [ ] Ask what camera, distance, lighting, and turbulence source (fan?) the rig uses.
- [x] Confirm whether we bring our own camera/LED/drone or use theirs. -> using our own
- [ ] Check the event rules on drone weight (the Mini 2 SE plus a beacon is over 250 g).

## 2. Real rig bring-up (before the event)
Goal: prove the real-camera path works end to end. Procedure: `docs/hardware_handoff.md`.
- [ ] Build the beacon: a constantly-lit red LED under a diffuser (the real rig uses `--beacon steady`).
- [ ] Bench test first (hand-held LED about 3 m away, `docs/hardware_handoff.md` step 0): lock, drift on a slow sway, jitter on a shake, `COAST` / `SEARCH` when covered.
- [ ] Dim the room and set `--zone` so lamps, screens and windows cannot steal the lock (steady tracking has no blink to reject them); keep over 15 lux for the drone.
- [ ] Set the real `--hfov` for the Action 5 Pro's dewarped standard mode. Calculated estimate: 14 mm equivalent on a 36 mm-wide frame -> 2*atan(18/14) = about 104 deg (about 107 if matched to the 43.3 mm diagonal). The 155 deg figure is the raw fisheye diagonal: do not use it. Measure it: put two marks a known distance apart at drone range, centred; count pixels; compare with the mrad/px printed at tracker start; move the LED a known angle to confirm. Every mrad value depends on it.
- [ ] Run `python tools/camera_check.py --fps 30 --steady` on Windows Python (not WSL) until it reports READY; note fps and `min_amplitude`.
- [ ] Exposure is locked on the Action 5 Pro; confirm intensity is steady at calm hover.
- [ ] Calibrate in calm hover: `python src/tracker.py --source camera --fps 30 --beacon steady --calibrate 15 --duration 1` (saves `calibration.json`).
- [ ] Measure the virtual-mount latency (`--mount-latency`, 0.06 s is a guess): quick known LED motion, time the response, or sweep it and pick the lowest calm-hover residual.
- [ ] Check: lens cover drops the state to `COAST` / `SEARCH` and tracking returns when removed.
- [ ] Check: first `TRACK` lock under 60 s, tracking held for a full 120 s.
- [ ] Record fallback CSVs (`calm.csv`, `fan.csv`, `obstruct.csv`, `demo.csv`) with `demo_runner.py --source zmq --record ...`.

## 3. Software gaps
- [x] **Fixed-camera residual.** Done in software: the residual is measured against a virtual `SimGimbal` mount (`--virtual-mount`, default on). Still to do: measure the Action 5 Pro's real latency and set `--mount-latency` (0.06 s is a guess), and present the residual as a modelled mount.
- [ ] Wire the real gimbal: a `gimbal.Gimbal` subclass for the driver and a `build_*` function in `tracker.py` (timestamp encoder reads to the frame time).
- [x] Pose on the real camera: `--tag-size` passes `AprilTagPose` to the tracker. Still to do: print a tag and check lean and range on the rig.
- [x] Pixel-to-angle conversion is now a pinhole model (`atan`, `mrad_per_px = 1000/focal_px`); the old linear hfov/width scale understated centre angles by about 30% at 104 deg.
- [x] `tracker.py --source camera --exposure N` locks exposure (skip it if the Action 5 Pro already holds it).
- [ ] Drone-detection ONNX model for `YoloDnnDetector` (only needed with a gimbal).
- [x] Live plots: `tools/live_dashboard.py run.jsonl` (run the sandbox with `--log`).

## 4. Calibration targets (on the rig)
- [ ] Calm hover: QBER about 3%, keys issued.
- [ ] Low fan: block QBER stays under about 5% so keys keep flowing (finite-size limit, see README), or use `--pool` / a longer `--block`.
- [ ] High fan / gust / obstruction: QBER at or above 11% or lock lost; no key issued.
- [ ] Decide the operating point: `--snr 50` (about 3% locked, the default) vs `--snr 10` (the original design brief's "10 times the noise" reading, about 6.4%). See "Operating point" in the README.
- [ ] Fill in `docs/calibration_log.md` with real entries (currently only templates).

## 5. Deliverable checks
- [ ] D1: acquisition < 60 s, measured on the real camera (currently only shown on simulated frames).
- [ ] D1: continuous tracking for 2 minutes under fan turbulence.
- [ ] D2: full run with the live camera showing sift, error correction, privacy amplification, QBER < 11%.
- [ ] D3: review `docs/d3_engineering_design.md` mass/power budget against real COTS part specs (<5 kg payload).
- [ ] Optional threat model: software decoy states only (no second optical channel in the demo).

## 6. Presentation prep
- [ ] Decide wording for the claims: "keys below about 5% QBER per 1 s block, abort gate at 11%" (not "keys up to 11%").
- [ ] Be ready to explain that basis/parity exchange are in-process calls standing in for the classical channel (no wireless bypass).
- [ ] Say clearly when a run is a CSV or simulated replay and not the live link.
- [ ] Present the error budget as a model with a preflight calibration step, not a measured law; `theta_fov` is a deliberately narrow *virtual* receiver.
- [ ] Rehearse `docs/live_demo_runbook.md` end to end, including the fallback (`--source csv --csv demo.csv`).
- [ ] Run `python tools/live_dashboard.py run.jsonl` during the demo for live charts, and `python tools/make_dashboard.py run.jsonl dashboard.html` for a saved visual.

## 7. Housekeeping
- [x] Commit the redesign and recent work.
- [x] Design `.docx` stays in the repo as `original_design.docx`, marked as an early, partly outdated brief.
- [ ] Delete `docs/transfer_function_derivation.md` once nobody needs the old model's reasoning.
- [ ] Re-run `pytest tests/ -v` after any error-budget or tracker change (about 4 minutes; `-m "not slow"` for a quick run).
