# Areas to improve

Found during the repo sweep on 2026-11-09. Ordered roughly by how much each one affects the
live demo. Items 1-3 and 8 are also tracked in `TODO.md`.

## 1. Fixed-camera residual (fixed)

The residual is now measured against a virtual pan-tilt mount (`SimGimbal`, 60 ms lag, 1.53 mrad
steps) that the tracker steers from its Kalman prediction (`--virtual-mount`, default on for
`--source camera`). In a 20 s windy sim at 30 fps the mean residual drops from 12.8 to 1.4 mrad;
drift and jitter are unchanged. The residual is a model of a mount, not a hardware measurement.
`--no-virtual-mount` restores the old image-centre offset.

## 2. Lean and range on the real camera (fixed)

`tracker.py --source camera --tag-size 0.09` now passes `vision.AprilTagPose` to the tracker.
Without the flag lean is 0 and range is 3 m. Not yet tried on the rig: print a tag and check it.

## 3. Exposure lock in `tracker.py` (fixed)

`--exposure N` sets a manual exposure (DirectShow: 2^N seconds). Leave it out if the Action 5 Pro
already holds exposure; confirm with `tools/camera_check.py`.

## 4. Calibration and the sandbox timeout (fixed)

`ZmqSubscriber` now starts its silence timeout only after the first message, so a sandbox
started before `--calibrate N` waits for the tracker.

## 5. `build_camera` capture internals (fixed)

`VideoCaptureSource.restart(duration)` replaces the direct `_t0` / `duration` assignments.

## 6. Small leftovers (fixed)

Removed the unused `TelemetryExhausted` and `_zmq` attributes, the `-0.0` in `tools/tune.py`
and the unused `quiet` argument of `describe_block`.

## 7. The test suite takes about 4 minutes (fixed)

Eleven tests (subprocess runs in `tests/test_demo_and_tools.py` and the simulated-camera tracker
runs in `tests/test_tracker.py`) now carry a `slow` marker, registered in `pytest.ini`.
`pytest -m "not slow"` runs the other 145 tests in under a minute.

## 8. The design `.docx` name and status (fixed)

The file is public-safe, so it stays in the repo, renamed to `original_design.docx`. The README
and docs now call it the early design brief, not the source of truth.


## 9. Infrared data link removed (fixed)

The demo uses a red LED and no infrared hardware. The optional photodiode path is gone:
`rx_signal` / `rx_background` in the telemetry schema, the matching branch and `p_bg` in the
error budget, their tests, and the doc references. Old recordings that still have those CSV
columns load fine (extra columns are ignored). `docs/d3_engineering_design.md` and
`docs/threat_model.md` still describe an 850 nm quantum channel for the deployed system, which
is a design proposal and not part of the demo.

## 10. Live plots (added)

`python tools/live_dashboard.py run.jsonl` serves the dashboard on http://127.0.0.1:8000 and
redraws the last 60 s every second while the sandbox runs with `--log run.jsonl`. It never shows
key material.
