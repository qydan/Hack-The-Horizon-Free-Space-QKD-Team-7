# Areas to improve

Found during the repo sweep on 2026-11-09. Ordered roughly by how much each one affects the
live demo. Items 1-3 and 8 are also tracked in `TODO.md`.

## 1. Fixed-camera residual (fixed)

The residual is now measured against a virtual pan-tilt mount (`SimGimbal`, 60 ms lag, 1.53 mrad
steps) that the tracker steers from its Kalman prediction (`--virtual-mount`, default on for
`--source camera`). In a 20 s windy sim at 30 fps the mean residual drops from 12.8 to 1.4 mrad;
drift and jitter are unchanged. The residual is a model of a mount, not a hardware measurement.
`--no-virtual-mount` restores the old image-centre offset.

## 2. No lean or range on the real camera

`vision.AprilTagPose` exists but `build_camera` in `src/tracker.py` does not pass a `pose_fn`,
so in camera mode `lean_deg` is 0 and `range_m` stays at `nominal_range_m` (3 m). The
drone-lean error bar on the dashboard is therefore empty on the real rig, which removes half of
the "two error bars" argument for D3.

## 3. Exposure is not locked by `tracker.py --source camera`

`build_camera` opens `VideoCaptureSource` with `manual_exposure=None`, so the camera runs with
whatever exposure the driver has. Auto-exposure hides intensity changes and breaks the
calibration. Only `tools/camera_check.py` has `--exposure` and `--dialog`. Add `--exposure` to
`tracker.py`.

## 4. Calibration and the sandbox timeout collide

`ZmqSubscriber` stops after 5 s of telemetry silence (`demo_runner.py --timeout`). The tracker
publishes nothing while `--calibrate N` runs, so a sandbox started first exits before the
tracker sends its first message. The docs work around it by calibrating in a separate run
(`--calibrate 15 --duration 1`). A cleaner fix is to start the timeout only after the first
message arrives.

## 5. `build_camera` pokes capture internals

`build_camera` sets `src._t0 = None` and `src.duration` directly to restart the capture clock
around calibration. A `restart(duration)` method on `vision.VideoCaptureSource` would be
cleaner.

## 6. Small leftovers

- `TelemetryExhausted` in `src/telemetry.py` is never raised or caught.
- `ZmqSubscriber._zmq` and `ControlSubscriber._zmq` are assigned but unused.
- `tools/tune.py` prints `-0.0` dB for the on-axis row.
- `describe_block` in `src/demo_runner.py` ignores its `quiet` argument.

## 7. The test suite takes about 4 minutes (fixed)

Ten tests (subprocess runs in `tests/test_demo_and_tools.py` and the simulated-camera tracker
runs in `tests/test_tracker.py`) now carry a `slow` marker, registered in `pytest.ini`.
`pytest -m "not slow"` runs the other 148 tests in about 30 s.

## 8. The design `.docx` name and status (fixed)

The file is public-safe, so it stays in the repo, renamed to `original_design.docx`. The README
and docs now call it the early design brief, not the source of truth.
