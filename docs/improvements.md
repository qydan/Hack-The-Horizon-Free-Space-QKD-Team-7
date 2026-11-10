# Areas to improve

Found during the repo sweep on 2026-11-09. Ordered roughly by how much each one affects the
live demo. Items 1-3 and 8 are also tracked in `TODO.md`.

## 1. Fixed-camera residual (biggest issue)

With no gimbal (`tracker.py --source camera`, `FixedGimbal`), `residual_mrad` is the beacon's
distance from the image centre. Nothing steers, so all of the drone's wander counts as ground
pointing error against the 10 mrad virtual receiver (`BudgetParams.theta_fov_mrad`).

At the default operating point (`--snr 50`), `tools/tune.py` reports 10.6 dB of extra loss to
reach the 11% gate. That is a residual of about 11 mrad, roughly 3 cm at 3 m, so a real hover
would abort almost constantly.

Options:
- Measure the residual from the calibrated boresight instead of the image centre.
- Emulate the gimbal in software (steer a virtual pointing direction with `SimGimbal` dynamics
  and report the residual against it).
- Wire in the real gimbal.

Where: `GroundTracker.update` in `src/tracker.py` (`resid = hypot(off)`).

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

## 7. The test suite takes about 7.5 minutes

Most of the time is the tracker tests, which render simulated camera frames. A `slow` pytest
marker (and `pytest -m "not slow"` for quick runs) would make the suite practical to run after
small changes.

## 8. The design `.docx` is untracked and names team members

`Drone QKD demo system diagram explained.docx` is cited as the source of truth by the README
and `docs/system_architecture.md` but is not in git, and it names team members. Decide whether
it belongs in the repo before committing; if not, reword the references.
