"""OpenCV acquisition and tracking (ported from the team's src/ tracker, retuned for the Action 5 Pro).

    frames (FrameSource) -> acquire.LockInDetector (7 Hz, linear red, full frame) -> state_machine
        SEARCH -> ACQUIRE -> TRACK <-> COAST
    TRACK: track.RoiTracker (lock-in in the ROI + sub-pixel centroid on this frame, rolling-shutter row time)
        -> kalman.AngleKalman (world angles = gimbal readback at the row time - pixel offset)
        -> pointing.Pointer (rig_gimbal: GimbalIO.set_target with velocity lead + PI;
                             rig_fixed: software virtual receiver with the frame latency + servo lag)
        -> motion_estimate (drift / jitter vs boresight, residual) + pose (ArUco lean / range)
        -> telemetry_out (I2 on 5556; aborts from the gate on 5557 drop the lock)

``tracker.Tracker`` wires one frame through all of it; ``main`` is the program.

No-bypass rule (tests B1/B2): modules here may import only ``sim_bridge.frame_source``,
``sim_bridge.gimbal_io`` and ``telemetry`` from this repository (plus numpy/scipy/opencv/stdlib). Ground truth,
``evaluation`` and ``gz.*`` are off limits, as is any reference to the truth port or the camera truth file.
The same code runs unchanged against the real hardware (UvcFrameSource + Storm32Gimbal / FixedMount).
"""
