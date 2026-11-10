"""[OPEN] OpenCV acquisition and tracking. Written by the team; this package only defines the interfaces.

No-bypass rule (tests B1/B2): modules here may import only ``sim_bridge.frame_source``,
``sim_bridge.gimbal_io`` and ``telemetry`` from this repository (plus numpy/scipy/opencv/stdlib). Ground truth,
``evaluation`` and ``gz.*`` are off limits, as is any reference to the truth port or the camera truth file.
The same code must run unchanged against the real hardware (UvcFrameSource + Storm32Gimbal / FixedMount).
"""
