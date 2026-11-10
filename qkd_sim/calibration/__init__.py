"""[OPEN] Calibration: intrinsics, mount alignment, gimbal response, rolling shutter, latency, preflight brightness.

Outputs go to config/calibration.yaml. evaluation/score.py compares them with the evaluation-only camera truth
and the configured emulation parameters; calibration code itself must never read either.
"""
