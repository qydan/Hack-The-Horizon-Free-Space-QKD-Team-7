"""[OPEN] Receiver pointing.

Inputs:  the filter state from ``tracking.kalman``.
Outputs: rig_fixed: a virtual receiver pointing (fixed boresight, or software-steered with a lag);
         rig_gimbal: ``GimbalIO.set_target(pan, tilt)`` commands (feedforward + PI), at most 50 Hz.
Config:  ``tracking.yaml: pointing.*`` (mode_rig_fixed, virtual_receiver_lag_s, feedforward_gain, pi_kp,
         pi_ki, command_rate_hz).

Rig behaviour: with ``FixedMount``, ``set_target`` is a no-op that logs a warning, so rig_fixed must not rely
on it. The residual reported in telemetry is relative to the (virtual or physical) receiver boresight.
"""
from __future__ import annotations

from typing import Any

from sim_bridge.gimbal_io import GimbalIO, GimbalState


class Pointer:
    """Common pointing interface for both rigs."""

    def __init__(self, cfg: dict[str, Any], rig: str, gimbal: GimbalIO) -> None:
        """Store configuration, rig name and the GimbalIO."""
        raise NotImplementedError

    def step(self, t: float, track_state: Any, gimbal_state: GimbalState) -> tuple[float, float]:
        """Update pointing; return the receiver boresight (az, el) in rad."""
        raise NotImplementedError

    def residual_rad(self, track_state: Any) -> tuple[float, float]:
        """Beacon direction minus receiver boresight (x, y) in rad."""
        raise NotImplementedError
