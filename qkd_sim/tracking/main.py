"""[OPEN] Tracker entry point: wires acquisition, tracking, filter, pointing, state machine and telemetry for
the chosen rig.

    python -m tracking.main --rig rig_fixed|rig_gimbal [--hardware]

Use ``sim_bridge.frame_source.make_frame_source`` and ``sim_bridge.gimbal_io.make_gimbal_io`` so the same code
runs in simulation and on the real hardware. tools/run_scenario.py launches this module once it is
implemented (it skips it while ``run`` raises NotImplementedError).
"""
from __future__ import annotations

from typing import Any


def run(rig: str, cfg: dict[str, Any], hardware: bool = False) -> None:
    """Run the tracker until interrupted."""
    raise NotImplementedError


def main() -> None:
    """Parse arguments and call :func:`run`."""
    raise NotImplementedError


if __name__ == "__main__":
    main()
