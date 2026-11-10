"""[OPEN] Sandbox entry point: subscribes to 5556 + 5560, runs blocks through the pipeline, publishes on 5557 + 5559.

Config: ``config/sandbox.yaml``.
"""
from __future__ import annotations

from typing import Any


def run(cfg: dict[str, Any]) -> None:
    """Run the sandbox until interrupted."""
    raise NotImplementedError


def main() -> None:
    """Parse arguments and call :func:`run`."""
    raise NotImplementedError


if __name__ == "__main__":
    main()
