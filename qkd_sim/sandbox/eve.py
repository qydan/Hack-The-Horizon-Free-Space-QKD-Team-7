"""Eavesdropper models toggled from the operator UI (port 5560): intercept-resend and photon-number splitting.

Ported from the Eve emulator in src/channel.py. The attack acts inside the channel simulation
(``channel_decoys.simulate_message``), not on finished detections:

* intercept-resend on a fraction of pulses: Eve measures in a random basis and resends, adding 25% error on the
  photons she touches. Caught by the QBER gate (half the pulses -> about 14.5% QBER).
* photon-number splitting (PNS): Eve keeps a photon of multi-photon pulses and blocks the rest; QBER stays low
  but the per-class gains stop following the Poisson model, which the decoy check catches.

Toggle message on 5560 (topic "eve"): ``{"mode": "intercept_resend", "fraction": 0.5}``, ``{"mode": "pns"}`` or
``{"mode": "off"}`` (``tools/eve_toggle.py`` sends these; ``scenarios/t7_eve.yaml`` schedules them).
Config: ``config/sandbox.yaml: eve``.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

MODES = ("off", "intercept_resend", "pns")


@dataclass
class EveConfig:
    intercept_fraction: float = 0.0     # 0..1, intercept-resend on this fraction of pulses
    pns: bool = False

    @property
    def active(self) -> bool:
        return self.intercept_fraction > 0.0 or self.pns


def intercept_resend_error(e_opt: float, fraction: float) -> float:
    """Photon error rate with intercept-resend on ``fraction`` of the pulses (wrong basis half the time, then a
    wrong bit half the time: 25%)."""
    q = 0.25 * min(max(fraction, 0.0), 1.0)
    return e_opt * (1.0 - q) + (1.0 - e_opt) * q


class Eve:
    """Eavesdropper attached to the channel."""

    def __init__(self, cfg: dict[str, Any] | None = None) -> None:
        """``cfg`` is config/sandbox.yaml (its ``eve`` section sets the initial mode)."""
        c = (cfg or {}).get("eve") or {}
        self.default_fraction = float(c.get("intercept_fraction", 0.5))
        self.mode = "off"
        self.fraction = 0.0
        self.set_mode(c.get("mode", "off"))

    def set_mode(self, mode: str | None, fraction: float | None = None) -> None:
        """Enable an attack mode ("intercept_resend", "pns") or disable it ("off" / None)."""
        mode = "off" if mode in (None, "", False) else str(mode)
        if mode not in MODES:
            raise ValueError(f"unknown Eve mode {mode!r}; expected one of {MODES}")
        self.mode = mode
        self.fraction = (self.default_fraction if fraction is None else float(fraction)) \
            if mode == "intercept_resend" else 0.0

    def handle(self, msg: dict[str, Any]) -> None:
        """Apply a toggle message from port 5560."""
        self.set_mode(msg.get("mode"), msg.get("fraction"))

    @property
    def config(self) -> EveConfig:
        return EveConfig(intercept_fraction=self.fraction, pns=self.mode == "pns")

    def apply(self, detections: dict[str, Any]) -> dict[str, Any]:
        """The attack is applied inside the channel simulation (it changes the per-class photon statistics,
        which cannot be done after the fact). Returns ``detections`` tagged with the active attack."""
        return {**detections, "eve": {"mode": self.mode, "fraction": self.fraction}}

    def describe(self) -> str:
        if self.mode == "intercept_resend":
            return f"intercept-resend on {100 * self.fraction:.0f}% of pulses"
        return "photon-number splitting" if self.mode == "pns" else "off"
