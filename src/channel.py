"""Channel + decoys + Eve emulator.

For every telemetry message the sandbox pretends to send a stream of weak pulses. Each
pulse is randomly signal (mean photon number mu), decoy (nu) or vacuum (0); its photon
count is a Poisson draw and every photon survives with the transmittance from the error
budget. Detector dark counts, background and a 2% optical error are added on top.

Per-class click and error counts are drawn from their exact binomial distributions
(statistically identical to simulating every pulse, but fast enough for the default 1 GHz source).

Eve emulator (off by default):
* intercept-resend on a fraction of pulses: adds 25% error on the photons it touches
* photon-number splitting (PNS): keeps a photon of multi-photon pulses and blocks the rest;
  QBER stays low but the per-class gains stop following the Poisson model, which the decoy
  check in ``decoy.analyze`` catches.
"""
from dataclasses import dataclass, field, replace
from typing import Dict, Optional

import numpy as np

from decoy import DecoyParams, expected_gain, simulate
from error_budget import Budget, BudgetParams

CLASSES = ("signal", "decoy", "vacuum")


@dataclass
class EveConfig:
    intercept_fraction: float = 0.0   # 0..1, intercept-resend on this fraction of pulses
    pns: bool = False

    @property
    def active(self) -> bool:
        return self.intercept_fraction > 0.0 or self.pns


@dataclass
class BlockCounts:
    """Pooled click statistics of a stretch of telemetry (a 100 ms interval or a key block)."""
    sent: Dict[str, int] = field(default_factory=lambda: {c: 0 for c in CLASSES})
    detected: Dict[str, int] = field(default_factory=lambda: {c: 0 for c in CLASSES})
    errors: Dict[str, int] = field(default_factory=lambda: {c: 0 for c in CLASSES})
    expected_detected: Dict[str, float] = field(default_factory=lambda: {c: 0.0 for c in CLASSES})
    exp_err: float = 0.0        # expected signal-class errors / clicks, from the error-budget formula
    exp_clicks: float = 0.0
    eta_pulses: float = 0.0     # sum of eta * pulses, for the mean transmittance
    pulses: int = 0
    locked_pulses: int = 0

    def add(self, other: "BlockCounts") -> None:
        for c in CLASSES:
            self.sent[c] += other.sent[c]
            self.detected[c] += other.detected[c]
            self.errors[c] += other.errors[c]
            self.expected_detected[c] += other.expected_detected[c]
        self.exp_err += other.exp_err
        self.exp_clicks += other.exp_clicks
        self.eta_pulses += other.eta_pulses
        self.pulses += other.pulses
        self.locked_pulses += other.locked_pulses

    @property
    def mean_eta(self) -> float:
        return self.eta_pulses / self.pulses if self.pulses else 0.0

    @property
    def expected_qber(self) -> float:
        """Pooled QBER predicted by the error budget (clicks-weighted, like the real one)."""
        return self.exp_err / self.exp_clicks if self.exp_clicks > 0 else 0.5

    @property
    def signal_qber(self) -> float:
        """Raw QBER of the signal class (errors / clicks on matched bases)."""
        d = self.detected["signal"]
        return self.errors["signal"] / d if d else 0.5

    def as_decoy_counts(self) -> Dict[str, Dict[str, int]]:
        return {c: {"sent": self.sent[c], "detected": self.detected[c], "errors": self.errors[c]}
                for c in CLASSES}

    def expected_gains(self) -> Dict[str, float]:
        return {c: self.expected_detected[c] / self.sent[c] for c in ("signal", "decoy") if self.sent[c]}


def intercept_resend_error(e_opt: float, fraction: float) -> float:
    """Photon error rate with intercept-resend on ``fraction`` of the pulses: Eve guesses the
    wrong basis half the time and then flips the bit half the time (25%)."""
    q = 0.25 * min(max(fraction, 0.0), 1.0)
    return e_opt * (1.0 - q) + (1.0 - e_opt) * q


def simulate_message(budget_params: BudgetParams, decoy_params: DecoyParams, b: Budget,
                     n_pulses: int, eve: EveConfig, rng: np.random.Generator) -> BlockCounts:
    """Send ``n_pulses`` pulses through the channel described by ``b``."""
    p = budget_params
    eta_eff = b.eta * p.eta_d
    dp = replace(decoy_params, mu=p.mu, dark_yield=min(0.5, p.p_dark + b.p_bg))
    e_photon = intercept_resend_error(p.e_opt, eve.intercept_fraction)

    raw = simulate(dp, 0.0, e_photon, pns_attack=eve.pns, rng=rng, eta=eta_eff, n_pulses=n_pulses)

    out = BlockCounts()
    for c, lam in (("signal", p.mu), ("decoy", dp.nu), ("vacuum", 0.0)):
        out.sent[c] = raw[c]["sent"]
        out.detected[c] = raw[c]["detected"]
        out.errors[c] = raw[c]["errors"]
        out.expected_detected[c] = raw[c]["sent"] * expected_gain(dp, lam, eta_eff)
    p_sig = 1.0 - np.exp(-p.mu * b.eta * p.eta_d)
    p_noise = p.p_dark + b.p_bg
    n_sig = out.sent["signal"]
    out.exp_clicks = n_sig * (p_sig + p_noise)
    out.exp_err = n_sig * (e_photon * p_sig + 0.5 * p_noise)
    out.eta_pulses = b.eta * n_pulses
    out.pulses = n_pulses
    out.locked_pulses = n_pulses if b.locked else 0
    return out
