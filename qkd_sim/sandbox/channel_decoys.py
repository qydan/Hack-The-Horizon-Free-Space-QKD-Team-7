"""Decoy-state channel model and analysis (ported from src/channel.py + src/decoy.py).

For every telemetry record the sandbox sends ``pulse_rate_hz * dt`` weak pulses. Each pulse is randomly signal
(mean photon number mu), decoy (nu) or vacuum; its photon count is Poisson and every photon survives with the
transmittance from the error budget (times the detector efficiency). Dark counts / background and the optical
error are added. Per-class click and error counts are drawn from their exact binomial distributions
(statistically identical to simulating every pulse, fast enough for a 1 GHz source).

    yield of an n-photon pulse     Y_n = Y0 + (1 - Y0)(1 - (1 - eta)^n)
    gain of intensity lambda       Q_l = sum_n Poisson(n; l) Y_n
    error rate                     E_l = (0.5 Y0 + e_d (Q_l - Y0)) / Q_l

Analysis: vacuum + weak-decoy bounds (Ma, Qi, Zhao, Lo 2005) with a Gaussian-style margin (``k_sigma``), plus a
consistency check of the per-class gains against the no-attack Poisson model, which a photon-number-splitting
attacker cannot fake. Hackathon-grade approximation, not a composable finite-key proof.

Config: ``config/sandbox.yaml`` (mu, eta_d, p_dark via the error budget; ``decoy.*``).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np

from sandbox.error_budget import Budget, BudgetParams
from sandbox.eve import EveConfig, intercept_resend_error

CLASSES = ("signal", "decoy", "vacuum")


# ----------------------------------------------------------------------------- decoy model
@dataclass
class DecoyParams:
    mu: float = 0.5
    nu: float = 0.1
    p_signal: float = 0.7
    p_decoy: float = 0.2
    eta0: float = 0.05              # standalone only: transmittance at intensity 1 (the sandbox passes eta)
    dark_yield: float = 2e-4        # Y0
    k_sigma: float = 5.0
    n_pulses: int = 20_000_000      # standalone only

    @classmethod
    def from_cfg(cls, cfg: dict[str, Any], budget: BudgetParams | None = None) -> "DecoyParams":
        d = cfg.get("decoy", {})
        p = cls(nu=float(d.get("nu", 0.1)), p_signal=float(d.get("p_signal", 0.7)),
                p_decoy=float(d.get("p_decoy", 0.2)), k_sigma=float(d.get("k_sigma", 5.0)))
        if budget is not None:
            p = replace(p, mu=budget.mu, dark_yield=min(0.5, budget.p_dark))
        return p


@dataclass
class DecoyResult:
    gain: dict[str, float]
    error: dict[str, float]
    y1_lower: float
    e1_upper: float
    single_photon_fraction: float
    consistent: bool
    note: str = ""

    @property
    def usable(self) -> bool:
        return self.single_photon_fraction > 0.0 and self.y1_lower > 0.0


def _poisson_pmf_ge2(lam: float) -> float:
    return 1.0 - math.exp(-lam) * (1.0 + lam)


def expected_gain(p: DecoyParams, lam: float, eta: float) -> float:
    """Q_lambda without an attacker."""
    return p.dark_yield + (1.0 - p.dark_yield) * (1.0 - math.exp(-eta * lam))


def class_statistics(params: DecoyParams, lam: float, eta: float, p_error: float, pns_attack: bool) -> dict[str, float]:
    """Exact per-pulse detection and error probabilities for one intensity class."""
    if pns_attack:
        # Eve blocks n <= 1 pulses and forwards n >= 2 (minus the photon she keeps) over a lossless channel,
        # thinned so the *signal* gain matches the no-attack expectation. She can't see the class, so the same
        # thinning hits decoy and vacuum pulses.
        p_multi_signal = _poisson_pmf_ge2(params.mu)
        forward = min(1.0, expected_gain(params, params.mu, eta) / p_multi_signal) if p_multi_signal > 0 else 0.0
        p_photon = forward * _poisson_pmf_ge2(lam)
    else:
        p_photon = 1.0 - math.exp(-eta * lam)
    p_dark_only = (1.0 - p_photon) * params.dark_yield
    return {"p_detect": p_photon + p_dark_only, "p_error": p_photon * p_error + 0.5 * p_dark_only}


def simulate(params: DecoyParams, intensity: float, p_error: float, pns_attack: bool = False, rng=None,
             eta: float | None = None, n_pulses: int | None = None) -> dict[str, dict[str, int]]:
    """One window of decoy-state pulses -> {class: {"sent", "detected", "errors"}} (exact binomial draws)."""
    rng = rng or np.random.default_rng()
    if eta is None:
        eta = params.eta0 * float(np.clip(intensity, 0.0, 1.0))
    n_total = params.n_pulses if n_pulses is None else n_pulses
    p_error = float(np.clip(p_error, 0.0, 0.5))
    sent = rng.multinomial(n_total, [params.p_signal, params.p_decoy, 1.0 - params.p_signal - params.p_decoy])
    out = {}
    for i, (name, lam) in enumerate((("signal", params.mu), ("decoy", params.nu), ("vacuum", 0.0))):
        st = class_statistics(params, lam, eta, p_error, pns_attack)
        detected = int(rng.binomial(sent[i], st["p_detect"]))
        errors = int(rng.binomial(sent[i], min(1.0, st["p_error"])))
        out[name] = {"sent": int(sent[i]), "detected": detected, "errors": min(errors, detected)}
    return out


def _upper(count: int, total: int, k: float) -> float:
    return (count + k * math.sqrt(count + 1.0)) / total


def _lower(count: int, total: int, k: float) -> float:
    return max(0.0, count - k * math.sqrt(count + 1.0)) / total


def analyze(params: DecoyParams, counts: dict[str, dict[str, int]], eta_expected: float,
            expected_gain_by_class: dict[str, float] | None = None) -> DecoyResult:
    """Vacuum + weak-decoy bounds on the single-photon yield, error rate and fraction, and the PNS consistency
    check. ``expected_gain_by_class`` overrides the no-attack model (needed when eta varied within the block:
    the mean of 1 - exp(-eta lam) is not 1 - exp(-mean(eta) lam))."""
    mu, nu, k = params.mu, params.nu, params.k_sigma
    s, d, v = counts["signal"], counts["decoy"], counts["vacuum"]
    q_mu = s["detected"] / s["sent"]
    q_nu = d["detected"] / d["sent"]
    q_vac = v["detected"] / v["sent"]
    q_nu_lo = _lower(d["detected"], d["sent"], k)
    q_mu_hi = _upper(s["detected"], s["sent"], k)
    y0_hi = _upper(v["detected"], v["sent"], k)
    y1_lower = max(0.0, (mu / (nu * (mu - nu))) * (
        q_nu_lo * math.exp(nu) - q_mu_hi * math.exp(mu) * (nu ** 2) / (mu ** 2)
        - ((mu ** 2 - nu ** 2) / (mu ** 2)) * y0_hi))
    err_nu_hi = _upper(d["errors"], d["sent"], k)
    if y1_lower > 0:
        e1_upper = min(0.5, max(0.0, (err_nu_hi * math.exp(nu) - 0.5 * _lower(v["detected"], v["sent"], k))
                                / (y1_lower * nu)))
    else:
        e1_upper = 0.5
    q1_lower = y1_lower * mu * math.exp(-mu)
    delta = min(1.0, q1_lower / q_mu_hi) if q_mu_hi > 0 else 0.0
    consistent, notes = True, []
    for name, lam, c in (("signal", mu, s), ("decoy", nu, d)):
        exp_q = expected_gain_by_class[name] if expected_gain_by_class else expected_gain(params, lam, eta_expected)
        if not (_lower(c["detected"], c["sent"], k + 2) <= exp_q <= _upper(c["detected"], c["sent"], k + 2)):
            consistent = False
            notes.append(f"{name} gain {c['detected'] / c['sent']:.5f} != model {exp_q:.5f}")
    return DecoyResult(gain={"signal": q_mu, "decoy": q_nu, "vacuum": q_vac},
                       error={"signal": s["errors"] / max(1, s["detected"]), "decoy": d["errors"] / max(1, d["detected"])},
                       y1_lower=y1_lower, e1_upper=e1_upper,
                       single_photon_fraction=delta if consistent else 0.0, consistent=consistent,
                       note="; ".join(notes))


def run_decoy_analysis(params: DecoyParams, intensity: float, p_error: float, pns_attack: bool = False,
                       rng=None) -> DecoyResult:
    """Simulate a standalone block and analyse it (tests)."""
    counts = simulate(params, intensity, p_error, pns_attack, rng)
    return analyze(params, counts, params.eta0 * float(np.clip(intensity, 0.0, 1.0)))


# ----------------------------------------------------------------------------- pooled click statistics
@dataclass
class BlockCounts:
    """Pooled click statistics of a stretch of telemetry (a 100 ms interval or a key block)."""
    sent: dict[str, int] = field(default_factory=lambda: {c: 0 for c in CLASSES})
    detected: dict[str, int] = field(default_factory=lambda: {c: 0 for c in CLASSES})
    errors: dict[str, int] = field(default_factory=lambda: {c: 0 for c in CLASSES})
    expected_detected: dict[str, float] = field(default_factory=lambda: {c: 0.0 for c in CLASSES})
    exp_err: float = 0.0
    exp_clicks: float = 0.0
    eta_pulses: float = 0.0
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
        """Pooled QBER predicted by the error budget (click-weighted, like the real one)."""
        return self.exp_err / self.exp_clicks if self.exp_clicks > 0 else 0.5

    @property
    def signal_qber(self) -> float:
        d = self.detected["signal"]
        return self.errors["signal"] / d if d else 0.5

    def as_decoy_counts(self) -> dict[str, dict[str, int]]:
        return {c: {"sent": self.sent[c], "detected": self.detected[c], "errors": self.errors[c]} for c in CLASSES}

    def expected_gains(self) -> dict[str, float]:
        return {c: self.expected_detected[c] / self.sent[c] for c in ("signal", "decoy") if self.sent[c]}


def simulate_message(budget_params: BudgetParams, decoy_params: DecoyParams, b: Budget, n_pulses: int,
                     eve: EveConfig, rng) -> BlockCounts:
    """Send ``n_pulses`` through the channel described by ``b`` (one telemetry record)."""
    p = budget_params
    eta_eff = b.eta * p.eta_d
    dp = replace(decoy_params, mu=p.mu, dark_yield=min(0.5, p.p_dark))
    e_photon = intercept_resend_error(p.e_opt, eve.intercept_fraction)
    raw = simulate(dp, 0.0, e_photon, pns_attack=eve.pns, rng=rng, eta=eta_eff, n_pulses=n_pulses)
    out = BlockCounts()
    for c, lam in (("signal", p.mu), ("decoy", dp.nu), ("vacuum", 0.0)):
        out.sent[c] = raw[c]["sent"]
        out.detected[c] = raw[c]["detected"]
        out.errors[c] = raw[c]["errors"]
        out.expected_detected[c] = raw[c]["sent"] * expected_gain(dp, lam, eta_eff)
    p_sig = 1.0 - np.exp(-p.mu * b.eta * p.eta_d)
    n_sig = out.sent["signal"]
    out.exp_clicks = n_sig * (p_sig + p.p_dark)
    out.exp_err = n_sig * (e_photon * p_sig + 0.5 * p.p_dark)
    out.eta_pulses = b.eta * n_pulses
    out.pulses = n_pulses
    out.locked_pulses = n_pulses if b.locked else 0
    return out


# ----------------------------------------------------------------------------- interface of the original stub
def detection_probabilities(budget: dict[str, float], cfg: dict[str, Any]) -> dict[str, float]:
    """Per-pulse detection probability of each class for a transmittance ``budget["eta"]`` (no attacker)."""
    bp = BudgetParams.from_cfg(cfg)
    dp = DecoyParams.from_cfg(cfg, bp)
    eta_eff = float(budget["eta"]) * bp.eta_d
    return {c: class_statistics(dp, lam, eta_eff, bp.e_opt, False)["p_detect"]
            for c, lam in (("signal", dp.mu), ("decoy", dp.nu), ("vacuum", 0.0))}


def simulate_detections(n_pulses: int, budget: dict[str, float], cfg: dict[str, Any], rng: Any,
                        eve: EveConfig | None = None) -> dict[str, Any]:
    """Draw detection events for ``n_pulses`` at transmittance ``budget["eta"]``; returns per-class counts."""
    bp = BudgetParams.from_cfg(cfg)
    b = Budget(float(budget["eta"]), 0.0, 1.0, 1.0, 1.0, float(budget["eta"]) > 0)
    return simulate_message(bp, DecoyParams.from_cfg(cfg, bp), b, int(n_pulses), eve or EveConfig(), rng).as_decoy_counts()
