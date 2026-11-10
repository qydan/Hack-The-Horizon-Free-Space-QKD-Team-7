"""Decoy-state BB84 with a photon-number-splitting (PNS) attacker.

Real "single-photon" sources are weak laser pulses: the photon number n is Poisson
distributed, so some pulses carry n >= 2 photons. A PNS attacker keeps one photon of
those pulses and learns the bit without adding any QBER. Decoy states defend by having
Alice randomly vary the pulse intensity (signal mu, decoy nu, vacuum 0); Bob's counts per
intensity class must follow the Poisson/yield model, which a PNS attacker cannot fake.

Channel model. In the sandbox (``channel.simulate_message``) the inputs come from the
error budget, which is driven by the measured telemetry:
    eta   = transmittance x detector efficiency (``error_budget.transmittance``)
    e_d   = optical error ``e_opt`` (raised by the Eve emulator's intercept-resend)
    Y0    = dark-count + background probability per pulse

Standalone (``run_decoy_analysis``, used by the tests) eta defaults to ``eta0 * intensity``.

    yield of an n-photon pulse     Y_n = Y0 + (1 - Y0)(1 - (1 - eta)^n)
    gain of intensity lambda       Q_l = sum_n Poisson(n; l) * Y_n
    error rate                     E_l = (0.5*Y0 + e_d*(Q_l - Y0)) / Q_l

Analysis: vacuum + weak decoy bounds (Ma, Qi, Zhao, Lo 2005) with a conservative
Gaussian-style statistical margin (``k_sigma``). This is a hackathon-grade approximation,
not a composable finite-key proof.
"""
import math
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

from qkd_core import binary_entropy  # noqa: F401  (re-exported for convenience)


@dataclass
class DecoyParams:
    mu: float = 0.5            # signal mean photon number
    nu: float = 0.1            # weak decoy mean photon number
    p_signal: float = 0.7      # fraction of pulses sent as signal
    p_decoy: float = 0.2       # fraction sent as decoy (remainder is vacuum)
    eta0: float = 0.05         # standalone only: transmittance at intensity == 1.0 (the sandbox passes eta)
    dark_yield: float = 2e-4   # Y0
    k_sigma: float = 5.0       # statistical margin
    n_pulses: int = 20_000_000  # standalone only: pulses per window (the sandbox passes pulse_rate x dt)


@dataclass
class DecoyResult:
    gain: Dict[str, float]            # observed Q per class
    error: Dict[str, float]           # observed E per class
    y1_lower: float                   # lower bound on single-photon yield
    e1_upper: float                   # upper bound on single-photon error rate
    single_photon_fraction: float     # lower bound on Delta = Q1 / Q_mu
    consistent: bool                  # do per-class gains match the no-attack model?
    note: str = ""

    @property
    def usable(self) -> bool:
        return self.single_photon_fraction > 0.0 and self.y1_lower > 0.0


def _poisson_pmf_ge2(lam: float) -> float:
    return 1.0 - math.exp(-lam) * (1.0 + lam)


def expected_gain(p: DecoyParams, lam: float, eta: float) -> float:
    """Q_lambda in the absence of an attacker."""
    return p.dark_yield + (1.0 - p.dark_yield) * (1.0 - math.exp(-eta * lam))


def class_statistics(params: DecoyParams, lam: float, eta: float, p_error: float,
                     pns_attack: bool) -> Dict[str, float]:
    """Exact per-pulse detection and error probabilities for one intensity class."""
    if pns_attack:
        # Eve blocks n <= 1 pulses and forwards n >= 2 pulses (minus the photon she keeps)
        # over a lossless channel, thinned so that the *signal* gain matches the no-attack
        # expectation. She cannot see the intensity class, so the same thinning applies to
        # decoy and vacuum pulses.
        p_multi_signal = _poisson_pmf_ge2(params.mu)
        forward_prob = min(1.0, expected_gain(params, params.mu, eta) / p_multi_signal) if p_multi_signal > 0 else 0.0
        p_photon = forward_prob * _poisson_pmf_ge2(lam)
    else:
        p_photon = 1.0 - math.exp(-eta * lam)
    p_dark_only = (1.0 - p_photon) * params.dark_yield
    return {
        "p_detect": p_photon + p_dark_only,
        # real photons flip with the optical error rate; dark counts are 50/50
        "p_error": p_photon * p_error + 0.5 * p_dark_only,
    }


def simulate(params: DecoyParams, intensity: float, p_error: float,
             pns_attack: bool = False, rng: Optional[np.random.Generator] = None,
             eta: Optional[float] = None, n_pulses: Optional[int] = None) -> Dict[str, Dict[str, int]]:
    """Simulate one window of decoy-state pulses. Returns {class: {"sent", "detected", "errors"}}.

    Counts are drawn from their exact binomial distributions, which is statistically
    identical to simulating every pulse but lets us use realistic pulse counts
    (``params.n_pulses``, e.g. a 20 MHz source over a 1 s window).

    ``pns_attack``: photon-number-splitting attacker, see ``class_statistics``.
    ``eta`` overrides the transmittance (default ``eta0 * intensity``); in the sandbox it
    comes from the error budget. ``n_pulses`` overrides ``params.n_pulses``.
    """
    rng = rng or np.random.default_rng()
    if eta is None:
        eta = params.eta0 * float(np.clip(intensity, 0.0, 1.0))
    n_total = params.n_pulses if n_pulses is None else n_pulses
    p_error = float(np.clip(p_error, 0.0, 0.5))
    probs = [params.p_signal, params.p_decoy, 1.0 - params.p_signal - params.p_decoy]
    sent = rng.multinomial(n_total, probs)
    out = {}
    for idx, (name, lam) in enumerate((("signal", params.mu), ("decoy", params.nu), ("vacuum", 0.0))):
        st = class_statistics(params, lam, eta, p_error, pns_attack)
        detected = int(rng.binomial(sent[idx], st["p_detect"]))
        errors = int(rng.binomial(sent[idx], min(1.0, st["p_error"])))
        out[name] = {"sent": int(sent[idx]), "detected": detected, "errors": min(errors, detected)}
    return out


def _upper(count: int, total: int, k: float) -> float:
    return (count + k * math.sqrt(count + 1.0)) / total


def _lower(count: int, total: int, k: float) -> float:
    return max(0.0, count - k * math.sqrt(count + 1.0)) / total


def analyze(params: DecoyParams, counts: Dict[str, Dict[str, int]], eta_expected: float,
            expected_gain_by_class: Optional[Dict[str, float]] = None) -> DecoyResult:
    """Vacuum + weak-decoy bounds on the single-photon yield, error rate and fraction.

    ``expected_gain_by_class`` ({"signal": Q, "decoy": Q}) overrides the no-attack model
    used by the consistency check. Needed when the transmittance varied during the block
    (the mean of 1 - exp(-eta*lam) is not 1 - exp(-mean(eta)*lam))."""
    mu, nu, k = params.mu, params.nu, params.k_sigma
    s, d, v = counts["signal"], counts["decoy"], counts["vacuum"]

    q_mu = s["detected"] / s["sent"]
    q_nu = d["detected"] / d["sent"]
    q_vac = v["detected"] / v["sent"]
    e_mu = s["errors"] / max(1, s["detected"])
    e_nu = d["errors"] / max(1, d["detected"])

    # Conservative directions for each term in Y1_L (increasing in Q_nu, decreasing in Q_mu, Y0)
    q_nu_lo = _lower(d["detected"], d["sent"], k)
    q_mu_hi = _upper(s["detected"], s["sent"], k)
    y0_hi = _upper(v["detected"], v["sent"], k)

    y1_lower = (mu / (nu * (mu - nu))) * (
        q_nu_lo * math.exp(nu)
        - q_mu_hi * math.exp(mu) * (nu ** 2) / (mu ** 2)
        - ((mu ** 2 - nu ** 2) / (mu ** 2)) * y0_hi
    )
    y1_lower = max(0.0, y1_lower)

    err_nu_hi = _upper(d["errors"], d["sent"], k)       # E_nu * Q_nu, upper
    if y1_lower > 0:
        e1_upper = min(0.5, max(0.0, (err_nu_hi * math.exp(nu) - 0.5 * _lower(v["detected"], v["sent"], k)) / (y1_lower * nu)))
    else:
        e1_upper = 0.5

    q1_lower = y1_lower * mu * math.exp(-mu)
    delta = min(1.0, q1_lower / q_mu_hi) if q_mu_hi > 0 else 0.0

    # Consistency check: observed gains should match the Poisson model for the measured eta
    consistent = True
    notes = []
    for name, lam, c in (("signal", mu, s), ("decoy", nu, d)):
        exp_q = (expected_gain_by_class[name] if expected_gain_by_class
                 else expected_gain(params, lam, eta_expected))
        lo = _lower(c["detected"], c["sent"], k + 2)
        hi = _upper(c["detected"], c["sent"], k + 2)
        if not (lo <= exp_q <= hi):
            consistent = False
            notes.append(f"{name} gain {c['detected']/c['sent']:.5f} != model {exp_q:.5f}")

    return DecoyResult(
        gain={"signal": q_mu, "decoy": q_nu, "vacuum": q_vac},
        error={"signal": e_mu, "decoy": e_nu},
        y1_lower=y1_lower,
        e1_upper=e1_upper,
        single_photon_fraction=delta if consistent else 0.0,
        consistent=consistent,
        note="; ".join(notes),
    )


def run_decoy_analysis(params: DecoyParams, intensity: float, p_error: float,
                       pns_attack: bool = False,
                       rng: Optional[np.random.Generator] = None) -> DecoyResult:
    """Simulate a block of decoy-state pulses and analyze it. Convenience wrapper.

    Beyond the key-rate bounds, the per-class gains are compared with the no-attack model
    (``consistent``). A mismatch means the channel is not behaving like a lossy fibre/air
    path (e.g. PNS), and the single-photon fraction is set to zero.
    """
    counts = simulate(params, intensity, p_error, pns_attack, rng)
    eta = params.eta0 * float(np.clip(intensity, 0.0, 1.0))
    return analyze(params, counts, eta)
