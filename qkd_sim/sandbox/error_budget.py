"""Error budget: I2 telemetry -> transmittance (fraction of photons that would arrive) -> QBER.

Ported from src/error_budget.py. Inputs are telemetry/schema.json records (dicts).

    eta = eta_geo(L) * exp(-2 theta_rx^2 / theta_fov^2) * eta_tx(theta_tx) * I

* eta_geo(L)  range loss: receiver lens area / beam spot area (``eta_geo_mode: fixed`` uses ``cal_range_m``)
* theta_rx    receiver pointing error = the measured tracking ``residual_mrad`` (gimbal or virtual receiver lag,
              steps and jitter are already inside it)
* theta_fov   *virtual* receiver half-width (1/e^2, ``theta_fov_virtual_mrad``), narrow enough that a few mrad
              of ground error visibly costs signal
* theta_tx    drone (transmitter) pointing = lean + drift, with a flat-topped eta_tx (about 1 inside +-5 deg)
* I           beacon brightness against the preflight / in-run reference (``intensity``)

``lean_deg`` / ``range_m`` may be null in telemetry (no tag pose): lean counts as 0, range as ``cal_range_m``.
Only TRACK and COAST count as locked; SEARCH and ACQUIRE carry no signal.

    QBER = (e_opt p_sig + p_noise / 2) / (p_sig + p_noise),  p_sig = 1 - exp(-mu eta eta_d),  p_noise = p_dark

Config: ``config/sandbox.yaml``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

LOCKED_STATES = ("TRACK", "COAST")


def lin_to_db(x: float) -> float:
    return -10.0 * math.log10(max(x, 1e-30))


@dataclass
class BudgetParams:
    lens_diameter_m: float = 0.025
    beam_full_angle_deg: float = 10.0
    theta_fov_mrad: float = 10.0
    tx_half_width_mrad: float = 87.0
    tx_flatness: int = 8
    mu: float = 0.5
    eta_d: float = 0.6
    e_opt: float = 0.02
    p_dark: float = 1.0e-5
    cal_range_m: float = 3.0
    eta_geo_mode: str = "lens_over_spot"
    abort_qber: float = 0.11

    @classmethod
    def from_cfg(cls, cfg: dict[str, Any]) -> "BudgetParams":
        p = cls(lens_diameter_m=float(cfg["lens_diameter_m"]), beam_full_angle_deg=float(cfg["beam_full_angle_deg"]),
                theta_fov_mrad=float(cfg["theta_fov_virtual_mrad"]), tx_half_width_mrad=float(cfg["tx_half_width_mrad"]),
                tx_flatness=int(cfg["tx_flatness"]), mu=float(cfg["mu"]), eta_d=float(cfg["eta_d"]),
                e_opt=float(cfg["e_opt"]), cal_range_m=float(cfg["cal_range_m"]),
                eta_geo_mode=str(cfg["eta_geo_mode"]), abort_qber=float(cfg["abort_qber"]))
        if cfg.get("p_dark") is None:
            p.calibrate_dark(float(cfg["on_axis_snr"]))
        else:
            p.p_dark = float(cfg["p_dark"])
        return p

    def calibrate_dark(self, on_axis_snr: float = 50.0) -> float:
        """Choose p_dark so the on-axis signal is ``on_axis_snr`` times the noise at the calibration range."""
        eta = geometric_loss(self, self.cal_range_m)
        p_sig = 1.0 - math.exp(-self.mu * eta * self.eta_d)
        self.p_dark = p_sig / on_axis_snr
        return self.p_dark


def geometric_loss(p: BudgetParams, range_m: float) -> float:
    """eta_geo(L) = receiver lens area / beam spot area (flat-topped beam), capped at 1."""
    spot_d = 2.0 * max(range_m, 1e-3) * math.tan(math.radians(p.beam_full_angle_deg / 2.0))
    return min(1.0, (p.lens_diameter_m / spot_d) ** 2)


def rx_pointing_factor(p: BudgetParams, residual_mrad: float) -> float:
    return math.exp(-2.0 * residual_mrad ** 2 / p.theta_fov_mrad ** 2)


def tx_pointing_angle_mrad(lean_deg: float, drift_mrad: float) -> float:
    """theta_tx = lean + drift (drift is already an angle)."""
    return math.radians(abs(lean_deg)) * 1000.0 + abs(drift_mrad)


def tx_pointing_factor(p: BudgetParams, theta_tx_mrad: float) -> float:
    """Flat-topped beam: about 1 inside the half-width, -3 dB at ``tx_half_width_mrad``."""
    return 1.0 / (1.0 + (theta_tx_mrad / p.tx_half_width_mrad) ** (2 * p.tx_flatness))


@dataclass
class Budget:
    eta: float              # transmittance (photons arriving / photons sent), before the detector
    eta_geo: float
    f_rx: float
    f_tx: float
    brightness: float
    locked: bool

    @property
    def loss_db(self) -> float:
        return lin_to_db(self.eta)


def record_range(p: BudgetParams, rec: dict[str, Any]) -> float:
    r = rec.get("range_m")
    return p.cal_range_m if (r is None or p.eta_geo_mode == "fixed") else float(r)


def record_lean(rec: dict[str, Any]) -> float:
    v = rec.get("lean_deg")
    return 0.0 if v is None else float(v)


def is_locked(rec: dict[str, Any]) -> bool:
    return rec.get("state") in LOCKED_STATES


def transmittance(p: BudgetParams, rec: dict[str, Any]) -> Budget:
    """One telemetry record -> transmittance."""
    geo = geometric_loss(p, record_range(p, rec))
    if not is_locked(rec):                     # beacon lost: nothing reaches the receiver
        return Budget(0.0, geo, 0.0, 0.0, 0.0, False)
    f_rx = rx_pointing_factor(p, float(rec["residual_mrad"]))
    f_tx = tx_pointing_factor(p, tx_pointing_angle_mrad(record_lean(rec), float(rec["drift_mrad"])))
    bright = max(0.0, min(float(rec["intensity"]), 1.0))
    return Budget(geo * f_rx * f_tx * bright, geo, f_rx, f_tx, bright, True)


def qber_estimate(p: BudgetParams, b: Budget, eve_fraction: float = 0.0) -> float:
    """Expected sifted QBER for a transmittance. Intercept-resend on ``eve_fraction`` of the pulses adds 25%
    error on those pulses' photons."""
    p_sig = 1.0 - math.exp(-p.mu * b.eta * p.eta_d)
    e = p.e_opt
    if eve_fraction > 0.0:
        q = 0.25 * min(1.0, eve_fraction)
        e = e * (1.0 - q) + (1.0 - e) * q
    return (e * p_sig + 0.5 * p.p_dark) / (p_sig + p.p_dark)


def pointing_loss_to_gate_db(p: BudgetParams) -> float:
    """How many dB of extra loss take the on-axis QBER up to the abort gate."""
    on = geometric_loss(p, p.cal_range_m)
    lo, hi = 0.0, 80.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if qber_estimate(p, Budget(on * 10 ** (-mid / 10), on, 1, 1, 1, True)) < p.abort_qber:
            lo = mid
        else:
            hi = mid
    return lo


# ----------------------------------------------------------------------------- interface of the original stub
def geometric_efficiency(record: dict[str, Any], cfg: dict[str, Any]) -> float:
    """Collection efficiency for one telemetry record (eta_geo_mode, theta_fov_virtual_mrad): the full
    transmittance before the detector."""
    return transmittance(BudgetParams.from_cfg(cfg), record).eta


def block_budget(records: list[dict[str, Any]], cfg: dict[str, Any]) -> dict[str, float]:
    """Aggregate one block of records (time-weighted) into channel parameters."""
    p = BudgetParams.from_cfg(cfg)
    if not records:
        return {"eta_mean": 0.0, "loss_db": lin_to_db(0.0), "qber_expected": 0.5, "locked_fraction": 0.0}
    ts = [float(r["t"]) for r in records]
    w = [max(0.0, b - a) for a, b in zip(ts, ts[1:])] + [0.0]
    if sum(w) == 0:
        w = [1.0] * len(records)
    bs = [transmittance(p, r) for r in records]
    tw = sum(w)
    eta = sum(wi * b.eta for wi, b in zip(w, bs)) / tw
    p_sig = sum(wi * (1.0 - math.exp(-p.mu * b.eta * p.eta_d)) for wi, b in zip(w, bs)) / tw
    q = (p.e_opt * p_sig + 0.5 * p.p_dark) / (p_sig + p.p_dark)
    return {"eta_mean": eta, "loss_db": lin_to_db(eta), "qber_expected": q,
            "locked_fraction": sum(wi for wi, b in zip(w, bs) if b.locked) / tw, "p_dark": p.p_dark}
