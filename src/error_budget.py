"""Error budget: telemetry -> transmittance (fraction of photons that would arrive) -> QBER.

    eta = eta_geo(L) * exp(-2 theta_rx^2 / theta_fov^2) * eta_tx(theta_tx) * I / I_cal

* eta_geo(L)  range loss: receiver lens area / beam spot area (about 31 dB at 5 m, 43 dB at 20 m)
* theta_rx    receiver pointing error = the measured tracking residual, so the ground gimbal's
              lag, step size and jitter are already inside it
* theta_fov   *virtual* receiver half-width (1/e^2, about 10 mrad), narrow enough that a few
              mrad of ground error visibly costs signal. The real view (about 54 mrad) would
              hide it.
* theta_tx    drone (transmitter) pointing = lean + drift / range, with a flat-topped
              eta_tx (about 1 inside +-5 deg, falling at the edge)
* I / I_cal   beacon brightness against the preflight calibration

QBER = (e_opt p_sig + p_noise / 2) / (p_sig + p_noise)
p_sig = 1 - exp(-mu eta eta_d),  p_noise = p_dark
"""
import math
from dataclasses import dataclass
from typing import Optional

from telemetry import TelemetryMessage

QBER_GATE = 0.11


def lin_to_db(x: float) -> float:
    return -10.0 * math.log10(max(x, 1e-30))


@dataclass
class BudgetParams:
    # range loss
    lens_diameter_m: float = 0.025          # receiver lens
    beam_full_angle_deg: float = 10.0       # drone LED / data-link beam
    # receiver (ground) pointing
    theta_fov_mrad: float = 10.0            # virtual receiver half-width
    # transmitter (drone) pointing
    tx_half_width_mrad: float = 87.0        # +-5 deg
    tx_flatness: int = 8                    # higher = flatter top, sharper edge
    # photon statistics
    mu: float = 0.5                         # signal mean photon number
    eta_d: float = 0.6                      # detector efficiency
    e_opt: float = 0.02                     # optical error (misalignment, imperfect polarisation)
    p_dark: float = 1.0e-5                  # dark-count probability per pulse (placeholder: default_params() calibrates it)
    # the range the preflight calibration was done at (the demo range)
    cal_range_m: float = 3.0

    def calibrate_dark(self, on_axis_snr: float = 50.0) -> float:
        """Choose p_dark so the on-axis signal is ``on_axis_snr`` times the noise at the demo
        range. Sets and returns p_dark."""
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
    """theta_tx = lean + drift / range. ``drift_mrad`` is already an angle (lateral offset
    divided by range), so it adds directly."""
    return math.radians(abs(lean_deg)) * 1000.0 + abs(drift_mrad)


def tx_pointing_factor(p: BudgetParams, theta_tx_mrad: float) -> float:
    """Flat-topped beam profile: about 1 inside the half-width, falling at the edge
    (-3 dB at ``tx_half_width_mrad``). Same shape as the link budget assumes."""
    return 1.0 / (1.0 + (theta_tx_mrad / p.tx_half_width_mrad) ** (2 * p.tx_flatness))


@dataclass
class Budget:
    eta: float              # transmittance (photons arriving / photons sent), before detector
    eta_geo: float
    f_rx: float
    f_tx: float
    brightness: float
    locked: bool

    @property
    def loss_db(self) -> float:
        return lin_to_db(self.eta)


def transmittance(p: BudgetParams, m: TelemetryMessage) -> Budget:
    """Turn one telemetry message into a transmittance and background level."""
    geo = geometric_loss(p, m.range_m)
    if not m.locked:                      # beacon lost: nothing reaches the receiver
        return Budget(0.0, geo, 0.0, 0.0, 0.0, False)

    f_rx = rx_pointing_factor(p, m.residual_mrad)
    f_tx = tx_pointing_factor(p, tx_pointing_angle_mrad(m.lean_deg, m.drift_mrad))
    bright = max(0.0, min(m.intensity, 1.0))
    return Budget(geo * f_rx * f_tx * bright, geo, f_rx, f_tx, bright, True)


def qber_estimate(p: BudgetParams, b: Budget, eve_fraction: float = 0.0) -> float:
    """Expected sifted QBER for a transmittance (the formula above). Intercept-resend on a
    fraction ``eve_fraction`` of pulses adds 25% error on those pulses' photons."""
    p_sig = 1.0 - math.exp(-p.mu * b.eta * p.eta_d)
    p_noise = p.p_dark
    e = p.e_opt
    if eve_fraction > 0.0:
        q = 0.25 * min(1.0, eve_fraction)
        e = e * (1.0 - q) + (1.0 - e) * q
    return (e * p_sig + 0.5 * p_noise) / (p_sig + p_noise)


def pointing_loss_to_gate_db(p: BudgetParams, b_on_axis: Optional[Budget] = None) -> float:
    """How many dB of extra loss take the on-axis QBER up to the 11% gate."""
    on = b_on_axis or Budget(geometric_loss(p, p.cal_range_m), geometric_loss(p, p.cal_range_m),
                             1.0, 1.0, 1.0, True)
    lo, hi = 0.0, 80.0
    for _ in range(60):
        mid = (lo + hi) / 2
        t = Budget(on.eta * 10 ** (-mid / 10), on.eta_geo, 1, 1, 1, True)
        if qber_estimate(p, t) < QBER_GATE:
            lo = mid
        else:
            hi = mid
    return lo


def default_params(on_axis_snr: float = 50.0) -> BudgetParams:
    p = BudgetParams()
    p.calibrate_dark(on_axis_snr)
    return p
