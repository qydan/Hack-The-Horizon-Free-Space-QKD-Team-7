import math

import pytest

from error_budget import (QBER_GATE, BudgetParams, default_params, geometric_loss, lin_to_db,
                          pointing_loss_to_gate_db, qber_estimate, transmittance, tx_pointing_angle_mrad,
                          tx_pointing_factor)
from telemetry import FINE_TRACK, SEARCH, TelemetryMessage


def m(res=0.5, lean=1.0, drift=2.0, inten=1.0, rng=3.0, state=FINE_TRACK, **kw):
    return TelemetryMessage(0.0, state, drift, 0.5, res, 0.3, lean, rng, inten, **kw)


P = default_params()


class TestRangeLoss:
    def test_matches_the_design_doc_link_budget(self):
        # "about 31 dB at 5 m and 43 dB at 20 m" for a 25 mm lens and a 10 degree beam
        assert lin_to_db(geometric_loss(P, 5.0)) == pytest.approx(31, abs=0.5)
        assert lin_to_db(geometric_loss(P, 20.0)) == pytest.approx(43, abs=0.5)

    def test_loss_grows_with_range_and_is_capped_at_one(self):
        assert geometric_loss(P, 10) < geometric_loss(P, 5) < geometric_loss(P, 1)
        assert geometric_loss(P, 0.001) == 1.0


class TestPointing:
    def test_receiver_residual_costs_signal(self):
        etas = [transmittance(P, m(res=r)).eta for r in (0, 2, 5, 10, 20)]
        assert etas == sorted(etas, reverse=True) and etas[-1] < 1e-3 * etas[0]

    def test_a_few_mrad_of_ground_error_is_visible(self):
        """The virtual receiver width is narrow enough that tracking visibly matters."""
        base = transmittance(P, m(res=0)).eta
        assert transmittance(P, m(res=3)).eta < 0.85 * base

    def test_measured_residual_is_what_counts_not_gimbal_lag(self):
        """Gimbal lag is already inside the residual, so it must not be charged twice."""
        a = transmittance(P, m(res=2))
        b = TelemetryMessage(0.0, FINE_TRACK, 2.0, 0.5, 2.0, 50.0, 1.0, 3.0, 1.0)
        assert a.eta == pytest.approx(transmittance(P, b).eta)

    def test_tx_beam_is_flat_topped(self):
        edge = P.tx_half_width_mrad
        assert tx_pointing_factor(P, 0) == 1.0
        assert tx_pointing_factor(P, 0.5 * edge) > 0.999
        assert tx_pointing_factor(P, edge) == pytest.approx(0.5)
        assert tx_pointing_factor(P, 1.5 * edge) < 0.01

    def test_sixty_mrad_drift_alone_is_free_but_lean_plus_drift_is_not(self):
        """Design doc: 60 mrad of hover drift is tolerated by the wide beam; lean is the tight limit."""
        assert tx_pointing_factor(P, tx_pointing_angle_mrad(0.0, 60.0)) > 0.97
        assert tx_pointing_factor(P, tx_pointing_angle_mrad(4.5, 60.0)) < 0.1

    def test_lean_and_drift_add(self):
        assert tx_pointing_angle_mrad(5.0, 10.0) == pytest.approx(math.radians(5) * 1000 + 10)

    def test_brightness_scales_transmittance(self):
        assert transmittance(P, m(inten=0.5)).eta == pytest.approx(0.5 * transmittance(P, m(inten=1.0)).eta)

    def test_lost_beacon_means_no_signal(self):
        b = transmittance(P, m(state=SEARCH))
        assert b.eta == 0 and not b.locked
        assert qber_estimate(P, b) == pytest.approx(0.5)


class TestIRLink:
    def test_measured_signal_replaces_the_pointing_factors(self):
        """S/S_cal already contains every pointing loss; using both would count it twice."""
        bad_pointing = m(res=15, lean=6, rx_signal=0.8, rx_background=0.0)
        good_pointing = m(res=0, lean=0, rx_signal=0.8, rx_background=0.0)
        assert transmittance(P, bad_pointing).eta == pytest.approx(transmittance(P, good_pointing).eta)
        assert transmittance(P, good_pointing).eta == pytest.approx(0.8 * geometric_loss(P, 3.0))

    def test_measured_background_raises_qber(self):
        quiet = transmittance(P, m(rx_signal=1.0, rx_background=0.0))
        lamp = transmittance(P, m(rx_signal=1.0, rx_background=30.0))
        assert qber_estimate(P, lamp) > qber_estimate(P, quiet) + 0.05

    def test_modelled_mode_has_zero_background(self):
        assert transmittance(P, m()).p_bg == 0.0


class TestQBER:
    def test_locked_qber_is_two_to_three_percent(self):
        q = qber_estimate(P, transmittance(P, m(res=0, lean=0, drift=0)))
        assert 0.02 <= q <= 0.035

    def test_qber_rises_monotonically_with_residual(self):
        qs = [qber_estimate(P, transmittance(P, m(res=r))) for r in (0, 4, 6, 8, 10, 12)]
        assert qs == sorted(qs) and qs[-1] > QBER_GATE

    def test_intercept_resend_on_half_the_pulses_gives_about_14_5_percent(self):
        b = transmittance(P, m(res=0, lean=0, drift=0))
        assert qber_estimate(P, b, eve_fraction=0.5) == pytest.approx(0.145, abs=0.01)

    def test_qber_never_above_half(self):
        assert qber_estimate(P, transmittance(P, m(res=1000))) <= 0.5

    def test_gate_crossing_is_a_fixed_loss_budget(self):
        db = pointing_loss_to_gate_db(P)
        assert 5 < db < 15

    def test_operating_point_is_configurable(self):
        hi, lo = default_params(200.0), default_params(10.0)
        b = lambda p: transmittance(p, m(res=0, lean=0, drift=0))
        assert qber_estimate(hi, b(hi)) < qber_estimate(P, b(P)) < qber_estimate(lo, b(lo))
        assert qber_estimate(lo, b(lo)) == pytest.approx(0.0636, abs=0.003)   # the doc's "10x noise" case
