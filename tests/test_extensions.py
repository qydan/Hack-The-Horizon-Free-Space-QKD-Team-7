"""Eve (intercept-resend), decoy-state PNS, and key pooling."""
import numpy as np
import pytest

from conftest import CALM
from decoy import DecoyParams, run_decoy_analysis
from key_pool import KeyPool
from qkd_core import QuantumCryptoEngine


def sifted_qber(engine, p, eve):
    a, ab, b, bb = engine.generate_and_transmit(p, eve_fraction=eve)
    m = ab == bb
    return float(np.mean(a[m] != b[m]))


class TestInterceptResend:
    def test_no_eve_matches_p_error(self):
        e = QuantumCryptoEngine(block_size=100000)
        assert sifted_qber(e, 0.03, 0.0) == pytest.approx(0.03, abs=0.01)

    def test_full_intercept_adds_25_percent(self):
        e = QuantumCryptoEngine(block_size=100000)
        assert sifted_qber(e, 0.0, 1.0) == pytest.approx(0.25, abs=0.01)

    @pytest.mark.parametrize("f", [0.25, 0.5, 0.75])
    def test_partial_intercept_scales_linearly(self, f):
        e = QuantumCryptoEngine(block_size=100000)
        assert sifted_qber(e, 0.0, f) == pytest.approx(0.25 * f, abs=0.01)

    def test_eve_forces_abort_on_otherwise_calm_channel(self, engine):
        p = engine.calculate_channel_error(**CALM)
        qber, _, _ = engine.sift_and_estimate(*engine.generate_and_transmit(p, eve_fraction=1.0))
        assert qber > 0.11

    def test_small_eve_can_hide_below_gate(self, engine):
        """Documenting the limit: a 10% intercept adds only ~2.5% QBER and stays under the gate."""
        p = engine.calculate_channel_error(**CALM)
        qber, _, _ = engine.sift_and_estimate(*engine.generate_and_transmit(p, eve_fraction=0.1))
        assert qber < 0.11


class TestDecoyPNS:
    P = DecoyParams()

    def run(self, pns, intensity=0.9, p_error=0.035, seed=1):
        return run_decoy_analysis(self.P, intensity, p_error, pns_attack=pns,
                                  rng=np.random.default_rng(seed))

    def test_honest_channel_certifies_single_photons(self):
        r = self.run(pns=False)
        assert r.consistent and r.usable
        true_y1 = self.P.eta0 * 0.9
        assert 0.6 * true_y1 < r.y1_lower <= 1.05 * true_y1   # lower bound, reasonably tight
        assert 0.4 < r.single_photon_fraction < 0.65           # ~ exp(-mu) = 0.607
        assert r.e1_upper < 0.2

    def test_pns_attack_is_detected_even_though_qber_is_low(self):
        r = self.run(pns=True)
        # The sifted-key QBER Bob sees is just the optical error: the naive check passes.
        assert r.error["signal"] < 0.11
        # ...but the decoy statistics expose it.
        assert not r.consistent
        assert r.single_photon_fraction == 0.0 and not r.usable

    @pytest.mark.parametrize("seed", range(5))
    def test_no_false_alarms_on_honest_channel(self, seed):
        assert self.run(pns=False, seed=seed).consistent

    @pytest.mark.parametrize("intensity", [0.3, 0.6, 1.0])
    def test_honest_across_intensities(self, intensity):
        r = self.run(pns=False, intensity=intensity)
        assert r.consistent

    def test_engine_uses_single_photon_fraction(self, engine):
        base = engine.secret_key_length(10000, 0.03, 2500, 1000)
        decoy = engine.secret_key_length(10000, 0.03, 2500, 1000, single_photon_fraction=0.6, e1_upper=0.03)
        assert decoy < base

    def test_pipeline_refuses_key_under_pns(self, engine):
        p = engine.calculate_channel_error(**CALM)
        qber, ar, br = engine.sift_and_estimate(*engine.generate_and_transmit(p))
        bad = self.run(pns=True)
        with pytest.raises(ValueError, match="Insufficient"):
            engine.reconcile_and_amplify(ar, br, qber=qber, sample_size=len(ar) // 4, decoy=bad)

    def test_pipeline_issues_key_with_honest_decoy(self):
        # Decoy accounting discards the multi-photon share, so a 32k block is marginal;
        # a 64k block (or pooling several windows) has comfortable margin.
        engine = QuantumCryptoEngine(block_size=65536)
        p = engine.calculate_channel_error(**CALM)
        qber, ar, br = engine.sift_and_estimate(*engine.generate_and_transmit(p))
        good = self.run(pns=False)
        key = engine.reconcile_and_amplify(ar, br, qber=qber, sample_size=len(ar) // 4, decoy=good)
        assert len(key) == 64
        assert engine.last_report["single_photon_fraction"] == good.single_photon_fraction


class TestKeyPool:
    def window(self, engine, p):
        q, a, b = engine.sift_and_estimate(*engine.generate_and_transmit(p))
        return q, a, b, int(round(len(a) * 0.25))

    def test_pool_extends_working_range_beyond_single_block(self):
        e = QuantumCryptoEngine(block_size=32768)
        p = 0.08  # single blocks give no key here
        q, a, b, m = self.window(e, p)
        with pytest.raises(ValueError):
            e.reconcile_and_amplify(a, b, qber=q, sample_size=m)
        pool = KeyPool(e)
        key = None
        for _ in range(12):
            q, a, b, m = self.window(e, p)
            pool.add(a, b, q, m)
            key = pool.try_distill()
            if key:
                break
        assert key and len(key) == 64
        assert pool.size == 0  # pool is emptied on success

    def test_pool_returns_none_until_enough(self, engine):
        pool = KeyPool(engine)
        q, a, b, m = self.window(engine, 0.08)
        pool.add(a, b, q, m)
        assert pool.try_distill() is None
        assert pool.size == len(a)

    def test_pool_caps_size_and_keeps_stats_consistent(self, engine):
        pool = KeyPool(engine, max_bits=30000)
        for _ in range(5):
            q, a, b, m = self.window(engine, 0.03)
            pool.add(a, b, q, m)
        assert pool.size <= 30000 + len(a)
        assert 0.0 <= pool.qber < 0.11
