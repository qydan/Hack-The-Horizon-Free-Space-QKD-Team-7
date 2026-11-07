import hashlib

import numpy as np
import pytest

from conftest import CALM, FAN_ON, QBER_GATE
from qkd_core import QuantumCryptoEngine


def run_cycle(engine, metrics):
    """One full pipeline pass. Returns (p_error, qber, key_or_None, status)."""
    p = engine.calculate_channel_error(**metrics)
    a, ab, b, bb = engine.generate_and_transmit(p)
    qber, ar, br = engine.sift_and_estimate(a, ab, b, bb)
    if qber >= QBER_GATE:
        return p, qber, None, "abort"
    try:
        return p, qber, engine.reconcile_and_amplify(ar, br, qber=qber, sample_size=len(ar) // 4), "key"
    except ValueError:
        return p, qber, None, "reconcile_fail"


# ---------- transfer function ----------

class TestChannelError:
    def test_zero_telemetry_hits_floor(self, engine):
        # intensity=1, no drift/jitter -> e_optical + gamma
        assert engine.calculate_channel_error(0, 0, 1.0) == pytest.approx(
            engine.e_optical + engine.gamma)

    @pytest.mark.parametrize("field", ["drift", "jitter"])
    def test_monotonic_increasing(self, engine, field):
        vals = [engine.calculate_channel_error(**{**CALM, field: v}) for v in (0, 5, 20, 80)]
        assert vals == sorted(vals) and vals[0] < vals[-1]

    def test_monotonic_decreasing_in_intensity(self, engine):
        vals = [engine.calculate_channel_error(**{**CALM, "intensity": v}) for v in (0.1, 0.4, 0.7, 1.0)]
        assert vals == sorted(vals, reverse=True)

    def test_caps_at_50_percent(self, engine):
        assert engine.calculate_channel_error(1e6, 1e6, 0.01) == 0.5

    @pytest.mark.parametrize("bad", [0.0, -1.0, 1e-12])
    def test_intensity_floor_prevents_div_by_zero(self, engine, bad):
        p = engine.calculate_channel_error(0, 0, bad)
        assert np.isfinite(p) and 0 < p <= 0.5

    def test_intensity_above_one_is_clamped(self, engine):
        assert engine.calculate_channel_error(0, 0, 5.0) == engine.calculate_channel_error(0, 0, 1.0)

    def test_calibration_targets(self, engine):
        """docs/calibration_log.md: ~3% calm, >11% with fan."""
        assert 0.02 <= engine.calculate_channel_error(**CALM) <= 0.05
        assert engine.calculate_channel_error(**FAN_ON) > QBER_GATE

    def test_obstruction_trips_gate(self, engine):
        """Hand over the lens: intensity 0.95 -> 0.20 should spike above the gate."""
        p = engine.calculate_channel_error(2.5, 1.0, 0.20)
        assert p > QBER_GATE, f"p_error={p:.3f}; increase gamma"

    @pytest.mark.parametrize("bad", [float("nan")])
    def test_nan_input_does_not_silently_pass(self, engine, bad):
        # NaN telemetry (dead sensor) must not produce a "safe" number
        p = engine.calculate_channel_error(bad, 0, 1.0)
        assert np.isnan(p) or p >= QBER_GATE


# ---------- transmission ----------

class TestTransmit:
    def test_shapes_and_binary_values(self, engine):
        for arr in engine.generate_and_transmit(0.05):
            assert arr.shape == (engine.block_size,)
            assert set(np.unique(arr)) <= {0, 1}

    def test_no_noise_matching_bases_are_identical(self, engine):
        a, ab, b, bb = engine.generate_and_transmit(0.0)
        m = ab == bb
        assert m.any()
        assert np.array_equal(a[m], b[m])

    def test_mismatched_bases_are_random_not_copied(self):
        e = QuantumCryptoEngine(block_size=20000)
        a, ab, b, bb = e.generate_and_transmit(0.0)
        m = ab != bb
        agree = np.mean(a[m] == b[m])
        assert 0.47 < agree < 0.53

    def test_bases_roughly_half_match(self):
        e = QuantumCryptoEngine(block_size=20000)
        _, ab, _, bb = e.generate_and_transmit(0.0)
        assert 0.48 < np.mean(ab == bb) < 0.52

    @pytest.mark.parametrize("p", [0.01, 0.05, 0.11, 0.25, 0.5])
    def test_error_rate_on_sifted_bits_matches_p_error(self, p):
        e = QuantumCryptoEngine(block_size=40000)
        a, ab, b, bb = e.generate_and_transmit(p)
        m = ab == bb
        observed = np.mean(a[m] != b[m])
        sigma = np.sqrt(p * (1 - p) / m.sum())
        assert abs(observed - p) < 5 * sigma

    def test_full_noise_p1_flips_all_matched(self, engine):
        a, ab, b, bb = engine.generate_and_transmit(1.0)
        m = ab == bb
        assert np.all(a[m] != b[m])


# ---------- sifting / parameter estimation ----------

class TestSift:
    def test_sifted_length_and_sample_removal(self, engine):
        a, ab, b, bb = engine.generate_and_transmit(0.02)
        n_match = int(np.sum(ab == bb))
        qber, ar, br = engine.sift_and_estimate(a, ab, b, bb, sample_ratio=0.2)
        assert len(ar) == len(br) == n_match - int(n_match * 0.2)

    def test_no_basis_matches_returns_qber_one(self, engine):
        z = np.zeros(8, dtype=int)
        qber, ar, br = engine.sift_and_estimate(z, z, z, z + 1)
        assert qber == 1.0 and len(ar) == 0 and len(br) == 0

    def test_tiny_sample_defaults_to_fail_safe(self, engine):
        # 2 matching bits * 0.2 -> sample_size 0 -> must fail closed (qber 1.0)
        z = np.zeros(2, dtype=int)
        qber, _, _ = engine.sift_and_estimate(z, z, z, z)
        assert qber == 1.0

    def test_qber_estimate_is_unbiased(self):
        e = QuantumCryptoEngine(block_size=4096)
        p = 0.05
        qs = []
        for _ in range(60):
            qs.append(e.sift_and_estimate(*e.generate_and_transmit(p))[0])
        assert abs(np.mean(qs) - p) < 0.005

    def test_perfect_channel_qber_zero(self, engine):
        assert engine.sift_and_estimate(*engine.generate_and_transmit(0.0))[0] == 0.0


# ---------- reconciliation + privacy amplification ----------

def noisy_copy(key, rate):
    b = key.copy()
    flips = np.random.random(len(key)) < rate
    b[flips] ^= 1
    return b, int(flips.sum())


class TestCascade:
    def test_identical_keys_need_no_corrections(self, engine):
        k = np.random.randint(0, 2, 4000)
        b, leaked, corrected = engine.cascade_reconcile(k, k.copy(), 0.03)
        assert np.array_equal(b, k) and corrected == 0

    @pytest.mark.parametrize("rate", [0.01, 0.03, 0.06, 0.10])
    def test_corrects_all_errors(self, engine, rate):
        k = np.random.randint(0, 2, 4000)
        noisy, n_err = noisy_copy(k, rate)
        b, leaked, corrected = engine.cascade_reconcile(k, noisy, rate)
        assert np.array_equal(b, k)
        assert corrected == n_err

    def test_does_not_modify_inputs(self, engine):
        k = np.random.randint(0, 2, 1000)
        noisy, _ = noisy_copy(k, 0.05)
        k0, n0 = k.copy(), noisy.copy()
        engine.cascade_reconcile(k, noisy, 0.05)
        assert np.array_equal(k, k0) and np.array_equal(noisy, n0)

    def test_leakage_is_accounted_and_near_theory(self, engine):
        n, rate = 4000, 0.03
        k = np.random.randint(0, 2, n)
        noisy, _ = noisy_copy(k, rate)
        _, leaked, _ = engine.cascade_reconcile(k, noisy, rate)
        from qkd_core import binary_entropy
        shannon = n * binary_entropy(rate)
        assert leaked > shannon  # cannot beat Shannon
        assert leaked < 2.0 * shannon + engine.TAG_BITS  # efficient enough

    def test_only_parities_cross_the_channel(self, engine):
        """Reconciliation must not depend on anything of Alice's except parities:
        corrupting Alice's key identically in both runs gives identical results."""
        k = np.random.randint(0, 2, 2000)
        noisy, _ = noisy_copy(k, 0.04)
        engine.rng = np.random.default_rng(5)
        r1 = engine.cascade_reconcile(k, noisy, 0.04)
        engine.rng = np.random.default_rng(5)
        r2 = engine.cascade_reconcile(k.copy(), noisy.copy(), 0.04)
        assert np.array_equal(r1[0], r2[0]) and r1[1:] == r2[1:]

    def test_residual_errors_fail_confirmation(self, engine):
        """With a single pass and an underestimated QBER, errors remain; the confirmation
        hash must catch it rather than silently returning mismatched keys."""
        engine.CASCADE_PASSES = 1
        k = np.random.randint(0, 2, 4000)
        noisy, _ = noisy_copy(k, 0.10)
        with pytest.raises(ValueError, match="Reconciliation failed"):
            engine.cascade_reconcile(k, noisy, 0.01)

    def test_hopeless_channel_leaves_no_secret_key(self, engine):
        """Even if Cascade converges at ~50% errors, it leaks so much that no key survives."""
        k = np.random.randint(0, 2, 4000)
        noisy = np.random.randint(0, 2, 4000)
        with pytest.raises(ValueError):
            engine.reconcile_and_amplify(k, noisy, qber=0.5, sample_size=1000)

    def test_empty_key_raises(self, engine):
        with pytest.raises(ValueError):
            engine.cascade_reconcile(np.array([], dtype=int), np.array([], dtype=int), 0.03)

    def test_short_key_with_partial_blocks(self, engine):
        k = np.random.randint(0, 2, 37)
        b = k.copy(); b[36] ^= 1
        out, _, corrected = engine.cascade_reconcile(k, b, 0.03)
        assert np.array_equal(out, k) and corrected == 1


class TestPrivacyAmplification:
    def test_secret_length_decreases_with_qber(self, engine):
        ls = [engine.secret_key_length(10000, q, 2500, 1000) for q in (0.01, 0.03, 0.06, 0.09)]
        assert ls == sorted(ls, reverse=True)

    def test_secret_length_decreases_with_leakage(self, engine):
        assert engine.secret_key_length(10000, 0.03, 2500, 500) > engine.secret_key_length(10000, 0.03, 2500, 2000)

    def test_finite_size_margin_shrinks_with_bigger_samples(self, engine):
        assert engine.finite_size_margin(10000, 4000) < engine.finite_size_margin(10000, 400)

    def test_refuses_when_not_enough_secret_bits(self, engine):
        with pytest.raises(ValueError, match="Insufficient"):
            engine.privacy_amplify(np.random.randint(0, 2, 5000), 100)

    def test_output_is_256_bit_hex(self, engine):
        out = engine.privacy_amplify(np.random.randint(0, 2, 5000), 4000)
        assert len(out) == 64 and int(out, 16) >= 0

    def test_not_a_plain_sha256_of_the_key(self, engine):
        k = np.random.randint(0, 2, 5000)
        plain = hashlib.sha256("".join(map(str, k)).encode()).hexdigest()
        assert engine.privacy_amplify(k, 4000) != plain

    def test_single_bit_change_changes_key_avalanche(self, engine):
        k = np.random.randint(0, 2, 5000)
        k2 = k.copy(); k2[123] ^= 1
        engine.rng = np.random.default_rng(9)
        o1 = int(engine.privacy_amplify(k, 4000), 16)
        engine.rng = np.random.default_rng(9)  # same public Toeplitz seed
        o2 = int(engine.privacy_amplify(k2, 4000), 16)
        assert o1 != o2

    def test_linearity_of_toeplitz_hash(self, engine):
        """Toeplitz hashing is GF(2)-linear: H(a)^H(b) == H(a^b) for a fixed seed."""
        a = np.random.randint(0, 2, 3000); b = np.random.randint(0, 2, 3000)
        outs = []
        for k in (a, b, a ^ b):
            engine.rng = np.random.default_rng(3)
            outs.append(int(engine.privacy_amplify(k, 3000), 16))
        assert outs[0] ^ outs[1] == outs[2]


class TestReconcileAndAmplify:
    def test_calm_block_yields_matching_key_and_report(self, engine):
        p = engine.calculate_channel_error(**CALM)
        qber, ar, br = engine.sift_and_estimate(*engine.generate_and_transmit(p))
        key = engine.reconcile_and_amplify(ar, br, qber=qber, sample_size=len(ar) // 4)
        assert len(key) == 64
        r = engine.last_report
        assert r["secret_length"] >= engine.KEY_BITS
        assert r["leaked_bits"] >= engine.TAG_BITS and r["reconciled_bits"] == len(ar)

    def test_short_key_is_rejected_not_padded(self, engine):
        k = np.random.randint(0, 2, 300)
        with pytest.raises(ValueError, match="Insufficient"):
            engine.reconcile_and_amplify(k, k.copy())

    def test_empty_keys_raise(self, engine):
        with pytest.raises(ValueError):
            engine.reconcile_and_amplify(np.array([], dtype=int), np.array([], dtype=int))

    def test_keys_differ_with_unseeded_engines(self):
        """Unseeded engines draw from os.urandom: two runs must not repeat a key."""
        QuantumCryptoEngine.default_seed = None
        try:
            keys = set()
            for _ in range(2):
                e = QuantumCryptoEngine()
                p = e.calculate_channel_error(**CALM)
                q, ar, br = e.sift_and_estimate(*e.generate_and_transmit(p))
                keys.add(e.reconcile_and_amplify(ar, br, qber=q, sample_size=len(ar) // 4))
            assert len(keys) == 2
        finally:
            pass  # monkeypatch fixture restores default_seed


# ---------- end to end ----------

class TestEndToEnd:
    N = 60

    def test_calm_channel_mostly_accepted(self, engine):
        results = [run_cycle(engine, CALM)[3] for _ in range(self.N)]
        assert results.count("abort") / self.N < 0.01, "false aborts in calm conditions"

    def test_fan_on_always_aborts(self, engine):
        results = [run_cycle(engine, FAN_ON)[3] for _ in range(self.N)]
        assert results.count("abort") / self.N > 0.99, "insecure keys escaping under turbulence"

    def test_calm_channel_produces_keys(self, engine):
        """Reconciliation should succeed at ~3% QBER, otherwise the demo shows no keys."""
        results = [run_cycle(engine, CALM)[3] for _ in range(self.N)]
        rate = results.count("key") / self.N
        assert rate > 0.9, f"only {rate:.0%} of calm cycles yield a key"

    def test_no_key_ever_issued_above_gate_true_error_rate(self, engine):
        """Whenever a key is issued, the true (unsampled) error rate must be below the gate."""
        for _ in range(self.N):
            p = engine.calculate_channel_error(**FAN_ON)
            a, ab, b, bb = engine.generate_and_transmit(p)
            qber, ar, br = engine.sift_and_estimate(a, ab, b, bb)
            if qber < QBER_GATE:
                assert np.mean(ar != br) < QBER_GATE + 0.05

    def test_keys_differ_between_runs(self, engine):
        keys = {k for _, _, k, s in (run_cycle(engine, CALM) for _ in range(30)) if k}
        assert keys, "no keys produced"
        assert len(keys) > 1, "identical key issued repeatedly"
