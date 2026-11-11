"""The key segment end to end (ported from src/sandbox.py): telemetry records in, interval and block records out.

    record -> error_budget.transmittance -> channel_decoys.simulate_message (+ Eve) -> pooled per interval / block
    block -> bb84 (sift, 20% QBER sample) -> gate (QBER, decoy) -> Cascade + Toeplitz (or key pool) -> key store

Every record becomes ``pulse_rate_hz * dt`` pulses. Clicks are pooled per ``interval_s`` (the live trace) and per
``block_s`` (the unit the gate decides on). An abort calls the gate's publisher (I8 re-acquire).
Records are plain dicts so they can be logged as JSONL and drawn by tools/make_dashboard.py.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Iterable

import numpy as np

from sandbox.bb84 import QuantumCryptoEngine
from sandbox.channel_decoys import BlockCounts, DecoyParams, analyze, simulate_message
from sandbox.error_budget import BudgetParams, lin_to_db, record_lean, record_range, transmittance
from sandbox.eve import Eve
from sandbox.gate import Gate
from sandbox.key_pool import KeyPool
from sandbox.key_store import KeyStore


@dataclass
class IntervalRecord:
    """One ~100 ms step of the live trace."""
    t: float
    state: str
    drift_mrad: float
    jitter_mrad: float
    residual_mrad: float
    lean_deg: float
    range_m: float
    intensity: float
    eta_db: float               # mean transmittance loss, dB
    qber_expected: float        # error-budget formula
    qber_measured: float        # from the simulated clicks (noisy at 100 ms)
    eve: str = "off"
    kind: str = "interval"


@dataclass
class BlockRecord:
    """The gate's decision on one key block."""
    t: float
    outcome: str                # key | abort | decoy_abort | no_key | no_link | pooling
    qber: float                 # pooled sampled QBER
    qber_expected: float
    sifted_bits: int = 0
    secret_bits: int | None = None
    leaked_bits: int | None = None
    errors_corrected: int | None = None
    pool_bits: int | None = None
    key: str | None = None
    decoy: dict | None = None
    eve: str = "off"
    note: str = ""
    kind: str = "block"

    @property
    def aborted(self) -> bool:
        return self.outcome in ("abort", "decoy_abort")


class Sandbox:
    """Consumes telemetry records (telemetry/schema.json dicts), emits IntervalRecord and BlockRecord."""

    def __init__(self, cfg: dict[str, Any], *, engine: QuantumCryptoEngine | None = None, eve: Eve | None = None,
                 gate: Gate | None = None, key_store: KeyStore | None = None, pool: bool | None = None,
                 rng: np.random.Generator | None = None):
        self.cfg = cfg
        self.engine = engine or QuantumCryptoEngine(cfg.get("bb84"))
        self.budget = BudgetParams.from_cfg(cfg)
        self.decoy = DecoyParams.from_cfg(cfg, self.budget)
        self.eve = eve or Eve(cfg)
        self.gate = gate or Gate(cfg)
        self.keys = key_store if key_store is not None else KeyStore(cfg)
        use_pool = cfg.get("pool", {}).get("enabled", False) if pool is None else pool
        self.pool = KeyPool(self.engine, int(cfg.get("pool", {}).get("max_bits", 400_000))) if use_pool else None
        self.pulse_rate = float(cfg["pulse_rate_hz"])
        self.interval_s = float(cfg["interval_s"])
        self.block_s = float(cfg["block_s"])
        self.max_gap = float(cfg.get("max_message_gap_s", 0.1))
        self.min_sifted_bits = int(cfg["min_sifted_bits"])
        self.rng = rng or self.engine.rng
        self._prev_t: float | None = None
        self._int_start = self._blk_start = 0.0
        self._int = BlockCounts()
        self._blk = BlockCounts()
        self._recs: list[dict[str, Any]] = []
        self.counts: dict[str, int] = {}

    # ------------------------------------------------------------------ streaming
    def feed(self, rec: dict[str, Any]) -> list:
        """Process one telemetry record; return any IntervalRecord / BlockRecord it completes."""
        out: list = []
        t = float(rec["t"])
        if self._prev_t is None or t < self._prev_t - 1.0:     # first record, or the clock restarted
            self._prev_t = self._int_start = self._blk_start = t
            self._int, self._blk, self._recs = BlockCounts(), BlockCounts(), []
            return out
        dt = t - self._prev_t
        if dt <= 0:
            return out
        self._prev_t = t
        b = transmittance(self.budget, rec)
        n = max(1, int(round(self.pulse_rate * min(dt, self.max_gap))))
        counts = simulate_message(self.budget, self.decoy, b, n, self.eve.config, self.rng)
        self._int.add(counts)
        self._blk.add(counts)
        self._recs.append(rec)
        if t - self._int_start >= self.interval_s:
            out.append(self._close_interval(t))
        if t - self._blk_start >= self.block_s:
            out.append(self._close_block(t))
        return out

    def run(self, records: Iterable[dict[str, Any]]):
        for rec in records:
            yield from self.feed(rec)

    # ------------------------------------------------------------------ records
    def _close_interval(self, t: float) -> IntervalRecord:
        recent = [r for r in self._recs if float(r["t"]) > self._int_start] or self._recs[-1:]
        mean = lambda f: float(np.mean([f(r) for r in recent]))  # noqa: E731
        c = self._int
        rec = IntervalRecord(
            t=round(t, 4), state=recent[-1]["state"],
            drift_mrad=mean(lambda r: r["drift_mrad"]), jitter_mrad=mean(lambda r: r["jitter_mrad"]),
            residual_mrad=mean(lambda r: r["residual_mrad"]), lean_deg=mean(record_lean),
            range_m=mean(lambda r: record_range(self.budget, r)), intensity=mean(lambda r: r["intensity"]),
            eta_db=lin_to_db(c.mean_eta), qber_expected=float(c.expected_qber), qber_measured=float(c.signal_qber),
            eve=self.eve.mode)
        self._int = BlockCounts()
        self._int_start = t
        return rec

    def _close_block(self, t: float) -> BlockRecord:
        rec = self._run_pipeline(t, self._blk)
        self.counts[rec.outcome] = self.counts.get(rec.outcome, 0) + 1
        self._blk = BlockCounts()
        self._recs = []
        self._blk_start = t
        if rec.aborted:
            self.gate.abort(t, f"{rec.outcome}: {rec.note}", rec.qber)
        self.keys.publish(asdict(rec))
        return rec

    # ------------------------------------------------------------------ BB84 + gate
    def _run_pipeline(self, t: float, c: BlockCounts) -> BlockRecord:
        eng = self.engine
        rec = BlockRecord(t=round(t, 4), outcome="", qber=0.5, qber_expected=float(c.expected_qber), eve=self.eve.mode)
        n_det = c.detected["signal"]
        if c.locked_pulses < 0.5 * c.pulses or n_det < 2 * self.min_sifted_bits:
            rec.outcome, rec.note = "no_link", "beacon not locked for most of the block / too few clicks"
            return rec
        a, ab, bb, bbases = eng.transmit_detected(n_det, c.signal_qber)
        qber, a_ret, b_ret = eng.sift_and_estimate(a, ab, bb, bbases)
        rec.qber, rec.sifted_bits = float(qber), int(len(a_ret))
        if rec.sifted_bits < self.min_sifted_bits:
            rec.outcome, rec.note = "no_link", "too few sifted bits"
            return rec
        dres = None
        if qber < self.gate.abort_qber:          # the decoy analysis only matters if QBER passes
            dres = analyze(self.decoy, c.as_decoy_counts(), c.mean_eta * self.budget.eta_d,
                           expected_gain_by_class=c.expected_gains())
            rec.decoy = {"consistent": dres.consistent, "y1_lower": dres.y1_lower, "e1_upper": dres.e1_upper,
                         "single_photon_fraction": dres.single_photon_fraction}
        outcome, note = self.gate.decide(qber, None if dres is None else {**rec.decoy, "note": dres.note})
        if outcome != "pass":
            rec.outcome, rec.note = outcome, note
            return rec
        f = eng.sample_fraction
        sample_size = int(round(len(a_ret) * f / (1.0 - f)))       # bits disclosed for the QBER sample
        if self.pool is not None:
            self.pool.add(a_ret, b_ret, qber, sample_size)
            key = self.pool.try_distill(decoy=dres)
            rec.pool_bits = self.pool.size
            if key is None:
                rec.outcome = "pooling"
                return rec
        else:
            try:
                key = eng.reconcile_and_amplify(a_ret, b_ret, qber=qber, sample_size=sample_size, decoy=dres)
            except ValueError as e:
                rec.outcome, rec.note = "no_key", str(e)
                return rec
        r = eng.last_report
        rec.outcome, rec.key = "key", key
        rec.secret_bits, rec.leaked_bits, rec.errors_corrected = r["secret_length"], r["leaked_bits"], r["errors_corrected"]
        self.keys.add(key, {"t": rec.t, "secret_bits": rec.secret_bits})
        return rec


def describe_block(rec: BlockRecord) -> str:
    """One console line per block (as src/demo_runner.py printed)."""
    q = f"QBER={rec.qber * 100:.1f}% (budget {rec.qber_expected * 100:.1f}%)"
    tail = {"key": lambda: f"KEY {rec.key[:16]}... ({rec.secret_bits} secret bits, {rec.leaked_bits} leaked)",
            "abort": lambda: f"ABORT: {rec.note}; tracker re-acquires",
            "decoy_abort": lambda: f"DECOY ABORT: {rec.note}; tracker re-acquires",
            "pooling": lambda: f"pooling ({rec.pool_bits} sifted bits)",
            "no_link": lambda: f"no link ({rec.note})"}.get(rec.outcome, lambda: f"no key ({rec.note})")()
    return f"t={rec.t:8.2f}s | {q} | {tail}"
