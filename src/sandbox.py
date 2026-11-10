"""Cyber sandbox (key segment): telemetry in, keys out.

    TelemetryMessage -> error budget -> channel + decoys -> Eve emulator
        -> BB84 pipeline (sift, Cascade, Toeplitz) -> QBER + decoy gate -> key store

Every message is turned into a transmittance and a stream of pulses. Clicks are pooled
per 100 ms interval (the live trace) and per key block (the unit the gate decides on).
The gate aborts on the *pooled* QBER of a block, because single frames often spike above
11% even when the average is fine. An abort is sent back to the tracker as a re-acquire
command (I8).

This module has no command line of its own; ``demo_runner.py`` drives it. As its own
program next to the tracker:
    python src/demo_runner.py --source zmq
"""
import json
import os
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Optional

import numpy as np

import decoy as decoy_mod
from channel import BlockCounts, EveConfig, simulate_message
from decoy import DecoyParams
from error_budget import QBER_GATE, BudgetParams, default_params, lin_to_db, transmittance
from key_pool import KeyPool
from qkd_core import QuantumCryptoEngine
from telemetry import TelemetryMessage


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
    eta_db: float                # mean transmittance loss, dB
    qber_expected: float         # error-budget formula
    qber_measured: float         # from the simulated clicks (noisy at 100 ms)
    kind: str = "interval"


@dataclass
class BlockRecord:
    """The gate's decision on one key block."""
    t: float
    outcome: str                 # key | abort | decoy_abort | no_key | no_link | pooling
    qber: float                  # pooled sampled QBER
    qber_expected: float
    sifted_bits: int = 0
    secret_bits: Optional[int] = None
    leaked_bits: Optional[int] = None
    errors_corrected: Optional[int] = None
    pool_bits: Optional[int] = None
    key: Optional[str] = None
    decoy: Optional[Dict] = None
    note: str = ""
    kind: str = "block"

    @property
    def aborted(self) -> bool:
        return self.outcome in ("abort", "decoy_abort")


class KeyStore:
    """Holds the finished keys and hands them to the operator (I4)."""

    def __init__(self, path: Optional[str] = None):
        self.keys: List[Dict] = []
        self.path = path

    def add(self, t: float, key: str, secret_bits: int) -> None:
        self.keys.append({"t": t, "key": key, "secret_bits": secret_bits})
        if self.path:
            with open(self.path, "a") as f:
                f.write(json.dumps(self.keys[-1]) + "\n")

    def __len__(self) -> int:
        return len(self.keys)


class Sandbox:
    """Consumes telemetry messages, emits IntervalRecords and BlockRecords.

    ``pulse_rate_hz`` is the emulated source rate. Real QKD links run at 100 MHz to GHz;
    the default keeps one 1 s block large enough for a finite-key run at the demo's QBER.
    """

    def __init__(self, engine: Optional[QuantumCryptoEngine] = None,
                 budget: Optional[BudgetParams] = None,
                 decoy: Optional[DecoyParams] = None,
                 eve: Optional[EveConfig] = None,
                 pulse_rate_hz: float = 1e9, interval_s: float = 0.1, block_s: float = 1.0,
                 pool: Optional[KeyPool] = None, key_store: Optional[KeyStore] = None,
                 on_abort: Optional[Callable[[BlockRecord], None]] = None,
                 min_sifted_bits: int = 2000, rng: Optional[np.random.Generator] = None):
        self.engine = engine or QuantumCryptoEngine()
        self.budget = budget or default_params()
        self.decoy = decoy or DecoyParams()
        self.eve = eve or EveConfig()
        self.pulse_rate = pulse_rate_hz
        self.interval_s = interval_s
        self.block_s = block_s
        self.pool = pool
        self.keys = key_store if key_store is not None else KeyStore()   # an empty KeyStore is falsy (__len__)
        self.on_abort = on_abort
        self.min_sifted_bits = min_sifted_bits
        self.rng = rng or self.engine.rng

        self._prev_t: Optional[float] = None
        self._int_start: Optional[float] = None
        self._blk_start: Optional[float] = None
        self._int = BlockCounts()
        self._blk = BlockCounts()
        self._msgs: List[TelemetryMessage] = []
        self.counts: Dict[str, int] = {}

    # ------------------------------------------------------------------ streaming

    def feed(self, msg: TelemetryMessage) -> List:
        """Process one telemetry message; returns any records completed by it."""
        out: List = []
        if self._prev_t is None:
            self._prev_t = msg.t
            self._int_start = self._blk_start = msg.t
            return out
        dt = msg.t - self._prev_t
        self._prev_t = msg.t
        if dt <= 0:
            return out

        b = transmittance(self.budget, msg)
        n = max(1, int(round(self.pulse_rate * dt)))
        counts = simulate_message(self.budget, self.decoy, b, n, self.eve, self.rng)
        self._int.add(counts)
        self._blk.add(counts)
        self._msgs.append(msg)

        if msg.t - self._int_start >= self.interval_s:
            out.append(self._close_interval(msg.t))
        if msg.t - self._blk_start >= self.block_s:
            out.append(self._close_block(msg.t))
        return out

    def run(self, source: Iterable[TelemetryMessage]) -> Iterable:
        """Generator over a telemetry source yielding records as they complete."""
        for msg in source:
            for rec in self.feed(msg):
                yield rec

    # ------------------------------------------------------------------ records

    def _close_interval(self, t: float) -> IntervalRecord:
        k = sum(1 for m in self._msgs if m.t > self._int_start)   # messages in this interval
        recent = self._msgs[-k:] if k else self._msgs[-1:]
        mean = lambda f: float(np.mean([f(m) for m in recent]))
        c = self._int
        rec = IntervalRecord(
            t=round(t, 4), state=recent[-1].state,
            drift_mrad=mean(lambda m: m.drift_mrad), jitter_mrad=mean(lambda m: m.jitter_mrad),
            residual_mrad=mean(lambda m: m.residual_mrad), lean_deg=mean(lambda m: m.lean_deg),
            range_m=mean(lambda m: m.range_m), intensity=mean(lambda m: m.intensity),
            eta_db=lin_to_db(c.mean_eta),
            qber_expected=float(c.expected_qber),
            qber_measured=float(c.signal_qber))
        self._int = BlockCounts()
        self._int_start = t
        return rec

    def _close_block(self, t: float) -> BlockRecord:
        c = self._blk
        qexp = float(c.expected_qber)
        rec = self._run_pipeline(t, c, qexp)
        self.counts[rec.outcome] = self.counts.get(rec.outcome, 0) + 1
        self._blk = BlockCounts()
        self._msgs = []
        self._blk_start = t
        if rec.aborted and self.on_abort:
            self.on_abort(rec)
        return rec

    # ------------------------------------------------------------------ BB84 + gate

    def _run_pipeline(self, t: float, c: BlockCounts, qexp: float) -> BlockRecord:
        eng = self.engine
        rec = BlockRecord(t=round(t, 4), outcome="", qber=0.5, qber_expected=qexp)

        n_det = c.detected["signal"]
        if c.locked_pulses < 0.5 * c.pulses or n_det < 2 * self.min_sifted_bits:
            rec.outcome, rec.note = "no_link", "beacon not locked for most of the block / too few clicks"
            return rec

        # BB84: transmit detected pulses, sift, sample 20% for QBER
        a, ab, bb_, bbases = eng.transmit_detected(n_det, c.signal_qber)
        qber, a_ret, b_ret = eng.sift_and_estimate(a, ab, bb_, bbases)
        rec.qber, rec.sifted_bits = float(qber), int(len(a_ret))
        if rec.sifted_bits < self.min_sifted_bits:
            rec.outcome, rec.note = "no_link", "too few sifted bits"
            return rec

        # Gate part 1: pooled QBER of the whole block
        if qber >= QBER_GATE:
            rec.outcome = "abort"
            rec.note = f"QBER {qber*100:.1f}% >= {QBER_GATE*100:.0f}%"
            return rec

        # Gate part 2: decoy-state check (catches PNS, certifies single-photon share)
        dres = decoy_mod.analyze(self.decoy_params_for_block(), c.as_decoy_counts(), c.mean_eta * self.budget.eta_d,
                                 expected_gain_by_class=c.expected_gains())
        rec.decoy = {"consistent": dres.consistent, "y1_lower": dres.y1_lower, "e1_upper": dres.e1_upper,
                     "single_photon_fraction": dres.single_photon_fraction}
        if not dres.consistent:
            rec.outcome, rec.note = "decoy_abort", dres.note or "decoy statistics inconsistent"
            return rec
        if not dres.usable:
            rec.outcome, rec.note = "no_key", "no certified single-photon yield"
            return rec

        sample_size = int(round(len(a_ret) * 0.25))   # 20% of sifted bits were sampled
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
        self.keys.add(t, key, r["secret_length"])
        return rec

    def decoy_params_for_block(self) -> DecoyParams:
        from dataclasses import replace
        return replace(self.decoy, mu=self.budget.mu, dark_yield=self.budget.p_dark)
