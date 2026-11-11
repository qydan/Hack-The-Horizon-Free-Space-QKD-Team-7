"""Cyber sandbox: telemetry in, keys out (ported from the team's src/ pipeline).

    I2 telemetry (5556) -> error_budget -> channel_decoys (+ eve) -> bb84 (sift, Cascade, Toeplitz)
        -> gate (pooled QBER < 11% and decoy check) -> key_store (5559);  gate abort -> 5557 (tracker re-acquires)

Modules: ``error_budget`` (telemetry -> transmittance -> QBER), ``channel_decoys`` (signal / decoy / vacuum
pulses, Poisson photons, dark counts, decoy-state bounds), ``eve`` (intercept-resend, photon-number splitting),
``bb84`` (``QuantumCryptoEngine``), ``key_pool`` (optional pooling), ``gate``, ``key_store``, ``pipeline``
(``Sandbox``: per-message streaming, 100 ms trace, per-block gate) and ``main`` (bus program / replay).

Subscribes to 5556 (telemetry) and 5560 (Eve toggle); publishes on 5557 (aborts) and 5559 (keys). These are its
only allowed ports (test B2). It must never read ground truth, evaluation/ or gz.*.
Configuration: config/sandbox.yaml.
"""
