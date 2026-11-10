"""[OPEN] Cyber sandbox: error budget -> decoy-state BB84 -> gate -> key store.

Subscribes to 5556 (telemetry) and 5560 (Eve toggle); publishes on 5557 (aborts) and 5559 (keys). These are its
only allowed ports (test B2). It must never read ground truth, evaluation/ or gz.*.
Config keys (values null until the team fills them): config/sandbox.yaml.
"""
