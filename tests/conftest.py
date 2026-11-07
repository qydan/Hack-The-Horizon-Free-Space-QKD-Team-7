import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from qkd_core import QuantumCryptoEngine  # noqa: E402

QBER_GATE = 0.11  # abort threshold used by demo_runner.py

CALM = {"drift": 2.5, "jitter": 1.0, "intensity": 0.9}
FAN_ON = {"drift": 35.0, "jitter": 50.0, "intensity": 0.25}


@pytest.fixture(autouse=True)
def seeded(monkeypatch):
    np.random.seed(1234)
    # Makes every QuantumCryptoEngine deterministic (test mode only; real runs use os.urandom)
    monkeypatch.setattr(QuantumCryptoEngine, "default_seed", 1234)


@pytest.fixture
def engine():
    return QuantumCryptoEngine(block_size=32768)
