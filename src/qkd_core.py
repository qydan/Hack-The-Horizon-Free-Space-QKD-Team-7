import numpy as np
import hashlib
from typing import Tuple

class QuantumCryptoEngine:
    def __init__(self, block_size: int = 2048):
        self.block_size = block_size
        
        # Physics calibration constants for the transfer function
        self.e_optical = 0.015
        self.alpha = 0.0015
        self.beta = 0.0012
        self.gamma = 0.012

    def calculate_channel_error(self, drift: float, jitter: float, intensity: float) -> float:
        """Translates raw sensor telemetry into a 0.0 to 1.0 quantum error likelihood."""
        intensity = max(0.01, min(intensity, 1.0))
        p_error = self.e_optical + (self.alpha * drift) + (self.beta * jitter) + (self.gamma / intensity)
        return min(p_error, 0.50)  # Max error is 50% (pure random noise)

    def generate_and_transmit(self, p_error: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Alice generates states. The channel injects errors based on P_error. 
        Bob measures the states.
        """
        # Alice generates random bits and bases (0: rectilinear, 1: diagonal)
        a_bits = np.random.randint(0, 2, size=self.block_size)
        a_bases = np.random.randint(0, 2, size=self.block_size)
        
        # Bob chooses random measurement bases
        b_bases = np.random.randint(0, 2, size=self.block_size)
        b_bits = np.copy(a_bits)
        
        # Quantum Channel Simulation: True where environment/Eve corrupts the photon
        noise_mask = np.random.random(size=self.block_size) < p_error
        
        for i in range(self.block_size):
            if a_bases[i] != b_bases[i]:
                # Mismatched bases yield a completely random bit (quantum collapse)
                b_bits[i] = np.random.randint(0, 2)
            elif noise_mask[i]:
                # Environmental flip
                b_bits[i] ^= 1
                
        return a_bits, a_bases, b_bits, b_bases

    def sift_and_estimate(self, a_bits: np.ndarray, a_bases: np.ndarray, 
                          b_bits: np.ndarray, b_bases: np.ndarray, 
                          sample_ratio: float = 0.2) -> Tuple[float, np.ndarray, np.ndarray]:
        """Discards basis mismatches and destructively samples for QBER."""
        # 1. Sifting
        matches = (a_bases == b_bases)
        a_sifted = a_bits[matches]
        b_sifted = b_bits[matches]
        
        if len(a_sifted) == 0:
            return 1.0, np.array([]), np.array([])
            
        # 2. Parameter Estimation (The 11% Gate)
        sample_size = int(len(a_sifted) * sample_ratio)
        sample_indices = np.random.choice(len(a_sifted), size=sample_size, replace=False)
        
        mismatches = np.sum(a_sifted[sample_indices] != b_sifted[sample_indices])
        qber = mismatches / sample_size if sample_size > 0 else 1.0
        
        # Remove the revealed sample bits from the secure key pool
        keep_indices = np.setdiff1d(np.arange(len(a_sifted)), sample_indices)
        return qber, a_sifted[keep_indices], b_sifted[keep_indices]

    def reconcile_and_amplify(self, a_keys: np.ndarray, b_keys: np.ndarray) -> str:
        """Fixes remaining parity errors and hashes the final AES key."""
        b_corrected = np.copy(b_keys)
        n = len(a_keys)
        block_size = 16
        
        # Cascade-style parity reconciliation
        for i in range(0, n, block_size):
            end = min(i + block_size, n)
            if np.sum(a_keys[i:end]) % 2 != np.sum(b_corrected[i:end]) % 2:
                # Discrepancy found, scan to correct the bit
                for j in range(i, end):
                    if a_keys[j] != b_corrected[j]:
                        b_corrected[j] = a_keys[j]
                        break
                        
        if not np.array_equal(a_keys, b_corrected):
            raise ValueError("Reconciliation failed to match keys.")
            
        # Privacy Amplification via SHA-256
        bit_string = "".join(map(str, a_keys)).encode('utf-8')
        return hashlib.sha256(bit_string).hexdigest()