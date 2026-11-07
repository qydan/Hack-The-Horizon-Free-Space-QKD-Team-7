import time
from telemetry import TelemetryProvider
from qkd_core import QuantumCryptoEngine
import numpy as np

# A temporary mock class until your hardware team finishes their OpenCV/Sensor script
class MockHardware(TelemetryProvider):
    def __init__(self):
        self.turbulence = False

    def get_metrics(self):
        if self.turbulence:
            return {"drift": np.random.uniform(20, 50), 
                    "jitter": np.random.uniform(30, 70), 
                    "intensity": np.random.uniform(0.1, 0.4)}
        else:
            return {"drift": np.random.uniform(0, 5), 
                    "jitter": np.random.uniform(0, 2), 
                    "intensity": np.random.uniform(0.8, 1.0)}

def main():
    # Dependency Injection: swap MockHardware with OpenCVHardware later
    hardware = MockHardware()
    engine = QuantumCryptoEngine(block_size=2048)
    
    print("--- QKD CYBER SANDBOX INITIALIZED ---")
    
    for cycle in range(5):
        # Simulate turning on the desk fan on cycle 3
        if cycle == 3:
            print("\n💨 [ENVIRONMENT] Desk fan activated. High turbulence expected.")
            hardware.turbulence = True
            
        metrics = hardware.get_metrics()
        p_error = engine.calculate_channel_error(**metrics)
        print(f"\nCycle {cycle+1} | Channel Error Prob: {p_error*100:.2f}%")
        
        # Execute BB84 Quantum Layer
        a_bits, a_bases, b_bits, b_bases = engine.generate_and_transmit(p_error)
        qber, a_retained, b_retained = engine.sift_and_estimate(a_bits, a_bases, b_bits, b_bases)
        
        print(f"Sampled QBER: {qber*100:.2f}%")
        
        if qber >= 0.11:
            print("🚨 SECURITY ABORT: QBER exceeds 11%. Key discarded.")
            continue
            
        print("✅ QBER < 11%. Channel secure. Reconciling...")
        
        try:
            final_key = engine.reconcile_and_amplify(a_retained, b_retained)
            print(f"🔑 AES-256 KEY: {final_key}")
        except ValueError as e:
            print(f"⚠️ {e}")
            
        time.sleep(1)

if __name__ == "__main__":
    main()