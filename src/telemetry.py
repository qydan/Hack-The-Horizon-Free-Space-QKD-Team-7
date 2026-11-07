from abc import ABC, abstractmethod
from typing import Dict

class TelemetryProvider(ABC):
    """
    Contract for physical link telemetry.
    The hardware team must implement this to return real-world data.
    """
    @abstractmethod
    def get_metrics(self) -> Dict[str, float]:
        """
        Must return:
        - "drift": float (spatial offset)
        - "jitter": float (vibration/movement speed)
        - "intensity": float (signal strength)
        """
        pass