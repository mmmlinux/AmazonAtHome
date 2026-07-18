"""
Simulated charger driver.

Instantly engages/disengages with a short artificial delay to mimic arm
movement.  Reports robot_detected and charging as True whenever engaged —
no physical sensor is simulated.

Used when chargers.yaml specifies type: simulated (the default).
"""

import time

from .charger_driver import ChargerDriver


class SimChargerDriver(ChargerDriver):
    """Simulates a charger station with no hardware."""

    ENGAGE_DELAY_S    = 0.05   # simulated arm-extend time
    DISENGAGE_DELAY_S = 0.05   # simulated arm-retract time

    SIMULATED_CURRENT_MA = 2000.0
    SIMULATED_VOLTAGE_V  = 24.0

    def engage(self, robot_id: str) -> bool:
        time.sleep(self.ENGAGE_DELAY_S)
        with self._lock:
            self._engaged        = True
            self._robot_detected = True
            self._charging       = True
            self._current_ma     = self.SIMULATED_CURRENT_MA
            self._voltage_v      = self.SIMULATED_VOLTAGE_V
        return True

    def disengage(self, robot_id: str) -> bool:
        time.sleep(self.DISENGAGE_DELAY_S)
        with self._lock:
            self._engaged        = False
            self._robot_detected = False
            self._charging       = False
            self._current_ma     = 0.0
            self._voltage_v      = 0.0
        return True
