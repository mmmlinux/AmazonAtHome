"""
Abstract base class for charger hardware drivers.

Each charging waypoint has one driver instance.  ChargerManager instantiates
the right subclass (SimChargerDriver or EspChargerDriver) based on chargers.yaml.

Subclasses maintain internal telemetry from periodic hardware feedback and
expose it via get_status().  The engage()/disengage() methods block until the
hardware confirms the command (or a timeout fires).
"""

import threading
from abc import ABC, abstractmethod


class ChargerDriver(ABC):
    """Abstract base for a single charging-station hardware driver."""

    def __init__(self, waypoint_id: str):
        self.waypoint_id = waypoint_id
        self._lock           = threading.Lock()
        self._engaged        = False
        self._robot_detected = False
        self._charging       = False
        self._current_ma     = 0.0
        self._voltage_v      = 0.0

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Establish communication with the charger. Called at startup."""

    def disconnect(self) -> None:
        """Tear down communication with the charger. Called at shutdown."""

    # ── Control ───────────────────────────────────────────────────────────────

    @abstractmethod
    def engage(self, robot_id: str) -> bool:
        """
        Command the charger to engage (extend arm / close contacts).

        Blocks until the charger hardware confirms it is actively charging,
        or until a timeout fires.  Returns True on confirmed success.
        """

    @abstractmethod
    def disengage(self, robot_id: str) -> bool:
        """
        Command the charger to disengage (retract arm / open contacts).

        Blocks until the charger hardware confirms the robot is released,
        or until a timeout fires.  Returns True on confirmed success.
        """

    # ── Status ────────────────────────────────────────────────────────────────

    def get_status(self) -> dict:
        """Return a thread-safe snapshot of charger status."""
        with self._lock:
            return {
                'waypoint_id':    self.waypoint_id,
                'engaged':        self._engaged,
                'robot_detected': self._robot_detected,
                'charging':       self._charging,
                'current_ma':     self._current_ma,
                'voltage_v':      self._voltage_v,
            }
