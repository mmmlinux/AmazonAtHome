"""
ChargerManager — ROS 2 node managing all charger hardware drivers.

Reads chargers.yaml and creates one driver per charging waypoint.  Exposes:

  Service  /charger/control   logistics_interfaces/srv/ChargerControl
    engage=True  : engage the charger at waypoint_id (blocks until confirmed)
    engage=False : disengage the charger at waypoint_id (blocks until released)

  Topic    /charger_state     logistics_interfaces/msg/ChargerState  (4 Hz)
    Snapshot of every configured charger's current status.

If no chargers_file is configured, or a waypoint has no entry in that file,
the /charger/control service returns success=True immediately so that the rest
of the system degrades gracefully to the old auto-charge behaviour.

Parameters
----------
  chargers_file  path to chargers.yaml  (required for real charger hardware)

chargers.yaml format
--------------------
  chargers:
    charge_1:
      type: simulated      # instant engage/disengage, no hardware
    charge_2:
      type: esp32
      ip:   192.168.1.50
      port: 80             # default 80
"""

import yaml

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from logistics_interfaces.msg import ChargerState, ChargerStatus
from logistics_interfaces.srv import ChargerControl

from .charger_driver import ChargerDriver
from .sim_charger_driver import SimChargerDriver
from .esp_charger_driver import EspChargerDriver


class ChargerManager(Node):
    """Manages all charger hardware drivers and exposes them over ROS."""

    STATUS_HZ = 4.0

    def __init__(self):
        super().__init__('charger_manager')

        self.declare_parameter('chargers_file', '')

        chargers_file = (
            self.get_parameter('chargers_file').get_parameter_value().string_value
        )

        self._drivers: dict[str, ChargerDriver] = {}
        self._load_chargers(chargers_file)

        cb = ReentrantCallbackGroup()

        self._state_pub = self.create_publisher(ChargerState, '/charger_state', 10)

        self.create_service(
            ChargerControl,
            '/charger/control',
            self._handle_control,
            callback_group=cb,
        )

        self.create_timer(
            1.0 / self.STATUS_HZ, self._publish_state, callback_group=cb
        )

        self.get_logger().info(
            f'ChargerManager ready — {len(self._drivers)} charger(s): '
            + (', '.join(self._drivers.keys()) or 'none')
        )

    # ── Setup ─────────────────────────────────────────────────────────────────

    def _load_chargers(self, chargers_file: str) -> None:
        if not chargers_file:
            self.get_logger().warning(
                'chargers_file not set — charging will be fully simulated '
                '(no ChargerControl needed)'
            )
            return

        try:
            with open(chargers_file, 'r') as f:
                config = yaml.safe_load(f)
        except OSError as exc:
            self.get_logger().error(f'Cannot read chargers_file: {exc}')
            return

        for waypoint_id, cfg in (config or {}).get('chargers', {}).items():
            driver = self._make_driver(waypoint_id, cfg)
            driver.connect()
            self._drivers[waypoint_id] = driver
            self.get_logger().info(
                f'  [{waypoint_id}] type={cfg.get("type", "simulated")}'
            )

    def _make_driver(self, waypoint_id: str, cfg: dict) -> ChargerDriver:
        ctype = cfg.get('type', 'simulated')
        if ctype == 'esp32':
            return EspChargerDriver(
                waypoint_id,
                ip=cfg['ip'],
                port=int(cfg.get('port', 80)),
                logger=self.get_logger(),
            )
        if ctype != 'simulated':
            self.get_logger().warning(
                f'[{waypoint_id}]: unknown type {ctype!r} — using simulated'
            )
        return SimChargerDriver(waypoint_id)

    # ── Service handler ───────────────────────────────────────────────────────

    def _handle_control(
        self,
        request: ChargerControl.Request,
        response: ChargerControl.Response,
    ) -> ChargerControl.Response:
        waypoint_id = request.waypoint_id
        robot_id    = request.robot_id
        engage      = request.engage
        action      = 'engage' if engage else 'disengage'

        driver = self._drivers.get(waypoint_id)
        if driver is None:
            # No driver for this charger — succeed silently so the system
            # continues to work without a chargers.yaml entry for every waypoint.
            self.get_logger().debug(
                f'ChargerControl: no driver for [{waypoint_id}] — pass-through'
            )
            response.success        = True
            response.message        = 'no_driver'
            response.robot_detected = True
            response.charging       = engage
            return response

        self.get_logger().info(
            f'ChargerControl: {action} [{waypoint_id}] for robot {robot_id}'
        )

        success = driver.engage(robot_id) if engage else driver.disengage(robot_id)

        status = driver.get_status()
        response.success        = success
        response.message        = 'ok' if success else 'timeout_or_error'
        response.robot_detected = status['robot_detected']
        response.charging       = status['charging']
        return response

    # ── Status publisher ──────────────────────────────────────────────────────

    def _publish_state(self) -> None:
        msg = ChargerState()
        for waypoint_id, driver in self._drivers.items():
            s  = driver.get_status()
            cs = ChargerStatus()
            cs.waypoint_id    = waypoint_id
            cs.engaged        = s['engaged']
            cs.robot_detected = s['robot_detected']
            cs.charging       = s['charging']
            cs.current_ma     = float(s['current_ma'])
            cs.voltage_v      = float(s['voltage_v'])
            msg.chargers.append(cs)
        self._state_pub.publish(msg)

    def destroy_node(self) -> None:
        for driver in self._drivers.values():
            try:
                driver.disconnect()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChargerManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
