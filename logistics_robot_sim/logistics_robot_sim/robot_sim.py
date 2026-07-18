"""
Robot Simulator — physics model for a single warehouse robot.

Accepts MoveToWaypoint action goals from nav_server and simulates travel by
sleeping for the appropriate wall-clock time, then reporting the result.
Simultaneously drains the battery while moving and charges it while stationary
at a recognised charging waypoint.

The node is launched once per robot under a unique namespace (robot_1/,
robot_2/, …) so all topic and action names are automatically scoped.

Actions served
--------------
  move_to_waypoint   logistics_interfaces/action/MoveToWaypoint
    Input : target_waypoint, distance (m), heading_deg, turn_deg
    Output: success, final_waypoint, message
    Feedback: current_waypoint, distance_remaining

Topics published
----------------
  robot_status   logistics_interfaces/msg/RobotStatus   (4 Hz + on change)
    current_waypoint, target_waypoint, is_moving,
    travel_progress (0–1), battery_level (%)

Parameters
----------
  travel_speed             m/s                    (default 1.0)
  start_waypoint           initial waypoint ID    (default 'charge_1')
  initial_battery          starting charge %      (default 100.0)
  discharge_rate_per_meter battery % lost per m   (default 0.5)
  charge_rate_per_second   battery % gained/s     (default 10.0)
  charging_waypoints       comma-separated list   (default 'charge_1,charge_2')
"""

import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from logistics_interfaces.action import MoveToWaypoint
from logistics_interfaces.msg import ChargerState, RobotStatus


class RobotSim(Node):
    """
    Simulated robot.

    - Action server: MoveToWaypoint  (nav_server → robot_sim)
    - Topic publisher: robot_status  (robot_sim → anyone subscribing)

    Battery model:
      - Discharges at `discharge_rate_per_meter` % per metre while moving.
      - Charges at `charge_rate_per_second` % per second while idle at a
        charging waypoint (listed in the `charging_waypoints` parameter).
    """

    CHARGE_TIMER_PERIOD = 0.5   # seconds between charging ticks

    def __init__(self):
        super().__init__('robot_sim')

        self.declare_parameter('travel_speed',           1.0)    # m/s
        self.declare_parameter('start_waypoint',         'charge_1')
        self.declare_parameter('initial_battery',        100.0)  # %
        self.declare_parameter('discharge_rate_per_meter', 0.5)  # % per metre
        self.declare_parameter('charge_rate_per_second',  10.0)  # % per second
        self.declare_parameter('charging_waypoints',     'charge_1,charge_2')

        self.speed: float = (
            self.get_parameter('travel_speed').get_parameter_value().double_value
        )
        self.current_waypoint: str = (
            self.get_parameter('start_waypoint').get_parameter_value().string_value
        )
        self.battery_level: float = (
            self.get_parameter('initial_battery').get_parameter_value().double_value
        )
        self._discharge_rate: float = (
            self.get_parameter('discharge_rate_per_meter').get_parameter_value().double_value
        )
        self._charge_rate: float = (
            self.get_parameter('charge_rate_per_second').get_parameter_value().double_value
        )
        charging_str = (
            self.get_parameter('charging_waypoints').get_parameter_value().string_value
        )
        self._charging_waypoints: set[str] = {
            w.strip() for w in charging_str.split(',') if w.strip()
        }

        # TODO: _is_moving is written by _execute_move (action callback) and
        # read by _charging_tick (timer callback) without a lock.  Both run on
        # the MultiThreadedExecutor's thread pool, so a data race is possible.
        # Protect with _battery_lock or a dedicated boolean Event.
        self._is_moving = False
        self._battery_lock = threading.Lock()

        # Charger engagement state — updated from /charger_state topic.
        # Maps waypoint_id → engaged (bool).  Until the first message arrives
        # (_charger_state_received=False) we fall back to auto-charging so the
        # system works even when ChargerManager is not running.
        self._charger_state_lock     = threading.Lock()
        self._charger_state_received = False
        self._engaged_chargers: dict[str, bool] = {}

        cb = ReentrantCallbackGroup()

        self._status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)

        self.create_subscription(
            ChargerState,
            '/charger_state',
            self._on_charger_state,
            10,
            callback_group=cb,
        )

        self._action_server = ActionServer(
            self,
            MoveToWaypoint,
            'move_to_waypoint',
            execute_callback=self._execute_move,
            callback_group=cb,
        )

        # Charging ticker — fires every CHARGE_TIMER_PERIOD seconds
        self.create_timer(
            self.CHARGE_TIMER_PERIOD,
            self._charging_tick,
            callback_group=cb,
        )

        self._publish_status(is_moving=False, target='')

        self.get_logger().info(
            f'Robot sim ready at [{self.current_waypoint}]  '
            f'speed={self.speed} m/s  battery={self.battery_level:.0f}%  '
            f'charging spots={self._charging_waypoints}'
        )

    # ── Charger state ─────────────────────────────────────────────────────────

    def _on_charger_state(self, msg: ChargerState) -> None:
        """Track which charger waypoints currently have an engaged charger."""
        with self._charger_state_lock:
            self._charger_state_received = True
            self._engaged_chargers = {
                cs.waypoint_id: cs.engaged for cs in msg.chargers
            }

    def _is_charger_engaged(self, waypoint_id: str) -> bool:
        """
        Return True if the charger at waypoint_id is currently engaged.

        Returns False if ChargerManager has not published any state yet
        (backwards-compatible: no lock = no block).
        """
        with self._charger_state_lock:
            if not self._charger_state_received:
                return False
            return self._engaged_chargers.get(waypoint_id, False)

    # ── Charging timer ────────────────────────────────────────────────────────

    def _charging_tick(self) -> None:
        """
        Called every CHARGE_TIMER_PERIOD seconds.

        Adds charge only when the robot is stationary at a recognised charging
        waypoint AND the charger is engaged.  Falls back to auto-charge if
        ChargerManager has not published any state (no charger_manager running).
        """
        if self._is_moving:
            return
        if self.current_waypoint not in self._charging_waypoints:
            return

        with self._charger_state_lock:
            if self._charger_state_received:
                # ChargerManager is running — only charge when engaged
                if not self._engaged_chargers.get(self.current_waypoint, False):
                    return

        with self._battery_lock:
            if self.battery_level < 100.0:
                gained = self._charge_rate * self.CHARGE_TIMER_PERIOD
                self.battery_level = min(100.0, self.battery_level + gained)
                self.get_logger().debug(f'Charging: {self.battery_level:.1f}%')
        self._publish_status(is_moving=False, target='')

    # ── Status publisher ──────────────────────────────────────────────────────

    def _publish_status(
        self,
        *,
        is_moving: bool,
        target: str,
        progress: float = 0.0,
    ) -> None:
        """
        Publish a RobotStatus snapshot.

        progress is the fraction of the current edge completed (0.0–1.0); the
        web UI uses this to interpolate the robot dot between waypoints.
        """
        msg = RobotStatus()
        msg.current_waypoint  = self.current_waypoint
        msg.target_waypoint   = target
        msg.is_moving         = is_moving
        msg.travel_progress   = float(progress)
        msg.battery_level     = float(self.battery_level)
        self._status_pub.publish(msg)

    # ── Action: MoveToWaypoint ────────────────────────────────────────────────

    def _execute_move(self, goal_handle):
        """
        Execute one movement leg (current waypoint → target waypoint).

        Divides the travel time into ~4 Hz update steps so that
        travel_progress advances smoothly and the web UI can animate the
        robot dot.  Battery is drained proportionally to distance each step.

        The turn_deg field is accepted but not physically simulated — it is
        there for future hardware integration where turning in place takes time.
        """
        target   = goal_handle.request.target_waypoint
        distance = float(goal_handle.request.distance)
        heading  = float(goal_handle.request.heading_deg)
        turn     = float(goal_handle.request.turn_deg)
        travel_time = distance / self.speed if self.speed > 0 else 0.0

        # Safety: block if the charger at our current waypoint is still engaged.
        # nav_server is expected to disengage before issuing any move command;
        # this is a last-resort guard against programming errors or timing races.
        CHARGER_RELEASE_TIMEOUT_S = 30.0
        deadline = time.monotonic() + CHARGER_RELEASE_TIMEOUT_S
        while self._is_charger_engaged(self.current_waypoint):
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f'Charger at [{self.current_waypoint}] still engaged after '
                    f'{CHARGER_RELEASE_TIMEOUT_S:.0f} s — proceeding anyway '
                    '(charger should have been disengaged before issuing a move)'
                )
                break
            time.sleep(0.1)

        self._is_moving = True
        try:
            self.get_logger().info(
                f'Moving [{self.current_waypoint}] -> [{target}]  '
                f'turn {turn:+.1f}° then go {distance:.1f} m  '
                f'heading={heading:.1f}°  ~{travel_time:.1f} s  '
                f'battery={self.battery_level:.1f}%'
            )

            # Publish feedback + status at ~4 Hz, draining battery per step
            update_hz     = 4.0
            steps         = max(1, int(travel_time * update_hz))
            step_time     = travel_time / steps
            dist_per_step = distance / steps

            for i in range(steps):
                time.sleep(step_time)
                progress = (i + 1) / steps

                with self._battery_lock:
                    self.battery_level = max(
                        0.0,
                        self.battery_level - dist_per_step * self._discharge_rate,
                    )

                feedback = MoveToWaypoint.Feedback()
                feedback.current_waypoint   = self.current_waypoint
                feedback.distance_remaining = distance * (1.0 - progress)
                goal_handle.publish_feedback(feedback)

                self._publish_status(is_moving=True, target=target, progress=progress)

            self.current_waypoint = target

        finally:
            self._is_moving = False

        self.get_logger().info(
            f'Arrived at [{target}]  battery={self.battery_level:.1f}%'
        )
        self._publish_status(is_moving=False, target='')

        result = MoveToWaypoint.Result()
        result.success      = True
        result.final_waypoint = target
        result.message      = f'Successfully moved to {target}'
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RobotSim()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
