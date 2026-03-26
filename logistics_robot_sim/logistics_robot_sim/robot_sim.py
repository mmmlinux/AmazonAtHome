import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from logistics_interfaces.action import MoveToWaypoint
from logistics_interfaces.msg import RobotStatus


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

        self._is_moving = False
        self._battery_lock = threading.Lock()

        cb = ReentrantCallbackGroup()

        self._status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)

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

    # ── Charging timer ────────────────────────────────────────────────────────

    def _charging_tick(self) -> None:
        if self._is_moving:
            return
        if self.current_waypoint not in self._charging_waypoints:
            return
        with self._battery_lock:
            if self.battery_level >= 100.0:
                return
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
        msg = RobotStatus()
        msg.current_waypoint  = self.current_waypoint
        msg.target_waypoint   = target
        msg.is_moving         = is_moving
        msg.travel_progress   = float(progress)
        msg.battery_level     = float(self.battery_level)
        self._status_pub.publish(msg)

    # ── Action: MoveToWaypoint ────────────────────────────────────────────────

    def _execute_move(self, goal_handle):
        target   = goal_handle.request.target_waypoint
        distance = float(goal_handle.request.distance)
        heading  = float(goal_handle.request.heading_deg)
        turn     = float(goal_handle.request.turn_deg)
        travel_time = distance / self.speed if self.speed > 0 else 0.0

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
