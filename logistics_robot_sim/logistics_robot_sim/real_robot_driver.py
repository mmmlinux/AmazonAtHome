"""
Real Robot Driver — skeleton for integrating a physical robot.

Drop-in replacement for robot_sim.  Implements the same two interfaces that
nav_server depends on so that everything above this layer (nav_server,
task_manager, web UI, traffic controller) works without modification.

Required interfaces
-------------------
Action server  move_to_waypoint   logistics_interfaces/action/MoveToWaypoint
  Receives a single-hop goal from nav_server (target waypoint name, edge
  distance in metres, absolute heading, turn angle).  Navigate the physical
  robot there and return success/failure.

Topic publisher  robot_status   logistics_interfaces/msg/RobotStatus  (≥ 4 Hz)
  Fields:
    current_waypoint  str    — last confirmed waypoint (used by nav_server and
                               traffic controller for collision avoidance)
    target_waypoint   str    — waypoint currently being approached
    is_moving         bool   — True while driving, False when stationary
    travel_progress   float  — 0.0–1.0 fraction of current edge completed
                               (the web UI animates the dot using this value)
    battery_level     float  — state of charge in % (0–100)

How to use
----------
  1. Fill in every section marked # TODO.
  2. Set robot_type_1:=real (or robot_type_2:=real) in your launch command:

       ros2 launch logistics_server logistics.launch.py robot_type_1:=real

     The launch file will skip robot_sim for that robot and expect your driver
     to be running (either started separately or added to the launch file).

  3. Your driver must be in the correct ROS namespace matching the robot ID:

       ros2 run logistics_robot_sim real_robot_driver --ros-args -r __ns:=/robot_1

Parameters (same as robot_sim where applicable)
----------
  start_waypoint      initial waypoint ID   (default 'charge_1')
  charging_waypoints  comma-separated list  (default 'charge_1,charge_2')
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


class RealRobotDriver(Node):

    STATUS_HZ = 4.0   # publish robot_status at this rate

    def __init__(self):
        super().__init__('robot_sim')   # keep name 'robot_sim' for topic compatibility

        self.declare_parameter('start_waypoint',    'charge_1')
        self.declare_parameter('charging_waypoints', 'charge_1,charge_2')

        self.current_waypoint: str = (
            self.get_parameter('start_waypoint').get_parameter_value().string_value
        )
        charging_str = (
            self.get_parameter('charging_waypoints').get_parameter_value().string_value
        )
        self._charging_waypoints: set[str] = {
            w.strip() for w in charging_str.split(',') if w.strip()
        }

        self._is_moving    = False
        self._target_wp    = ''
        self._progress     = 0.0
        self._battery_lock = threading.Lock()
        self._battery      = 100.0   # updated by _read_battery

        # Charger engagement state from /charger_state topic.
        # Falls back to no-lock behaviour if ChargerManager is not running.
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

        self.create_timer(1.0 / self.STATUS_HZ, self._status_tick, callback_group=cb)

        # TODO: any hardware initialisation (serial port open, ROS driver setup, etc.)

        self.get_logger().info(
            f'Real robot driver ready at [{self.current_waypoint}]  '
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
        """Return True if the charger at waypoint_id is currently engaged."""
        with self._charger_state_lock:
            if not self._charger_state_received:
                return False
            return self._engaged_chargers.get(waypoint_id, False)

    # ── Hardware interface — implement these ──────────────────────────────────

    def _send_move_command(self, target_waypoint: str,
                           distance_m: float,
                           heading_deg: float,
                           turn_deg: float) -> None:
        """
        Command the physical robot to begin moving to target_waypoint.

        Called once at the start of each hop.  The method should be
        non-blocking — actual completion is polled in _is_move_complete().

        heading_deg : absolute heading (0 = East, 90 = North, CCW positive)
        turn_deg    : signed turn from current heading before driving
                      (positive = CCW / left, negative = CW / right)
        distance_m  : edge length in metres

        TODO: translate waypoint name / heading / distance into a command
              understood by your robot platform (e.g. ROS Nav2 goal, serial
              command, etc.) and send it.
        """
        raise NotImplementedError

    def _is_move_complete(self) -> bool:
        """
        Return True when the robot has finished the current movement.

        Called in a polling loop inside _execute_move.

        TODO: check your robot's navigation feedback to determine if the
              move goal has been reached (e.g. nav2 result, encoder counts,
              QR code / SLAM localisation at target position, etc.)
        """
        raise NotImplementedError

    def _get_travel_progress(self) -> float:
        """
        Return estimated completion fraction (0.0–1.0) for the current hop.

        Used to animate the robot dot in the web UI.  A simple time-based
        estimate is fine if you don't have odometry.

        TODO: return progress from your localisation / odometry source, or
              estimate it from elapsed time vs expected travel time.
        """
        raise NotImplementedError

    def _read_battery(self) -> float:
        """
        Return current battery state of charge in percent (0.0–100.0).

        Called by the status timer.

        TODO: read from your robot's battery management system (BMS topic,
              serial packet, REST API, etc.).  Return a float in [0, 100].
        """
        raise NotImplementedError

    def _abort_move(self) -> None:
        """
        Stop any in-progress motion immediately.

        Called when the action server receives a cancel request.

        TODO: send an e-stop or velocity-zero command to your robot platform.
        """
        pass   # optional — implement if your platform supports it

    # ── Action server ─────────────────────────────────────────────────────────

    def _execute_move(self, goal_handle):
        """
        Execute one movement hop: drive to target_waypoint and report back.

        The polling loop calls _is_move_complete() at STATUS_HZ until the move
        finishes or the goal is cancelled.  Progress is updated each tick so
        the web UI animates smoothly.
        """
        target   = goal_handle.request.target_waypoint
        distance = float(goal_handle.request.distance)
        heading  = float(goal_handle.request.heading_deg)
        turn     = float(goal_handle.request.turn_deg)

        # Block if the charger at our current position is still engaged.
        CHARGER_RELEASE_TIMEOUT_S = 30.0
        deadline = time.monotonic() + CHARGER_RELEASE_TIMEOUT_S
        while self._is_charger_engaged(self.current_waypoint):
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f'Charger at [{self.current_waypoint}] still engaged after '
                    f'{CHARGER_RELEASE_TIMEOUT_S:.0f} s — proceeding anyway'
                )
                break
            time.sleep(0.1)

        self._is_moving = True
        self._target_wp = target
        self._progress  = 0.0

        self.get_logger().info(
            f'Moving [{self.current_waypoint}] → [{target}]  '
            f'turn {turn:+.1f}°  distance={distance:.1f} m  heading={heading:.1f}°'
        )

        try:
            self._send_move_command(target, distance, heading, turn)

            poll_interval = 1.0 / self.STATUS_HZ
            while not self._is_move_complete():
                if goal_handle.is_cancel_requested:
                    self._abort_move()
                    goal_handle.canceled()
                    result = MoveToWaypoint.Result()
                    result.success       = False
                    result.final_waypoint = self.current_waypoint
                    result.message       = 'Cancelled'
                    return result

                self._progress = self._get_travel_progress()

                feedback = MoveToWaypoint.Feedback()
                feedback.current_waypoint   = self.current_waypoint
                feedback.distance_remaining = distance * (1.0 - self._progress)
                goal_handle.publish_feedback(feedback)

                time.sleep(poll_interval)

            self.current_waypoint = target
            self._progress        = 1.0

        finally:
            self._is_moving = False
            self._target_wp = ''
            self._progress  = 0.0

        self.get_logger().info(f'Arrived at [{target}]')

        result = MoveToWaypoint.Result()
        result.success        = True
        result.final_waypoint = target
        result.message        = f'Successfully moved to {target}'
        goal_handle.succeed()
        return result

    # ── Status publisher ──────────────────────────────────────────────────────

    def _status_tick(self) -> None:
        """Publish a RobotStatus snapshot at STATUS_HZ."""
        try:
            battery = self._read_battery()
            with self._battery_lock:
                self._battery = battery
        except NotImplementedError:
            # _read_battery not implemented yet — publish last known value
            with self._battery_lock:
                battery = self._battery

        msg = RobotStatus()
        msg.current_waypoint = self.current_waypoint
        msg.target_waypoint  = self._target_wp
        msg.is_moving        = self._is_moving
        msg.travel_progress  = float(self._progress)
        msg.battery_level    = float(battery)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealRobotDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
