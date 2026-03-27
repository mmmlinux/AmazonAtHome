import heapq
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import yaml

from logistics_interfaces.action import LogisticsTask, MoveToWaypoint
from logistics_interfaces.msg import RobotStatus, WarehouseState


def _parse_map(data: dict) -> tuple[dict, dict]:
    """
    Parse a polar-edge map dict and return (waypoint_info, graph).

    Waypoint x/y positions are computed via BFS from the origin waypoint
    using each edge's compass bearing and distance.

    bearing convention: 0 = North (up), 90 = East (right), clockwise positive.
    Positions are stored as SVG coordinates (y increases downward).
    """
    display   = data.get('display', {})
    origin_x  = float(display.get('origin_x', 300))
    origin_y  = float(display.get('origin_y', 50))
    scale     = float(display.get('scale', 30))   # pixels per metre
    origin_id = data['origin']

    # Build undirected adjacency: wp -> [(neighbour, bearing_to_neighbour, distance)]
    adjacency: dict[str, list] = {}
    for edge in data['edges']:
        a, b   = edge['from'], edge['to']
        bear   = float(edge['bearing'])
        dist   = float(edge['distance'])
        adjacency.setdefault(a, []).append((b,  bear,            dist))
        adjacency.setdefault(b, []).append((a, (bear + 180) % 360, dist))

    # BFS from origin to assign SVG positions
    svg_pos: dict[str, tuple[float, float]] = {origin_id: (origin_x, origin_y)}
    queue = [origin_id]
    while queue:
        current = queue.pop(0)
        cx, cy = svg_pos[current]
        for neighbour, bearing, distance in adjacency.get(current, []):
            if neighbour not in svg_pos:
                rad = math.radians(bearing)
                nx = cx + distance * scale * math.sin(rad)   # East  = +X
                ny = cy - distance * scale * math.cos(rad)   # North = -Y (SVG flipped)
                svg_pos[neighbour] = (nx, ny)
                queue.append(neighbour)

    # Attach computed positions to waypoint info
    waypoint_info = {}
    for wp_id, wp_data in data['waypoints'].items():
        info = dict(wp_data)
        if wp_id in svg_pos:
            info['x'], info['y'] = svg_pos[wp_id]
        waypoint_info[wp_id] = info

    # Build navigation graph (undirected, keyed by distance)
    graph: dict[str, dict[str, float]] = {wp: {} for wp in data['waypoints']}
    for edge in data['edges']:
        a, b, dist = edge['from'], edge['to'], float(edge['distance'])
        graph[a][b] = dist
        graph[b][a] = dist

    return waypoint_info, graph


class NavServer(Node):
    def __init__(self):
        super().__init__('nav_server')

        self.declare_parameter('map_file', '')
        self.declare_parameter('robot_start', 'charge_1')
        self.declare_parameter('min_battery',      50.0)   # empty-robot charge threshold
        self.declare_parameter('critical_battery', 25.0)   # loaded-robot emergency-drop threshold

        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        self.robot_position = self.get_parameter('robot_start').get_parameter_value().string_value
        self._min_battery:      float = self.get_parameter('min_battery').get_parameter_value().double_value
        self._critical_battery: float = self.get_parameter('critical_battery').get_parameter_value().double_value

        self.graph: dict[str, dict[str, float]] = {}
        self.waypoint_info: dict = {}
        self._load_map(map_file)

        # Track the robot's current heading so each new leg can compute
        # the relative turn angle.  None = unknown (first move of a session).
        self.robot_heading_deg: float | None = None

        # Battery state — updated via robot_status subscription
        self._battery_level: float = 100.0
        self._battery_lock = threading.Lock()

        # Slot occupancy — updated via warehouse_state topic
        self._slot_occupied: dict[str, bool] = {}

        # Reentrant groups so the action client callbacks can fire while the
        # action server execute callback is blocking on threading.Event.
        cb_group = ReentrantCallbackGroup()

        self._status_sub = self.create_subscription(
            RobotStatus, 'robot_status', self._on_robot_status, 10,
            callback_group=cb_group,
        )
        self.create_subscription(
            WarehouseState, '/warehouse_state', self._on_warehouse_state, 10,
            callback_group=cb_group,
        )

        self._move_client = ActionClient(
            self, MoveToWaypoint, 'move_to_waypoint', callback_group=cb_group
        )

        self._task_server = ActionServer(
            self,
            LogisticsTask,
            'logistics_task',
            execute_callback=self._execute_task,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f'Nav server ready. {len(self.graph)} waypoints loaded. '
            f'Robot starting at: {self.robot_position}'
        )

    # ------------------------------------------------------------------
    # Map loading
    # ------------------------------------------------------------------

    def _load_map(self, map_file: str) -> None:
        if not map_file:
            self.get_logger().fatal('map_file parameter is required')
            raise RuntimeError('map_file parameter not set')

        with open(map_file, 'r') as f:
            data = yaml.safe_load(f)

        self.waypoint_info, self.graph = _parse_map(data)

        self.get_logger().info(
            f'Map loaded from {map_file}: '
            f'{len(self.graph)} waypoints, {len(data["edges"])} edges'
        )

    # ------------------------------------------------------------------
    # Robot status subscription
    # ------------------------------------------------------------------

    def _on_robot_status(self, msg: RobotStatus) -> None:
        with self._battery_lock:
            self._battery_level = float(msg.battery_level)

    def _on_warehouse_state(self, msg: WarehouseState) -> None:
        self._slot_occupied = {s.waypoint_id: s.occupied for s in msg.slots}

    # ------------------------------------------------------------------
    # Battery lockout
    # ------------------------------------------------------------------

    def _find_nearest_charger(self) -> str | None:
        """Return the nearest charging waypoint reachable from current position."""
        chargers = [
            wp for wp, info in self.waypoint_info.items()
            if info.get('type') == 'charging'
        ]
        if not chargers:
            return None
        best_wp, best_dist = None, float('inf')
        for wp in chargers:
            if wp == self.robot_position:
                return wp
            _, d = self._dijkstra(self.robot_position, wp)
            if d < best_dist:
                best_dist = d
                best_wp = wp
        return best_wp

    def _check_low_battery_carrying(self, goal_handle, result, path_taken: list) -> bool:
        """
        When carrying a box, check if battery is below 20%. If so, abort
        immediately so the task manager can handle the emergency drop.
        Does NOT navigate to a charger — that is the task manager's job.
        Returns True if the goal was aborted (caller must return immediately).
        """
        with self._battery_lock:
            level = self._battery_level
        if level >= self._critical_battery:
            return False

        self.get_logger().warning(
            f'Critical battery ({level:.1f}%) while carrying — aborting for emergency drop'
        )
        fb = LogisticsTask.Feedback()
        fb.current_waypoint = self.robot_position
        fb.detail = f'Critical battery ({level:.0f}%) — emergency drop needed'
        goal_handle.publish_feedback(fb)

        result.success = False
        result.diverted_to_charge = True
        result.path_taken = path_taken
        result.message = 'low_battery_carrying'
        goal_handle.abort()
        return True

    def _handle_low_battery(self, goal_handle, result, path_taken: list) -> bool:
        """
        Check whether battery is below 20%. If so, navigate to the nearest charger
        and hold until battery is BOTH above 20% AND has gained at least 5% since
        charging started, then abort the current goal so the task can be re-queued.

        Returns True if the goal was aborted (caller must return immediately).
        Returns False if battery is fine and the task should continue.

        Only call this when the robot is NOT carrying a box.
        """
        with self._battery_lock:
            level = self._battery_level

        if level >= self._min_battery:
            return False

        self.get_logger().warning(
            f'Low battery ({level:.1f}%) — diverting to charger'
        )

        charger = self._find_nearest_charger()
        if charger is None:
            self.get_logger().error('No charging waypoint found in map')
            result.success = False
            result.diverted_to_charge = False
            result.path_taken = path_taken
            result.message = 'low_battery_no_charger'
            goal_handle.abort()
            return True

        # Navigate to charger if not already there
        current_type = self.waypoint_info.get(self.robot_position, {}).get('type', '')
        if current_type != 'charging':
            fb = LogisticsTask.Feedback()
            fb.current_waypoint = self.robot_position
            fb.detail = f'Low battery ({level:.0f}%) — going to charger...'
            goal_handle.publish_feedback(fb)
            self._send_move_to(charger)

        resume_at = 80.0

        with self._battery_lock:
            charge_start = self._battery_level
        self.get_logger().info(
            f'Holding at charger until battery ≥ {resume_at:.0f}% '
            f'(currently {charge_start:.1f}%)'
        )

        while True:
            with self._battery_lock:
                level = self._battery_level
            if level >= resume_at:
                break
            fb = LogisticsTask.Feedback()
            fb.current_waypoint = self.robot_position
            fb.detail = f'Charging: {level:.0f}% / {resume_at:.0f}%'
            goal_handle.publish_feedback(fb)
            time.sleep(0.5)

        self.get_logger().info(
            f'Battery recovered to {level:.1f}% — aborting task for re-queue'
        )
        result.success = False
        result.diverted_to_charge = True
        result.path_taken = path_taken
        result.message = 'diverted_to_charge'
        goal_handle.abort()
        return True

    def _send_move_to(self, destination: str) -> bool:
        """Navigate to destination using full Dijkstra path, updating robot_position."""
        path, _ = self._dijkstra(self.robot_position, destination)
        if path is None:
            return False
        for i in range(1, len(path)):
            prev_wp = path[i - 1]
            next_wp = path[i]
            edge_dist = self.graph[prev_wp][next_wp]
            heading = self._heading(prev_wp, next_wp)
            turn = (
                self._turn(self.robot_heading_deg, heading)
                if self.robot_heading_deg is not None
                else 0.0
            )
            if not self._send_move_command(next_wp, edge_dist, heading, turn):
                return False
            self.robot_position = next_wp
            self.robot_heading_deg = heading
        return True

    # ------------------------------------------------------------------
    # Angle helpers
    # ------------------------------------------------------------------

    def _heading(self, from_wp: str, to_wp: str) -> float:
        """
        Absolute heading in degrees for the leg from_wp → to_wp.

        Coordinate convention (matches the warehouse_map.yaml x/y layout):
          0°  = East  (+X on the map)
          90° = North (+Y up on screen; SVG Y is inverted so we negate it)
        Counterclockwise positive.
        """
        p1 = self.waypoint_info[from_wp]
        p2 = self.waypoint_info[to_wp]
        dx =  (p2['x'] - p1['x'])
        dy = -(p2['y'] - p1['y'])   # invert SVG Y so up = positive
        return math.degrees(math.atan2(dy, dx))

    def _turn(self, from_heading: float, to_heading: float) -> float:
        """
        Shortest signed turn from from_heading to to_heading (degrees).
        Positive = CCW (left), negative = CW (right).
        """
        diff = (to_heading - from_heading + 180.0) % 360.0 - 180.0
        return diff

    # ------------------------------------------------------------------
    # Pathfinding
    # ------------------------------------------------------------------

    def _dijkstra(self, start: str, end: str) -> tuple[list[str] | None, float]:
        """Return (path, total_distance) or (None, inf) if unreachable."""
        dist = {node: float('inf') for node in self.graph}
        prev: dict[str, str | None] = {node: None for node in self.graph}
        dist[start] = 0.0
        heap: list[tuple[float, str]] = [(0.0, start)]

        while heap:
            d, u = heapq.heappop(heap)
            if d > dist[u]:
                continue
            if u == end:
                break
            for v, w in self.graph[u].items():
                alt = dist[u] + w
                if alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u
                    heapq.heappush(heap, (alt, v))

        if dist[end] == float('inf'):
            return None, float('inf')

        path: list[str] = []
        cur: str | None = end
        while cur is not None:
            path.append(cur)
            cur = prev[cur]
        path.reverse()
        return path, dist[end]

    # ------------------------------------------------------------------
    # Action server: LogisticsTask  (operator → nav_server)
    # ------------------------------------------------------------------

    def _execute_task(self, goal_handle):
        destination         = goal_handle.request.destination_waypoint
        carrying_box        = goal_handle.request.carrying_box
        skip_battery_check  = goal_handle.request.skip_battery_check
        self.get_logger().info(
            f'New task: send robot to [{destination}]  '
            f'carrying_box={carrying_box}  skip_battery_check={skip_battery_check}'
        )

        result = LogisticsTask.Result()
        result.diverted_to_charge = False

        # Pre-nav battery check (skipped for emergency-drop navigation).
        # Not carrying: divert to nearest charger, charge to 80%, then abort for re-queue.
        # Carrying: abort immediately — task manager handles the emergency drop.
        if not skip_battery_check:
            if not carrying_box:
                if self._handle_low_battery(goal_handle, result, [self.robot_position]):
                    return result
            else:
                if self._check_low_battery_carrying(goal_handle, result, [self.robot_position]):
                    return result

        if destination not in self.graph:
            result.success = False
            result.message = f'Unknown waypoint: {destination}'
            goal_handle.abort()
            return result

        if destination == self.robot_position:
            result.success = True
            result.path_taken = [destination]
            result.message = 'Already at destination'
            goal_handle.succeed()
            return result

        path, total_dist = self._dijkstra(self.robot_position, destination)

        if path is None:
            result.success = False
            result.message = f'No path from {self.robot_position} to {destination}'
            goal_handle.abort()
            return result

        self.get_logger().info(
            f'Path: {" -> ".join(path)}  (total {total_dist:.1f} m)'
        )

        # Publish initial feedback with the full planned path
        feedback = LogisticsTask.Feedback()
        feedback.planned_path = path
        feedback.current_waypoint = self.robot_position
        feedback.steps_completed = 0
        feedback.total_steps = len(path) - 1
        feedback.detail = f'Navigating to {destination}'
        goal_handle.publish_feedback(feedback)

        path_taken = [self.robot_position]

        for i in range(1, len(path)):
            if goal_handle.is_cancel_requested:
                result.success = False
                result.path_taken = path_taken
                result.message = 'Task cancelled'
                goal_handle.canceled()
                return result

            prev_wp = path[i - 1]
            next_wp = path[i]
            edge_dist = self.graph[prev_wp][next_wp]

            heading = self._heading(prev_wp, next_wp)
            turn = (
                self._turn(self.robot_heading_deg, heading)
                if self.robot_heading_deg is not None
                else 0.0
            )

            self.get_logger().info(
                f'Step {i}/{len(path) - 1}: [{next_wp}]  '
                f'dist={edge_dist:.1f} m  '
                f'heading={heading:.1f}°  turn={turn:+.1f}°'
            )

            if not self._send_move_command(next_wp, edge_dist, heading, turn):
                result.success = False
                result.path_taken = path_taken
                result.message = f'Robot failed to reach {next_wp}'
                goal_handle.abort()
                return result

            self.robot_position = next_wp
            self.robot_heading_deg = heading
            path_taken.append(next_wp)

            # Mid-nav battery check after each hop (skipped for emergency-drop nav)
            if not skip_battery_check:
                if not carrying_box:
                    if self._handle_low_battery(goal_handle, result, path_taken):
                        return result
                else:
                    if self._check_low_battery_carrying(goal_handle, result, path_taken):
                        return result

            feedback.current_waypoint = self.robot_position
            feedback.steps_completed = i
            feedback.detail = f'Step {i}/{feedback.total_steps} — at {self.robot_position}'
            goal_handle.publish_feedback(feedback)

        result.success = True
        result.path_taken = path_taken
        result.message = f'Arrived at {destination}'
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # Action client: MoveToWaypoint  (nav_server → robot_sim)
    # ------------------------------------------------------------------

    def _send_move_command(
        self, waypoint: str, distance: float,
        heading_deg: float = 0.0, turn_deg: float = 0.0,
    ) -> bool:
        """Send a single-step move goal and block until it completes."""
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Robot sim action server not available')
            return False

        done = threading.Event()
        result_holder: list[bool] = [False]

        def on_result(future):
            result_holder[0] = future.result().result.success
            done.set()

        def on_goal_response(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(f'Move to {waypoint} was rejected')
                done.set()
                return
            gh.get_result_async().add_done_callback(on_result)

        goal = MoveToWaypoint.Goal()
        goal.target_waypoint = waypoint
        goal.distance = float(distance)
        goal.heading_deg = float(heading_deg)
        goal.turn_deg = float(turn_deg)

        self._move_client.send_goal_async(goal).add_done_callback(on_goal_response)

        if not done.wait(timeout=120.0):
            self.get_logger().error(f'Timeout waiting for move to {waypoint}')
            return False

        return result_holder[0]


def main(args=None):
    rclpy.init(args=args)
    node = NavServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
