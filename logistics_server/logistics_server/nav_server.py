"""
Navigation Server — pathfinding, task execution, and collision avoidance.

One instance runs per robot, scoped under its namespace (robot_1/, robot_2/).
It owns the Dijkstra graph, processes LogisticsTask action goals from the task
manager, and drives the robot hop-by-hop via MoveToWaypoint action goals sent
to robot_sim.

Collision avoidance
-------------------
Before stepping onto any waypoint the nav_server calls the global
TrafficControllerNode (traffic/acquire service).  The TC grants or denies
permission; on denial the nav_server waits and retries.  If the TC detects a
deadlock it publishes a yield signal on /{robot_id}/yield_request, which
interrupts the retry loop and triggers a one-hop retreat.

Each step also reruns Dijkstra with a +50 m penalty on peer-robot positions so
the planned route naturally routes around occupied waypoints without hard-blocking.

Battery lockout
---------------
Battery thresholds are enforced at the nav_server level (not task_manager) so
the robot always handles its own diversion.

  < min_battery  (empty robot) : navigate to nearest free charger, charge to 80%,
                                  then abort the goal for re-queue.
  < critical_battery (carrying): abort immediately; task_manager handles
                                  the emergency drop.

Actions served
--------------
  logistics_task   logistics_interfaces/action/LogisticsTask   (task_manager → nav_server)

Actions used
------------
  move_to_waypoint   logistics_interfaces/action/MoveToWaypoint   (nav_server → robot_sim)

Services used
-------------
  /traffic/acquire   logistics_interfaces/srv/AcquireWaypoint
  /traffic/release   logistics_interfaces/srv/ReleaseWaypoint

Topics subscribed
-----------------
  robot_status         logistics_interfaces/msg/RobotStatus    (own battery/position)
  yield_request        std_msgs/msg/Empty                      (TC deadlock signal)
  /warehouse_state     logistics_interfaces/msg/WarehouseState (slot occupancy)
  /{peer}/robot_status logistics_interfaces/msg/RobotStatus    (peer positions)

Parameters
----------
  map_file          path to warehouse YAML
  robot_start       starting waypoint ID   (default 'charge_1')
  peers             comma-separated peer robot IDs (e.g. 'robot_2')
  min_battery       empty-robot charge threshold % (default 50.0)
  critical_battery  loaded-robot emergency-drop threshold % (default 25.0)
"""

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

from std_msgs.msg import Empty

from logistics_interfaces.action import LogisticsTask, MoveToWaypoint
from logistics_interfaces.msg import RobotStatus, WarehouseState
from logistics_interfaces.srv import AcquireWaypoint, ReleaseWaypoint

# Collision-avoidance movement priorities (higher = gets waypoint first).
# Separate from the display priorities on TaskStatus.
_PRIO_IDLE     = 0
_PRIO_CHARGING = 1   # diverting to charger (yields to working robots)
_PRIO_EMPTY    = 2   # travelling without box
_PRIO_LOADED   = 3   # travelling with box — don't block deliveries


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
        self.declare_parameter('peers', '')           # comma-separated peer robot IDs
        self._robot_id = self.get_namespace().lstrip('/')
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

        # Current collision-avoidance movement priority (set per task).
        self._current_priority: int = _PRIO_IDLE

        # Set when the traffic controller signals this robot to yield.
        self._yield_event = threading.Event()

        # Battery state — updated via robot_status subscription
        self._battery_level: float = 100.0
        self._battery_lock = threading.Lock()

        # Slot occupancy — updated via warehouse_state topic
        self._slot_occupied: dict[str, bool] = {}

        # Peer robot positions: peer_id → current_waypoint
        self._peer_positions: dict[str, str] = {}

        # Reentrant groups so the action client callbacks can fire while the
        # action server execute callback is blocking on threading.Event.
        cb_group = ReentrantCallbackGroup()

        self._status_sub = self.create_subscription(
            RobotStatus, 'robot_status', self._on_robot_status, 10,
            callback_group=cb_group,
        )
        self.create_subscription(
            Empty, 'yield_request', self._on_yield, 10, callback_group=cb_group,
        )
        self.create_subscription(
            WarehouseState, '/warehouse_state', self._on_warehouse_state, 10,
            callback_group=cb_group,
        )

        # Subscribe to each peer's robot_status to track their positions.
        peers_param = self.get_parameter('peers').get_parameter_value().string_value
        for peer in (p.strip() for p in peers_param.split(',') if p.strip()):
            self.create_subscription(
                RobotStatus,
                f'/{peer}/robot_status',
                lambda msg, pid=peer: self._on_peer_status(pid, msg),
                10,
                callback_group=cb_group,
            )

        self._move_client = ActionClient(
            self, MoveToWaypoint, 'move_to_waypoint', callback_group=cb_group
        )

        # Traffic controller clients for collision avoidance.
        self._acquire_cli = self.create_client(
            AcquireWaypoint, '/traffic/acquire', callback_group=cb_group,
        )
        self._release_cli = self.create_client(
            ReleaseWaypoint, '/traffic/release', callback_group=cb_group,
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

        # Claim starting position after the executor has started spinning so
        # the async service call can complete.  (Calling _acquire_wp directly
        # in __init__ hangs because call_async callbacks never fire until spin()
        # is running, causing repeated 5-second timeouts and a 30s delay.)
        self._startup_timer = self.create_timer(0.5, self._claim_start_position)

    # ------------------------------------------------------------------
    # Startup
    # ------------------------------------------------------------------

    def _claim_start_position(self) -> None:
        """One-shot timer: claim the starting waypoint once the executor is spinning."""
        self._startup_timer.cancel()
        self._acquire_wp(self.robot_position, _PRIO_IDLE)
        self.get_logger().info(f'[{self._robot_id}] Starting position {self.robot_position} claimed')

    # ------------------------------------------------------------------
    # Map loading
    # ------------------------------------------------------------------

    def _load_map(self, map_file: str) -> None:
        """
        Parse the warehouse YAML and populate self.waypoint_info and self.graph.

        Raises RuntimeError if map_file is empty (misconfigured launch).
        """
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
        """Update battery level from the robot_sim's periodic status publish."""
        with self._battery_lock:
            self._battery_level = float(msg.battery_level)

    def _on_peer_status(self, peer_id: str, msg: RobotStatus) -> None:
        """Track peer robot positions for Dijkstra penalty and charger avoidance."""
        self._peer_positions[peer_id] = msg.current_waypoint

    def _on_warehouse_state(self, msg: WarehouseState) -> None:
        """Cache slot occupancy so battery-divert logic can find empty storage spots."""
        self._slot_occupied = {s.waypoint_id: s.occupied for s in msg.slots}

    # ------------------------------------------------------------------
    # Traffic control helpers
    # ------------------------------------------------------------------

    def _on_yield(self, _msg: Empty) -> None:
        """Traffic controller signals this robot to yield (deadlock detected)."""
        self.get_logger().info(f'[{self._robot_id}] Yield signal received — will retreat')
        self._yield_event.set()

    def _acquire_wp(self, waypoint: str, priority: int, timeout: float = 30.0) -> bool:
        """
        Block until this robot acquires the waypoint lock, or timeout expires.

        Returns True  — granted (or TC unavailable, proceed without locking).
        Returns False — yield signal received; caller should call _yield_retreat()
                        then replan.

        Higher-priority robots use a shorter retry interval so they naturally
        get ahead of lower-priority ones when both are waiting on the same spot.
        """
        if not self._acquire_cli.wait_for_service(timeout_sec=2.0):
            return True  # TC not running — no-op

        deadline = time.time() + timeout
        # Higher-priority robots poll more aggressively so they naturally win
        # contested waypoints over lower-priority ones.
        retry_interval = max(0.05, 0.3 - priority * 0.05)

        while time.time() < deadline:
            # Check for yield signal before each attempt.
            if self._yield_event.is_set():
                self._yield_event.clear()
                return False

            req = AcquireWaypoint.Request()
            req.robot_id = self._robot_id
            req.waypoint = waypoint
            req.priority = priority

            # Bridge the async service call back to synchronous with an Event.
            # The default callback group is Reentrant, so this fires on the
            # executor thread while we block here on the action thread.
            done = threading.Event()
            holder: list = [None]

            def _cb(fut, _h=holder, _d=done):
                _h[0] = fut.result()
                _d.set()

            self._acquire_cli.call_async(req).add_done_callback(_cb)
            done.wait(timeout=5.0)  # 5 s covers slow TC under load

            if holder[0] is not None and holder[0].granted:
                return True

            time.sleep(retry_interval)

        self.get_logger().warning(
            f'[{self._robot_id}] Timeout acquiring {waypoint} — proceeding anyway'
        )
        return True  # safety valve: proceed rather than hang forever

    def _release_wp(self, waypoint: str) -> None:
        """
        Asynchronously release a waypoint in the traffic controller.

        Fire-and-forget: we don't need the response — the TC will unblock any
        robot that was waiting on this waypoint as a side effect.
        """
        if not self._release_cli.wait_for_service(timeout_sec=1.0):
            return
        req = ReleaseWaypoint.Request()
        req.robot_id = self._robot_id
        req.waypoint = waypoint
        self._release_cli.call_async(req)

    def _yield_retreat(self, blocked_wp: str, priority: int) -> bool:
        """
        Back up one hop to break a deadlock.

        Picks an adjacent waypoint that is not the one we're blocked on,
        preferring junctions and chargers.  Acquires the retreat waypoint,
        moves there, and releases the current position.

        Returns True if the retreat succeeded (robot_position is updated).
        Returns False if no retreat is possible (caller should wait and retry).
        """
        candidates = [
            wp for wp in self.graph.get(self.robot_position, {}).keys()
            if wp != blocked_wp
        ]
        if not candidates:
            return False

        junction_types = frozenset({'intersection', 'charging'})
        junctions = [wp for wp in candidates
                     if self.waypoint_info.get(wp, {}).get('type') in junction_types]
        retreat_wp = junctions[0] if junctions else candidates[0]

        self.get_logger().info(
            f'[{self._robot_id}] Deadlock yield: '
            f'{self.robot_position} → {retreat_wp}'
        )

        if not self._acquire_cli.wait_for_service(timeout_sec=1.0):
            return False

        req = AcquireWaypoint.Request()
        req.robot_id = self._robot_id
        req.waypoint = retreat_wp
        req.priority = priority

        done = threading.Event()
        holder: list = [None]

        def _cb(fut, _h=holder, _d=done):
            _h[0] = fut.result()
            _d.set()

        self._acquire_cli.call_async(req).add_done_callback(_cb)
        done.wait(timeout=3.0)

        if not holder[0] or not holder[0].granted:
            return False

        edge_dist = self.graph[self.robot_position][retreat_wp]
        heading   = self._heading(self.robot_position, retreat_wp)
        turn      = (
            self._turn(self.robot_heading_deg, heading)
            if self.robot_heading_deg is not None else 0.0
        )
        old_pos = self.robot_position
        if not self._send_move_command(retreat_wp, edge_dist, heading, turn):
            return False

        self._release_wp(old_pos)
        self.robot_position = retreat_wp
        self.robot_heading_deg = heading
        return True

    # ------------------------------------------------------------------
    # Battery lockout
    # ------------------------------------------------------------------

    def _find_nearest_charger(self) -> str | None:
        """
        Return the nearest free charging waypoint reachable from current position.

        Chargers currently occupied by peer robots are skipped.  If every charger
        is occupied (unlikely but possible), falls back to the nearest one anyway
        so the robot never gets stranded with a dead battery.
        """
        occupied_by_peers = set(self._peer_positions.values())

        all_chargers = [
            wp for wp, info in self.waypoint_info.items()
            if info.get('type') == 'charging'
        ]
        if not all_chargers:
            return None

        # Prefer free chargers; fall back to any charger only if all are taken.
        free_chargers = [wp for wp in all_chargers if wp not in occupied_by_peers]
        candidates    = free_chargers if free_chargers else all_chargers

        if self.robot_position in candidates:
            return self.robot_position

        best_wp, best_dist = None, float('inf')
        for wp in candidates:
            _, d = self._dijkstra(self.robot_position, wp)
            if d < best_dist:
                best_dist = d
                best_wp   = wp
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

    def _send_move_to(self, destination: str, priority: int = _PRIO_CHARGING) -> bool:
        """Navigate to destination using full Dijkstra path, updating robot_position."""
        peer_wps: set[str] = {
            wp for wp in self._peer_positions.values()
            if wp and wp != self.robot_position
        }
        path, _ = self._dijkstra(
            self.robot_position, destination,
            penalized_wps=peer_wps or None,
        )
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
            self._acquire_wp(next_wp, priority)
            if not self._send_move_command(next_wp, edge_dist, heading, turn):
                return False
            self._release_wp(prev_wp)
            self.robot_position = next_wp
            self.robot_heading_deg = heading
        return True

    # ------------------------------------------------------------------
    # Angle helpers
    # ------------------------------------------------------------------

    def _heading(self, from_wp: str, to_wp: str) -> float:
        """
        Absolute heading in degrees for the leg from_wp → to_wp.

        SVG coordinates have Y increasing downward, so dy is negated before
        calling atan2 to produce a standard screen-space heading where North
        (up on screen) maps to 90°.  The result is passed to robot_sim purely
        for informational logging; navigation uses distance, not heading.

        Coordinate convention:
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

    def _dijkstra(
        self,
        start: str,
        end: str,
        penalized_wps: set[str] | None = None,
        penalty: float = 50.0,
    ) -> tuple[list[str] | None, float]:
        """
        Return (path, total_distance) or (None, inf) if unreachable.

        penalized_wps  — waypoints to add `penalty` metres of cost to when
                         entering.  Used to route around peer-robot positions
                         without hard-blocking those waypoints.
        """
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
                edge_cost = w + (penalty if penalized_wps and v in penalized_wps else 0.0)
                alt = dist[u] + edge_cost
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
        """
        Action server execute callback — navigate the robot to a destination.

        Called by the task_manager via a LogisticsTask goal.  The method runs
        in a background thread (MultiThreadedExecutor) so blocking calls here
        do not stall other callbacks.

        Hop-by-hop loop
        ~~~~~~~~~~~~~~~
        Rather than planning once and following the path blindly, the robot
        replans at every hop using the current peer positions as a soft penalty.
        This means the route adapts dynamically if another robot moves into the
        way.  Each hop:
          1. Replan Dijkstra with +50 m peer-position penalty.
          2. Call _acquire_wp — blocks until the TC grants the next waypoint.
             If a yield signal fires during the wait, _acquire_wp returns False.
          3. On yield: _yield_retreat backs up one hop then replans from the new
             position; if retreat fails (no free neighbour), wait 1 s and retry.
          4. On acquire grant: send the MoveToWaypoint goal to robot_sim and wait.
          5. Release the previous waypoint and advance robot_position.
        """
        destination         = goal_handle.request.destination_waypoint
        carrying_box        = goal_handle.request.carrying_box
        skip_battery_check  = goal_handle.request.skip_battery_check
        priority = _PRIO_LOADED if carrying_box else _PRIO_EMPTY
        self._current_priority = priority
        self.get_logger().info(
            f'New task: send robot to [{destination}]  '
            f'carrying_box={carrying_box}  skip_battery_check={skip_battery_check}  '
            f'priority={priority}'
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
        step_count = 0

        # Re-plan at every hop so the route adapts to where peers currently are.
        while self.robot_position != destination:
            if goal_handle.is_cancel_requested:
                result.success = False
                result.path_taken = path_taken
                result.message = 'Task cancelled'
                goal_handle.canceled()
                return result

            # Build penalty set from known peer positions (exclude our own spot).
            peer_wps: set[str] = {
                wp for wp in self._peer_positions.values()
                if wp and wp != self.robot_position
            }

            path, _ = self._dijkstra(
                self.robot_position, destination,
                penalized_wps=peer_wps or None,
            )
            if path is None or len(path) < 2:
                result.success = False
                result.path_taken = path_taken
                result.message = f'No path from {self.robot_position} to {destination}'
                goal_handle.abort()
                return result

            next_wp   = path[1]
            edge_dist = self.graph[self.robot_position][next_wp]
            heading   = self._heading(self.robot_position, next_wp)
            turn      = (
                self._turn(self.robot_heading_deg, heading)
                if self.robot_heading_deg is not None else 0.0
            )

            self.get_logger().info(
                f'Step {step_count + 1} → [{next_wp}]  '
                f'dist={edge_dist:.1f} m  remaining={len(path) - 1} hops'
                + (f'  avoiding={peer_wps}' if peer_wps else '')
            )

            # Acquire next waypoint.  Returns False if a yield signal was received.
            if not self._acquire_wp(next_wp, priority):
                if not self._yield_retreat(next_wp, priority):
                    time.sleep(1.0)  # no retreat possible — wait and replan
                # Either way: position may have changed, replan next iteration.
                continue

            if not self._send_move_command(next_wp, edge_dist, heading, turn):
                result.success = False
                result.path_taken = path_taken
                result.message = f'Robot failed to reach {next_wp}'
                goal_handle.abort()
                return result

            prev_wp = self.robot_position
            self._release_wp(prev_wp)
            self.robot_position   = next_wp
            self.robot_heading_deg = heading
            path_taken.append(next_wp)
            step_count += 1

            # Mid-nav battery check after each hop (skipped for emergency-drop nav)
            if not skip_battery_check:
                if not carrying_box:
                    if self._handle_low_battery(goal_handle, result, path_taken):
                        return result
                else:
                    if self._check_low_battery_carrying(goal_handle, result, path_taken):
                        return result

            feedback.planned_path     = path
            feedback.current_waypoint = self.robot_position
            feedback.steps_completed  = step_count
            feedback.total_steps      = step_count + len(path) - 2
            feedback.detail = f'Step {step_count} — at {self.robot_position}'
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
        """
        Send a single-hop MoveToWaypoint goal to robot_sim and block until done.

        Uses a threading.Event + done callback pattern because the ROS 2 action
        client API is async-only — we bridge back to synchronous here so the
        calling execute callback can use normal sequential logic.

        Returns True on success, False if the server was unavailable, rejected
        the goal, or timed out (120 s safety valve).
        """
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
