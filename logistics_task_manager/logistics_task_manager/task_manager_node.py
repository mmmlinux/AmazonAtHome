"""
Task Manager — multi-robot dispatcher.

Maintains one shared FIFO task queue.  When a task becomes available the
dispatcher assigns it to the closest idle robot.  A robot only returns to its
nearest charging station when the queue is empty; otherwise it immediately
becomes available for the next task.

A robot that is currently returning to charge can be interrupted and reassigned
as long as its battery is above the minimum threshold (MIN_BATTERY).  The
return trip is navigated hop-by-hop so the interrupt takes effect at the next
waypoint boundary, avoiding conflicting navigation goals.

Parameters
----------
robots            Comma-separated robot IDs (default: 'robot_1')
                  Each ID must match the ROS namespace of a running
                  robot_sim + nav_server pair.
map_file          Warehouse YAML map — used for distance-based selection.
charge_waypoint   Fallback charger if map is unavailable (default: 'charge_1')
load_waypoint     Loading-dock waypoint  (default: 'dock_1')
unload_waypoint   Unloading-dock waypoint (default: 'dock_1')

Topics published  (one per robot)
-----------------
  {robot_id}/task_status   logistics_interfaces/msg/TaskStatus

Services advertised
-------------------
  submit_task   logistics_interfaces/srv/SubmitTask

Actions used     (one client per robot)
------------
  {robot_id}/logistics_task   logistics_interfaces/action/LogisticsTask
"""

import heapq
import queue as _queue
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import yaml

from logistics_interfaces.action import LogisticsTask
from logistics_interfaces.msg import RobotStatus, TaskStatus, WarehouseState
from logistics_interfaces.srv import SetSlotOccupancy, SubmitTask


# Battery must be at or above this level for a robot to accept a new task
# while returning to charge.  Matches the nav_server's charge-lockout threshold.
MIN_BATTERY = 20.0


# ── Per-robot agent ───────────────────────────────────────────────────────────

class RobotAgent:
    """
    Tracks live state for one robot and owns its action client and
    task_status publisher.

    is_busy      True while executing a user task.
    is_returning True while navigating back to a charger (can be interrupted).

    A simple lock serialises writes to internal state fields.
    """

    def __init__(self, robot_id: str, node: Node, cb: ReentrantCallbackGroup):
        self.robot_id    = robot_id
        self.is_busy     = False
        self.is_returning = False

        # Signals the return-to-charge loop to abort after the current hop
        self._interrupt_return = threading.Event()

        self._lock             = threading.Lock()
        self._current_waypoint = 'charge_1'
        self._battery_level    = 100.0

        # Last-published task fields — merged into every publish so callers
        # only need to pass the fields that changed.
        self._task_status        = 'idle'
        self._task_type          = ''
        self._task_detail        = 'Idle at charging station'
        self._movement_priority  = TaskStatus.PRIORITY_IDLE

        self._action_client = ActionClient(
            node, LogisticsTask, f'{robot_id}/logistics_task',
            callback_group=cb,
        )
        self._status_pub = node.create_publisher(
            TaskStatus, f'{robot_id}/task_status', 10,
        )

    # ── Robot status updates (from robot_status topic) ────────────────────────

    def on_robot_status(self, msg: RobotStatus) -> None:
        with self._lock:
            self._current_waypoint = msg.current_waypoint
            self._battery_level    = float(msg.battery_level)

    @property
    def waypoint(self) -> str:
        with self._lock:
            return self._current_waypoint

    @property
    def battery(self) -> float:
        with self._lock:
            return self._battery_level

    # ── Task status publishing ────────────────────────────────────────────────

    def publish(self, queue_size: int, **kwargs) -> None:
        """
        Publish a TaskStatus message.  Only the fields given in kwargs are
        updated; all other fields retain their previous values.
        """
        with self._lock:
            if 'task_status'       in kwargs: self._task_status       = kwargs['task_status']
            if 'task_type'         in kwargs: self._task_type         = kwargs['task_type']
            if 'task_detail'       in kwargs: self._task_detail       = kwargs['task_detail']
            if 'movement_priority' in kwargs: self._movement_priority = kwargs['movement_priority']

            msg = TaskStatus()
            msg.robot_id          = self.robot_id
            msg.task_status       = self._task_status
            msg.task_type         = self._task_type
            msg.task_detail       = self._task_detail
            msg.queue_size        = queue_size
            msg.movement_priority = self._movement_priority

        self._status_pub.publish(msg)


# ── Task manager node ─────────────────────────────────────────────────────────

class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('task_manager')

        self.declare_parameter('robots',           'robot_1')
        self.declare_parameter('map_file',         '')
        self.declare_parameter('charge_waypoint',  'charge_1')
        self.declare_parameter('load_waypoint',    'dock_1')
        self.declare_parameter('unload_waypoint',  'dock_1')

        robots_str      = self.get_parameter('robots').get_parameter_value().string_value
        map_file        = self.get_parameter('map_file').get_parameter_value().string_value
        self._charge_wp = self.get_parameter('charge_waypoint').get_parameter_value().string_value
        self._load_wp   = self.get_parameter('load_waypoint').get_parameter_value().string_value
        self._unload_wp = self.get_parameter('unload_waypoint').get_parameter_value().string_value

        robot_ids = [r.strip() for r in robots_str.split(',') if r.strip()]

        self._waypoint_info: dict = {}
        self._graph: dict         = {}
        self._docks: list[str]    = []   # all loading/unloading docks in the map
        if map_file:
            self._load_map(map_file)
        else:
            self.get_logger().warning('No map_file set — closest-robot selection disabled')

        # Fallback: use the explicit params if map didn't supply docks
        if not self._docks:
            self._docks = list({self._load_wp, self._unload_wp})

        self._task_queue: _queue.Queue = _queue.Queue()
        self._slot_occupied: dict[str, bool] = {}

        cb = ReentrantCallbackGroup()

        # Build one agent per robot
        self._robots: list[RobotAgent] = []
        for rid in robot_ids:
            agent = RobotAgent(rid, self, cb)
            self._robots.append(agent)
            self.create_subscription(
                RobotStatus, f'{rid}/robot_status',
                lambda msg, a=agent: a.on_robot_status(msg),
                10, callback_group=cb,
            )

        self._submit_srv = self.create_service(
            SubmitTask, 'submit_task', self._on_submit_task, callback_group=cb,
        )

        self._occupancy_client = self.create_client(
            SetSlotOccupancy, 'set_slot_occupancy', callback_group=cb,
        )

        self.create_subscription(
            WarehouseState, 'warehouse_state', self._on_warehouse_state, 10,
            callback_group=cb,
        )

        # Publish initial idle status so web node gets something on connect
        for agent in self._robots:
            agent.publish(queue_size=0,
                          task_status='idle', task_type='',
                          task_detail='Idle at charging station')

        threading.Thread(target=self._dispatcher, daemon=True).start()

        self.get_logger().info(
            f'Task manager ready  robots={robot_ids}  '
            f'load={self._load_wp}  unload={self._unload_wp}'
        )

    # ── Map helpers ───────────────────────────────────────────────────────────

    def _load_map(self, map_file: str) -> None:
        with open(map_file) as f:
            data = yaml.safe_load(f)
        self._waypoint_info = {
            wp_id: dict(wp_data)
            for wp_id, wp_data in data['waypoints'].items()
        }
        self._graph = {wp: {} for wp in data['waypoints']}
        for edge in data['edges']:
            a, b = edge['from'], edge['to']
            d    = float(edge['distance'])
            self._graph[a][b] = d
            self._graph[b][a] = d

        dock_types = {'loading_unloading', 'loading', 'unloading'}
        self._docks = [
            wp for wp, info in self._waypoint_info.items()
            if info.get('type') in dock_types
        ]
        self.get_logger().info(
            f'Map loaded: {len(self._graph)} waypoints, {len(data["edges"])} edges, '
            f'{len(self._docks)} docks ({self._docks})'
        )

    def _dist(self, a: str, b: str) -> float:
        """Dijkstra shortest distance between two waypoints."""
        if not self._graph or a == b:
            return 0.0
        d = {n: float('inf') for n in self._graph}
        d[a] = 0.0
        heap = [(0.0, a)]
        while heap:
            cost, u = heapq.heappop(heap)
            if cost > d[u]:
                continue
            if u == b:
                break
            for v, w in self._graph.get(u, {}).items():
                alt = d[u] + w
                if alt < d[v]:
                    d[v] = alt
                    heapq.heappush(heap, (alt, v))
        return d.get(b, float('inf'))

    def _path_to(self, a: str, b: str) -> list[str]:
        """Dijkstra shortest path; returns ordered list of waypoint IDs."""
        if not self._graph or a == b:
            return [a]
        d    = {n: float('inf') for n in self._graph}
        prev: dict[str, str | None] = {n: None for n in self._graph}
        d[a] = 0.0
        heap = [(0.0, a)]
        while heap:
            cost, u = heapq.heappop(heap)
            if cost > d[u]:
                continue
            if u == b:
                break
            for v, w in self._graph.get(u, {}).items():
                alt = d[u] + w
                if alt < d[v]:
                    d[v] = alt
                    prev[v] = u
                    heapq.heappush(heap, (alt, v))
        if d.get(b, float('inf')) == float('inf'):
            return [a]      # unreachable — caller falls back to direct nav
        path: list[str] = []
        cur: str | None = b
        while cur is not None:
            path.append(cur)
            cur = prev[cur]
        path.reverse()
        return path

    def _nearest_charger(self, from_wp: str) -> str:
        chargers = [
            wp for wp, info in self._waypoint_info.items()
            if info.get('type') == 'charging'
        ]
        if not chargers:
            return self._charge_wp
        if from_wp in chargers:
            return from_wp
        return min(chargers, key=lambda c: self._dist(from_wp, c))

    # ── Warehouse state subscription ──────────────────────────────────────────

    def _on_warehouse_state(self, msg: WarehouseState) -> None:
        self._slot_occupied = {s.waypoint_id: s.occupied for s in msg.slots}

    # ── Service: SubmitTask ───────────────────────────────────────────────────

    def _on_submit_task(self, req: SubmitTask.Request,
                        resp: SubmitTask.Response) -> SubmitTask.Response:
        self._task_queue.put({'type': req.task_type, 'slot': req.slot})
        qs = self._task_queue.qsize()
        for agent in self._robots:
            agent.publish(queue_size=qs)
        self.get_logger().info(
            f'Task queued: {req.task_type} {req.slot}  (queue depth: {qs})'
        )
        resp.ok      = True
        resp.message = 'Task queued'
        return resp

    # ── Dispatcher ───────────────────────────────────────────────────────────

    def _dispatcher(self) -> None:
        """
        Pulls tasks one at a time from the queue and assigns each to the
        closest available robot.

        'Available' means either:
          - fully idle (not busy, not returning), OR
          - returning to charge with battery >= MIN_BATTERY (interrupted)
        """
        while True:
            task = self._task_queue.get()
            qs   = self._task_queue.qsize()
            for agent in self._robots:
                agent.publish(queue_size=qs)

            first_wp = (
                task['slot'] if task['type'] == 'pickup'
                else self._load_wp
            )

            robot = None
            while robot is None:
                # Prefer fully idle robots
                idle = [r for r in self._robots
                        if not r.is_busy and not r.is_returning]
                if idle:
                    robot = min(idle, key=lambda r: self._dist(r.waypoint, first_wp))
                    robot.is_busy = True
                    break

                # Interrupt a returning robot if battery allows
                returnable = [r for r in self._robots
                              if r.is_returning and r.battery >= MIN_BATTERY]
                if returnable:
                    robot = min(returnable, key=lambda r: self._dist(r.waypoint, first_wp))
                    robot._interrupt_return.set()
                    # Wait for the current hop to finish and the return thread to yield
                    while robot.is_returning:
                        time.sleep(0.05)
                    robot.is_busy = True
                    break

                time.sleep(0.05)

            threading.Thread(
                target=self._execute_task,
                args=(robot, task),
                daemon=True,
            ).start()

    # ── Task execution ────────────────────────────────────────────────────────

    def _qs(self) -> int:
        return self._task_queue.qsize()

    def _pick_dock(self, robot: RobotAgent, *, need_occupied: bool) -> str:
        """
        Find and reserve the closest dock in the required state.
          need_occupied=False  drop-off: pick a free dock, reserve it immediately
          need_occupied=True   pick-up:  pick a dock that has an item

        If no dock is currently available, blocks and retries every second.
        Reserving a free dock (need_occupied=False) calls set_slot_occupancy
        immediately so no other robot races to the same dock.
        """
        while True:
            if need_occupied:
                candidates = [d for d in self._docks
                              if self._slot_occupied.get(d, False)]
            else:
                candidates = [d for d in self._docks
                              if not self._slot_occupied.get(d, False)]

            if candidates:
                dock = min(candidates, key=lambda d: self._dist(robot.waypoint, d))
                if not need_occupied:
                    # Reserve immediately so other robots don't pick the same dock
                    self._set_occupancy(dock, True)
                return dock

            msg = ('Waiting for a dock with an item...'
                   if need_occupied else 'Waiting for a free dock...')
            robot.publish(queue_size=self._qs(), task_detail=msg)
            time.sleep(1.0)

    def _set_occupancy(self, waypoint_id: str, occupied: bool) -> None:
        """Fire-and-forget occupancy update to the warehouse state node."""
        if not self._occupancy_client.service_is_ready():
            self.get_logger().warning(
                f'Occupancy service not ready — skipping {waypoint_id}'
            )
            return
        req = SetSlotOccupancy.Request()
        req.waypoint_id = waypoint_id
        req.occupied    = occupied
        self._occupancy_client.call_async(req)

    def _execute_task(self, robot: RobotAgent, task: dict) -> None:
        try:
            if task['type'] == 'pickup':
                self._run_pickup(robot, task['slot'])
            elif task['type'] == 'delivery':
                self._run_delivery(robot, task['slot'])
            else:
                self.get_logger().warning(f'Unknown task type: {task["type"]}')
        except Exception as exc:
            self.get_logger().error(f'[{robot.robot_id}] Task error: {exc}')
            robot.publish(queue_size=self._qs(), task_status='error',
                          task_detail=str(exc),
                          movement_priority=TaskStatus.PRIORITY_IDLE)

        robot.is_busy = False

        # Return to charge only when queue is empty
        if self._task_queue.empty():
            self._return_to_charge(robot)

    def _return_to_charge(self, robot: RobotAgent) -> None:
        """
        Navigate hop-by-hop to the nearest charger.
        Aborts after the current hop if _interrupt_return is set by the dispatcher
        (meaning a new task was assigned).
        """
        charger = self._nearest_charger(robot.waypoint)

        robot.is_returning = True
        robot._interrupt_return.clear()

        robot.publish(queue_size=0, task_status='running', task_type='',
                      task_detail=f'Queue empty — returning to {charger}...',
                      movement_priority=TaskStatus.PRIORITY_RETURNING_CHARGE)

        if self._graph:
            path = self._path_to(robot.waypoint, charger)
            completed = True
            for waypoint in path[1:]:
                if robot._interrupt_return.is_set():
                    completed = False
                    break
                self._nav_to(robot, waypoint)
        else:
            # No map — navigate directly (not interruptible mid-trip)
            completed = not robot._interrupt_return.is_set()
            if completed:
                self._nav_to(robot, charger)

        if completed:
            robot.publish(queue_size=0, task_status='idle', task_type='',
                          task_detail='Idle at charging station',
                          movement_priority=TaskStatus.PRIORITY_IDLE)

        robot.is_returning = False

    def _run_pickup(self, robot: RobotAgent, slot: str) -> None:
        self.get_logger().info(f'[{robot.robot_id}] Pickup: {slot}')

        # Skip if slot is known to be empty
        if not self._slot_occupied.get(slot, True):
            self.get_logger().warning(
                f'[{robot.robot_id}] Pickup skipped — {slot} is empty'
            )
            robot.publish(queue_size=self._qs(), task_status='idle',
                          task_type='', task_detail=f'Skipped: {slot} is empty',
                          movement_priority=TaskStatus.PRIORITY_IDLE)
            return

        robot.publish(queue_size=self._qs(), task_status='running',
                      task_type='pickup', task_detail=f'Navigating to {slot}...',
                      movement_priority=TaskStatus.PRIORITY_TRAVELLING_EMPTY)
        if not self._nav_to(robot, slot):
            robot.publish(queue_size=self._qs(), task_status='error',
                          task_detail=f'Could not reach {slot}',
                          movement_priority=TaskStatus.PRIORITY_IDLE)
            return

        robot.publish(queue_size=self._qs(), task_detail=f'Picking up box at {slot}...',
                      movement_priority=TaskStatus.PRIORITY_LOADED)
        time.sleep(2.0)
        self._set_occupancy(slot, False)

        # Find and reserve the closest free unload dock
        unload_dock = self._pick_dock(robot, need_occupied=False)

        robot.publish(queue_size=self._qs(),
                      task_detail=f'Navigating to {unload_dock}...')
        if not self._nav_to(robot, unload_dock):
            self._set_occupancy(unload_dock, False)   # release reservation on failure
            robot.publish(queue_size=self._qs(), task_status='error',
                          task_detail=f'Could not reach {unload_dock}',
                          movement_priority=TaskStatus.PRIORITY_IDLE)
            return

        robot.publish(queue_size=self._qs(), task_detail=f'Dropping off box at {unload_dock}...')
        time.sleep(2.0)
        # Dock stays occupied (item is now there)

    def _run_delivery(self, robot: RobotAgent, slot: str) -> None:
        self.get_logger().info(f'[{robot.robot_id}] Delivery: {slot}')

        # Find a load dock that has an item ready
        load_dock = self._pick_dock(robot, need_occupied=True)

        robot.publish(queue_size=self._qs(), task_status='running',
                      task_type='delivery',
                      task_detail=f'Navigating to {load_dock}...',
                      movement_priority=TaskStatus.PRIORITY_TRAVELLING_EMPTY)
        if not self._nav_to(robot, load_dock):
            robot.publish(queue_size=self._qs(), task_status='error',
                          task_detail=f'Could not reach {load_dock}',
                          movement_priority=TaskStatus.PRIORITY_IDLE)
            return

        robot.publish(queue_size=self._qs(),
                      task_detail=f'Picking up box from {load_dock}...',
                      movement_priority=TaskStatus.PRIORITY_LOADED)
        time.sleep(2.0)
        self._set_occupancy(load_dock, False)

        robot.publish(queue_size=self._qs(),
                      task_detail=f'Delivering to {slot}...')
        if not self._nav_to(robot, slot):
            robot.publish(queue_size=self._qs(), task_status='error',
                          task_detail=f'Could not reach {slot}',
                          movement_priority=TaskStatus.PRIORITY_IDLE)
            return

        robot.publish(queue_size=self._qs(),
                      task_detail=f'Placing box at {slot}...')
        time.sleep(2.0)
        self._set_occupancy(slot, True)

    # ── Navigation helper ─────────────────────────────────────────────────────

    def _nav_to(self, robot: RobotAgent, waypoint: str) -> bool:
        """Send a LogisticsTask to this robot's nav_server and block until done."""
        if not robot._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'[{robot.robot_id}] Nav server not available')
            return False

        done           = threading.Event()
        success_holder: list[bool] = [False]

        def on_result(future):
            success_holder[0] = future.result().result.success
            done.set()

        def on_feedback(fb_msg):
            fb     = fb_msg.feedback
            detail = fb.detail if fb.detail else (
                f'Step {fb.steps_completed}/{fb.total_steps}'
                f' — at {fb.current_waypoint}'
            )
            robot.publish(queue_size=self._qs(), task_detail=detail)

        def on_goal(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(
                    f'[{robot.robot_id}] Goal to {waypoint} rejected'
                )
                done.set()
                return
            gh.get_result_async().add_done_callback(on_result)

        goal = LogisticsTask.Goal()
        goal.destination_waypoint = waypoint
        robot._action_client.send_goal_async(
            goal, feedback_callback=on_feedback
        ).add_done_callback(on_goal)

        done.wait(timeout=300.0)
        return success_holder[0]


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
