"""
Task Manager node for the warehouse logistics system.

Responsibilities:
  - Accepts task submissions via the SubmitTask ROS service
  - Maintains a FIFO task queue
  - Executes tasks sequentially, sending LogisticsTask action goals to nav_server
  - Returns to the charging station only when the queue is empty
  - Publishes TaskStatus at every state change so other nodes (e.g. web UI)
    can display live progress

Topics published:
  task_status  (logistics_interfaces/msg/TaskStatus)

Services advertised:
  submit_task  (logistics_interfaces/srv/SubmitTask)

Actions used:
  logistics_task  (logistics_interfaces/action/LogisticsTask)
"""

import queue as _queue
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from logistics_interfaces.action import LogisticsTask
from logistics_interfaces.msg import TaskStatus
from logistics_interfaces.srv import SubmitTask


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager')

        self.declare_parameter('charge_waypoint', 'charge_1')
        self.declare_parameter('load_waypoint',   'dock_1')
        self.declare_parameter('unload_waypoint', 'dock_1')

        self._charge_wp = self.get_parameter('charge_waypoint').get_parameter_value().string_value
        self._load_wp   = self.get_parameter('load_waypoint').get_parameter_value().string_value
        self._unload_wp = self.get_parameter('unload_waypoint').get_parameter_value().string_value

        self._task_queue: _queue.Queue = _queue.Queue()

        # Current status — written by task runner thread, read by publish helper.
        # Protected by _status_lock.
        self._status_lock = threading.Lock()
        self._status = TaskStatus()
        self._status.task_status = 'idle'
        self._status.task_type   = ''
        self._status.task_detail = 'Idle at charging station'
        self._status.queue_size  = 0

        cb = ReentrantCallbackGroup()

        self._status_pub = self.create_publisher(TaskStatus, 'task_status', 10)

        self._submit_srv = self.create_service(
            SubmitTask, 'submit_task', self._on_submit_task,
            callback_group=cb,
        )

        self._task_client = ActionClient(
            self, LogisticsTask, 'logistics_task', callback_group=cb,
        )

        # Publish initial status so subscribers get a value on connection
        self._publish_status()

        threading.Thread(target=self._task_runner, daemon=True).start()

        self.get_logger().info(
            f'Task manager ready  '
            f'charge={self._charge_wp}  load={self._load_wp}  unload={self._unload_wp}'
        )

    # ── Service: SubmitTask ───────────────────────────────────────────────────

    def _on_submit_task(self, request: SubmitTask.Request,
                        response: SubmitTask.Response) -> SubmitTask.Response:
        self._task_queue.put({'type': request.task_type, 'slot': request.slot})
        self._set_status(queue_size=self._task_queue.qsize())
        self.get_logger().info(
            f'Task queued: {request.task_type} {request.slot}  '
            f'(queue depth: {self._task_queue.qsize()})'
        )
        response.ok      = True
        response.message = 'Task queued'
        return response

    # ── Status helpers ────────────────────────────────────────────────────────

    def _set_status(self, **kwargs) -> None:
        with self._status_lock:
            for k, v in kwargs.items():
                setattr(self._status, k, v)
        self._publish_status()

    def _publish_status(self) -> None:
        with self._status_lock:
            msg = TaskStatus()
            msg.task_status = self._status.task_status
            msg.task_type   = self._status.task_type
            msg.task_detail = self._status.task_detail
            msg.queue_size  = self._status.queue_size
        self._status_pub.publish(msg)

    # ── Task runner ───────────────────────────────────────────────────────────

    def _task_runner(self) -> None:
        while True:
            task = self._task_queue.get()
            self._set_status(queue_size=self._task_queue.qsize())

            try:
                if task['type'] == 'pickup':
                    self._run_pickup(task['slot'])
                elif task['type'] == 'delivery':
                    self._run_delivery(task['slot'])
                else:
                    self.get_logger().warning(f'Unknown task type: {task["type"]}')
            except Exception as exc:
                self.get_logger().error(f'Task error: {exc}')
                self._set_status(task_status='error', task_detail=str(exc))

            # Only return to the charger when there are no more queued tasks
            if self._task_queue.empty():
                self._set_status(
                    task_detail='Queue empty — returning to charging station...'
                )
                self._nav_to(self._charge_wp)
                self._set_status(
                    task_status='idle', task_type='',
                    task_detail='Idle at charging station', queue_size=0,
                )
            else:
                remaining = self._task_queue.qsize()
                self._set_status(
                    queue_size=remaining,
                    task_detail=f'{remaining} task(s) remaining in queue',
                )

    # ── Task execution ────────────────────────────────────────────────────────

    def _run_pickup(self, slot: str) -> None:
        self.get_logger().info(f'Pickup task: {slot}')
        self._set_status(task_status='running', task_type='pickup',
                         task_detail=f'Navigating to {slot}...')

        if not self._nav_to(slot):
            self._set_status(task_status='error',
                             task_detail=f'Could not reach {slot}')
            return

        self._set_status(task_detail=f'Picking up box at {slot}...')
        time.sleep(2.0)

        self._set_status(task_detail=f'Navigating to {self._unload_wp}...')
        if not self._nav_to(self._unload_wp):
            self._set_status(task_status='error',
                             task_detail='Could not reach unloading dock')
            return

        self._set_status(task_detail='Dropping off box...')
        time.sleep(2.0)

    def _run_delivery(self, slot: str) -> None:
        self.get_logger().info(f'Delivery task: {slot}')
        self._set_status(task_status='running', task_type='delivery',
                         task_detail=f'Navigating to {self._load_wp}...')

        if not self._nav_to(self._load_wp):
            self._set_status(task_status='error',
                             task_detail='Could not reach loading dock')
            return

        self._set_status(task_detail='Picking up box from loading dock...')
        time.sleep(2.0)

        self._set_status(task_detail=f'Delivering to {slot}...')
        if not self._nav_to(slot):
            self._set_status(task_status='error',
                             task_detail=f'Could not reach {slot}')
            return

        self._set_status(task_detail=f'Placing box at {slot}...')
        time.sleep(2.0)

    # ── Navigation helper ─────────────────────────────────────────────────────

    def _nav_to(self, waypoint: str) -> bool:
        """Send a LogisticsTask goal and block until complete."""
        if not self._task_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav server not available')
            return False

        done = threading.Event()
        success_holder: list[bool] = [False]

        def on_result(future):
            success_holder[0] = future.result().result.success
            done.set()

        def on_feedback(fb_msg):
            fb = fb_msg.feedback
            detail = fb.detail if fb.detail else (
                f'Step {fb.steps_completed}/{fb.total_steps}'
                f' — at {fb.current_waypoint}'
            )
            self._set_status(task_detail=detail)

        def on_goal(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(f'Goal to {waypoint} rejected')
                done.set()
                return
            gh.get_result_async().add_done_callback(on_result)

        goal = LogisticsTask.Goal()
        goal.destination_waypoint = waypoint
        self._task_client.send_goal_async(
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
