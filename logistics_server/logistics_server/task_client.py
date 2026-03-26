"""
Simple CLI client to send a LogisticsTask goal to the nav server.

Usage:
    ros2 run logistics_server task_client <destination_waypoint>
"""
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from logistics_interfaces.action import LogisticsTask


class TaskClient(Node):
    def __init__(self):
        super().__init__('task_client')
        self._client = ActionClient(self, LogisticsTask, 'logistics_task')

    def send_task(self, destination: str) -> None:
        self.get_logger().info(f'Waiting for nav_server...')
        self._client.wait_for_server()

        goal = LogisticsTask.Goal()
        goal.destination_waypoint = destination

        print(f'\nSending robot to: {destination}')
        self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        ).add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        if fb.steps_completed == 0:
            print(f'  Planned path: {" -> ".join(fb.planned_path)}')
        print(f'  Step {fb.steps_completed}/{fb.total_steps} — at [{fb.current_waypoint}]')

    def _on_goal_response(self, future) -> None:
        gh = future.result()
        if not gh.accepted:
            print('Goal rejected by nav_server.')
            rclpy.shutdown()
            return
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        if result.success:
            print(f'\nDone. Path taken: {" -> ".join(result.path_taken)}')
        else:
            print(f'\nFailed: {result.message}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print('Usage: task_client <destination_waypoint>')
        print()
        print('Waypoint types:')
        print('  charging    : charge_1')
        print('  loading     : load_1')
        print('  unloading   : unload_1')
        print('  quick_access: quick_1')
        print('  storage     : shelf_A1, shelf_A2, shelf_B1, shelf_B2')
        print('  intersection: inter_1, inter_2, inter_3, inter_4')
        rclpy.shutdown()
        return

    node = TaskClient()
    node.send_task(sys.argv[1])
    rclpy.spin(node)


if __name__ == '__main__':
    main()
