"""
Warehouse State Node — owns slot and dock occupancy.

Loads all trackable waypoints from the map on startup (storage, quick_access,
loading, unloading, loading_unloading) and initialises them as empty.

Topics published
-----------------
  warehouse_state   logistics_interfaces/msg/WarehouseState
    Published at 1 Hz and immediately after any state change so subscribers
    always have fresh data.

Services advertised
-------------------
  set_slot_occupancy   logistics_interfaces/srv/SetSlotOccupancy

Parameters
----------
  map_file   path to warehouse YAML map
"""

import threading
import yaml

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from logistics_interfaces.msg import SlotStatus, WarehouseState
from logistics_interfaces.srv import SetSlotOccupancy


TRACKABLE_TYPES = frozenset({
    'storage', 'quick_access', 'loading', 'unloading', 'loading_unloading',
})


class WarehouseStateNode(Node):

    def __init__(self):
        super().__init__('warehouse_state')

        self.declare_parameter('map_file', '')
        map_file = self.get_parameter('map_file').get_parameter_value().string_value

        self._lock = threading.Lock()
        self._occupancy: dict[str, bool] = {}

        if map_file:
            self._load_map(map_file)
        else:
            self.get_logger().warning('No map_file — no slots tracked')

        cb = ReentrantCallbackGroup()

        self._state_pub = self.create_publisher(WarehouseState, 'warehouse_state', 10)

        self._set_srv = self.create_service(
            SetSlotOccupancy, 'set_slot_occupancy', self._on_set, callback_group=cb,
        )

        # Periodic publish so newly-connected subscribers get state within 1 s
        self.create_timer(1.0, self._publish, callback_group=cb)

        self.get_logger().info(
            f'Warehouse state ready — tracking {len(self._occupancy)} slots'
        )

    # ── Map loading ───────────────────────────────────────────────────────────

    def _load_map(self, map_file: str) -> None:
        with open(map_file) as f:
            data = yaml.safe_load(f)
        for wp_id, wp_data in data['waypoints'].items():
            if wp_data.get('type') in TRACKABLE_TYPES:
                self._occupancy[wp_id] = False
        self.get_logger().info(
            f'Map loaded: {len(self._occupancy)} trackable waypoints'
        )

    # ── Publisher ─────────────────────────────────────────────────────────────

    def _publish(self) -> None:
        with self._lock:
            msg = WarehouseState()
            msg.slots = [
                SlotStatus(waypoint_id=wp_id, occupied=occ)
                for wp_id, occ in self._occupancy.items()
            ]
        self._state_pub.publish(msg)

    # ── Service: SetSlotOccupancy ─────────────────────────────────────────────

    def _on_set(
        self,
        req: SetSlotOccupancy.Request,
        resp: SetSlotOccupancy.Response,
    ) -> SetSlotOccupancy.Response:
        with self._lock:
            if req.waypoint_id not in self._occupancy:
                resp.ok      = False
                resp.message = f'Unknown waypoint: {req.waypoint_id}'
                return resp
            self._occupancy[req.waypoint_id] = req.occupied

        self.get_logger().info(
            f'Slot {req.waypoint_id} → {"occupied" if req.occupied else "empty"}'
        )
        self._publish()
        resp.ok      = True
        resp.message = 'ok'
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseStateNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
