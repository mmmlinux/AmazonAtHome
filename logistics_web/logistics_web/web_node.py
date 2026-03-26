"""
Web bridge node for the warehouse logistics system.

This node is purely a UI/status bridge — it contains no robot logic.

Endpoints:
  GET  /               → web UI (index.html)
  GET  /map            → warehouse map JSON (waypoints + edges)
  POST /task/pickup    {"slot": "<waypoint>"}  → forwarded to task_manager
  POST /task/delivery  {"slot": "<waypoint>"}  → forwarded to task_manager
  WS   /ws             → real-time RobotState JSON (polled at 10 Hz per client)

Data sources:
  robot_status  topic  → physical robot position, battery, motion
  task_status   topic  → task queue state from task_manager
"""

import asyncio
import json
import math
import threading
from dataclasses import asdict, dataclass
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import yaml

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel

from ament_index_python.packages import get_package_share_directory

from logistics_interfaces.msg import RobotStatus, TaskStatus
from logistics_interfaces.srv import SubmitTask


# ── Shared state ──────────────────────────────────────────────────────────────
# Written by ROS callbacks, read by WebSocket poll loops.

@dataclass
class RobotState:
    # Physical robot state (from robot_status topic)
    current_waypoint: str = 'charge_1'
    target_waypoint: str = ''
    is_moving: bool = False
    travel_progress: float = 0.0
    battery_level: float = 100.0
    # Task state (from task_status topic)
    task_status: str = 'idle'      # idle | running | error
    task_type: str = ''
    task_detail: str = 'Idle at charging station'
    queue_size: int = 0


_state = RobotState()
_state_lock = threading.Lock()


def _update_state(**kwargs) -> None:
    with _state_lock:
        for k, v in kwargs.items():
            setattr(_state, k, v)


def _snapshot() -> str:
    with _state_lock:
        return json.dumps(asdict(_state))


# ── Map position calculator ──────────────────────────────────────────────────

def _parse_positions(data: dict) -> dict:
    """
    Compute SVG x/y for every waypoint via BFS from the origin,
    using each edge's compass bearing and distance.
    """
    display   = data.get('display', {})
    origin_x  = float(display.get('origin_x', 300))
    origin_y  = float(display.get('origin_y', 50))
    scale     = float(display.get('scale', 30))
    origin_id = data['origin']

    adjacency: dict[str, list] = {}
    for edge in data['edges']:
        a, b   = edge['from'], edge['to']
        bear   = float(edge['bearing'])
        dist   = float(edge['distance'])
        adjacency.setdefault(a, []).append((b,  bear,             dist))
        adjacency.setdefault(b, []).append((a, (bear + 180) % 360, dist))

    svg_pos: dict[str, tuple[float, float]] = {origin_id: (origin_x, origin_y)}
    queue = [origin_id]
    while queue:
        current = queue.pop(0)
        cx, cy = svg_pos[current]
        for neighbour, bearing, distance in adjacency.get(current, []):
            if neighbour not in svg_pos:
                rad = math.radians(bearing)
                svg_pos[neighbour] = (
                    cx + distance * scale * math.sin(rad),
                    cy - distance * scale * math.cos(rad),
                )
                queue.append(neighbour)

    waypoints = {}
    for wp_id, wp_data in data['waypoints'].items():
        info = dict(wp_data)
        if wp_id in svg_pos:
            info['x'], info['y'] = svg_pos[wp_id]
        waypoints[wp_id] = info
    return waypoints


# ── FastAPI app ───────────────────────────────────────────────────────────────

app = FastAPI()
_map_data: dict = {}
_static_dir: Path = Path()
_web_node: 'WebNode | None' = None


@app.get('/')
async def index():
    return FileResponse(str(_static_dir / 'index.html'))


@app.get('/map')
def get_map():
    return JSONResponse(_map_data)


class TaskRequest(BaseModel):
    slot: str


@app.post('/task/pickup')
def post_pickup(req: TaskRequest):
    if _web_node is None:
        return {'ok': False, 'error': 'Node not ready'}
    return _web_node.submit_task('pickup', req.slot)


@app.post('/task/delivery')
def post_delivery(req: TaskRequest):
    if _web_node is None:
        return {'ok': False, 'error': 'Node not ready'}
    return _web_node.submit_task('delivery', req.slot)


@app.websocket('/ws')
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    last_sent: str = ''
    try:
        while True:
            snap = _snapshot()
            if snap != last_sent:
                await ws.send_text(snap)
                last_sent = snap
            await asyncio.sleep(0.1)
    except (WebSocketDisconnect, Exception):
        pass


# ── ROS node ──────────────────────────────────────────────────────────────────

class WebNode(Node):
    def __init__(self):
        super().__init__('logistics_web')

        self.declare_parameter('map_file',  '')
        self.declare_parameter('web_host',  '0.0.0.0')
        self.declare_parameter('web_port',  8080)

        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        host     = self.get_parameter('web_host').get_parameter_value().string_value
        port     = self.get_parameter('web_port').get_parameter_value().integer_value

        global _map_data, _static_dir, _web_node
        _map_data   = self._load_map(map_file)
        _static_dir = Path(get_package_share_directory('logistics_web')) / 'static'
        _web_node   = self

        cb = ReentrantCallbackGroup()

        self._robot_status_sub = self.create_subscription(
            RobotStatus, 'robot_status', self._on_robot_status, 10,
            callback_group=cb,
        )
        self._task_status_sub = self.create_subscription(
            TaskStatus, 'task_status', self._on_task_status, 10,
            callback_group=cb,
        )
        self._submit_client = self.create_client(
            SubmitTask, 'submit_task', callback_group=cb,
        )

        threading.Thread(target=self._run_web, args=(host, port), daemon=True).start()

        self.get_logger().info(f'Web UI → http://{host}:{port}')

    # ── Map ───────────────────────────────────────────────────────────────────

    def _load_map(self, map_file: str) -> dict:
        if not map_file:
            raise RuntimeError('map_file parameter is required')
        with open(map_file) as f:
            data = yaml.safe_load(f)
        data['waypoints'] = _parse_positions(data)
        return data

    # ── ROS topic callbacks ───────────────────────────────────────────────────

    def _on_robot_status(self, msg: RobotStatus) -> None:
        _update_state(
            current_waypoint=msg.current_waypoint,
            target_waypoint=msg.target_waypoint,
            is_moving=msg.is_moving,
            travel_progress=float(msg.travel_progress),
            battery_level=float(msg.battery_level),
        )

    def _on_task_status(self, msg: TaskStatus) -> None:
        _update_state(
            task_status=msg.task_status,
            task_type=msg.task_type,
            task_detail=msg.task_detail,
            queue_size=msg.queue_size,
        )

    # ── Service call: SubmitTask ──────────────────────────────────────────────

    def submit_task(self, task_type: str, slot: str) -> dict:
        if not self._submit_client.wait_for_service(timeout_sec=5.0):
            return {'ok': False, 'error': 'Task manager not available'}

        done = threading.Event()
        result_holder: list = [None]

        def on_response(future):
            result_holder[0] = future.result()
            done.set()

        req = SubmitTask.Request()
        req.task_type = task_type
        req.slot      = slot
        self._submit_client.call_async(req).add_done_callback(on_response)

        if not done.wait(timeout=10.0):
            return {'ok': False, 'error': 'Timeout contacting task manager'}

        resp = result_holder[0]
        return {'ok': resp.ok, 'message': resp.message}

    # ── Web server ────────────────────────────────────────────────────────────

    def _run_web(self, host: str, port: int) -> None:
        config = uvicorn.Config(app, host=host, port=port, log_level='warning')
        uvicorn.Server(config).run()


def main(args=None):
    rclpy.init(args=args)
    node = WebNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
