"""
Web bridge node — pure UI / status relay, no robot logic.

Subscribes to every robot's robot_status and task_status topics and
merges them into a single JSON object pushed over WebSocket at 10 Hz.

WebSocket payload shape
-----------------------
{
  "robots": {
    "<robot_id>": {
      "current_waypoint": str,
      "target_waypoint":  str,
      "is_moving":        bool,
      "travel_progress":  float,
      "battery_level":    float,
      "task_status":      str,   // idle | running | error
      "task_type":        str,
      "task_detail":      str
    },
    ...
  },
  "queue_size": int
}

Endpoints
---------
  GET  /               web UI
  GET  /map            warehouse map JSON
  POST /task/pickup    {slot} → forwarded to task_manager via SubmitTask service
  POST /task/delivery  {slot} → forwarded to task_manager via SubmitTask service
  WS   /ws             live state stream

Parameters
----------
  map_file   path to warehouse YAML
  web_host   bind address (default 0.0.0.0)
  web_port   HTTP port    (default 8080)
  robots     comma-separated robot IDs matching the ROS namespaces in use
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

from logistics_interfaces.msg import RobotStatus, TaskStatus, WarehouseState
from logistics_interfaces.srv import SetSlotOccupancy, SubmitTask


# ── Shared state ──────────────────────────────────────────────────────────────

@dataclass
class RobotState:
    current_waypoint:  str   = ''
    target_waypoint:   str   = ''
    is_moving:         bool  = False
    travel_progress:   float = 0.0
    battery_level:     float = 100.0
    task_status:       str   = 'idle'
    task_type:         str   = ''
    task_detail:       str   = 'Initialising...'
    movement_priority: int   = 0


class _State:
    def __init__(self):
        self.robots:      dict[str, RobotState] = {}
        self.queue_size:  int                   = 0
        self.queue_items: list[str]             = []
        self.slots:       dict[str, bool]       = {}
        self.lock = threading.Lock()

    def snapshot(self) -> str:
        with self.lock:
            return json.dumps({
                'robots':      {rid: asdict(s) for rid, s in self.robots.items()},
                'queue_size':  self.queue_size,
                'queue_items': list(self.queue_items),
                'slots':       dict(self.slots),
            })


_state = _State()


# ── Map helper ────────────────────────────────────────────────────────────────

def _parse_positions(data: dict) -> dict:
    display   = data.get('display', {})
    origin_x  = float(display.get('origin_x', 300))
    origin_y  = float(display.get('origin_y', 50))
    scale     = float(display.get('scale', 30))
    origin_id = data['origin']

    adjacency: dict = {}
    for edge in data['edges']:
        a, b   = edge['from'], edge['to']
        bear   = float(edge['bearing'])
        dist   = float(edge['distance'])
        adjacency.setdefault(a, []).append((b,  bear,             dist))
        adjacency.setdefault(b, []).append((a, (bear + 180) % 360, dist))

    svg_pos = {origin_id: (origin_x, origin_y)}
    queue = [origin_id]
    while queue:
        cur = queue.pop(0)
        cx, cy = svg_pos[cur]
        for nb, bear, dist in adjacency.get(cur, []):
            if nb not in svg_pos:
                rad = math.radians(bear)
                svg_pos[nb] = (
                    cx + dist * scale * math.sin(rad),
                    cy - dist * scale * math.cos(rad),
                )
                queue.append(nb)

    waypoints = {}
    for wp_id, wp_data in data['waypoints'].items():
        info = dict(wp_data)
        if wp_id in svg_pos:
            info['x'], info['y'] = svg_pos[wp_id]
        waypoints[wp_id] = info
    return waypoints


# ── FastAPI app ───────────────────────────────────────────────────────────────

app = FastAPI()
_map_data:      dict            = {}
_raw_map_data:  dict            = {}
_map_file_path: str             = ''
_static_dir:    Path            = Path()
_web_node:      'WebNode | None' = None


@app.get('/')
async def index():
    return FileResponse(str(_static_dir / 'index.html'))


@app.get('/map-editor')
async def map_editor():
    return FileResponse(str(_static_dir / 'map_editor.html'))


@app.get('/map-view')
async def map_view():
    return FileResponse(str(_static_dir / 'map_view.html'))


@app.get('/map')
def get_map():
    return JSONResponse(_map_data)


class MapSaveRequest(BaseModel):
    yaml_content: str


# TODO: Add authentication/authorization to all mutating endpoints.
# Currently any client on the network can submit tasks, toggle slot occupancy,
# and overwrite the warehouse map file.  At minimum, require a shared secret
# API key in the Authorization header for POST /task/*, POST /slot/set, and
# POST /map/save.  For production, consider JWT or session-based auth.
@app.post('/map/save')
def post_map_save(req: MapSaveRequest):
    global _map_data, _raw_map_data
    if not _map_file_path:
        return JSONResponse({'ok': False, 'error': 'Map file path not configured'})
    try:
        data = yaml.safe_load(req.yaml_content)
        if not data or 'waypoints' not in data or 'edges' not in data:
            return JSONResponse({'ok': False, 'error': 'Invalid map YAML: missing waypoints or edges'})
        with open(_map_file_path, 'w') as f:
            f.write(req.yaml_content)
        _raw_map_data = data
        enriched = dict(data)
        enriched['waypoints'] = _parse_positions(data)
        _map_data = enriched
        return JSONResponse({'ok': True, 'message': 'Map saved and reloaded'})
    except Exception as e:
        return JSONResponse({'ok': False, 'error': str(e)})


@app.post('/map/parse')
def post_map_parse(req: MapSaveRequest):
    try:
        data = yaml.safe_load(req.yaml_content)
        if not data or 'waypoints' not in data:
            return JSONResponse({'ok': False, 'error': 'Invalid map YAML structure'})
        enriched = dict(data)
        enriched['waypoints'] = _parse_positions(data)
        return JSONResponse({'ok': True, 'data': enriched})
    except Exception as e:
        return JSONResponse({'ok': False, 'error': str(e)})


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


class SlotToggleRequest(BaseModel):
    waypoint_id: str
    occupied: bool


@app.post('/slot/set')
def post_slot_set(req: SlotToggleRequest):
    if _web_node is None:
        return {'ok': False, 'error': 'Node not ready'}
    return _web_node.set_slot_occupancy(req.waypoint_id, req.occupied)


# TODO: Add a manual/override control mode for operators:
#   POST /robot/{id}/estop   — send emergency stop to a specific robot
#   POST /robot/{id}/resume  — clear estop and allow autonomous operation
#   POST /robot/{id}/goto    — manually command a robot to a waypoint
# This is essential for recovering from stuck or misbehaving robots without
# restarting the system or physically intervening.
@app.websocket('/ws')
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    last_sent = ''
    try:
        while True:
            snap = _state.snapshot()
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
        self.declare_parameter('robots',    'robot_1,robot_2')

        map_file   = self.get_parameter('map_file').get_parameter_value().string_value
        host       = self.get_parameter('web_host').get_parameter_value().string_value
        port       = self.get_parameter('web_port').get_parameter_value().integer_value
        robots_str = self.get_parameter('robots').get_parameter_value().string_value

        robot_ids = [r.strip() for r in robots_str.split(',') if r.strip()]

        global _map_data, _static_dir, _web_node
        _map_data   = self._load_map(map_file)
        _static_dir = Path(get_package_share_directory('logistics_web')) / 'static'
        _web_node   = self

        # Pre-populate state for every robot so the WebSocket payload is
        # consistent from the first frame
        with _state.lock:
            for rid in robot_ids:
                _state.robots[rid] = RobotState()

        cb = ReentrantCallbackGroup()

        for rid in robot_ids:
            self.create_subscription(
                RobotStatus, f'{rid}/robot_status',
                lambda msg, r=rid: self._on_robot_status(msg, r),
                10, callback_group=cb,
            )
            self.create_subscription(
                TaskStatus, f'{rid}/task_status',
                lambda msg, r=rid: self._on_task_status(msg, r),
                10, callback_group=cb,
            )

        self._submit_client = self.create_client(
            SubmitTask, 'submit_task', callback_group=cb,
        )
        self._occupancy_client = self.create_client(
            SetSlotOccupancy, 'set_slot_occupancy', callback_group=cb,
        )

        self.create_subscription(
            WarehouseState, 'warehouse_state', self._on_warehouse_state, 10,
            callback_group=cb,
        )

        threading.Thread(target=self._run_web, args=(host, port), daemon=True).start()
        self.get_logger().info(f'Web UI → http://{host}:{port}  robots={robot_ids}')

    # ── Map ───────────────────────────────────────────────────────────────────

    def _load_map(self, map_file: str) -> dict:
        global _raw_map_data, _map_file_path
        if not map_file:
            raise RuntimeError('map_file parameter is required')
        with open(map_file) as f:
            data = yaml.safe_load(f)
        _raw_map_data = data
        _map_file_path = map_file
        enriched = dict(data)
        enriched['waypoints'] = _parse_positions(data)
        return enriched

    # ── Topic callbacks ───────────────────────────────────────────────────────

    def _on_robot_status(self, msg: RobotStatus, robot_id: str) -> None:
        with _state.lock:
            s = _state.robots.get(robot_id)
            if s is None:
                return
            s.current_waypoint = msg.current_waypoint
            s.target_waypoint  = msg.target_waypoint
            s.is_moving        = msg.is_moving
            s.travel_progress  = float(msg.travel_progress)
            s.battery_level    = float(msg.battery_level)

    def _on_task_status(self, msg: TaskStatus, robot_id: str) -> None:
        with _state.lock:
            s = _state.robots.get(robot_id)
            if s is None:
                return
            s.task_status       = msg.task_status
            s.task_type         = msg.task_type
            s.task_detail       = msg.task_detail
            s.movement_priority = msg.movement_priority
            _state.queue_size   = msg.queue_size
            _state.queue_items  = list(msg.queue_items)

    def _on_warehouse_state(self, msg: WarehouseState) -> None:
        with _state.lock:
            _state.slots = {s.waypoint_id: s.occupied for s in msg.slots}

    # ── Service: SubmitTask ───────────────────────────────────────────────────

    def submit_task(self, task_type: str, slot: str) -> dict:
        if not self._submit_client.wait_for_service(timeout_sec=5.0):
            return {'ok': False, 'error': 'Task manager not available'}

        done          = threading.Event()
        result_holder: list = [None]

        def on_response(future):
            result_holder[0] = future.result()
            done.set()

        req           = SubmitTask.Request()
        req.task_type = task_type
        req.slot      = slot
        self._submit_client.call_async(req).add_done_callback(on_response)

        if not done.wait(timeout=10.0):
            return {'ok': False, 'error': 'Timeout contacting task manager'}

        resp = result_holder[0]
        return {'ok': resp.ok, 'message': resp.message}

    def set_slot_occupancy(self, waypoint_id: str, occupied: bool) -> dict:
        if not self._occupancy_client.wait_for_service(timeout_sec=5.0):
            return {'ok': False, 'error': 'Warehouse state node not available'}

        done          = threading.Event()
        result_holder: list = [None]

        def on_response(future):
            result_holder[0] = future.result()
            done.set()

        req             = SetSlotOccupancy.Request()
        req.waypoint_id = waypoint_id
        req.occupied    = occupied
        self._occupancy_client.call_async(req).add_done_callback(on_response)

        if not done.wait(timeout=10.0):
            return {'ok': False, 'error': 'Timeout'}

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
