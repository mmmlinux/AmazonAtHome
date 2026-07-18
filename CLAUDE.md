# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

This is a ROS 2 (Humble) Python workspace built with colcon. The workspace root is the repo root.

```bash
# Build all packages
colcon build

# Source environment (required after build and in any new terminal)
source install/setup.bash

# Launch the full system
ros2 launch logistics_server logistics

# Launch with custom parameters
ros2 launch logistics_server logistics map_file:=/path/to/map.yaml travel_speed:=2.0 web_port:=9000
```

Web UI is available at `http://localhost:8080` after launch.

## Debugging

```bash
# Monitor robot positions/battery
ros2 topic echo /robot_1/robot_status

# Monitor task assignment
ros2 topic echo /robot_1/task_status

# Monitor slot occupancy
ros2 topic echo /warehouse_state

# Manually submit a task
ros2 service call /submit_task logistics_interfaces/srv/SubmitTask "{task_type: 'pickup', slot: 'shelf_A1'}"

# Manually toggle a slot
ros2 service call /set_slot_occupancy logistics_interfaces/srv/SetSlotOccupancy "{waypoint_id: 'shelf_A1', occupied: true}"
```

There are no automated tests. The `logistics_server/logistics_server/task_client.py` is a manual test utility for submitting tasks from the CLI.

## Architecture

Six ROS 2 packages, each with a single primary node:

| Package | Node | Namespace | Role |
|---|---|---|---|
| `logistics_interfaces` | — | — | Shared msg/srv/action definitions only |
| `logistics_robot_sim` | `robot_sim` | `robot_1/`, `robot_2/` | Simulates physical robot movement and battery |
| `logistics_robot_sim` | `hector_driver` | `hector_1/`, etc. | Bridges nav_server to the physical Hector robot over WebSocket |
| `logistics_server` | `nav_server` | `robot_1/`, `robot_2/` | Dijkstra pathfinding, task execution, battery lockout |
| `logistics_task_manager` | `task_manager` | global | FIFO queue, robot assignment, dock selection |
| `logistics_warehouse` | `warehouse_state` | global | Single source of truth for slot/dock occupancy |
| `logistics_web` | `web_node` | global | FastAPI + WebSocket UI bridge; pure relay, no state |

Multi-robot scaling uses ROS namespaces (`robot_1/`, `robot_2/`): `robot_sim`/`hector_driver` and `nav_server` are launched once per robot with no code changes.

### Key Data Flows

**Task execution**: Web UI → POST `/task/pickup` → `logistics_web` calls `submit_task` service → `task_manager` FIFO queue → action goal on `{robot_id}/logistics_task` → `nav_server` (Dijkstra path) → action goals on `move_to_waypoint` → `robot_sim` or `hector_driver` (leg-by-leg movement).

**Occupancy**: `warehouse_state` is the authority. `task_manager` calls `set_slot_occupancy` **atomically before navigating** to a dock (prevents race conditions). Web UI can also toggle occupancy directly.

**Robot position animation**: `robot_sim`/`hector_driver` publishes `robot_status` at 4 Hz with `travel_progress` (0.0–1.0 along current edge). The web UI interpolates the SVG robot dot position using this value.

### Critical Design Details

**Battery lockout** lives in `nav_server`, not `task_manager`. When battery < 20% at task start, nav_server navigates to the nearest charger and holds until battery ≥ 20% AND has gained ≥ 5%, publishing feedback throughout.

**Return-to-charge is interruptible**: When the task queue empties, robots navigate home hop-by-hop. `task_manager` sets a `threading.Event` (`_interrupt_return`) between hops if a new task arrives and battery ≥ 20%. The return thread checks this event after each hop.

**Smart dock selection**: `task_manager` auto-discovers docks from the map (waypoint types `loading`/`unloading`/`loading_unloading`). Pickup finds the nearest **occupied** dock; delivery finds the nearest **unoccupied** dock. Waits and retries every second if none available.

**Movement priority constants** (on `TaskStatus` message): `PRIORITY_IDLE=0`, `PRIORITY_TRAVELLING_EMPTY=1`, `PRIORITY_LOADED=2`, `PRIORITY_RETURNING_CHARGE=3`. These drive color-coding in the web UI.

### Warehouse Map

`logistics_server/config/warehouse_map.yaml` — polar-edge YAML: waypoints with `(type, description)` and edges with `(bearing_degrees, distance_meters)`. `nav_server` and `task_manager` both load this file. SVG positions are computed via BFS from origin (`charge_1` at SVG `(320, 60)`, scale 25px/m, North = −Y).

Waypoint types tracked for occupancy: `storage`, `quick_access`, `loading`, `unloading`, `loading_unloading`.

### Thread Safety Pattern

All nodes use `MultiThreadedExecutor`. Shared mutable state is guarded by `threading.Lock`. `robot_sim` and `nav_server` use `ReentrantCallbackGroup` so action callbacks and timer callbacks can fire concurrently. `task_manager`'s shared task queue uses the thread-safe `queue.Queue`.

## Hector Robot Integration

To run a physical Hector robot alongside (or instead of) simulated robots, set `robot_type: hector` in `robots.yaml`.  The launch file starts a `hector_driver` node, which acts as a **WebSocket server** that Hector connects to over WiFi.

### robots.yaml entry

```yaml
- id:                 hector_1
  robot_type:         hector
  start_waypoint:     charge_1
  home_charger:       charge_1
  charging_waypoints: charge_1
  min_battery:        50.0
  critical_battery:   25.0
  ws_port:            8765      # each Hector robot needs a unique port
```

### Starting hector_agent on the Pi Zero W

```bash
# Install dependency (once)
pip install websockets

# Run — replace IP with the WarehouseManager machine's address
python hector_agent.py --ws ws://192.168.1.100:8765 --port /dev/ttyS0
```

The agent reconnects automatically if the WiFi link drops.

### WebSocket protocol (hector_driver ↔ hector_agent)

All messages are UTF-8 JSON. The WM node is the server; Hector is the client.

**WM → Hector**

| `type` | Fields | Meaning |
|---|---|---|
| `move` | `id`, `target_waypoint`, `distance_m`, `heading_deg`, `turn_deg` | Execute one navigation hop |
| `abort` | — | Stop immediately (e-stop) |

**Hector → WM**

| `type` | Fields | Meaning |
|---|---|---|
| `status` | `battery_pct`, `progress`, `moving`, `left_enc`, `right_enc`, `lift_up`, `payload_present` | Periodic telemetry at 4 Hz |
| `move_done` | `id`, `success`, `message` | Hop complete or failed |
| `fault` | `code`, `data` | Boskov fault forwarded to ROS log |

### Motion model

Each hop is two phases: **rotate** (`ROTATE` command) then **drive** (`MOVE` with `duration_ms`). Both are timed dead-reckoning — Boskov ACKs the command immediately and runs the motion autonomously; `hector_agent` sleeps for the computed duration while tracking progress.

Calibration constants at the top of `hector_agent.py`:

| Constant | Default | Meaning |
|---|---|---|
| `DRIVE_SPEED_PCT` | 50 | Motor power % for straight driving |
| `TURN_SPEED_PCT` | 40 | Motor power % for rotation |
| `DRIVE_SPEED_M_S` | 0.4 | Actual forward speed at `DRIVE_SPEED_PCT` (tune first) |
| `TURN_RATE_DEG_S` | 90.0 | Must match Boskov's `TURN_RATE_AT_100_DEG_PER_SEC` |
