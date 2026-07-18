# Warehouse Logistics System — Project Summary

A ROS 2 Humble Python system that simulates a multi-robot warehouse.  Robots are dispatched from a shared task queue, navigate a polar-edge map, manage their own battery, and report live status to a web UI.

---

## Packages

| Package | Type | Purpose |
|---|---|---|
| `logistics_interfaces` | CMake / interface | All shared ROS messages, services, and actions |
| `logistics_robot_sim` | Python | Simulated robot: movement + battery model |
| `logistics_server` | Python | Nav server: Dijkstra pathfinding, battery lockout, task execution |
| `logistics_task_manager` | Python | Dispatcher: shared FIFO queue, dock selection, return-to-charge |
| `logistics_warehouse` | Python | Warehouse state: slot and dock occupancy |
| `logistics_web` | Python | Web bridge: FastAPI + WebSocket live UI |

---

## ROS Interfaces

### Messages

**RobotStatus**
```
string  current_waypoint
string  target_waypoint
bool    is_moving
float32 travel_progress    # 0–1 along current segment
float32 battery_level      # 0–100 %
```

**TaskStatus**
```
int32 PRIORITY_IDLE             = 0   # not moving with intent
int32 PRIORITY_TRAVELLING_EMPTY = 1   # heading to pick something up
int32 PRIORITY_LOADED           = 2   # carrying item to drop-off
int32 PRIORITY_RETURNING_CHARGE = 3   # returning to charger

string robot_id
string task_status        # idle | running | error
string task_type          # pickup | delivery | ''
string task_detail        # human-readable description
int32  queue_size
int32  movement_priority
```

**SlotStatus**
```
string waypoint_id
bool   occupied
```

**WarehouseState**
```
SlotStatus[] slots
```

### Services

**SubmitTask**
```
string task_type    # pickup | delivery
string slot
---
bool   ok
string message
```

**SetSlotOccupancy**
```
string waypoint_id
bool   occupied
---
bool   ok
string message
```

### Actions

**MoveToWaypoint** — single-leg move from nav_server → robot_sim
```
Goal:     string target_waypoint, float32 distance, heading_deg, turn_deg
Result:   bool success, string final_waypoint, message
Feedback: string current_waypoint, float32 distance_remaining
```

**LogisticsTask** — full multi-hop task from task_manager → nav_server
```
Goal:     string destination_waypoint
Result:   bool success, string[] path_taken, string message
Feedback: string[] planned_path, string current_waypoint,
          int32 steps_completed, total_steps, string detail
```

---

## Nodes

### `robot_sim` (namespaced: `robot_1/`, `robot_2/`)
**Package**: logistics_robot_sim

Simulates a mobile robot.

| Parameter | Default | Description |
|---|---|---|
| `travel_speed` | 1.0 | m/s |
| `start_waypoint` | charge_1 | Starting position |
| `initial_battery` | 100.0 | % |
| `discharge_rate_per_meter` | 0.5 | % per meter |
| `charge_rate_per_second` | 10.0 | % per second |
| `charging_waypoints` | charge_1,charge_2 | Comma-separated charger IDs |

Publishes `robot_status` at ~4 Hz during movement and every 0.5 s while idle at a charger.
Provides `move_to_waypoint` action.

**Battery model**: Drains linearly during movement; auto-charges when idle at a charging waypoint.  A `threading.Lock` prevents race conditions between the move thread and the charge timer.

---

### `nav_server` (namespaced: `robot_1/`, `robot_2/`)
**Package**: logistics_server

Plans paths and executes `LogisticsTask` goals.

| Parameter | Default | Description |
|---|---|---|
| `map_file` | required | Path to warehouse YAML |
| `robot_start` | charge_1 | Starting waypoint |

Subscribes to `robot_status` (battery tracking) and `/warehouse_state` (occupancy, available for future path-planning).
Provides `logistics_task` action; uses `move_to_waypoint` action.

**Battery lockout**: If battery < 20 % when a task arrives, the robot first navigates to the nearest charger and waits until battery ≥ 20 % AND has gained ≥ 5 % since docking (whichever condition completes second).

**Pathfinding**: Dijkstra shortest path.  Per-leg headings are computed from waypoint positions (0° = East, 90° = North, CCW positive) and relative turn angles are passed to the robot for simulation.

---

### `task_manager` (global)
**Package**: logistics_task_manager

Dispatches tasks from a shared FIFO queue to the best available robot.

| Parameter | Default | Description |
|---|---|---|
| `robots` | robot_1 | Comma-separated robot IDs |
| `map_file` | — | Warehouse map (Dijkstra for robot selection) |
| `charge_waypoint` | charge_1 | Fallback charger if no map |
| `load_waypoint` | dock_1 | Fallback load dock if no map |
| `unload_waypoint` | dock_1 | Fallback unload dock if no map |

Subscribes to `{robot_id}/robot_status` and `warehouse_state`.
Publishes `{robot_id}/task_status`.
Provides `submit_task` service; uses `set_slot_occupancy` and `{robot_id}/logistics_task`.

**Dispatcher logic**:
1. Pull task from queue.
2. Prefer fully idle robots (not busy, not returning to charge).
3. If none, check robots currently returning to charge — interrupt any with battery ≥ 20 % by signalling their `_interrupt_return` event and waiting for the current navigation hop to finish.
4. Among candidates, pick the robot closest (Dijkstra) to the task's first waypoint.
5. Run the task in a dedicated thread.

**Dock selection**: All `loading_unloading` / `loading` / `unloading` waypoints are auto-discovered from the map as a dock pool.
- Drop-off (pickup task): find closest dock that is **not occupied**; reserve it immediately via `set_slot_occupancy` to prevent two robots racing to the same dock.
- Pick-up (delivery task): find closest dock that **is occupied** (has an item).
- If no suitable dock exists, the robot waits and retries every second.

**Slot check**: Before navigating to a storage slot for a pickup, the robot verifies the slot is occupied.  If it is known to be empty, the task is skipped and the robot returns to idle/charge.

**Return to charge**: Only when the queue is empty.  Navigated hop-by-hop so the dispatcher can abort it (between hops) when a new task arrives.

---

### `warehouse_state` (global)
**Package**: logistics_warehouse

Owns occupancy state for all trackable waypoints.

| Parameter | Default | Description |
|---|---|---|
| `map_file` | required | Path to warehouse YAML |

Tracked types: `storage`, `quick_access`, `loading`, `unloading`, `loading_unloading`.  All start empty on boot.

Provides `set_slot_occupancy` service.
Publishes `warehouse_state` at 1 Hz and immediately after any change.

---

### `logistics_web` (global)
**Package**: logistics_web

Aggregates ROS state and serves the web UI.

| Parameter | Default | Description |
|---|---|---|
| `map_file` | required | Warehouse map |
| `web_host` | 0.0.0.0 | Bind address |
| `web_port` | 8080 | HTTP port |
| `robots` | robot_1,robot_2 | Comma-separated robot IDs |

Subscribes to `{robot_id}/robot_status`, `{robot_id}/task_status`, and `warehouse_state`.
Uses `submit_task` and `set_slot_occupancy` services.

**HTTP endpoints**:
```
GET  /               Web UI (index.html)
GET  /map            Warehouse map JSON with computed SVG positions
POST /task/pickup    { slot }  →  submit pickup task
POST /task/delivery  { slot }  →  submit delivery task
POST /slot/set       { waypoint_id, occupied }  →  manual occupancy toggle
WS   /ws             Live state stream (10 Hz, change-triggered)
```

**WebSocket payload**:
```json
{
  "robots": {
    "<robot_id>": {
      "current_waypoint": "string",
      "target_waypoint":  "string",
      "is_moving":        bool,
      "travel_progress":  float,
      "battery_level":    float,
      "task_status":      "idle|running|error",
      "task_type":        "pickup|delivery|''",
      "task_detail":      "string",
      "movement_priority": int
    }
  },
  "queue_size": int,
  "slots": { "<waypoint_id>": bool }
}
```

---

## Launch File

**`logistics_server/launch/logistics.launch.py`**

Launch arguments: `map_file`, `travel_speed` (default 1.0 m/s), `web_port` (default 8080).

Starts 7 nodes in order:
1. `robot_sim` — namespace `robot_1`, starts at `charge_1`
2. `nav_server` — namespace `robot_1`
3. `robot_sim` — namespace `robot_2`, starts at `charge_2`
4. `nav_server` — namespace `robot_2`
5. `warehouse_state` — global
6. `task_manager` — global, manages both robots
7. `logistics_web` — global

---

## Warehouse Map

**`logistics_server/config/warehouse_map.yaml`**

```
shelf_A2─shelf_A1─┐                      ┌─shelf_E1─shelf_E2
shelf_B2─shelf_B1─┤                      ├─shelf_F1─shelf_F2
shelf_C2─shelf_C1─┤                      ├─shelf_G1─shelf_G2
shelf_D2─shelf_D1─┤                      ├─shelf_H1─shelf_H2
                charge_1 ──────────── charge_2
                   │                      │
                inter_L1             inter_R1
                   │                      │
                inter_L2 ──────────── inter_R2
                   │                      │
                inter_L3 ──────────── inter_R3
                   │                      │
                inter_L4 ──────────── inter_R4
                   │                      │
                inter_L5             inter_R5
                   │                      │
                quick_1               quick_2
                   │                      │
                dock_1  ────────────  dock_2
```

**Waypoint counts**: 2 charging, 16 storage, 2 quick-access, 2 loading/unloading, 10 intersections = **32 total**
**Edges**: 35
**Map format**: Polar edges (bearing in compass degrees, distance in metres); positions computed via BFS from `charge_1` as origin.

---

## Web UI

Single-page SVG map plus a right-hand control panel.

- **Map**: Renders all waypoints and edges; robot dots animate along edges using `travel_progress`.  Trackable waypoints (storage, docks) are clickable to toggle occupancy — occupied nodes show a white centre dot and white outline ring.
- **Fleet Status card**: One card per robot showing location, task detail, colour-coded movement priority badge, and battery bar.
- **Task Control**: Drop-down of all storage and quick-access slots; Pick Up / Put Box buttons submit tasks to the queue.
- **Legend**: Waypoint type colours + occupied indicator explanation.
- **Connection indicator**: Green dot in header; shows "Disconnected" with auto-reconnect on WebSocket drop.

---

## Key Design Decisions

| Concern | Decision |
|---|---|
| Battery lockout ownership | Nav server (robot concern, not UI concern) |
| Task queue ownership | Dedicated `task_manager` node (not web node) |
| Occupancy ownership | Dedicated `warehouse_state` node (authoritative source of truth) |
| Multi-robot namespacing | ROS namespaces (`robot_1/`, `robot_2/`) — no code changes to robot nodes |
| Dock reservation | Immediate `set_slot_occupancy` call before navigating prevents two robots racing |
| Return-to-charge interruption | Hop-by-hop navigation + `threading.Event`; dispatcher waits for current hop to finish |
| Skip empty pickups | Occupancy check before dispatch avoids wasted trips |
| Wait vs. abort on no dock | Wait and retry — operator or other robot will eventually free a dock |
| Web UI role | Pure relay — subscribes to topics, forwards service calls; owns no state |
