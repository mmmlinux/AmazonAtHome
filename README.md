# Amazon at Home — Warehouse Logistics Simulator

100% vibe coded slop guaranteed.

A ROS 2 (Humble) simulation of a multi-robot warehouse logistics system. Two simulated robots pick up and deliver boxes across a warehouse, navigating via Dijkstra pathfinding, managing battery, and coordinating through a shared task queue. A live web UI shows robot positions, battery levels, slot occupancy, and the task queue.

## Quick Start (Docker)

```bash
docker compose up
```

Open `http://localhost:8080` in your browser.

To pass launch arguments:

```bash
docker compose run --rm warehouse travel_speed:=3.0 min_battery_1:=40.0
```

`./start.sh` is a thin convenience wrapper around `docker compose up` — it just
`mkdir -p logs` first and passes through any arguments (e.g. `./start.sh --build`).
It runs in the foreground (`Ctrl+C` stops it) and brings up every service defined
in `docker-compose.yml` (`warehouse` and `git-daemon`), not just `warehouse`.

## Quick Start (Native ROS 2 Humble)

```bash
# Build
colcon build
source install/setup.bash

# Launch
ros2 launch logistics_server logistics.launch.py

# Launch with custom parameters
ros2 launch logistics_server logistics.launch.py travel_speed:=2.0 web_port:=9000
```

## Launch Parameters

| Parameter | Default | Description |
|---|---|---|
| `travel_speed` | `4.0` | Robot movement speed (m/s) |
| `web_port` | `8080` | Web UI port |
| `map_file` | built-in map | Path to custom warehouse map YAML |
| `min_battery_1` / `min_battery_2` | `50.0` | Low-battery threshold per robot (%) — triggers charge diversion when empty |
| `critical_battery_1` / `critical_battery_2` | `25.0` | Critical threshold per robot (%) — triggers emergency drop when loaded |

## Battery Behaviour

- **Empty robot below `min_battery`**: navigates to the nearest charger, charges to 80%, then re-queues its task at the front of the queue for another robot to pick up.
- **Loaded robot below `critical_battery`**: drops the box at the nearest empty storage slot, queues a pickup for that slot at the front of the queue, then navigates to a charger and charges to 80%.
- **Loaded robot between `critical_battery` and `min_battery`**: continues delivery normally.
- **Idle robots**: automatically return to their home charger when the queue is empty. The return trip is interrupted if a new task arrives and battery ≥ `min_battery`.

## Pathfinding

Navigation is handled by `nav_server` (`logistics_server` package) using **Dijkstra's algorithm** on an undirected weighted graph, where edge weights are physical distances in metres.

### Map Format

The warehouse is defined in `logistics_server/config/warehouse_map.yaml` using a **polar-edge** format: edges are described by compass bearing and distance rather than absolute coordinates. Waypoint positions for the web UI are derived at startup by BFS from the origin node, converting each edge's bearing + distance into SVG `(x, y)` coordinates (North = −Y, East = +X, scale 20 px/m).

```yaml
waypoints:
  spine_W1:
    type: intersection
    description: "West corridor junction — aisle A"

edges:
  - from: charge_2
    to:   spine_W1
    bearing: 180    # South
    distance: 4.0   # metres
```

Waypoint types: `charging`, `intersection`, `storage`, `quick_access`, `loading`, `unloading`, `loading_unloading`.

### How a Journey Works

1. `task_manager` sends an action goal to `nav_server` with a `destination_waypoint`.
2. `nav_server` acquires a waypoint lock on its current position from the traffic controller, then enters its hop-by-hop loop.
3. Before each hop, `nav_server` reruns Dijkstra from the current position, adding a **+50 m soft penalty** to any waypoint currently occupied by a peer robot. This causes the planned route to naturally route around peers without hard-blocking those waypoints.
4. `nav_server` requests permission for the next waypoint from the traffic controller (`traffic/acquire`). If denied, it waits and retries. If a yield signal arrives during the wait, it executes a one-hop retreat instead.
5. Once the waypoint is granted, `nav_server` sends a `MoveToWaypoint` action goal to `robot_sim`. Each leg carries the edge distance, absolute heading, and relative turn angle.
6. `robot_sim` moves the robot at `travel_speed` m/s, publishing `travel_progress` (0.0–1.0) at 4 Hz so the web UI can smoothly interpolate the robot dot between waypoints.
7. After the hop completes, `nav_server` releases the previous waypoint and advances its position. Battery is checked after every hop.

### Nearest-Charger and Nearest-Storage Selection

When a robot needs to divert to charge or find an emergency drop spot, it runs Dijkstra from its current position to every candidate waypoint and picks the one with the shortest total path distance — not straight-line distance. This means it correctly accounts for the warehouse aisle layout rather than cutting through walls.

For charger selection, waypoints currently occupied by peer robots are excluded so two robots never attempt to use the same charging station. If all chargers are occupied (unlikely), the robot falls back to the nearest one.

## Collision Avoidance

Collision avoidance is handled by a centralised **Traffic Controller** node (`traffic_controller`, in the `logistics_server` package). Every `nav_server` must acquire a lock from the TC before stepping onto a waypoint, and must release it after leaving.

### Waypoint and aisle locking

- **Waypoint lock**: at most one robot may occupy a given waypoint at a time.
- **Aisle lock**: at most one robot may be inside an aisle (or QA area) at a time. The TC detects aisle membership from waypoint names (`aisle_X`, `shelf_X`, `qa_W/E`) and junction names (`spine_W1`–`spine_W5`, `spine_E1`–`spine_E5`).

### Movement priorities

Robots advertise a priority with every acquire request. Higher-priority robots win contested waypoints and poll the TC more aggressively.

| Priority | Value | When used |
|---|---|---|
| `PRIORITY_IDLE` | 0 | Not moving |
| `PRIORITY_CHARGING` | 1 | Diverting to a charger |
| `PRIORITY_EMPTY` | 2 | Travelling without a box |
| `PRIORITY_LOADED` | 3 | Carrying a box |

### Deadlock detection and resolution

The TC tracks each robot's pending (blocked) request. When robot A is waiting on a waypoint held by B, and B is simultaneously waiting on a waypoint held by A, that is a circular wait. The TC resolves it by publishing a yield signal to one robot:

- **Aisle-exit vs aisle-entry**: the entering robot always yields — the robot already inside must exit before a new one can enter.
- **All other cases**: the lower-priority robot yields. Equal priority → lexicographically lower robot ID yields (deterministic tie-break).

### Proactive junction blocking

If a robot is parked at the entry junction of an aisle that is already occupied, the TC yields it immediately (without waiting for a full deadlock cycle to form), allowing the occupying robot to exit cleanly.

### Yield and retreat

When a `nav_server` receives a yield signal it backs up one hop to a free neighbouring waypoint (preferring intersections and chargers), releases its current position, and then replans from the new location.

## Architecture

Seven ROS 2 nodes across six packages:

| Package | Node | Namespace | Role |
|---|---|---|---|
| `logistics_interfaces` | — | — | Shared msg/srv/action definitions |
| `logistics_robot_sim` | `robot_sim` | `robot_N/` | Simulates robot movement and battery drain/charge |
| `logistics_server` | `nav_server` | `robot_N/` | Dijkstra pathfinding, task execution, battery management |
| `logistics_server` | `traffic_controller` | global | Centralised waypoint/aisle locking and deadlock resolution |
| `logistics_task_manager` | `task_manager` | global | FIFO task queue, robot assignment, smart dock selection |
| `logistics_warehouse` | `warehouse_state` | global | Authoritative slot and dock occupancy state |
| `logistics_web` | `web_node` | global | FastAPI + WebSocket bridge to the web UI |

`robot_sim` and `nav_server` are launched once per robot under their own ROS namespace (`robot_1/`, `robot_2/`), so adding more robots requires no code changes.

## Debugging

```bash
# Robot position, battery, and movement progress
ros2 topic echo /robot_1/robot_status

# Task assignment, queue depth, and movement priority
ros2 topic echo /robot_1/task_status

# Slot and dock occupancy
ros2 topic echo /warehouse_state

# Submit a task manually
ros2 service call /submit_task logistics_interfaces/srv/SubmitTask "{task_type: 'pickup', slot: 'shelf_A1'}"

# Toggle slot occupancy manually
ros2 service call /set_slot_occupancy logistics_interfaces/srv/SetSlotOccupancy "{waypoint_id: 'shelf_A1', occupied: true}"

# Manually acquire a waypoint lock (traffic controller)
ros2 service call /traffic/acquire logistics_interfaces/srv/AcquireWaypoint "{robot_id: 'debug', waypoint: 'shelf_A1', priority: 0}"

# Manually release a waypoint lock
ros2 service call /traffic/release logistics_interfaces/srv/ReleaseWaypoint "{robot_id: 'debug', waypoint: 'shelf_A1'}"
```

`logistics_server/logistics_server/task_client.py` is a CLI utility for submitting tasks directly to a single `nav_server` (bypasses the task manager queue).
