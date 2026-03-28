# Amazon at Home — Warehouse Logistics Simulator

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
2. `nav_server` runs Dijkstra from its current position to the destination, producing an ordered list of waypoints (the path).
3. The full planned path is sent as the first feedback message so the web UI can display it immediately.
4. `nav_server` walks the path **hop by hop**, sending each leg as a separate `MoveToWaypoint` action goal to `robot_sim`. Each leg includes the edge distance, absolute heading, and relative turn angle.
5. `robot_sim` moves the robot along the leg at `travel_speed` m/s, publishing `travel_progress` (0.0–1.0) at 4 Hz so the web UI can smoothly interpolate the robot dot between waypoints.
6. Battery is checked **after every hop** (not just at task start), so a robot that runs low mid-journey diverts immediately rather than completing a long route.

### Nearest-Charger and Nearest-Storage Selection

When a robot needs to divert to charge or find an emergency drop spot, it runs Dijkstra from its current position to every candidate waypoint and picks the one with the shortest total path distance — not straight-line distance. This means it correctly accounts for the warehouse aisle layout rather than cutting through walls.

## Architecture

Six ROS 2 packages:

| Package | Node | Role |
|---|---|---|
| `logistics_interfaces` | — | Shared msg/srv/action definitions |
| `logistics_robot_sim` | `robot_sim` | Simulates robot movement and battery drain/charge |
| `logistics_server` | `nav_server` | Dijkstra pathfinding, task execution, battery management |
| `logistics_task_manager` | `task_manager` | FIFO task queue, robot assignment, smart dock selection |
| `logistics_warehouse` | `warehouse_state` | Authoritative slot and dock occupancy state |
| `logistics_web` | `web_node` | FastAPI + WebSocket bridge to the web UI |

Both `robot_sim` and `nav_server` are launched once per robot under their own ROS namespace (`robot_1/`, `robot_2/`), so adding more robots requires no code changes.

## Debugging

```bash
# Robot position and battery
ros2 topic echo /robot_1/robot_status

# Task assignment and queue
ros2 topic echo /robot_1/task_status

# Slot occupancy
ros2 topic echo /warehouse_state

# Submit a task manually
ros2 service call /submit_task logistics_interfaces/srv/SubmitTask "{task_type: 'pickup', slot: 'shelf_A1'}"

# Toggle slot occupancy manually
ros2 service call /set_slot_occupancy logistics_interfaces/srv/SetSlotOccupancy "{waypoint_id: 'shelf_A1', occupied: true}"
```

`logistics_server/logistics_server/task_client.py` is a CLI utility for submitting tasks interactively.
