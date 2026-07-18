"""
Warehouse logistics launch file.

Fleet configuration is loaded from robots.yaml (override with robots_file:=...).
Operational knobs (travel_speed, web_port, map_file) are kept as launch args
because they are commonly tweaked at the command line without editing a file.

Usage examples
--------------
  # Default (all simulated, built-in map)
  ros2 launch logistics_server logistics.launch.py

  # Custom config file
  ros2 launch logistics_server logistics.launch.py robots_file:=/path/to/robots.yaml

  # Operational overrides
  ros2 launch logistics_server logistics.launch.py travel_speed:=2.0 web_port:=9000
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg         = get_package_share_directory('logistics_server')
    default_map          = os.path.join(pkg, 'config', 'warehouse_map.yaml')
    default_robots_file  = os.path.join(pkg, 'config', 'robots.yaml')
    default_chargers_file = os.path.join(pkg, 'config', 'chargers.yaml')

    # ── Resolve chargers_file and robots_file early ───────────────────────────
    chargers_file = os.environ.get('CHARGERS_FILE', default_chargers_file)

    # ── Resolve robots_file early so we can read it as plain Python ──────────
    # LaunchConfiguration substitutions are evaluated lazily by the launch
    # framework, but we need the file path *now* to load the fleet config and
    # drive conditional node creation.  We read the env override if present,
    # otherwise fall back to the package default.
    robots_file = os.environ.get('ROBOTS_FILE', default_robots_file)

    with open(robots_file) as f:
        fleet = yaml.safe_load(f)

    robots = fleet['robots']   # list of per-robot dicts

    robot_ids    = [r['id'] for r in robots]
    peer_map     = {r['id']: [x['id'] for x in robots if x['id'] != r['id']]
                   for r in robots}
    home_chargers   = ','.join(r['home_charger']   for r in robots)
    min_batteries   = ','.join(str(r['min_battery']) for r in robots)

    nodes = [
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map,
            description='Path to the warehouse map YAML file',
        ),
        DeclareLaunchArgument(
            'travel_speed',
            default_value='4.0',
            description='Simulated robot travel speed in m/s (ignored for real robots)',
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',
            description='Port for the web UI',
        ),
    ]

    # ── Per-robot nodes ───────────────────────────────────────────────────────
    for robot in robots:
        rid         = robot['id']
        robot_type  = robot.get('robot_type', 'simulated')
        start_wp    = robot['start_waypoint']
        min_bat     = float(robot.get('min_battery',      50.0))
        crit_bat    = float(robot.get('critical_battery', 25.0))
        peers       = ','.join(peer_map[rid])

        if robot_type == 'simulated':
            # robot_sim: simulates movement and battery for this robot
            nodes.append(Node(
                package='logistics_robot_sim',
                executable='robot_sim',
                name='robot_sim',
                namespace=rid,
                parameters=[{
                    'travel_speed':             LaunchConfiguration('travel_speed'),
                    'start_waypoint':           start_wp,
                    'initial_battery':          100.0,
                    'discharge_rate_per_meter': float(robot.get('discharge_rate_per_meter', 0.25)),
                    'charge_rate_per_second':   float(robot.get('charge_rate_per_second',   5.0)),
                    'charging_waypoints':       robot.get('charging_waypoints', start_wp),
                }],
                output='screen',
            ))
        elif robot_type == 'hector':
            # hector_driver: bridges nav_server to the physical Hector robot
            # over WebSocket.  Hector connects to this node on boot.
            nodes.append(Node(
                package='logistics_robot_sim',
                executable='hector_driver',
                name='robot_sim',
                namespace=rid,
                parameters=[{
                    'start_waypoint':    start_wp,
                    'charging_waypoints': robot.get('charging_waypoints', start_wp),
                    'ws_port':           int(robot.get('ws_port', 8765)),
                }],
                output='screen',
            ))
        else:
            # real robot: user supplies their own driver that implements
            # move_to_waypoint action + robot_status topic in this namespace.
            # See logistics_robot_sim/real_robot_driver.py for a skeleton.
            pass

        # nav_server always launches regardless of robot type
        nodes.append(Node(
            package='logistics_server',
            executable='nav_server',
            name='nav_server',
            namespace=rid,
            parameters=[{
                'map_file':         LaunchConfiguration('map_file'),
                'robot_start':      start_wp,
                'peers':            peers,
                'min_battery':      min_bat,
                'critical_battery': crit_bat,
            }],
            output='screen',
        ))

    # ── Shared / global nodes ─────────────────────────────────────────────────
    nodes += [
        # Warehouse state (slot / dock occupancy)
        Node(
            package='logistics_warehouse',
            executable='warehouse_state',
            name='warehouse_state',
            parameters=[{
                'map_file': LaunchConfiguration('map_file'),
            }],
            output='screen',
        ),

        # Task manager (shared queue, dispatches to all robots)
        Node(
            package='logistics_task_manager',
            executable='task_manager',
            name='task_manager',
            parameters=[{
                'robots':          ','.join(robot_ids),
                'map_file':        LaunchConfiguration('map_file'),
                'charge_waypoint': robots[0]['home_charger'],
                'home_chargers':   home_chargers,
                'min_batteries':   min_batteries,
                'load_waypoint':   'dock_1',
                'unload_waypoint': 'dock_1',
            }],
            output='screen',
        ),

        # Traffic controller (waypoint / aisle locking for collision avoidance)
        Node(
            package='logistics_server',
            executable='traffic_controller',
            name='traffic_controller',
            output='screen',
        ),

        # Charger manager (engage/disengage each physical charger station)
        Node(
            package='logistics_server',
            executable='charger_manager',
            name='charger_manager',
            parameters=[{
                'chargers_file': chargers_file,
            }],
            output='screen',
        ),

        # Web UI
        Node(
            package='logistics_web',
            executable='web_node',
            name='logistics_web',
            parameters=[{
                'map_file': LaunchConfiguration('map_file'),
                'web_port': LaunchConfiguration('web_port'),
                'robots':   ','.join(robot_ids),
            }],
            output='screen',
        ),
    ]

    return LaunchDescription(nodes)
