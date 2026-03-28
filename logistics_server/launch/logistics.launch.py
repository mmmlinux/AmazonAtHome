import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('logistics_server')
    default_map = os.path.join(pkg, 'config', 'warehouse_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map,
            description='Path to the warehouse map YAML file',
        ),
        DeclareLaunchArgument(
            'travel_speed',
            default_value='4.0',
            description='Simulated robot travel speed in m/s',
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',
            description='Port for the web UI',
        ),
        DeclareLaunchArgument(
            'min_battery_1',
            default_value='50.0',
            description='Battery % at which robot_1 (empty) stops and charges',
        ),
        DeclareLaunchArgument(
            'min_battery_2',
            default_value='50.0',
            description='Battery % at which robot_2 (empty) stops and charges',
        ),
        DeclareLaunchArgument(
            'critical_battery_1',
            default_value='25.0',
            description='Battery % at which robot_1 (loaded) does an emergency drop',
        ),
        DeclareLaunchArgument(
            'critical_battery_2',
            default_value='25.0',
            description='Battery % at which robot_2 (loaded) does an emergency drop',
        ),

        # ── Robot 1 (west side, home: charge_2) ──────────────────────────────
        Node(
            package='logistics_robot_sim',
            executable='robot_sim',
            name='robot_sim',
            namespace='robot_1',
            parameters=[{
                'travel_speed':             LaunchConfiguration('travel_speed'),
                'start_waypoint':           'charge_2',
                'initial_battery':          100.0,
                'discharge_rate_per_meter': 0.25,
                'charge_rate_per_second':   5.0,
                'charging_waypoints':       'charge_1,charge_2,charge_3,charge_4',
            }],
            output='screen',
        ),
        Node(
            package='logistics_server',
            executable='nav_server',
            name='nav_server',
            namespace='robot_1',
            parameters=[{
                'map_file':         LaunchConfiguration('map_file'),
                'robot_start':      'charge_2',
                'peers':            'robot_2',
                'min_battery':      LaunchConfiguration('min_battery_1'),
                'critical_battery': LaunchConfiguration('critical_battery_1'),
            }],
            output='screen',
        ),

        # ── Robot 2 (east side, home: charge_3) ──────────────────────────────
        Node(
            package='logistics_robot_sim',
            executable='robot_sim',
            name='robot_sim',
            namespace='robot_2',
            parameters=[{
                'travel_speed':             LaunchConfiguration('travel_speed'),
                'start_waypoint':           'charge_3',
                'initial_battery':          100.0,
                'discharge_rate_per_meter': 0.25,
                'charge_rate_per_second':   5.0,
                'charging_waypoints':       'charge_1,charge_2,charge_3,charge_4',
            }],
            output='screen',
        ),
        Node(
            package='logistics_server',
            executable='nav_server',
            name='nav_server',
            namespace='robot_2',
            parameters=[{
                'map_file':         LaunchConfiguration('map_file'),
                'robot_start':      'charge_3',
                'peers':            'robot_1',
                'min_battery':      LaunchConfiguration('min_battery_2'),
                'critical_battery': LaunchConfiguration('critical_battery_2'),
            }],
            output='screen',
        ),

        # ── Warehouse state (slot / dock occupancy) ───────────────────────────
        Node(
            package='logistics_warehouse',
            executable='warehouse_state',
            name='warehouse_state',
            parameters=[{
                'map_file': LaunchConfiguration('map_file'),
            }],
            output='screen',
        ),

        # ── Task manager (shared queue, dispatches to both robots) ─────────────
        Node(
            package='logistics_task_manager',
            executable='task_manager',
            name='task_manager',
            parameters=[{
                'robots':          'robot_1,robot_2',
                'map_file':        LaunchConfiguration('map_file'),
                'charge_waypoint': 'charge_2',
                'home_chargers':   'charge_2,charge_3',
                'min_batteries':   '50.0,50.0',
                'load_waypoint':   'dock_1',
                'unload_waypoint': 'dock_1',
            }],
            output='screen',
        ),

        # ── Traffic controller (waypoint/aisle locking for collision avoidance) ─
        Node(
            package='logistics_server',
            executable='traffic_controller',
            name='traffic_controller',
            output='screen',
        ),

        # ── Web UI ────────────────────────────────────────────────────────────
        Node(
            package='logistics_web',
            executable='web_node',
            name='logistics_web',
            parameters=[{
                'map_file': LaunchConfiguration('map_file'),
                'web_port': LaunchConfiguration('web_port'),
                'robots':   'robot_1,robot_2',
            }],
            output='screen',
        ),
    ])
