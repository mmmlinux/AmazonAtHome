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
            default_value='1.0',
            description='Simulated robot travel speed in m/s',
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',
            description='Port for the web UI',
        ),

        # ── Robot 1 (west side, starts at charge_1) ───────────────────────────
        Node(
            package='logistics_robot_sim',
            executable='robot_sim',
            name='robot_sim',
            namespace='robot_1',
            parameters=[{
                'travel_speed':             LaunchConfiguration('travel_speed'),
                'start_waypoint':           'charge_1',
                'initial_battery':          100.0,
                'discharge_rate_per_meter': 0.5,
                'charge_rate_per_second':   10.0,
                'charging_waypoints':       'charge_1,charge_2',
            }],
            output='screen',
        ),
        Node(
            package='logistics_server',
            executable='nav_server',
            name='nav_server',
            namespace='robot_1',
            parameters=[{
                'map_file':    LaunchConfiguration('map_file'),
                'robot_start': 'charge_1',
            }],
            output='screen',
        ),

        # ── Robot 2 (east side, starts at charge_2) ───────────────────────────
        Node(
            package='logistics_robot_sim',
            executable='robot_sim',
            name='robot_sim',
            namespace='robot_2',
            parameters=[{
                'travel_speed':             LaunchConfiguration('travel_speed'),
                'start_waypoint':           'charge_2',
                'initial_battery':          100.0,
                'discharge_rate_per_meter': 0.5,
                'charge_rate_per_second':   10.0,
                'charging_waypoints':       'charge_1,charge_2',
            }],
            output='screen',
        ),
        Node(
            package='logistics_server',
            executable='nav_server',
            name='nav_server',
            namespace='robot_2',
            parameters=[{
                'map_file':    LaunchConfiguration('map_file'),
                'robot_start': 'charge_2',
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
                'charge_waypoint': 'charge_1',
                'load_waypoint':   'dock_1',
                'unload_waypoint': 'dock_1',
            }],
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
