"""
Main bringup launch file for RMF simulation and tests.

This launch file:
0. Reads test config to determine num_robots; generates fleet config + nav graph
1. Launches the robot simulator with specified number of robots
2. Launches robot controllers for each robot (1s delay)
3. Launches the fleet adapter (2s delay)
4. Runs the specified test from rmf_tests (configurable delay, default 5s)

Arguments:
- test_name: Name of the test to run from rmf_tests (default: test_navigation)
- test_config: Path to test config YAML that specifies num_robots etc.
- test_delay: Seconds before starting the test (default: 5.0)
"""

import os
import subprocess
import sys
import yaml as pyyaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


NAV_GRAPH_PATH = '/tmp/rmf_nav_graph.yaml'
FLEET_CONFIG_PATH = '/tmp/rmf_fleet_config.yaml'

# Will be set by generate_config_files after reading test config
_num_robots = 1


def generate_config_files(context):
    """Read test config for num_robots, then generate nav graph and fleet
    config."""
    global _num_robots

    test_config_path = LaunchConfiguration('test_config').perform(context)
    map_yaml = LaunchConfiguration('map_yaml').perform(context)

    # Read num_robots from the test config YAML
    with open(test_config_path, 'r') as f:
        test_config = pyyaml.safe_load(f)
    _num_robots = int(test_config.get('num_robots', 1))

    bringup_share = get_package_share_directory('rmf_bringup')
    nav_graph_script = os.path.join(bringup_share, 'scripts',
                                    'generate_nav_graph.py')
    fleet_config_script = os.path.join(bringup_share, 'scripts',
                                       'generate_fleet_config.py')

    # Generate nav graph from building YAML
    subprocess.check_call([
        sys.executable, nav_graph_script,
        '--building', map_yaml,
        '--output', NAV_GRAPH_PATH,
    ])

    # Generate fleet config with N robots
    subprocess.check_call([
        sys.executable, fleet_config_script,
        '--num-robots', str(_num_robots),
        '--output', FLEET_CONFIG_PATH,
    ])

    return []


def generate_sim_node(context):
    """Generate the simulator node using num_robots from test config."""
    return [Node(
        package='robots_cv_sim',
        executable='sim_node',
        name='robot_simulator',
        output='screen',
        parameters=[{
            'map_yaml': LaunchConfiguration('map_yaml'),
            'map_img_path': LaunchConfiguration('map_img_path'),
            'start_config': LaunchConfiguration('start_config'),
            'num_robots_override': _num_robots,
        }]
    )]


def generate_controller_nodes(context):
    """Generate controller nodes for each robot."""
    nodes = []
    for i in range(1, _num_robots + 1):
        node = Node(
            package='robot_controller',
            executable='controller',
            name=f'robot_{i}_controller',
            arguments=[f'robot_id:={i}'],
            output='screen'
        )
        nodes.append(node)

    return nodes


def generate_fleet_adapter_node(context):
    """Generate the fleet adapter node."""
    return [Node(
        package='fleet_adapter',
        executable='fleet_adapter_node',
        name='fleet_adapter',
        output='screen',
        parameters=[{
            'nav_graph_path': NAV_GRAPH_PATH,
            'fleet_name': 'fleet_1',
        }]
    )]


def generate_test_process(context):
    """Generate the test execution process."""
    test_name = LaunchConfiguration('test_name').perform(context)

    test_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rmf_tests', test_name,
        ],
        output='screen',
        shell=False
    )

    return [test_process]


def generate_launch_description():
    # Get package directories
    robots_cv_sim_share = get_package_share_directory('robots_cv_sim')
    traffic_editor_assets_share = get_package_share_directory(
        'traffic_editor_assets')
    rmf_tests_share = get_package_share_directory('rmf_tests')

    default_map_yaml = os.path.join(
        traffic_editor_assets_share, 'map.building.yaml')
    default_map_png = os.path.join(
        traffic_editor_assets_share, 'map.png')
    default_start_config = os.path.join(
        robots_cv_sim_share, 'config', 'start_config.yaml')
    default_test_config = os.path.join(
        rmf_tests_share, 'config', 'test_navigation.yaml')

    return LaunchDescription([
        # ── Declare arguments ──────────────────────────────────────────
        DeclareLaunchArgument(
            'test_name',
            default_value='test_navigation',
            description='Name of the test executable from rmf_tests package'
        ),
        DeclareLaunchArgument(
            'test_config',
            default_value=default_test_config,
            description='Path to test config YAML (specifies num_robots, etc.)'
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_yaml,
            description='Path to map.building.yaml'
        ),
        DeclareLaunchArgument(
            'map_img_path',
            default_value=default_map_png,
            description='Path to map image'
        ),
        DeclareLaunchArgument(
            'start_config',
            default_value=default_start_config,
            description='Path to robot start config yaml'
        ),
        DeclareLaunchArgument(
            'test_delay',
            default_value='5.0',
            description='Delay in seconds before starting the test'
        ),

        # ── Step 0: read test config + generate config files ───────────
        OpaqueFunction(function=generate_config_files),

        # ── Step 1: launch the simulator ───────────────────────────────
        OpaqueFunction(function=generate_sim_node),

        # ── Step 2: controllers (1s delay for sim) ─────────────────────
        TimerAction(
            period=1.0,
            actions=[OpaqueFunction(function=generate_controller_nodes)]
        ),

        # ── Step 3: rmf_traffic_schedule (1.5s delay) ──────────────────
        # Must be running before the fleet adapter connects to the schedule
        TimerAction(
            period=1.5,
            actions=[
                Node(
                    package='rmf_traffic_ros2',
                    executable='rmf_traffic_schedule',
                    name='rmf_traffic_schedule',
                    output='screen',
                )
            ]
        ),

        # ── Step 4: fleet adapter (3s delay — after schedule is up) ───
        TimerAction(
            period=3.0,
            actions=[OpaqueFunction(function=generate_fleet_adapter_node)]
        ),

        # ── Step 5: test (configurable delay, default 5s) ─────────────
        TimerAction(
            period=LaunchConfiguration('test_delay'),
            actions=[OpaqueFunction(function=generate_test_process)]
        ),
    ])
