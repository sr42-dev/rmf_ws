"""
Main bringup launch file for RMF simulation and tests.

This launch file:
0. Generates fleet config + nav graph from building map
1. Launches the robot simulator with specified number of robots
2. Launches robot controllers for each robot (1s delay)
3. Runs the specified test from rmf_tests (configurable delay, default 5s)

Arguments:
- num_robots: Number of robots to spawn (default: 1)
- test_name: Name of the test to run from rmf_tests (default: test_navigation)
- test_delay: Seconds before starting the test (default: 5.0)
"""

import os
import subprocess
import sys
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


def generate_config_files(context):
    """Generate nav graph and fleet config before launching nodes."""
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    map_yaml = LaunchConfiguration('map_yaml').perform(context)

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
        '--num-robots', str(num_robots),
        '--output', FLEET_CONFIG_PATH,
    ])

    return []


def generate_controller_nodes(context):
    """Generate controller nodes for each robot."""
    num_robots = int(LaunchConfiguration('num_robots').perform(context))

    nodes = []
    for i in range(1, num_robots + 1):
        node = Node(
            package='robot_controller',
            executable='controller',
            name=f'robot_{i}_controller',
            arguments=[f'robot_id:={i}'],
            output='screen'
        )
        nodes.append(node)

    return nodes



def generate_test_process(context):
    """Generate the test execution process."""
    test_name = LaunchConfiguration('test_name').perform(context)
    num_robots = LaunchConfiguration('num_robots').perform(context)

    test_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rmf_tests', test_name,
            '--ros-args',
            '-p', f'num_robots:={num_robots}'
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

    default_map_yaml = os.path.join(
        traffic_editor_assets_share, 'map.building.yaml')
    default_map_png = os.path.join(
        traffic_editor_assets_share, 'map.png')
    default_start_config = os.path.join(
        robots_cv_sim_share, 'config', 'start_config.yaml')

    return LaunchDescription([
        # ── Declare arguments ──────────────────────────────────────────
        DeclareLaunchArgument(
            'num_robots',
            default_value='1',
            description='Number of robots to spawn and control'
        ),
        DeclareLaunchArgument(
            'test_name',
            default_value='test_navigation',
            description='Name of the test executable from rmf_tests package'
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

        # ── Step 0: generate config files ──────────────────────────────
        OpaqueFunction(function=generate_config_files),

        # ── Step 1: launch the simulator ───────────────────────────────
        Node(
            package='robots_cv_sim',
            executable='sim_node',
            name='robot_simulator',
            output='screen',
            parameters=[{
                'map_yaml': LaunchConfiguration('map_yaml'),
                'map_img_path': LaunchConfiguration('map_img_path'),
                'start_config': LaunchConfiguration('start_config'),
                'num_robots_override': LaunchConfiguration('num_robots'),
            }]
        ),

        # ── Step 2: controllers (1s delay for sim) ─────────────────────
        TimerAction(
            period=1.0,
            actions=[OpaqueFunction(function=generate_controller_nodes)]
        ),

        # ── Step 3: test (configurable delay, default 5s) ─────────────
        TimerAction(
            period=LaunchConfiguration('test_delay'),
            actions=[OpaqueFunction(function=generate_test_process)]
        ),
    ])
