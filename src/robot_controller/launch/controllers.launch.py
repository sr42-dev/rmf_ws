from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots',
            default_value='1',
            description='Number of robots to control'
        ),
        OpaqueFunction(function=generate_controller_nodes)
    ])
