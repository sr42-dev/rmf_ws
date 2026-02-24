import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Default paths
    default_map_yaml = '/home/dev/ros2_ws/src/rmf/traffic_editor_assets/map.building.yaml'
    default_map_png = '/home/dev/ros2_ws/src/rmf/traffic_editor_assets/map.png'
    
    # Config file in this package
    pkg_share = get_package_share_directory('robots_cv_sim')
    default_config = os.path.join(pkg_share, 'config', 'robots.yaml')
    
    return LaunchDescription([
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
            default_value=default_config,
            description='Path to robot config yaml'
        ),
        DeclareLaunchArgument(
            'num_robots',
            default_value='100',
            description='Number of robots to spawn'
        ),
        
        Node(
            package='robots_cv_sim',
            executable='sim_node',
            name='robot_simulator',
            output='screen',
            parameters=[{
                'map_yaml': LaunchConfiguration('map_yaml'),
                'map_img_path': LaunchConfiguration('map_img_path'),
                'start_config': LaunchConfiguration('start_config'),
                'num_robots_override': LaunchConfiguration('num_robots')
            }]
        )
    ])
