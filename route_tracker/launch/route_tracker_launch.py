import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    route_tracker_param_dir = LaunchConfiguration(
        'route_tracker_launch.py',
        default=os.path.join(
            get_package_share_directory('route_tracker'),
            'param',
            'kec_route_tracker.yaml'))
    print("test");
    return LaunchDescription([
        DeclareLaunchArgument(
            'route_tracker',
            default_value=route_tracker_param_dir,
            description=''),

        Node(
            package='route_tracker',
            executable='route_tracker_node',
            parameters=[route_tracker_param_dir],
            output='screen'),
    ])