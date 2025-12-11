#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('multi_lidar_sync')
    
    # Configuration file path
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the LiDAR configuration file'
    )
    
    # Multi-LiDAR sync node
    multi_lidar_sync_node = Node(
        package='multi_lidar_sync',
        executable='multi_lidar_sync_node',
        name='multi_lidar_sync_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        config_arg,
        multi_lidar_sync_node
    ])
