#!/usr/bin/env python3
"""
ROS2 Launch：启动机器人区域检测节点

使用方式：
  ros2 launch robot_area_detector robot_area_detector.launch.py
  ros2 launch robot_area_detector robot_area_detector.launch.py robot_frame:=gimbal_yaw_fake
  ros2 launch robot_area_detector robot_area_detector.launch.py areas_yaml:=/path/to/areas.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_area_detector')
    polygon_manager_dir = get_package_share_directory('polygon_manager')
    default_config = os.path.join(pkg_dir, 'config', 'robot_area_detector.yaml')
    areas_yaml_default = os.path.join(polygon_manager_dir, 'config', 'areas.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_config,
            description='Parameter YAML file'),
        DeclareLaunchArgument(
            'areas_yaml',
            default_value=areas_yaml_default,
            description='Path to areas.yaml'),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='Map frame'),
        DeclareLaunchArgument(
            'robot_frame', default_value='chassis',
            description='Robot frame'),

        Node(
            package='robot_area_detector',
            executable='robot_area_detector_node',
            name='robot_area_detector_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'areas_yaml': LaunchConfiguration('areas_yaml'),
                    'map_frame': LaunchConfiguration('map_frame'),
                    'robot_frame': LaunchConfiguration('robot_frame'),
                },
            ],
        ),
    ])
