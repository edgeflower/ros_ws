#!/usr/bin/env python3
"""
ROS2 Launch 文件：启动敌方禁区检测节点

使用方式：
  ros2 launch enemy_forbidden_area_detector enemy_forbidden_area_detector.launch.py

可用参数：
  areas_yaml_path:=/path/to/areas.yaml
  target_topic:=/target_tracking
  enable_visualization:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('enemy_forbidden_area_detector')
    default_config = os.path.join(package_dir, 'config', 'enemy_forbidden_area_detector.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'areas_yaml_path',
            default_value='/home/sentry/Desktop/ros_ws/src/polygon_manager/config/areas.yaml',
            description='Path to the areas YAML file'
        ),
        DeclareLaunchArgument(
            'target_topic',
            default_value='/target_tracking',
            description='Topic for enemy target tracking'
        ),
        DeclareLaunchArgument(
            'result_topic',
            default_value='/enemy_in_forbidden_area',
            description='Topic for forbidden area detection result'
        ),
        DeclareLaunchArgument(
            'inside_threshold',
            default_value='5',
            description='Consecutive inside detections to confirm forbidden'
        ),
        DeclareLaunchArgument(
            'outside_threshold',
            default_value='10',
            description='Consecutive outside detections to confirm safe'
        ),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='true',
            description='Enable RViz visualization markers'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_config,
            description='Path to the parameter YAML file'
        ),

        Node(
            package='enemy_forbidden_area_detector',
            executable='enemy_forbidden_area_detector_node',
            name='enemy_forbidden_area_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'areas_yaml_path': LaunchConfiguration('areas_yaml_path'),
                    'target_topic': LaunchConfiguration('target_topic'),
                    'result_topic': LaunchConfiguration('result_topic'),
                    'inside_threshold': LaunchConfiguration('inside_threshold'),
                    'outside_threshold': LaunchConfiguration('outside_threshold'),
                    'enable_visualization': LaunchConfiguration('enable_visualization'),
                },
            ],
        ),
    ])
