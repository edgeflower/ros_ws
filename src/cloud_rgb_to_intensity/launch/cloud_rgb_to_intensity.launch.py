#!/usr/bin/env python3
"""
Launch file for cloud_rgb_to_intensity node.

Converts PointXYZRGB (odin1/cloud_slam) to PointXYZI (registered_scan)
for terrain_analysis compatibility.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='odin1/cloud_slam',
        description='Input PointXYZRGB topic'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='registered_scan',
        description='Output PointXYZI topic'
    )

    intensity_mode_arg = DeclareLaunchArgument(
        'intensity_mode',
        default_value='zero',
        description='Intensity mode: zero, gray, or height'
    )

    # Node
    cloud_converter_node = Node(
        package='cloud_rgb_to_intensity',
        executable='cloud_rgb_to_intensity_node',
        name='cloud_rgb_to_intensity',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'intensity_mode': LaunchConfiguration('intensity_mode'),
        }],
        remappings=[
            # Optional: add remappings here if needed
        ]
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        intensity_mode_arg,
        cloud_converter_node,
    ])
