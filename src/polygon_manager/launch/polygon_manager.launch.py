#!/usr/bin/env python3
"""
ROS2 Launch 文件：启动禁区管理系统

使用方式：
  ros2 launch polygon_manager polygon_manager.launch.py

可用参数：
  config_file:=/path/to/config.yaml
  update_frequency:=20.0
  use_rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """
    LaunchDescription 配置函数
    
    在这里定义所有的 ROS 节点和参数
    """
    
    # 获取包目录
    package_dir = get_package_share_directory('polygon_manager')
    
    # 获取配置文件路径
    config_file = LaunchConfiguration('config_file')
    rviz_config = os.path.join(package_dir, 'config', 'polygon_manager.rviz')
    
    # 多边形管理节点
    polygon_manager_node = Node(
        package='polygon_manager',
        executable='polygon_manager_node',
        name='polygon_manager_node',
        output='screen',
        parameters=[
            {'config_file': config_file},
            {'map_file': LaunchConfiguration('map_file')},
            {'update_frequency': LaunchConfiguration('update_frequency')},
        ],
        remappings=[
            ('/clicked_point', '/clicked_point'),
            ('/enemy_position', '/enemy_position'),
            ('/enemy_area_state', '/enemy_area_state'),
            ('/map', '/map'),
        ]
    )
    
    # RViz 可视化（如果启用）
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    return [polygon_manager_node, rviz_node]


def generate_launch_description():
    """
    生成 LaunchDescription
    """
    
    package_dir = get_package_share_directory('polygon_manager')
    default_config = os.path.join(package_dir, 'config', 'areas.yaml')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to the polygon configuration YAML file'
        ),
        
        DeclareLaunchArgument(
            'update_frequency',
            default_value='10.0',
            description='Polygon marker update frequency (Hz)'
        ),
        
        DeclareLaunchArgument(
            'map_file',
            default_value='/home/sentry/Desktop/ros_ws/test_gai.yaml',
            description='Path to offline map YAML file (empty = no offline map)'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz2'
        ),
        
        # 执行配置函数
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    """
    直接运行此脚本时调用
    """
    desc = generate_launch_description()
    # 可以在这里添加额外的调试代码
