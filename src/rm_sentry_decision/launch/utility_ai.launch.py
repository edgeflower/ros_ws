"""
Utility AI 决策节点 Launch 文件

启动 utility_ai_node 并加载参数配置文件。

使用方法:
  ros2 launch rm_sentry_decision utility_ai.launch.py

可选参数:
  params_file: 参数文件路径（默认使用 config/utility_ai.yaml）
"""

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # 获取 rm_sentry_decision 包的共享目录路径
    pkg_share = get_package_share_directory('rm_sentry_decision')

    # 默认参数文件路径
    default_params_file = os.path.join(pkg_share, 'config', 'utility_ai.yaml')

    # 解决 workspace overlay 库冲突：
    # 当 underlay (如 /home/lu/3V3/install) 也包含 rm_decision_interfaces 时，
    # 旧版库会在 LD_LIBRARY_PATH 中排在前面，导致符号查找失败。
    # 这里将 overlay 的 lib 目录前置到 LD_LIBRARY_PATH，确保新版库优先加载。
    pkg_prefix = get_package_prefix('rm_sentry_decision')
    overlay_lib = os.path.join(os.path.dirname(pkg_prefix), 'rm_decision_interfaces', 'lib')

    # 创建 utility_ai_node 节点
    utility_ai_node = Node(
        package='rm_sentry_decision',
        executable='utility_ai_node',
        name='utility_ai_node',
        output='screen',                    # 日志输出到终端
        parameters=[default_params_file],   # 加载参数文件
        # 如果需要在 namespace 下运行，取消下面的注释
        # namespace='red_standard_robot1',
        # remappings=[
        #     ('/game_status', '/red_standard_robot1/game_status'),
        #     ('/robot_status', '/red_standard_robot1/robot_status'),
        #     ('/target_tracking', '/red_standard_robot1/target_tracking'),
        #     ('/lidar_odometry', '/red_standard_robot1/lidar_odometry'),
        #     ('/robot_area_status', '/red_standard_robot1/robot_area_status'),
        #     ('/sentry_decision', '/red_standard_robot1/sentry_decision'),
        # ],
    )

    return LaunchDescription([
        # 将 overlay 的 rm_decision_interfaces/lib 前置到 LD_LIBRARY_PATH
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=overlay_lib + ':' + os.environ.get('LD_LIBRARY_PATH', ''),
        ),
        utility_ai_node,
    ])
