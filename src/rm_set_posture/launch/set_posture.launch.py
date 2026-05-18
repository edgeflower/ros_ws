import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('rm_set_posture'), 'config', 'set_posture.yaml')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')

    namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='robot namespace')

    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否启用ros模拟时间')

    log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')

    use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='是否以组件模式加载到容器中')

    container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='rm_container',
        description='组件容器名称')

    # ---------- 独立节点模式 ----------
    standalone_node = Node(
        package='rm_set_posture',
        executable='set_posture_node',
        namespace=namespace,
        parameters=[config_dir, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=3,
        condition=UnlessCondition(use_composition),
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # ---------- 组件模式：加载到已有容器 ----------
    load_composable = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='rm_set_posture',
                plugin='SetPostureNode',
                name='set_posture_node',
                namespace=namespace,
                parameters=[config_dir, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                ],
            ),
        ],
        condition=IfCondition(use_composition),
    )

    ld = LaunchDescription()
    ld.add_action(namespace_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(log_level_cmd)
    ld.add_action(use_composition_cmd)
    ld.add_action(container_name_cmd)
    ld.add_action(standalone_node)
    # ld.add_action(load_composable)
    return ld
