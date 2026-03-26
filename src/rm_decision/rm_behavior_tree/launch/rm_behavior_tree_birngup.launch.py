import os  # 导入操作系统接口模块，用于路径拼接
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node , SetRemap , PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    bt_config_dir = os.path.join(get_package_share_directory('rm_behavior_tree'), 'config')
    
    style = LaunchConfiguration('style')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level'),

    bt_xml_dir = PathJoinSubstitution([bt_config_dir, style ]) , ".xml"
    namespace = LaunchConfiguration('namespace')

    
    style_cmd = DeclareLaunchArgument(
            'style',
            default_value="map1",
            description="选取哪一个进攻防御方式   test0	test test2	rmul2025_01 test3  "
        )

    use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value="true",
            description="是否启用ros模拟时间"
        )
    
    namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value="", # red_standard_robot1
            description="robot namespace"
    )
    
    log_level_cmd = DeclareLaunchArgument(
            'log_level',
            default_value="info",
            description="log level"
        
    )

    rm_behavior_tree_node = Node(
        package='rm_behavior_tree',
        executable='rm_behavior_tree_me',
        namespace=namespace,
        respawn=True,         # 如果节点崩溃则自动重启节点
        respawn_delay=3,      # 节点重启前等待 3 秒
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                'style': bt_xml_dir,
                'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ],
    )
    bringup_group = GroupAction(
        actions=[
            PushRosNamespace(namespace= namespace),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),
            rm_behavior_tree_node,
        ]

    )
    ld = LaunchDescription()
    
    ld.add_action(namespace_cmd)
    ld.add_action(style_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(log_level_cmd)
    #ld.add_action(bringup_group)
    ld.add_action(rm_behavior_tree_node)
    
    
    return ld
