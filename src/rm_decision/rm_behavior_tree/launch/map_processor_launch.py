import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('rm_behavior_tree')
    config_dir = os.path.join(pkg_dir, 'config')

    # Load YAML configuration
    map_processor_config = os.path.join(config_dir, 'map_processor.yaml')

    # Declare launch arguments
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/map',
        description='Topic name for the occupancy grid map'
    )

    sample_step_arg = DeclareLaunchArgument(
        'sample_step',
        default_value='2.0',
        description='Sampling step size in meters'
    )

    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.3',
        description='Robot radius in meters for dilation'
    )

    ray_count_arg = DeclareLaunchArgument(
        'ray_count',
        default_value='36',
        description='Number of rays for observation scoring'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='0.1',
        description='Update rate in Hz'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to use simulation time'
    )

    # Map processor node
    map_processor_node = Node(
        package='rm_behavior_tree',
        executable='map_processor_node',
        name='map_processor_node',
        output='screen',
        parameters=[
            map_processor_config,  # Load YAML config first
            {  # Command-line overrides
                'map_topic': LaunchConfiguration('map_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(map_topic_arg)
    ld.add_action(sample_step_arg)
    ld.add_action(robot_radius_arg)
    ld.add_action(ray_count_arg)
    ld.add_action(update_rate_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(map_processor_node)

    return ld
