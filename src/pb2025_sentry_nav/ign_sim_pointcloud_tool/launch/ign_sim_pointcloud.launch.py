from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    ign_sim_pointcloud_node = Node(
        package="ign_sim_pointcloud_tool",
        executable="ign_sim_pointcloud_tool_node",
        namespace="",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time},
                    {"reliability_qos": "best_effort"}],
    )

    ld = LaunchDescription()

    ld.add_action(ign_sim_pointcloud_node)

    return ld
