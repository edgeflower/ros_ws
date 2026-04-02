#ifndef RM_BEHAVIOR_TREE_PLUGINS_ACTION_GET_ROBOT_LOCATION_
#define RM_BEHAVIOR_TREE_PLUGINS_ACTION_GET_ROBOT_LOCATION_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace rm_behavior_tree {

class GetRobotLocationAction : public BT::RosTopicSubNode<nav_msgs::msg::Odometry>
{
public:
    GetRobotLocationAction(const std::string& name,
                          const BT::NodeConfig& config,
                          const BT::RosNodeParams& params);

    static BT::PortsList providedPorts()
    {
        BT::PortsList custom_ports = {
            
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("robot_pose")
        };
        return BT::RosTopicSubNode<nav_msgs::msg::Odometry>::providedBasicPorts(custom_ports);
    }

    BT::NodeStatus onTick(const std::shared_ptr<nav_msgs::msg::Odometry>& last_msg) override;

private:
    geometry_msgs::msg::PoseStamped robot_pose_;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE_PLUGINS_ACTION_GET_ROBOT_LOCATION_
