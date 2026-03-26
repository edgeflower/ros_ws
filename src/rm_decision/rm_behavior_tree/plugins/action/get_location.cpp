#include "rm_behavior_tree/plugins/action/get_location.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>

#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>

#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace rm_behavior_tree {
GetLocationAction::GetLocationAction(
    const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<nav_msgs::msg::Odometry>(name, conf, params)
{
}

BT::NodeStatus GetLocationAction::onTick(
    const std::shared_ptr<nav_msgs::msg::Odometry>& last_msg)
{
    if (last_msg) {
        setOutput("robot_location", *last_msg);

        RCLCPP_DEBUG(logger(), "[GetLocation] pose.position.x: %f", last_msg->pose.pose.position.x);
        RCLCPP_DEBUG(logger(), "[GetLocation] pose.position.y: %f", last_msg->pose.pose.position.y);
        RCLCPP_DEBUG(logger(), "[GetLocation] pose.position.z: %f", last_msg->pose.pose.position.z);

        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_WARN(logger(), "[GetLocation] Waiting for /odometry message...");
        return BT::NodeStatus::FAILURE;
    }
}
} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"

CreateRosNodePlugin(rm_behavior_tree::GetLocationAction, "GetLocation");