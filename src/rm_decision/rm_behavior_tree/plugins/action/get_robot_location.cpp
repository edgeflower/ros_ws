#include "rm_behavior_tree/plugins/action/get_robot_location.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>

namespace rm_behavior_tree {

GetRobotLocationAction::GetRobotLocationAction(
    const std::string& name,
    const BT::NodeConfig& config,
    const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<nav_msgs::msg::Odometry>(name, config, params)
{
}

BT::NodeStatus GetRobotLocationAction::onTick(
    const std::shared_ptr<nav_msgs::msg::Odometry>& last_msg)
{
    if (!last_msg) {
        RCLCPP_WARN(node_->get_logger(), "[GetRobotLocation] Waiting for odometry message...");
        return BT::NodeStatus::FAILURE;
    }

    // Convert Odometry to PoseStamped
    robot_pose_.header.stamp = node_->now();
    robot_pose_.header.frame_id = last_msg->header.frame_id;

    robot_pose_.pose.position.x = last_msg->pose.pose.position.x;
    robot_pose_.pose.position.y = last_msg->pose.pose.position.y;
    robot_pose_.pose.position.z = last_msg->pose.pose.position.z;

    robot_pose_.pose.orientation = last_msg->pose.pose.orientation;

    // Output the robot pose
    if (setOutput("robot_pose", robot_pose_)) {
        RCLCPP_DEBUG(node_->get_logger(),
                    "[GetRobotLocation] robot x: %.2f, y: %.2f",
                    last_msg->pose.pose.position.x,
                    last_msg->pose.pose.position.y);
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "[GetRobotLocation] Failed to set output port");
        return BT::NodeStatus::FAILURE;
    }
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::GetRobotLocationAction, "GetRobotLocation")
