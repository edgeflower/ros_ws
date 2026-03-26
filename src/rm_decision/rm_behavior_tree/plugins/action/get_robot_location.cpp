#include "rm_behavior_tree/plugins/action/get_robot_location.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <functional>
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/logging.hpp>
namespace rm_behavior_tree {
GetRobotLocationAction::GetRobotLocationAction(
    const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{
    node_ = std::make_shared<rclcpp::Node>("get_robot_location");

    odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/red_standard_robot1/odometry", 10,
        std::bind(&GetRobotLocationAction::updateOdometry, this, std::placeholders::_1));
}
BT::NodeStatus GetRobotLocationAction::tick()
{   robot_x_ = 0.0;
    robot_y_ = 0.0;
    rclcpp::spin_some(node_);

    if (setOutput("robot_pose", std::make_pair(robot_x_, robot_y_))) {
        RCLCPP_INFO(node_->get_logger(),"robot x 坐标 %.2f  robot y 坐标 %.2f", robot_x_ , robot_y_);
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_WARN(node_->get_logger(),"妈的，出错了");
        return BT::NodeStatus::FAILURE;
    }
}

}
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::GetRobotLocationAction>("GetRobotLocation");
}
