#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__CANCEL_NAV2_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__CANCEL_NAV2_GOAL_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace rm_behavior_tree
{

class CancelNav2Goal : public BT::SyncActionNode
{
public:
  CancelNav2Goal(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("action_name", "/navigate_to_pose", "导航 action server 名称"),
      BT::InputPort<double>("timeout", 1.0, "等待取消响应的超时时间（秒）")
    };
  }

  BT::NodeStatus tick() override;

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  rclcpp_action::Client<NavigateToPose>::SharedPtr getClient(const std::string & action_name);

  rclcpp::Node::SharedPtr node_;
  std::string current_action_name_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__CANCEL_NAV2_GOAL_HPP_
