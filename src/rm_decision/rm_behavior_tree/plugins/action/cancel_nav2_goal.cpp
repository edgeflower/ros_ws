#include "rm_behavior_tree/plugins/action/cancel_nav2_goal.hpp"

#include <algorithm>
#include <chrono>
#include <future>

#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

CancelNav2Goal::CancelNav2Goal(
  const std::string & name,
  const BT::NodeConfiguration & config,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config),
  node_(params.nh)
{
  if (!node_) {
    throw BT::RuntimeError("CancelNav2Goal requires a valid ROS node");
  }
}

rclcpp_action::Client<CancelNav2Goal::NavigateToPose>::SharedPtr
CancelNav2Goal::getClient(const std::string & action_name)
{
  if (!action_client_ || current_action_name_ != action_name) {
    current_action_name_ = action_name;
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, current_action_name_);
  }
  return action_client_;
}

BT::NodeStatus CancelNav2Goal::tick()
{
  std::string action_name = "/navigate_to_pose";
  getInput("action_name", action_name);
  if (action_name.empty()) {
    action_name = "/navigate_to_pose";
  }

  double timeout_sec = 1.0;
  getInput("timeout", timeout_sec);
  timeout_sec = std::max(0.0, timeout_sec);

  RCLCPP_INFO(
    node_->get_logger(),
    "CancelNav2Goal: cancel all goals on %s",
    action_name.c_str());

  auto client = getClient(action_name);
  const auto timeout = std::chrono::duration<double>(timeout_sec);

  if (!client->wait_for_action_server(timeout)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "CancelNav2Goal: action server %s not available",
      action_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto cancel_future = client->async_cancel_all_goals();
  if (cancel_future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_WARN(
      node_->get_logger(),
      "CancelNav2Goal: timeout waiting for cancel response from %s",
      action_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto cancel_response = cancel_future.get();
  RCLCPP_INFO(
    node_->get_logger(),
    "CancelNav2Goal: cancel service returned code %d",
    cancel_response->return_code);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::CancelNav2Goal, "CancelNav2Goal");
