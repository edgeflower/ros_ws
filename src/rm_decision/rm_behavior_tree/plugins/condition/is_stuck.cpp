#include "rm_behavior_tree/plugins/condition/is_stuck.hpp"
#include <cmath>
#include <rclcpp/logging.hpp>

namespace rm_behavior_tree
{

IsStuckAction::IsStuckAction(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus IsStuckAction::tick()
{
  // Reset state when node becomes IDLE (halted or tree reset)
  if (status() == BT::NodeStatus::IDLE) {
    initialized_ = false;
  }

  // Get input ports
  auto cmd_vel_opt = getInput<geometry_msgs::msg::Twist>("cmd_vel");
  auto robot_pose_opt = getInput<geometry_msgs::msg::PoseStamped>("robot_pose");

  if (!cmd_vel_opt || !robot_pose_opt) {
    RCLCPP_WARN(
      rclcpp::get_logger("IsStuck"),
      "Missing input ports: cmd_vel=%s, robot_pose=%s",
      cmd_vel_opt ? "ok" : "missing",
      robot_pose_opt ? "ok" : "missing");
    return BT::NodeStatus::FAILURE;
  }

  auto cmd_vel = cmd_vel_opt.value();
  auto robot_pose = robot_pose_opt.value();

  // Get thresholds with defaults
  double velocity_threshold = 1.0;
  double distance_threshold = 0.5;
  double time_window = 3.0;

  getInput("velocity_threshold", velocity_threshold);
  getInput("distance_threshold", distance_threshold);
  getInput("time_window", time_window);

  // Calculate current linear speed from cmd_vel
  double linear_speed = std::sqrt(
    cmd_vel.linear.x * cmd_vel.linear.x +
    cmd_vel.linear.y * cmd_vel.linear.y);

  // If velocity command is below threshold, reset and return SUCCESS
  if (linear_speed < velocity_threshold) {
    initialized_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  // Velocity is above threshold - check if robot is moving
  auto current_time = rclcpp::Clock().now();
  auto current_position = robot_pose.pose.position;

  if (!initialized_) {
    // Start new detection window
    start_position_ = current_position;
    window_start_time_ = current_time;
    initialized_ = true;
    RCLCPP_DEBUG(
      rclcpp::get_logger("IsStuck"),
      "Starting stuck detection window at (%.2f, %.2f)",
      start_position_.x, start_position_.y);
    return BT::NodeStatus::SUCCESS;
  }

  // Calculate elapsed time and distance moved
  double elapsed = (current_time - window_start_time_).seconds();
  double dx = current_position.x - start_position_.x;
  double dy = current_position.y - start_position_.y;
  double distance_moved = std::sqrt(dx * dx + dy * dy);

  RCLCPP_DEBUG(
    rclcpp::get_logger("IsStuck"),
    "Elapsed: %.2fs, Distance: %.2fm, Velocity: %.2fm/s",
    elapsed, distance_moved, linear_speed);

  // Check if time window has elapsed
  if (elapsed >= time_window) {
    if (distance_moved < distance_threshold) {
      // Robot is stuck: high velocity command but little movement
      RCLCPP_WARN(
        rclcpp::get_logger("IsStuck"),
        "Robot STUCK detected! Velocity: %.2fm/s, Distance moved: %.2fm in %.2fs",
        linear_speed, distance_moved, elapsed);
      // Reset for next detection cycle
      initialized_ = false;
      return BT::NodeStatus::FAILURE;
    } else {
      // Robot is moving normally, reset window
      RCLCPP_DEBUG(
        rclcpp::get_logger("IsStuck"),
        "Robot moving normally. Resetting detection window.");
      initialized_ = false;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsStuckAction>("IsStuck");
}
