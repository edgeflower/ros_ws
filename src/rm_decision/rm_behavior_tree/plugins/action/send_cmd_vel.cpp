#include "rm_behavior_tree/plugins/action/send_cmd_vel.hpp"

#include <chrono>
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

namespace rm_behavior_tree
{

SendCmdVelAction::SendCmdVelAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config), Node("send_cmd_vel_node")
{
}

BT::NodeStatus SendCmdVelAction::onStart()
{
  getInput("linear_x", linear_x_);
  getInput("linear_y", linear_y_);
  getInput("angular_z", angular_z_);

  std::string topic_name;
  getInput("topic_name", topic_name);

  double frequency;
  getInput("frequency", frequency);
  auto period = std::chrono::duration<double>(1.0 / frequency);

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10);
  timer_ = this->create_wall_timer(period, std::bind(&SendCmdVelAction::publishCmdVel, this));

  RCLCPP_INFO(
    this->get_logger(), "[SendCmdVel] Started: lx=%.2f ly=%.2f az=%.2f @ %.1fHz -> %s",
    linear_x_, linear_y_, angular_z_, frequency, topic_name.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SendCmdVelAction::onRunning()
{
  return BT::NodeStatus::RUNNING;
}

void SendCmdVelAction::onHalted()
{
  timer_.reset();

  auto msg = geometry_msgs::msg::Twist();
  cmd_vel_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "[SendCmdVel] Halted, zero velocity sent");
}

void SendCmdVelAction::publishCmdVel()
{
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear_x_;
  msg.linear.y = linear_y_;
  msg.angular.z = angular_z_;
  cmd_vel_pub_->publish(msg);
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::SendCmdVelAction>("SendCmdVel");
}
