#include "rm_behavior_tree/plugins/action/back_up.hpp"

#include <cmath>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace rm_behavior_tree
{

BackUpAction::BackUpAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config), Node("back_up_node")
{
  // 创建 cmd_vel 发布者
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_chassis", 10);

  // TF 设置
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus BackUpAction::onStart()
{
  // 获取参数
  getInput("backup_dist", backup_dist_);
  getInput("backup_speed", backup_speed_);
  getInput("time_allowance", time_allowance_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);

  // 重置状态
  started_ = false;

  // 获取起始位置
  if (!getCurrentPose(start_pose_)) {
    RCLCPP_WARN(this->get_logger(), "[BackUp] Failed to get initial pose");
    return BT::NodeStatus::FAILURE;
  }

  start_time_ = this->now();
  started_ = true;

  RCLCPP_INFO(
    this->get_logger(), "[BackUp] Starting: dist=%.2fm, speed=%.2fm/s",
    backup_dist_, backup_speed_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BackUpAction::onRunning()
{
  auto current_time = this->now();

  // 检查超时
  double elapsed = (current_time - start_time_).seconds();
  if (elapsed > time_allowance_) {
    RCLCPP_WARN(this->get_logger(), "[BackUp] Timeout after %.1fs", elapsed);
    stopRobot();
    return BT::NodeStatus::FAILURE;
  }

  // 获取当前位置
  geometry_msgs::msg::PoseStamped current_pose;
  if (!getCurrentPose(current_pose)) {
    RCLCPP_WARN(this->get_logger(), "[BackUp] Failed to get current pose");
    stopRobot();
    return BT::NodeStatus::FAILURE;
  }

  // 计算已移动距离
  double dx = current_pose.pose.position.x - start_pose_.pose.position.x;
  double dy = current_pose.pose.position.y - start_pose_.pose.position.y;
  double distance_traveled = std::hypot(dx, dy);

  // 检查是否完成
  if (distance_traveled >= backup_dist_) {
    RCLCPP_INFO(this->get_logger(), "[BackUp] Completed: traveled %.2fm", distance_traveled);
    stopRobot();
    return BT::NodeStatus::SUCCESS;
  }

  // 发送后退命令（负速度）
  publishVelocity(-std::abs(backup_speed_));

  return BT::NodeStatus::RUNNING;
}

void BackUpAction::onHalted()
{
  stopRobot();
  RCLCPP_INFO(this->get_logger(), "[BackUp] Halted");
}

bool BackUpAction::getCurrentPose(geometry_msgs::msg::PoseStamped & pose)
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero);

    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "[BackUp] TF error: %s", ex.what());
    return false;
  }
}

void BackUpAction::stopRobot()
{
  publishVelocity(0.0);
}

void BackUpAction::publishVelocity(double linear_x)
{
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear_x;
  msg.linear.y = 0.0;
  msg.angular.z = 0.0;
  cmd_vel_pub_->publish(msg);
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::BackUpAction>("BackUp");
}
