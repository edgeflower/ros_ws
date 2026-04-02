// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_decision_interfaces/msg/robot_control.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace fake_vel_transform
{
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  void syncCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
    const nav_msgs::msg::Path::ConstSharedPtr & local_plan);
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdSpinCallback(rm_decision_interfaces::msg::RobotControl::SharedPtr msg);
  void publishTransform();
  geometry_msgs::msg::Twist transformVelocity(
    const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<rm_decision_interfaces::msg::RobotControl>::SharedPtr cmd_spin_sub_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_filter_;
  message_filters::Subscriber<nav_msgs::msg::Path> local_plan_sub_filter_;
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, nav_msgs::msg::Path>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string robot_base_frame_;
  std::string fake_robot_base_frame_;
  std::string odom_topic_;
  std::string local_plan_topic_;
  std::string cmd_spin_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  float spin_speed_;

  std::mutex cmd_vel_mutex_;
  geometry_msgs::msg::Twist::SharedPtr latest_cmd_vel_;
  double current_robot_base_angle_;
  rclcpp::Time last_controller_activate_time_;

  std::string smooth_mode_;
  int window_size_;
  double alpha_;
  float chassis_vel_multiplier_ = 1.0;

  std::deque<geometry_msgs::msg::Twist> vel_buffer_;
  geometry_msgs::msg::Twist smoothed_vel_{};

  geometry_msgs::msg::Twist smoothVelocity(const geometry_msgs::msg::Twist & new_vel)
  {
    geometry_msgs::msg::Twist result;
    if (smooth_mode_ == "average") {
      vel_buffer_.push_back(new_vel);
      if ((int)vel_buffer_.size() > window_size_) vel_buffer_.pop_front();

      result.linear.x = 0.0;
      result.linear.y = 0.0;
      result.angular.z = 0.0;
      for (const auto & v : vel_buffer_) {
        result.linear.x += v.linear.x;
        result.linear.y += v.linear.y;
        result.angular.z += v.angular.z;
      }
      result.linear.x /= vel_buffer_.size();
      result.linear.y /= vel_buffer_.size();
      result.angular.z /= vel_buffer_.size();
    } else if (smooth_mode_ == "exponential") {
      smoothed_vel_.linear.x = alpha_ * new_vel.linear.x + (1 - alpha_) * smoothed_vel_.linear.x;
      smoothed_vel_.linear.y = alpha_ * new_vel.linear.y + (1 - alpha_) * smoothed_vel_.linear.y;
      smoothed_vel_.angular.z = alpha_ * new_vel.angular.z + (1 - alpha_) * smoothed_vel_.angular.z;
      result = smoothed_vel_;
    } else {
      result = new_vel;
    }
    return result;
  }
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
