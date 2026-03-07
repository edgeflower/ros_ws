// Copyright 2025 Shenzhen Beijing Moscow University Polar Bear Robotics Team
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

#include "pb_omni_pid_pursuit_controller/critics/goal_angle_critic.hpp"

#include <cmath>

#include "nav2_util/node_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace pb_omni_pid_pursuit_controller
{

GoalAngleCritic::GoalAngleCritic()
: CriticFunction("GoalAngleCritic"),
  threshold_to_consider_(1.0)
{
}

void GoalAngleCritic::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::string & critic_name)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in GoalAngleCritic::configure");
  }

  // Declare parameters
  std::string param_prefix = plugin_name + "." + critic_name;

  declare_parameter_if_not_declared(
    node, param_prefix + ".enabled", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, param_prefix + ".weight", rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(
    node, param_prefix + ".threshold_to_consider", rclcpp::ParameterValue(1.0));

  // Get parameters
  enabled_ = getParam<bool>(parent, param_prefix + ".enabled", true);
  weight_ = getParam<double>(parent, param_prefix + ".weight", 10.0);
  threshold_to_consider_ = getParam<double>(
    parent, param_prefix + ".threshold_to_consider", 1.0);

  // Validate parameters
  if (threshold_to_consider_ < 0.1) threshold_to_consider_ = 0.1;
}

double GoalAngleCritic::score(
  const Trajectory & trajectory,
  const CriticData & data,
  bool & collision)
{
  // This critic doesn't detect collisions
  // collision parameter is not modified

  if (!enabled_ || trajectory.empty() || !data.has_goal_pose) {
    return 0.0;
  }

  // Get goal position and desired heading
  double goal_x = data.goal_pose.position.x;
  double goal_y = data.goal_pose.position.y;
  double goal_yaw = tf2::getYaw(data.goal_pose.orientation);

  // Evaluate at the end of the trajectory
  const auto & final_point = trajectory.back();

  // Compute distance to goal
  double dx = goal_x - final_point.x;
  double dy = goal_y - final_point.y;
  double dist_to_goal = hypot(dx, dy);

  // Only apply cost if close to goal
  if (dist_to_goal > threshold_to_consider_) {
    return 0.0;
  }

  // Compute heading error
  double heading_error = normalizeAngle(goal_yaw - final_point.theta);

  // Cost increases with heading error (normalized to [0, 1])
  // Using squared error for stronger penalty on large misalignments
  double angle_cost = (heading_error * heading_error) / (M_PI * M_PI);

  // Scale cost by proximity to goal (closer = higher weight)
  double proximity_scale = 1.0 - (dist_to_goal / threshold_to_consider_);

  return angle_cost * proximity_scale;
}

}  // namespace pb_omni_pid_pursuit_controller
