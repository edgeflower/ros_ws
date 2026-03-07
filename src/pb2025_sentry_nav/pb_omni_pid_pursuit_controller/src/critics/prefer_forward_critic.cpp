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

#include "pb_omni_pid_pursuit_controller/critics/prefer_forward_critic.hpp"

#include <cmath>
#include <algorithm>

#include "nav2_util/node_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace pb_omni_pid_pursuit_controller
{

PreferForwardCritic::PreferForwardCritic()
: CriticFunction("PreferForwardCritic"),
  power_(1.0)
{
}

void PreferForwardCritic::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::string & critic_name)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in PreferForwardCritic::configure");
  }

  // Declare parameters
  std::string param_prefix = plugin_name + "." + critic_name;

  declare_parameter_if_not_declared(
    node, param_prefix + ".enabled", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, param_prefix + ".weight", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(
    node, param_prefix + ".power", rclcpp::ParameterValue(1));

  // Get parameters
  enabled_ = getParam<bool>(parent, param_prefix + ".enabled", true);
  weight_ = getParam<double>(parent, param_prefix + ".weight", 5.0);
  power_ = getParam<double>(parent, param_prefix + ".power", 1.0);

  // Validate parameters
  if (power_ < 0.5) power_ = 0.5;
  if (power_ > 3.0) power_ = 3.0;
}

double PreferForwardCritic::score(
  const Trajectory & trajectory,
  const CriticData & data,
  bool & collision)
{
  (void)data;  // Unused
  // This critic doesn't detect collisions
  // collision parameter is not modified

  if (!enabled_ || trajectory.empty()) {
    return 0.0;
  }

  double total_penalty = 0.0;
  size_t count = 0;

  // Evaluate the entire trajectory
  for (const auto & point : trajectory.points) {
    // Penalize backward motion (negative vx)
    double backward_penalty = 0.0;
    if (point.vx < 0.0) {
      double p = 1.0;
      for (int j = 0; j < (int)power_; j++) {
        p *= (-point.vx);
      }
      backward_penalty = p;
    }

    // Penalize lateral motion (non-zero vy)
    // Use absolute value since both positive and negative lateral motion is penalized
    double p = 1.0;
    for (int j = 0; j < (int)power_; j++) {
      p *= fabs(point.vy);
    }
    double lateral_penalty = p;

    total_penalty += backward_penalty + lateral_penalty;
    count++;
  }

  if (count > 0) {
    return total_penalty / static_cast<double>(count);
  }
  return 0.0;
}

}  // namespace pb_omni_pid_pursuit_controller
