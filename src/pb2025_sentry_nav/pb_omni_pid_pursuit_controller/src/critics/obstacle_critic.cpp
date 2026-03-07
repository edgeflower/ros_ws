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

#include "pb_omni_pid_pursuit_controller/critics/obstacle_critic.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#ifndef HUGE_VAL
#define HUGE_VAL (1.0 / 0.0)
#endif

#ifndef isfinite
#define isfinite(x) std::isfinite(x)
#endif

using nav2_util::declare_parameter_if_not_declared;
using namespace nav2_costmap_2d;  // NOLINT

namespace pb_omni_pid_pursuit_controller
{

ObstacleCritic::ObstacleCritic()
: CriticFunction("ObstacleCritic"),
  inflation_radius_(0.5),
  consider_footprint_(false)
{
}

void ObstacleCritic::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::string & critic_name)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in ObstacleCritic::configure");
  }

  // Declare parameters
  std::string param_prefix = plugin_name + "." + critic_name;

  declare_parameter_if_not_declared(
    node, param_prefix + ".enabled", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, param_prefix + ".weight", rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(
    node, param_prefix + ".inflation_radius", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, param_prefix + ".consider_footprint", rclcpp::ParameterValue(false));

  // Get parameters
  enabled_ = getParam<bool>(parent, param_prefix + ".enabled", true);
  weight_ = getParam<double>(parent, param_prefix + ".weight", 10.0);
  inflation_radius_ = getParam<double>(parent, param_prefix + ".inflation_radius", 0.5);
  consider_footprint_ = getParam<bool>(parent, param_prefix + ".consider_footprint", false);

  // Validate parameters
  if (inflation_radius_ < 0.0) inflation_radius_ = 0.0;
}

double ObstacleCritic::getCostAtPosition(
  double wx, double wy,
  nav2_costmap_2d::Costmap2D * costmap) const
{
  if (!costmap) {
    return 0.0;
  }

  unsigned int mx, my;
  if (!costmap->worldToMap(wx, wy, mx, my)) {
    // Outside costmap bounds - treat as free space
    return 0.0;
  }

  unsigned char cost = costmap->getCost(mx, my);

  // Check if in collision
  if (cost >= INSCRIBED_INFLATED_OBSTACLE) {
    return HUGE_VAL;
  }

  // Normalize cost to [0, 1]
  return static_cast<double>(cost) / 254.0;
}

double ObstacleCritic::score(
  const Trajectory & trajectory,
  const CriticData & data,
  bool & collision)
{
  collision = false;

  if (!enabled_ || trajectory.empty() || !data.costmap) {
    return 0.0;
  }

  double max_cost = 0.0;

  for (const auto & point : trajectory.points) {
    double cost = getCostAtPosition(point.x, point.y, data.costmap);

    if (!isfinite(cost)) {
      // Collision detected
      collision = true;
      return HUGE_VAL;
    }

    if (cost > max_cost) {
      max_cost = cost;
    }
  }

  return max_cost;
}

}  // namespace pb_omni_pid_pursuit_controller
