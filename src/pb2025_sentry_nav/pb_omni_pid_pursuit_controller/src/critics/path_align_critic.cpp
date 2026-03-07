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

#include "pb_omni_pid_pursuit_controller/critics/path_align_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_util/node_utils.hpp"

#ifndef HUGE_VAL
#define HUGE_VAL (1.0 / 0.0)
#endif

using nav2_util::declare_parameter_if_not_declared;

namespace pb_omni_pid_pursuit_controller
{

PathAlignCritic::PathAlignCritic()
: CriticFunction("PathAlignCritic"),
  power_(1.0),
  threshold_(0.5)
{
}

void PathAlignCritic::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::string & critic_name)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in PathAlignCritic::configure");
  }

  // Declare parameters
  std::string param_prefix = plugin_name + "." + critic_name;

  declare_parameter_if_not_declared(
    node, param_prefix + ".enabled", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, param_prefix + ".weight", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(
    node, param_prefix + ".power", rclcpp::ParameterValue(1));
  declare_parameter_if_not_declared(
    node, param_prefix + ".threshold", rclcpp::ParameterValue(0.5));

  // Get parameters
  enabled_ = getParam<bool>(parent, param_prefix + ".enabled", true);
  weight_ = getParam<double>(parent, param_prefix + ".weight", 5.0);
  power_ = getParam<double>(parent, param_prefix + ".power", 1.0);
  threshold_ = getParam<double>(parent, param_prefix + ".threshold", 0.5);

  // Validate parameters
  if (power_ < 0.5) power_ = 0.5;
  if (power_ > 3.0) power_ = 3.0;
  if (threshold_ < 0.0) threshold_ = 0.0;
}

double PathAlignCritic::pointToSegmentDistance(
  double px, double py,
  double x1, double y1,
  double x2, double y2) const
{
  // Vector from p1 to p2
  double dx = x2 - x1;
  double dy = y2 - y1;

  // Vector from p1 to point
  double lx = px - x1;
  double ly = py - y1;

  // Projection of point onto line (parameterized by t)
  double segment_length_sq = dx * dx + dy * dy;

  double t;
  if (segment_length_sq < 1e-9) {
    // Segment is a point
    t = 0.0;
  } else {
    t = (lx * dx + ly * dy) / segment_length_sq;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
  }

  // Closest point on segment
  double cx = x1 + t * dx;
  double cy = y1 + t * dy;

  // Distance to closest point
  double dist_x = px - cx;
  double dist_y = py - cy;
  return hypot(dist_x, dist_y);
}

double PathAlignCritic::findClosestPointOnPath(
  double x, double y,
  const nav_msgs::msg::Path & path,
  double & closest_x,
  double & closest_y) const
{
  if (path.poses.empty()) {
    closest_x = x;
    closest_y = y;
    return 0.0;
  }

  double min_dist = HUGE_VAL;
  closest_x = path.poses[0].pose.position.x;
  closest_y = path.poses[0].pose.position.y;

  // Check distance to each path vertex
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & pose = path.poses[i].pose.position;
    double dist = hypot(x - pose.x, y - pose.y);
    if (dist < min_dist) {
      min_dist = dist;
      closest_x = pose.x;
      closest_y = pose.y;
    }
  }

  // Check distance to each segment
  for (size_t i = 0; i < path.poses.size() - 1; ++i) {
    const auto & p1 = path.poses[i].pose.position;
    const auto & p2 = path.poses[i + 1].pose.position;

    double dist = pointToSegmentDistance(x, y, p1.x, p1.y, p2.x, p2.y);
    if (dist < min_dist) {
      min_dist = dist;
      // Closest point on segment is computed inside pointToSegmentDistance
      // but we need to compute it again to get the coordinates
      double dx = p2.x - p1.x;
      double dy = p2.y - p1.y;
      double lx = x - p1.x;
      double ly = y - p1.y;
      double segment_length_sq = dx * dx + dy * dy;

      if (segment_length_sq > 1e-9) {
        double t = (lx * dx + ly * dy) / segment_length_sq;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        closest_x = p1.x + t * dx;
        closest_y = p1.y + t * dy;
      }
    }
  }

  return min_dist;
}

double PathAlignCritic::score(
  const Trajectory & trajectory,
  const CriticData & data,
  bool & collision)
{
  collision = false;

  if (!enabled_ || trajectory.empty() || data.reference_path.poses.empty()) {
    return 0.0;
  }

  double total_cost = 0.0;
  size_t evaluation_points = 0;

  // Sample points along trajectory for evaluation
  // Use every few points to reduce computation
  size_t step = trajectory.size() / 10;
  if (step < 1) step = 1;

  for (size_t i = 0; i < trajectory.size(); i += step) {
    const auto & point = trajectory[i];

    double closest_x, closest_y;
    double dist = findClosestPointOnPath(
      point.x, point.y, data.reference_path, closest_x, closest_y);

    // Apply threshold - distances below threshold have zero cost
    if (dist > threshold_) {
      double excess_dist = dist - threshold_;
      double p = 1.0;
      for (int j = 0; j < (int)power_; j++) {
        p *= excess_dist;
      }
      total_cost += p;
    }
    evaluation_points++;
  }

  if (evaluation_points > 0) {
    return total_cost / static_cast<double>(evaluation_points);
  }
  return 0.0;
}

}  // namespace pb_omni_pid_pursuit_controller
