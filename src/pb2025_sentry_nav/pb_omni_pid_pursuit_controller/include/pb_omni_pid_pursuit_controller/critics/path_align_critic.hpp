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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_

#include "pb_omni_pid_pursuit_controller/critics/critic_function.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <vector>

namespace pb_omni_pid_pursuit_controller
{

/**
 * @class PathAlignCritic
 * @brief Critic that penalizes deviation from the reference path
 *
 * This critic evaluates how well a trajectory follows the reference path
 * by computing distances from trajectory points to the path segments.
 *
 * The cost is computed as:
 * - For each trajectory point, find the closest point on the reference path
 * - Compute distance to that point
 * - Apply power function for non-linear scaling
 * - Sum (or average) the costs
 *
 * Parameters:
 * - weight: Scaling factor for this critic's cost
 * - power: Power to raise distances to (higher = more penalize large deviations)
 * - threshold: Distance threshold (points within threshold have zero cost)
 */
class PathAlignCritic : public CriticFunction
{
public:
  PathAlignCritic();
  ~PathAlignCritic() override = default;

  /**
   * @brief Configure from ROS parameters
   * @param parent Parent node
   * @param plugin_name Plugin name for namespacing
   * @param critic_name Critic name
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::string & critic_name) override;

  /**
   * @brief Evaluate trajectory alignment to path
   * @param trajectory Trajectory to evaluate
   * @param data Context data
   * @param collision Output parameter (set to true if collision)
   * @return Alignment cost
   */
  double score(
    const Trajectory & trajectory,
    const CriticData & data,
    bool & collision) override;

private:
  double power_;           // Power for distance cost
  double threshold_;       // Distance threshold for zero cost

  /**
   * @brief Find the closest point on the reference path to a given point
   * @param x Query point X
   * @param y Query point Y
   * @param path Reference path
   * @param closest_x Output closest X on path
   * @param closest_y Output closest Y on path
   * @return Distance to closest point
   */
  double findClosestPointOnPath(
    double x, double y,
    const nav_msgs::msg::Path & path,
    double & closest_x,
    double & closest_y) const;

  /**
   * @brief Compute distance from point to line segment
   * @param px Point X
   * @param py Point Y
   * @param x1 Segment start X
   * @param y1 Segment start Y
   * @param x2 Segment end X
   * @param y2 Segment end Y
   * @return Distance to segment
   */
  double pointToSegmentDistance(
    double px, double py,
    double x1, double y1,
    double x2, double y2) const;
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_
