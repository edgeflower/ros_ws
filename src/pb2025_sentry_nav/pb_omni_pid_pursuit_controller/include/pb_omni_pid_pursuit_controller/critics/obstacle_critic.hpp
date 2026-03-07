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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__OBSTACLE_CRITIC_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__OBSTACLE_CRITIC_HPP_

#include "pb_omni_pid_pursuit_controller/critics/critic_function.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace pb_omni_pid_pursuit_controller
{

/**
 * @class ObstacleCritic
 * @brief Critic that evaluates collision risk using the costmap
 *
 * This critic checks the costmap at each trajectory point to detect
 * potential collisions. Points in lethal or inscribed inflated obstacle
 * zones are marked as collisions.
 *
 * The cost returned is the maximum costmap cost encountered along the trajectory.
 *
 * Parameters:
 * - weight: Scaling factor for this critic's cost
 * - inflation_radius: Additional safety margin around obstacles (in meters)
 * - consider_footprint: Whether to consider the robot's footprint (default: false for point model)
 */
class ObstacleCritic : public CriticFunction
{
public:
  ObstacleCritic();
  ~ObstacleCritic() override = default;

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
   * @brief Evaluate obstacle collision risk
   * @param trajectory Trajectory to evaluate
   * @param data Context data (must have valid costmap)
   * @param collision Output parameter, set to true if collision detected
   * @return Obstacle cost
   */
  double score(
    const Trajectory & trajectory,
    const CriticData & data,
    bool & collision) override;

private:
  double inflation_radius_;   // Additional safety margin (meters)
  bool consider_footprint_;   // Whether to consider robot footprint

  /**
   * @brief Get costmap cost at a world position
   * @param wx World X coordinate
   * @param wy World Y coordinate
   * @param costmap Pointer to costmap
   * @return Cost value (0-253, or infinity if outside map)
   */
  double getCostAtPosition(
    double wx, double wy,
    nav2_costmap_2d::Costmap2D * costmap) const;
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__OBSTACLE_CRITIC_HPP_
