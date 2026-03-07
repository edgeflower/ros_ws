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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__GOAL_ANGLE_CRITIC_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__GOAL_ANGLE_CRITIC_HPP_

#include "pb_omni_pid_pursuit_controller/critics/critic_function.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <tf2/utils.h>

namespace pb_omni_pid_pursuit_controller
{

/**
 * @class GoalAngleCritic
 * @brief Critic that penalizes heading misalignment when approaching the goal
 *
 * This critic evaluates how well the robot's heading aligns with the goal's
 * desired heading when the robot is close to the goal.
 *
 * The cost is only applied when the robot is within `threshold_to_consider`
 * distance of the goal. The cost increases with the angle difference between
 * the robot's heading and the goal's desired heading.
 *
 * Parameters:
 * - weight: Scaling factor for this critic's cost
 * - threshold_to_consider: Distance from goal to start applying this critic
 */
class GoalAngleCritic : public CriticFunction
{
public:
  GoalAngleCritic();
  ~GoalAngleCritic() override = default;

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
   * @brief Evaluate goal angle alignment
   * @param trajectory Trajectory to evaluate
   * @param data Context data (must have goal_pose set)
   * @param collision Output parameter (not modified by this critic)
   * @return Angle alignment cost
   */
  double score(
    const Trajectory & trajectory,
    const CriticData & data,
    bool & collision) override;

private:
  double threshold_to_consider_;  // Distance from goal to activate critic

  /**
   * @brief Normalize angle to [-pi, pi]
   * @param angle Input angle
   * @return Normalized angle
   */
  inline double normalizeAngle(double angle) const
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__GOAL_ANGLE_CRITIC_HPP_
