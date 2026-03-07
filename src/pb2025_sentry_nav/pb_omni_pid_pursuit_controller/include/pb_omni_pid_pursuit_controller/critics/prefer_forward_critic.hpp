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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_

#include "pb_omni_pid_pursuit_controller/critics/critic_function.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace pb_omni_pid_pursuit_controller
{

/**
 * @class PreferForwardCritic
 * @brief Critic that penalizes backward and lateral motion
 *
 * For omni-directional robots, this critic encourages forward motion
 * by penalizing backward (negative x) and lateral (y) velocities.
 *
 * The cost is computed based on the velocity components:
 * - Negative vx (backward motion): penalty proportional to magnitude
 * - vy (lateral motion): penalty proportional to magnitude
 *
 * Parameters:
 * - weight: Scaling factor for this critic's cost
 * - power: Power to raise velocity penalties to (higher = more discourage non-forward)
 */
class PreferForwardCritic : public CriticFunction
{
public:
  PreferForwardCritic();
  ~PreferForwardCritic() override = default;

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
   * @brief Evaluate forward motion preference
   * @param trajectory Trajectory to evaluate
   * @param data Context data
   * @param collision Output parameter (not modified by this critic)
   * @return Forward preference cost
   */
  double score(
    const Trajectory & trajectory,
    const CriticData & data,
    bool & collision) override;

private:
  double power_;  // Power for velocity penalty
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_
