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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__CRITIC_FUNCTION_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__CRITIC_FUNCTION_HPP_

#include "pb_omni_pid_pursuit_controller/trajectory.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <memory>
#include <string>
#include <vector>

namespace pb_omni_pid_pursuit_controller
{

/**
 * @struct CriticData
 * @brief Context data passed to critics for evaluation
 */
struct CriticData
{
  // Robot state
  geometry_msgs::msg::Pose robot_pose;          // Current robot pose in costmap frame

  // Reference path (in costmap frame)
  nav_msgs::msg::Path reference_path;           // Local reference path

  // Costmap for obstacle checking
  nav2_costmap_2d::Costmap2D * costmap;         // Pointer to costmap

  // Control parameters
  double model_dt;                              // Time step for dynamics
  int time_steps;                               // Number of steps

  // Goal information
  geometry_msgs::msg::Pose goal_pose;           // Goal pose
  bool has_goal_pose;                           // Whether goal is available

  // Control limits
  double v_linear_max;                          // Max linear velocity
  double v_angular_max;                         // Max angular velocity

  CriticData()
  : costmap(nullptr),
    model_dt(0.05),
    time_steps(20),
    has_goal_pose(false),
    v_linear_max(2.0),
    v_angular_max(2.0)
  {
  }
};

/**
 * @class CriticFunction
 * @brief Base class for trajectory evaluation critics in MPPI controller
 *
 * Critics evaluate different aspects of a trajectory:
 * - Path alignment (how well it follows the reference)
 * - Goal approach (heading toward goal)
 * - Obstacle avoidance (costmap costs)
 * - Control preferences (forward motion, smoothness, etc.)
 *
 * Each critic returns a cost score; lower is better.
 * Total cost = sum(weight_i * cost_i) for all enabled critics.
 */
class CriticFunction
{
public:
  /**
   * @brief Constructor
   * @param name Name of the critic
   */
  explicit CriticFunction(const std::string & name)
  : name_(name), weight_(1.0), enabled_(true)
  {
  }

  /**
   * @brief Destructor
   */
  virtual ~CriticFunction() = default;

  /**
   * @brief Configure the critic from ROS parameters
   * @param parent Parent node
   * @param plugin_name Plugin name for parameter namespacing
   * @param critic_name Name of this critic
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::string & critic_name) = 0;

  /**
   * @brief Initialize the critic (called after configure)
   */
  virtual void initialize()
  {
  }

  /**
   * @brief Evaluate a trajectory and return its cost
   *
   * @param trajectory The trajectory to evaluate
   * @param data Context data for evaluation
   * @param collision Output parameter, set to true if trajectory is in collision
   * @return Cost score (lower is better)
   */
  virtual double score(
    const Trajectory & trajectory,
    const CriticData & data,
    bool & collision) = 0;

  /**
   * @brief Check if this critic is enabled
   * @return True if enabled
   */
  bool isEnabled() const
  {
    return enabled_;
  }

  /**
   * @brief Enable or disable this critic
   * @param enabled True to enable
   */
  void setEnabled(bool enabled)
  {
    enabled_ = enabled;
  }

  /**
   * @brief Get the weight for this critic
   * @return Weight multiplier
   */
  double getWeight() const
  {
    return weight_;
  }

  /**
   * @brief Set the weight for this critic
   * @param weight New weight value
   */
  void setWeight(double weight)
  {
    weight_ = weight;
  }

  /**
   * @brief Get the name of this critic
   * @return Critic name
   */
  const std::string & getName() const
  {
    return name_;
  }

protected:
  std::string name_;
  double weight_;
  bool enabled_;

  /**
   * @brief Helper to get parameter with default
   * @param node Node to get parameter from
   * @param param_name Full parameter name
   * @param default_value Default value if not found
   * @return Parameter value
   */
  template<typename T>
  T getParam(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
    const std::string & param_name,
    const T & default_value)
  {
    auto n = node.lock();
    if (!n) {
      return default_value;
    }

    T value;
    try {
      if (n->get_parameter(param_name, value)) {
        return value;
      }
    } catch (...) {
      // Parameter not found or wrong type
    }
    return default_value;
  }
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__CRITICS__CRITIC_FUNCTION_HPP_
