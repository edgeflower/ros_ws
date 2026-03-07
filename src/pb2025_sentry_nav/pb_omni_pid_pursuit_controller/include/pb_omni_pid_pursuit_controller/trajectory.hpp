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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__TRAJECTORY_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__TRAJECTORY_HPP_

#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace pb_omni_pid_pursuit_controller
{

/**
 * @struct TrajectoryPoint
 * @brief Single point in a trajectory with state information
 */
struct TrajectoryPoint
{
  double x;          // X position (m)
  double y;          // Y position (m)
  double theta;      // Yaw angle (rad)
  double vx;         // Linear velocity X (m/s)
  double vy;         // Linear velocity Y (m/s)
  double wz;         // Angular velocity (rad/s)
  double time_from_start;  // Time from trajectory start (s)

  TrajectoryPoint()
  : x(0.0), y(0.0), theta(0.0), vx(0.0), vy(0.0), wz(0.0), time_from_start(0.0)
  {
  }

  TrajectoryPoint(double x_, double y_, double theta_)
  : x(x_), y(y_), theta(theta_), vx(0.0), vy(0.0), wz(0.0), time_from_start(0.0)
  {
  }
};

/**
 * @struct Trajectory
 * @brief A trajectory containing a sequence of states
 *
 * Used by MPPI critics to evaluate candidate trajectories.
 */
struct Trajectory
{
  std::vector<TrajectoryPoint> points;

  Trajectory() = default;

  /**
   * @brief Reserve space for a specific number of points
   * @param size Number of points to reserve
   */
  void reserve(size_t size)
  {
    points.reserve(size);
  }

  /**
   * @brief Add a point to the trajectory
   * @param point Point to add
   */
  void push_back(const TrajectoryPoint & point)
  {
    points.push_back(point);
  }

  /**
   * @brief Get the number of points in the trajectory
   * @return Number of points
   */
  size_t size() const
  {
    return points.size();
  }

  /**
   * @brief Check if trajectory is empty
   * @return True if empty
   */
  bool empty() const
  {
    return points.empty();
  }

  /**
   * @brief Clear all points from trajectory
   */
  void clear()
  {
    points.clear();
  }

  /**
   * @brief Get the final point of the trajectory
   * @return Final trajectory point
   */
  const TrajectoryPoint & back() const
  {
    return points.back();
  }

  /**
   * @brief Get a point at specific index
   * @param index Index of point
   * @return Reference to point
   */
  TrajectoryPoint & operator[](size_t index)
  {
    return points[index];
  }

  /**
   * @brief Get a point at specific index (const)
   * @param index Index of point
   * @return Const reference to point
   */
  const TrajectoryPoint & operator[](size_t index) const
  {
    return points[index];
  }
};

/**
 * @struct ControlSample
 * @brief A control input sample for MPPI optimization
 */
struct ControlSample
{
  double vx;          // Linear velocity X (m/s)
  double vy;          // Linear velocity Y (m/s)
  double wz;          // Angular velocity (rad/s)
  double cost;        // Associated trajectory cost
  bool collision;     // Whether trajectory has collision

  ControlSample()
  : vx(0.0), vy(0.0), wz(0.0), cost(0.0), collision(false)
  {
  }

  ControlSample(double vx_, double vy_, double wz_)
  : vx(vx_), vy(vy_), wz(wz_), cost(0.0), collision(false)
  {
  }
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__TRAJECTORY_HPP_
