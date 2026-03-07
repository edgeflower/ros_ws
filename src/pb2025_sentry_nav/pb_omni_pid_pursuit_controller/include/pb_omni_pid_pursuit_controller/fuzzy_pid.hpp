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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__FUZZY_PID_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__FUZZY_PID_HPP_

#include "pb_omni_pid_pursuit_controller/pid.hpp"
#include "pb_omni_pid_pursuit_controller/fuzzy_controller.hpp"
#include <memory>
#include <cmath>

namespace pb_omni_pid_pursuit_controller
{

/**
 * @class FuzzyPID
 * @brief PID controller with fuzzy logic based parameter adaptation
 *
 * This controller extends the standard PID by using fuzzy logic to dynamically
 * adjust Kp, Ki, Kd parameters based on:
 * - Error magnitude (e)
 * - Error change rate (ec)
 *
 * The fuzzy adaptation helps:
 * - Fast response when error is large (high Kp, low Ki)
 * - Reduce overshoot when error change is high (low Kp, high Kd)
 * - Eliminate steady-state error when near target (high Ki)
 *
 * Usage is identical to standard PID, but gains are automatically adjusted.
 */
class FuzzyPID
{
public:
  /**
   * @brief Constructor for FuzzyPID
   *
   * @param dt Loop interval time (seconds)
   * @param max Maximum output value
   * @param min Minimum output value
   * @param kp Base proportional gain
   * @param kd Base derivative gain
   * @param ki Base integral gain
   * @param e_max Maximum error for normalization (default: 2.0)
   * @param ec_max Maximum error change for normalization (default: 5.0)
   * @param kp_adjustment_ratio Kp adjustment range ratio (default: 0.5 = ±50%)
   * @param ki_adjustment_ratio Ki adjustment range ratio (default: 0.3 = ±30%)
   * @param kd_adjustment_ratio Kd adjustment range ratio (default: 0.4 = ±40%)
   */
  FuzzyPID(
    double dt, double max, double min, double kp, double kd, double ki,
    double e_max = 2.0,
    double ec_max = 5.0,
    double kp_adjustment_ratio = 0.5,
    double ki_adjustment_ratio = 0.3,
    double kd_adjustment_ratio = 0.4);

  /**
   * @brief Compute PID output with fuzzy parameter adaptation
   *
   * @param set_point Target value
   * @param pv Current process variable
   * @return Control output (clamped to [min, max])
   */
  double calculate(double set_point, double pv);

  /**
   * @brief Reset internal PID state (error and integral)
   */
  void reset();

  /**
   * @brief Set the base PID gains
   * @param kp Base proportional gain
   * @param ki Base integral gain
   * @param kd Base derivative gain
   */
  void setBaseGains(double kp, double ki, double kd);

  /**
   * @brief Get the current (adapted) PID gains
   * @param kp Output current Kp
   * @param ki Output current Ki
   * @param kd Output current Kd
   */
  void getCurrentGains(double & kp, double & ki, double & kd) const;

  /**
   * @brief Enable or disable fuzzy adaptation
   * @param enable True to enable fuzzy adaptation, false for standard PID
   */
  void setFuzzyEnabled(bool enable) {fuzzy_enabled_ = enable;}

  /**
   * @brief Check if fuzzy adaptation is enabled
   * @return True if fuzzy adaptation is enabled
   */
  bool isFuzzyEnabled() const {return fuzzy_enabled_;}

  /**
   * @brief Set the integral error directly (for PID compatibility)
   * @param sum_error Value to set integral to
   */
  void setSumError(double sum_error);

  /**
   * @brief Get access to the underlying fuzzy controller
   * @return Pointer to the fuzzy controller
   */
  std::shared_ptr<FuzzyController> getFuzzyController() {return fuzzy_;}

private:
  // Base PID parameters
  double kp_base_;
  double ki_base_;
  double kd_base_;

  // Current adapted parameters
  double kp_current_;
  double ki_current_;
  double kd_current_;

  // Internal standard PID controller
  std::unique_ptr<PID> pid_;

  // Fuzzy logic controller for adaptation
  std::shared_ptr<FuzzyController> fuzzy_;

  // Fuzzy adaptation enable flag
  bool fuzzy_enabled_;

  // Previous error for computing error change rate
  double prev_error_;

  // Control period
  double dt_;
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__FUZZY_PID_HPP_
