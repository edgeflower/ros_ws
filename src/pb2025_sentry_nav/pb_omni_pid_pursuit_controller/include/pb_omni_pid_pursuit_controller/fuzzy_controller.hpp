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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__FUZZY_CONTROLLER_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__FUZZY_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

namespace pb_omni_pid_pursuit_controller
{

/**
 * @brief Enum for fuzzy linguistic terms
 *
 * Seven linguistic variables for fuzzy sets:
 * NB - Negative Big, NM - Negative Medium, NS - Negative Small
 * ZO - Zero, PS - Positive Small, PM - Positive Medium, PB - Positive Big
 */
enum class FuzzyTerm
{
  NB = 0,  // Negative Big
  NM = 1,  // Negative Medium
  NS = 2,  // Negative Small
  ZO = 3,  // Zero
  PS = 4,  // Positive Small
  PM = 5,  // Positive Medium
  PB = 6   // Positive Big
};

/**
 * @class FuzzyController
 * @brief Mamdani-style fuzzy inference engine for PID parameter adaptation
 *
 * This controller uses fuzzy logic to adjust PID parameters based on:
 * - Error (e): Difference between setpoint and process variable
 * - Error Change (ec): Rate of change of error
 *
 * Output: Adjustments to Kp, Ki, Kd parameters
 *
 * The controller uses:
 * - Triangular membership functions for input/output variables
 * - Mamdani inference (min for implication, max for aggregation)
 * - Centroid defuzzification
 */
class FuzzyController
{
public:
  /**
   * @brief Constructor for FuzzyController
   * @param e_max Maximum error value for normalization
   * @param ec_max Maximum error change value for normalization
   * @param kp_adjustment_ratio Maximum adjustment ratio for Kp (e.g., 0.5 = ±50%)
   * @param ki_adjustment_ratio Maximum adjustment ratio for Ki
   * @param kd_adjustment_ratio Maximum adjustment ratio for Kd
   */
  FuzzyController(
    double e_max = 2.0, double ec_max = 5.0,
    double kp_adjustment_ratio = 0.5,
    double ki_adjustment_ratio = 0.3,
    double kd_adjustment_ratio = 0.4);

  /**
   * @brief Compute fuzzy PID parameter adjustments
   * @param error Current error (setpoint - process_variable)
   * @param error_change Current rate of change of error
   * @param[out] dkp Adjustment to Kp (in range [-1, 1], multiply by base Kp)
   * @param[out] dki Adjustment to Ki (in range [-1, 1], multiply by base Ki)
   * @param[out] dkd Adjustment to Kd (in range [-1, 1], multiply by base Kd)
   */
  void computeAdjustments(
    double error, double error_change,
    double & dkp, double & dki, double & dkd);

  /**
   * @brief Set the maximum error range for normalization
   * @param e_max New maximum error value
   */
  void setErrorMax(double e_max) {e_max_ = e_max;}

  /**
   * @brief Set the maximum error change range for normalization
   * @param ec_max New maximum error change value
   */
  void setErrorChangeMax(double ec_max) {ec_max_ = ec_max;}

  /**
   * @brief Set the Kp adjustment ratio
   * @param ratio New Kp adjustment ratio
   */
  void setKpAdjustmentRatio(double ratio) {kp_adjustment_ratio_ = ratio;}

  /**
   * @brief Set the Ki adjustment ratio
   * @param ratio New Ki adjustment ratio
   */
  void setKiAdjustmentRatio(double ratio) {ki_adjustment_ratio_ = ratio;}

  /**
   * @brief Set the Kd adjustment ratio
   * @param ratio New Kd adjustment ratio
   */
  void setKdAdjustmentRatio(double ratio) {kd_adjustment_ratio_ = ratio;}

private:
  // Normalization bounds
  double e_max_;
  double ec_max_;

  // Output adjustment ratios
  double kp_adjustment_ratio_;
  double ki_adjustment_ratio_;
  double kd_adjustment_ratio_;

  // Membership function centers for normalized input [-1, 1]
  static constexpr std::array<double, 7> mf_centers_ = {
    -1.0, -0.66, -0.33, 0.0, 0.33, 0.66, 1.0
  };

  /**
   * @brief Triangular membership function
   * @param x Input value (normalized to [-1, 1])
   * @param center Center of the triangle
   * @param width Width of the triangle (default: 0.34)
   * @return Membership degree in [0, 1]
   */
  inline double triangularMF(double x, double center, double width = 0.34) const
  {
    return std::max(0.0, 1.0 - std::abs(x - center) / width);
  }

  /**
   * @brief Compute membership degrees for all fuzzy terms
   * @param value Normalized input value [-1, 1]
   * @return Array of membership degrees for NB, NM, NS, ZO, PS, PM, PB
   */
  std::array<double, 7> computeMembershipDegrees(double value) const;

  /**
   * @brief Fuzzy inference rule for Kp adjustment
   *
   * Rule strategy:
   * - When error is large: Increase Kp (fast response)
   * - When error change is large: Decrease Kp (reduce overshoot)
   * - When error is near zero: Small adjustment (maintain stability)
   *
   * @param e_mf Membership degrees for error
   * @param ec_mf Membership degrees for error change
   * @return Adjusted Kp value in [-1, 1]
   */
  double inferKpAdjustment(
    const std::array<double, 7> & e_mf,
    const std::array<double, 7> & ec_mf) const;

  /**
   * @brief Fuzzy inference rule for Ki adjustment
   *
   * Rule strategy:
   * - When error is large: Decrease Ki (avoid integral windup)
   * - When error is near zero: Increase Ki (eliminate steady-state error)
   * - When error change is large: Decrease Ki (reduce overshoot)
   *
   * @param e_mf Membership degrees for error
   * @param ec_mf Membership degrees for error change
   * @return Adjusted Ki value in [-1, 1]
   */
  double inferKiAdjustment(
    const std::array<double, 7> & e_mf,
    const std::array<double, 7> & ec_mf) const;

  /**
   * @brief Fuzzy inference rule for Kd adjustment
   *
   * Rule strategy:
   * - When error change is large: Increase Kd (enhance damping)
   * - When error is large: Slight increase (predictive action)
   * - When near steady state: Decrease Kd (reduce noise sensitivity)
   *
   * @param e_mf Membership degrees for error
   * @param ec_mf Membership degrees for error change
   * @return Adjusted Kd value in [-1, 1]
   */
  double inferKdAdjustment(
    const std::array<double, 7> & e_mf,
    const std::array<double, 7> & ec_mf) const;

  /**
   * @brief Centroid defuzzification
   * @param fired_weights Array of (firing_strength * output_term_value)
   * @param firing_strengths Array of firing strengths
   * @return Defuzzified output value
   */
  double centroidDefuzzification(
    const std::array<double, 7> & fired_weights,
    const std::array<double, 7> & firing_strengths) const;
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__FUZZY_CONTROLLER_HPP_
