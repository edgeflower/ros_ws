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

#include "pb_omni_pid_pursuit_controller/fuzzy_controller.hpp"

#include <cmath>
#include <algorithm>

namespace pb_omni_pid_pursuit_controller
{

// Fuzzy rule tables for Kp, Ki, Kd adjustments
// Each entry corresponds to output term for error(row) x error_change(col)
// Numeric values: NB=-3, NM=-2, NS=-1, ZO=0, PS=1, PM=2, PB=3

// Kp rules: error large -> increase Kp, error change large -> decrease Kp
static constexpr int kp_rules_[7][7] = {
  // NB=-3 NM=-2 NS=-1 ZO=0 PS=1 PM=2 PB=3 (ec)
  { 3,    2,    2,    1,    1,    0,    0},   // NB (e)
  { 3,    2,    2,    1,    1,    0,   -1},   // NM
  { 2,    2,    1,    1,    0,   -1,   -1},   // NS
  { 2,    2,    1,    0,   -1,   -2,   -2},   // ZO
  { 1,    1,    0,   -1,   -1,   -2,   -2},   // PS
  { 1,    0,   -1,   -2,   -2,   -2,   -3},   // PM
  { 0,    0,   -2,   -2,   -2,   -3,   -3}    // PB
};

// Ki rules: error large -> decrease Ki, error near zero -> increase Ki
static constexpr int ki_rules_[7][7] = {
  // NB=-3 NM=-2 NS=-1 ZO=0 PS=1 PM=2 PB=3 (ec)
  {-3,   -3,   -2,   -2,   -1,    0,    0},   // NB (e)
  {-3,   -3,   -2,   -1,   -1,    0,    0},   // NM
  {-3,   -2,   -1,   -1,    0,    1,    1},   // NS
  {-2,   -2,   -1,    0,    1,    2,    2},   // ZO
  {-2,   -1,    0,    1,    1,    2,    3},   // PS
  { 0,    0,    1,    2,    2,    3,    3},   // PM
  { 0,    0,    1,    2,    3,    3,    3}    // PB
};

// Kd rules: error change large -> increase Kd, near steady state -> decrease Kd
static constexpr int kd_rules_[7][7] = {
  // NB=-3 NM=-2 NS=-1 ZO=0 PS=1 PM=2 PB=3 (ec)
  { 1,   -1,   -3,   -3,   -3,   -2,    1},   // NB (e)
  { 1,   -1,   -3,   -2,   -2,   -1,    0},   // NM
  { 0,   -1,   -2,   -2,   -1,   -1,    0},   // NS
  { 0,   -1,   -1,   -1,   -1,   -1,    0},   // ZO
  { 0,    0,    0,    0,    0,    0,    0},   // PS
  { 3,    1,    1,    1,    1,    1,    3},   // PM
  { 3,    2,    2,    2,    1,    1,    3}    // PB
};

// Output term values for defuzzification (normalized to [-1, 1])
static constexpr double output_values_[7] = {
  -1.0, -0.66, -0.33, 0.0, 0.33, 0.66, 1.0
};

FuzzyController::FuzzyController(
  double e_max, double ec_max,
  double kp_adjustment_ratio,
  double ki_adjustment_ratio,
  double kd_adjustment_ratio)
: e_max_(e_max),
  ec_max_(ec_max),
  kp_adjustment_ratio_(kp_adjustment_ratio),
  ki_adjustment_ratio_(ki_adjustment_ratio),
  kd_adjustment_ratio_(kd_adjustment_ratio)
{
}

std::array<double, 7> FuzzyController::computeMembershipDegrees(double value) const
{
  std::array<double, 7> memberships;

  for (size_t i = 0; i < 7; ++i) {
    memberships[i] = triangularMF(value, mf_centers_[i], 0.34);
  }

  return memberships;
}

double FuzzyController::centroidDefuzzification(
  const std::array<double, 7> & fired_weights,
  const std::array<double, 7> & firing_strengths) const
{
  double numerator = 0.0;
  double denominator = 0.0;

  for (size_t i = 0; i < 7; ++i) {
    numerator += fired_weights[i];
    denominator += firing_strengths[i];
  }

  return (denominator > 1e-9) ? (numerator / denominator) : 0.0;
}

double FuzzyController::inferKpAdjustment(
  const std::array<double, 7> & e_mf,
  const std::array<double, 7> & ec_mf) const
{
  std::array<double, 7> fired_weights = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> firing_strengths = {0, 0, 0, 0, 0, 0, 0};

  // Mamdani inference: min for implication, max for aggregation
  for (size_t e_idx = 0; e_idx < 7; ++e_idx) {
    for (size_t ec_idx = 0; ec_idx < 7; ++ec_idx) {
      double firing_strength = std::min(e_mf[e_idx], ec_mf[ec_idx]);

      if (firing_strength > 1e-6) {
        int output_term = kp_rules_[e_idx][ec_idx];
        int term_idx = output_term + 3;  // Map [-3, 3] to [0, 6]

        fired_weights[term_idx] = std::max(
          fired_weights[term_idx],
          firing_strength * output_values_[term_idx]);
        firing_strengths[term_idx] = std::max(
          firing_strengths[term_idx],
          firing_strength);
      }
    }
  }

  return centroidDefuzzification(fired_weights, firing_strengths);
}

double FuzzyController::inferKiAdjustment(
  const std::array<double, 7> & e_mf,
  const std::array<double, 7> & ec_mf) const
{
  std::array<double, 7> fired_weights = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> firing_strengths = {0, 0, 0, 0, 0, 0, 0};

  for (size_t e_idx = 0; e_idx < 7; ++e_idx) {
    for (size_t ec_idx = 0; ec_idx < 7; ++ec_idx) {
      double firing_strength = std::min(e_mf[e_idx], ec_mf[ec_idx]);

      if (firing_strength > 1e-6) {
        int output_term = ki_rules_[e_idx][ec_idx];
        int term_idx = output_term + 3;

        fired_weights[term_idx] = std::max(
          fired_weights[term_idx],
          firing_strength * output_values_[term_idx]);
        firing_strengths[term_idx] = std::max(
          firing_strengths[term_idx],
          firing_strength);
      }
    }
  }

  return centroidDefuzzification(fired_weights, firing_strengths);
}

double FuzzyController::inferKdAdjustment(
  const std::array<double, 7> & e_mf,
  const std::array<double, 7> & ec_mf) const
{
  std::array<double, 7> fired_weights = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> firing_strengths = {0, 0, 0, 0, 0, 0, 0};

  for (size_t e_idx = 0; e_idx < 7; ++e_idx) {
    for (size_t ec_idx = 0; ec_idx < 7; ++ec_idx) {
      double firing_strength = std::min(e_mf[e_idx], ec_mf[ec_idx]);

      if (firing_strength > 1e-6) {
        int output_term = kd_rules_[e_idx][ec_idx];
        int term_idx = output_term + 3;

        fired_weights[term_idx] = std::max(
          fired_weights[term_idx],
          firing_strength * output_values_[term_idx]);
        firing_strengths[term_idx] = std::max(
          firing_strengths[term_idx],
          firing_strength);
      }
    }
  }

  return centroidDefuzzification(fired_weights, firing_strengths);
}

void FuzzyController::computeAdjustments(
  double error, double error_change,
  double & dkp, double & dki, double & dkd)
{
  // Normalize inputs to [-1, 1]
  double e_norm = std::clamp(error / e_max_, -1.0, 1.0);
  double ec_norm = std::clamp(error_change / ec_max_, -1.0, 1.0);

  // Compute membership degrees for both inputs
  auto e_mf = computeMembershipDegrees(e_norm);
  auto ec_mf = computeMembershipDegrees(ec_norm);

  // Apply fuzzy rules for each parameter
  dkp = inferKpAdjustment(e_mf, ec_mf);
  dki = inferKiAdjustment(e_mf, ec_mf);
  dkd = inferKdAdjustment(e_mf, ec_mf);

  // Scale by adjustment ratios
  dkp *= kp_adjustment_ratio_;
  dki *= ki_adjustment_ratio_;
  dkd *= kd_adjustment_ratio_;
}

}  // namespace pb_omni_pid_pursuit_controller
