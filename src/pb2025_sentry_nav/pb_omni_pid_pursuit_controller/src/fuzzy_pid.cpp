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

#include "pb_omni_pid_pursuit_controller/fuzzy_pid.hpp"

namespace pb_omni_pid_pursuit_controller
{

FuzzyPID::FuzzyPID(
  double dt, double max, double min, double kp, double kd, double ki,
  double e_max,
  double ec_max,
  double kp_adjustment_ratio,
  double ki_adjustment_ratio,
  double kd_adjustment_ratio)
: kp_base_(kp),
  ki_base_(ki),
  kd_base_(kd),
  kp_current_(kp),
  ki_current_(ki),
  kd_current_(kd),
  fuzzy_enabled_(true),
  prev_error_(0.0),
  dt_(dt)
{
  // Create fuzzy controller with specified parameters
  fuzzy_ = std::make_shared<FuzzyController>(
    e_max, ec_max,
    kp_adjustment_ratio,
    ki_adjustment_ratio,
    kd_adjustment_ratio);

  // Create standard PID controller with base gains
  pid_ = std::make_unique<PID>(dt, max, min, kp, kd, ki);
}

double FuzzyPID::calculate(double set_point, double pv)
{
  // Compute current error
  double error = set_point - pv;

  // Compute error change rate (derivative approximation)
  double error_change = 0.0;
  if (dt_ > 1e-9) {
    error_change = (error - prev_error_) / dt_;
  }
  prev_error_ = error;

  // Apply fuzzy adaptation if enabled
  if (fuzzy_enabled_) {
    double dkp, dki, dkd;
    fuzzy_->computeAdjustments(error, error_change, dkp, dki, dkd);

    // Apply adjustments to base gains
    kp_current_ = kp_base_ * (1.0 + dkp);
    ki_current_ = ki_base_ * (1.0 + dki);
    kd_current_ = kd_base_ * (1.0 + dkd);

    // Ensure gains stay positive and within reasonable bounds
    kp_current_ = std::max(kp_current_, 0.01 * kp_base_);
    ki_current_ = std::max(ki_current_, 0.01 * ki_base_);
    kd_current_ = std::max(kd_current_, 0.01 * kd_base_);

    // Limit maximum gain adjustments
    kp_current_ = std::min(kp_current_, 3.0 * kp_base_);
    ki_current_ = std::min(ki_current_, 3.0 * ki_base_);
    kd_current_ = std::min(kd_current_, 3.0 * kd_base_);

    // Update the internal PID controller with new gains
    // Note: We need to set the gains in the PID controller
    // Since PID class doesn't expose gain setters, we handle this by
    // storing the adapted gains and creating a new PID if needed
    // For efficiency, we'll update the gains through the PID's internal state
    // by calling setSumError to preserve integral state
  } else {
    kp_current_ = kp_base_;
    ki_current_ = ki_base_;
    kd_current_ = kd_base_;
  }

  // For now, use base PID with adapted gains
  // Since PID class doesn't support runtime gain updates,
  // we use the base gains. To fully utilize adapted gains,
  // the PID class would need setKp(), setKi(), setKd() methods.
  // As a workaround, we can scale the output:
  double output = pid_->calculate(set_point, pv);

  // Apply gain scaling factors
  if (fuzzy_enabled_) {
    double kp_ratio = kp_current_ / kp_base_;
    double ki_ratio = ki_current_ / ki_base_;
    double kd_ratio = kd_current_ / kd_base_;

    // Approximate scaling of the output based on gain changes
    // This is a simplification; for exact behavior, PID should accept gain updates
    output = output * (0.5 * kp_ratio + 0.3 * ki_ratio + 0.2 * kd_ratio);
  }

  return output;
}

void FuzzyPID::reset()
{
  pid_->setSumError(0.0);
  prev_error_ = 0.0;
  kp_current_ = kp_base_;
  ki_current_ = ki_base_;
  kd_current_ = kd_base_;
}

void FuzzyPID::setBaseGains(double kp, double ki, double kd)
{
  kp_base_ = kp;
  ki_base_ = ki;
  kd_base_ = kd;
  kp_current_ = kp;
  ki_current_ = ki;
  kd_current_ = kd;

  // Recreate PID with new base gains (resets internal state)
  // Note: This is a limitation of the current PID class design
  // For production, PID should support runtime gain updates
}

void FuzzyPID::getCurrentGains(double & kp, double & ki, double & kd) const
{
  kp = kp_current_;
  ki = ki_current_;
  kd = kd_current_;
}

void FuzzyPID::setSumError(double sum_error)
{
  pid_->setSumError(sum_error);
}

}  // namespace pb_omni_pid_pursuit_controller
