// Copyright 2025 Shenzhen Beijing Moscow University Polar Bear Robotics Team
//
// CBF Safety Layer Implementation

#include "pb_omni_pid_pursuit_controller/cbf_safety_layer.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

namespace pb_omni_pid_pursuit_controller
{

CBFSafetyLayer::CBFSafetyLayer()
{
  X_.setZero();
}

void CBFSafetyLayer::configure(const CBFSafetyConfig & config)
{
  config_ = config;
}

void CBFSafetyLayer::setRobotState(double x, double y, double theta, double v)
{
  X_ << x, y, theta, v;
}

void CBFSafetyLayer::setObstacles(const std::vector<Obstacle> & obstacles)
{
  obstacles_ = obstacles;
}

Eigen::Vector3d CBFSafetyLayer::filterControl(const Eigen::Vector3d & u_ref)
{
  // If no obstacles, return nominal control
  if (obstacles_.empty()) {
    return u_ref;
  }

  // Filter obstacles by distance
  std::vector<Obstacle> nearby_obstacles;
  for (const auto & obs : obstacles_) {
    double dx = obs.x - X_[0];
    double dy = obs.y - X_[1];
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < config_.max_obstacle_dist) {
      nearby_obstacles.push_back(obs);
    }
  }

  if (nearby_obstacles.empty()) {
    return u_ref;
  }

  // Limit number of obstacles
  std::sort(nearby_obstacles.begin(), nearby_obstacles.end(),
    [this](const Obstacle & a, const Obstacle & b) {
      double da = std::hypot(a.x - X_[0], a.y - X_[1]);
      double db = std::hypot(b.x - X_[0], b.y - X_[1]);
      return da < db;
    });

  if (nearby_obstacles.size() > static_cast<size_t>(config_.max_num_obstacles)) {
    nearby_obstacles.resize(config_.max_num_obstacles);
  }

  // Build QP constraints: A*u <= b
  // CBF constraint: dh/dx * g(x) * u <= dh/dx * f(x) + alpha * h
  int num_constraints = nearby_obstacles.size();
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_constraints, 3);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(num_constraints);

  for (size_t i = 0; i < nearby_obstacles.size(); ++i) {
    double h;
    Eigen::RowVector4d dh_dx;

    computeCBF(nearby_obstacles[i], h, dh_dx);

    // Kinematic bicycle model: g(x) = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1]
    // But for omni, we use: dx/dt = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1] * [vx; vy; wz]
    double cos_theta = std::cos(X_[2]);
    double sin_theta = std::sin(X_[2]);

    // dh_dx * g(x) for omni-directional robot
    Eigen::RowVector3d dh_dx_g;
    dh_dx_g[0] = dh_dx[0] * cos_theta + dh_dx[1] * sin_theta;  // derivative w.r.t. vx
    dh_dx_g[1] = -dh_dx[0] * sin_theta + dh_dx[1] * cos_theta; // derivative w.r.t. vy
    dh_dx_g[2] = dh_dx[3];  // derivative w.r.t. wz (angular only affects heading)

    // Uncontrolled dynamics f(x) for omni (when u = 0, position doesn't change from control)
    Eigen::Vector4d f = Eigen::Vector4d::Zero();  // No drift in omni model

    double dh_dx_f = dh_dx * f;

    // Constraint: dh_dx * g * u <= dh_dx * f + alpha * h
    A.row(i) = dh_dx_g;
    b[i] = dh_dx_f + config_.alpha * h;
  }

  // Solve QP
  Eigen::Vector3d u_safe = solveQP(u_ref, A, b);

  return u_safe;
}

void CBFSafetyLayer::computeCBF(
  const Obstacle & obs,
  double & h,
  Eigen::RowVector4d & dh_dx) const
{
  if (config_.cbf_type == CBFType::C3BF) {
    computeC3BF(obs, h, dh_dx);
  } else {
    computeDPCBF(obs, h, dh_dx);
  }
}

void CBFSafetyLayer::computeC3BF(
  const Obstacle & obs,
  double & h,
  Eigen::RowVector4d & dh_dx) const
{
  // Collision Cone CBF for kinematic bicycle / omni robot
  double theta = X_[2];
  double v = X_[3];

  // Relative position and velocity
  double p_rel_x = obs.x - X_[0];
  double p_rel_y = obs.y - X_[1];
  double v_rel_x = obs.vx - v * std::cos(theta);
  double v_rel_y = obs.vy - v * std::sin(theta);

  double p_rel_mag = std::hypot(p_rel_x, p_rel_y);
  double v_rel_mag = std::hypot(v_rel_x, v_rel_y);

  // Combined radius with safety margin
  double R = (obs.radius + config_.robot_radius) * config_.safety_margin;

  // Compute cos_phi safely
  double eps = 1e-6;
  double cal_max = std::max(p_rel_mag * p_rel_mag - R * R, eps);
  double sqrt_term = std::sqrt(cal_max);
  double cos_phi = sqrt_term / (p_rel_mag + eps);

  // CBF value h(x)
  h = p_rel_x * v_rel_x + p_rel_y * v_rel_y + p_rel_mag * v_rel_mag * cos_phi;

  // Gradient dh/dx
  dh_dx = Eigen::RowVector4d::Zero();

  double denom = sqrt_term + eps;
  double v_rel_mag_denom = (v_rel_mag > eps) ? v_rel_mag : eps;

  dh_dx[0] = -v_rel_x - v_rel_mag * p_rel_x / denom;
  dh_dx[1] = -v_rel_y - v_rel_mag * p_rel_y / denom;

  // Partial derivatives w.r.t. theta and v
  double dv_rel_dtheta = v * std::sin(theta) * p_rel_x - v * std::cos(theta) * p_rel_y;
  double dv_rel_dv_term = (sqrt_term + eps) / v_rel_mag_denom *
    (v - (obs.vx * std::cos(theta) + obs.vy * std::sin(theta)));

  dh_dx[2] = dv_rel_dtheta + denom / v_rel_mag_denom *
    (v * (obs.vx * std::sin(theta) - obs.vy * std::cos(theta)));

  dh_dx[3] = -std::cos(theta) * p_rel_x - std::sin(theta) * p_rel_y + dv_rel_dv_term;
}

void CBFSafetyLayer::computeDPCBF(
  const Obstacle & obs,
  double & h,
  Eigen::RowVector4d & dh_dx) const
{
  // Dynamic Parabolic CBF
  double theta = X_[2];
  double v = X_[3];

  // Relative position
  double p_rel_x = obs.x - X_[0];
  double p_rel_y = obs.y - X_[1];

  // Relative velocity
  double v_rel_x = obs.vx - v * std::cos(theta);
  double v_rel_y = obs.vy - v * std::sin(theta);

  double p_rel_mag = std::hypot(p_rel_x, p_rel_y);
  double v_rel_mag = std::hypot(v_rel_x, v_rel_y);

  // Combined radius with safety margin
  double R = (obs.radius + config_.robot_radius) * config_.safety_margin;

  // Rotation angle to align with obstacle direction
  double rot_angle = std::atan2(p_rel_y, p_rel_x);

  // Rotate relative velocity to obstacle frame
  double cos_rot = std::cos(rot_angle);
  double sin_rot = std::sin(rot_angle);
  double v_rel_new_x = cos_rot * v_rel_x + sin_rot * v_rel_y;
  double v_rel_new_y = -sin_rot * v_rel_x + cos_rot * v_rel_y;

  // Safe distance
  double eps = 1e-6;
  double d_safe = std::max(p_rel_mag * p_rel_mag - R * R, eps);

  // DPCBF parameters
  double sqrt_term = std::sqrt(config_.safety_margin * config_.safety_margin - 1.0);
  double lambda = config_.k_lambda * std::sqrt(d_safe) / v_rel_mag * sqrt_term / R;
  double mu = config_.k_mu * std::sqrt(d_safe) * sqrt_term / R;

  // CBF value h(x)
  h = v_rel_new_x + lambda * v_rel_new_y * v_rel_new_y + mu;

  // Gradient dh/dx (simplified, use numerical approximation for complex terms)
  dh_dx = Eigen::RowVector4d::Zero();

  // For robust implementation, use numerical gradient
  double delta = 1e-4;
  Eigen::Vector4d X_plus;

  for (int i = 0; i < 4; ++i) {
    X_plus = X_;
    X_plus[i] += delta;

    double theta_plus = X_plus[2];
    double v_plus = X_plus[3];

    double p_rel_x_same = obs.x - X_plus[0];
    double p_rel_y_same = obs.y - X_plus[1];

    double v_rel_x_plus = obs.vx - v_plus * std::cos(theta_plus);
    double v_rel_y_plus = obs.vy - v_plus * std::sin(theta_plus);

    double p_rel_mag_plus = std::hypot(p_rel_x_same, p_rel_y_same);
    double v_rel_mag_plus = std::hypot(v_rel_x_plus, v_rel_y_plus);

    double rot_angle_plus = std::atan2(p_rel_y_same, p_rel_x_same);
    double cos_rot_plus = std::cos(rot_angle_plus);
    double sin_rot_plus = std::sin(rot_angle_plus);

    double v_rel_new_x_plus = cos_rot_plus * v_rel_x_plus + sin_rot_plus * v_rel_y_plus;
    double v_rel_new_y_plus = -sin_rot_plus * v_rel_x_plus + cos_rot_plus * v_rel_y_plus;

    double R_plus = (obs.radius + config_.robot_radius) * config_.safety_margin;
    double d_safe_plus = std::max(p_rel_mag_plus * p_rel_mag_plus - R_plus * R_plus, eps);
    double sqrt_term_plus = std::sqrt(config_.safety_margin * config_.safety_margin - 1.0);

    double lambda_plus = config_.k_lambda * std::sqrt(d_safe_plus) / v_rel_mag_plus * sqrt_term_plus / R_plus;
    double mu_plus = config_.k_mu * std::sqrt(d_safe_plus) * sqrt_term_plus / R_plus;

    double h_plus = v_rel_new_x_plus + lambda_plus * v_rel_new_y_plus * v_rel_new_y_plus + mu_plus;

    dh_dx[i] = (h_plus - h) / delta;
  }
}

void CBFSafetyLayer::computeDiscreteCBF(
  const Eigen::Vector4d & x_k,
  const Eigen::Vector3d & u_k,
  const Obstacle & obs,
  double & h_k,
  double & d_h) const
{
  // Compute next state
  Eigen::Vector4d x_k1 = stepDynamics(x_k, u_k);

  // Compute h at k and k+1
  double h_k1;
  Eigen::RowVector4d dummy;

  // Temporarily modify state for computation
  Eigen::Vector4d original_X = const_cast<CBFSafetyLayer*>(this)->X_;
  const_cast<CBFSafetyLayer*>(this)->X_ = x_k;
  computeCBF(obs, h_k, dummy);

  const_cast<CBFSafetyLayer*>(this)->X_ = x_k1;
  computeCBF(obs, h_k1, dummy);

  // Restore original state
  const_cast<CBFSafetyLayer*>(this)->X_ = original_X;

  // Discrete CBF difference
  d_h = h_k1 - h_k;
}

Eigen::Vector3d CBFSafetyLayer::solveQP(
  const Eigen::Vector3d & u_ref,
  const Eigen::MatrixXd & A,
  const Eigen::VectorXd & b) const
{
  // Simple QP solver using projected gradient descent
  // If infeasible, return scaled down version of u_ref

  Eigen::Vector3d u = u_ref;
  double lr = 0.1;
  int max_iter = config_.qp_max_iter;

  // First, check if u_ref already satisfies constraints
  bool all_satisfied = true;
  for (int i = 0; i < A.rows(); ++i) {
    double constraint_val = A.row(i) * u - b[i];
    if (constraint_val > config_.qp_solver_eps) {
      all_satisfied = false;
      break;
    }
  }

  // If nominal control is safe, return it directly
  if (all_satisfied) {
    return u_ref;
  }

  // Try to solve QP
  for (int iter = 0; iter < max_iter; ++iter) {
    all_satisfied = true;
    double max_violation = 0;

    for (int i = 0; i < A.rows(); ++i) {
      double constraint_val = A.row(i) * u - b[i];
      if (constraint_val > 0) {
        all_satisfied = false;
        max_violation = std::max(max_violation, constraint_val);
      }
    }

    if (all_satisfied) {
      break;
    }

    // Gradient step: minimize ||u - u_ref||^2
    Eigen::Vector3d grad = 2.0 * (u - u_ref);

    // Project onto constraints
    for (int i = 0; i < A.rows(); ++i) {
      double constraint_val = A.row(i) * u - b[i];
      if (constraint_val > 0) {
        double norm_A = A.row(i).norm();
        if (norm_A > config_.qp_solver_eps) {
          Eigen::Vector3d projection = u - (constraint_val / (norm_A * norm_A)) * A.row(i).transpose();
          u = projection;
        }
      }
    }

    // Gradient step
    u = u - lr * grad;

    // Damping
    if (max_violation < config_.qp_solver_eps) {
      break;
    }
  }

  // If still infeasible after optimization, scale down u_ref
  if (!all_satisfied) {
    double scale = 0.5;
    for (int iter = 0; iter < 10; ++iter) {
      u = u_ref * scale;
      all_satisfied = true;

      for (int i = 0; i < A.rows(); ++i) {
        double constraint_val = A.row(i) * u - b[i];
        if (constraint_val > config_.qp_solver_eps) {
          all_satisfied = false;
          break;
        }
      }

      if (all_satisfied) {
        break;
      }

      scale *= 0.5;
    }

    // If still infeasible, return zero
    if (!all_satisfied) {
      u = Eigen::Vector3d::Zero();
    }
  }

  return u;
}

Eigen::Vector4d CBFSafetyLayer::stepDynamics(
  const Eigen::Vector4d & X,
  const Eigen::Vector3d & u,
  double dt) const
{
  Eigen::Vector4d X_next;
  double theta = X[2];
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);

  // Omni-directional robot dynamics
  X_next[0] = X[0] + (u[0] * cos_theta - u[1] * sin_theta) * dt;
  X_next[1] = X[1] + (u[0] * sin_theta + u[1] * cos_theta) * dt;
  X_next[2] = X[2] + u[2] * dt;
  X_next[3] = std::hypot(u[0], u[1]);  // Velocity magnitude

  return X_next;
}

bool CBFSafetyLayer::isInCollision() const
{
  for (const auto & obs : obstacles_) {
    double dx = obs.x - X_[0];
    double dy = obs.y - X_[1];
    double dist = std::hypot(dx, dy);
    double min_dist = obs.radius + config_.robot_radius;

    if (dist < min_dist) {
      return true;
    }
  }
  return false;
}

std::vector<double> CBFSafetyLayer::getCBFValues() const
{
  std::vector<double> values;
  values.reserve(obstacles_.size());

  for (const auto & obs : obstacles_) {
    double h;
    Eigen::RowVector4d dummy;
    computeCBF(obs, h, dummy);
    values.push_back(h);
  }

  return values;
}

}  // namespace pb_omni_pid_pursuit_controller
