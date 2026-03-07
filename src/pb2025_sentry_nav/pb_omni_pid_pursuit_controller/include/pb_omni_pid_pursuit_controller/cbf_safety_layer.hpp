// Copyright 2025 Shenzhen Beijing Moscow University Polar Bear Robotics Team
//
// CBF Safety Layer for Omni PID Pursuit Controller
// Provides safety guarantee using Control Barrier Functions

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__CBF_SAFETY_LAYER_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__CBF_SAFETY_LAYER_HPP_

#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace pb_omni_pid_pursuit_controller
{

/**
 * @brief Obstacle representation [x, y, radius, vx, vy]
 */
struct Obstacle
{
  double x;        // obstacle x position
  double y;        // obstacle y position
  double radius;   // obstacle radius
  double vx;       // obstacle velocity x (optional)
  double vy;       // obstacle velocity y (optional)

  Obstacle(double x_ = 0, double y_ = 0, double r = 0, double vx_ = 0, double vy_ = 0)
  : x(x_), y(y_), radius(r), vx(vx_), vy(vy_) {}
};

/**
 * @brief CBF Type enumeration
 */
enum class CBFType
{
  C3BF,   // Collision Cone CBF
  DPCBF   // Dynamic Parabolic CBF
};

/**
 * @brief CBF Safety Layer Configuration
 */
struct CBFSafetyConfig
{
  CBFType cbf_type = CBFType::C3BF;
  double robot_radius = 0.3;
  double safety_margin = 1.05;       // s parameter, multiplies total radius
  double alpha = 1.0;                // CBF class K function parameter
  double max_obstacle_dist = 5.0;    // maximum distance to consider obstacles
  int max_num_obstacles = 10;        // maximum number of obstacles to consider

  // QP Solver parameters
  double qp_solver_eps = 1e-6;
  int qp_max_iter = 1000;

  // DPCBF specific parameters
  double k_lambda = 0.1;    // DPCBF lambda parameter
  double k_mu = 0.5;        // DPCBF mu parameter
};

/**
 * @brief CBF Safety Layer for obstacle avoidance
 *
 * Given a nominal control input u_ref, computes the closest safe control u
 * that satisfies the CBF constraint: h_dot >= -alpha * h
 *
 * For omni-directional robot: u = [vx, vy, wz]
 */
class CBFSafetyLayer
{
public:
  /**
   * @brief Constructor
   */
  CBFSafetyLayer();

  /**
   * @brief Configure the CBF safety layer
   * @param config Configuration parameters
   */
  void configure(const CBFSafetyConfig & config);

  /**
   * @brief Set current robot state
   * @param x Robot x position
   * @param y Robot y position
   * @param theta Robot heading angle
   * @param v Robot current velocity magnitude
   */
  void setRobotState(double x, double y, double theta, double v);

  /**
   * @brief Set obstacles
   * @param obstacles Vector of obstacles
   */
  void setObstacles(const std::vector<Obstacle> & obstacles);

  /**
   * @brief Filter control input through CBF safety layer
   * @param u_ref Nominal control input [vx, vy, wz]
   * @return Safe control input [vx, vy, wz]
   */
  Eigen::Vector3d filterControl(const Eigen::Vector3d & u_ref);

  /**
   * @brief Compute CBF value h and its gradient
   * @param obs Obstacle to compute CBF for
   * @param h Output CBF value
   * @param dh_dx Output CBF gradient [1x4]
   */
  void computeCBF(
    const Obstacle & obs,
    double & h,
    Eigen::RowVector4d & dh_dx) const;

  /**
   * @brief Compute discrete-time CBF
   * @param x_k Current state [x, y, theta, v]
   * @param u_k Control input [vx, vy, wz]
   * @param obs Obstacle
   * @param h_k Output CBF value at time k
   * @param d_h Output CBF difference h_{k+1} - h_k
   */
  void computeDiscreteCBF(
    const Eigen::Vector4d & x_k,
    const Eigen::Vector3d & u_k,
    const Obstacle & obs,
    double & h_k,
    double & d_h) const;

  /**
   * @brief Solve QP: min ||u - u_ref||^2 s.t. A*u <= b
   * @param u_ref Nominal control
   * @param A Constraint matrix (num_constraints x 3)
   * @param b Constraint vector (num_constraints)
   * @return Optimal control
   */
  Eigen::Vector3d solveQP(
    const Eigen::Vector3d & u_ref,
    const Eigen::MatrixXd & A,
    const Eigen::VectorXd & b) const;

  /**
   * @brief Check if robot is in collision with obstacles
   * @return true if collision detected
   */
  bool isInCollision() const;

  /**
   * @brief Get current CBF values for all obstacles
   * @return Vector of CBF values
   */
  std::vector<double> getCBFValues() const;

private:
  CBFSafetyConfig config_;

  // Robot state [x, y, theta, v]
  Eigen::Vector4d X_;

  // Obstacles
  std::vector<Obstacle> obstacles_;

  /**
   * @brief Compute C3BF (Collision Cone CBF)
   */
  void computeC3BF(
    const Obstacle & obs,
    double & h,
    Eigen::RowVector4d & dh_dx) const;

  /**
   * @brief Compute DPCBF (Dynamic Parabolic CBF)
   */
  void computeDPCBF(
    const Obstacle & obs,
    double & h,
    Eigen::RowVector4d & dh_dx) const;

  /**
   * @brief Compute kinematic bicycle dynamics
   * @param X Current state
   * @param u Control input
   * @param dt Time step
   * @return Next state
   */
  Eigen::Vector4d stepDynamics(
    const Eigen::Vector4d & X,
    const Eigen::Vector3d & u,
    double dt = 0.05) const;
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__CBF_SAFETY_LAYER_HPP_
