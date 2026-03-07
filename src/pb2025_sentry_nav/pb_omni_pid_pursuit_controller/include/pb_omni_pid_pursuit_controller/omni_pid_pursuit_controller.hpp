

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <random>
#include <limits>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "pb_omni_pid_pursuit_controller/pid.hpp"
#include "pb_omni_pid_pursuit_controller/fuzzy_pid.hpp"
#include "pb_omni_pid_pursuit_controller/trajectory.hpp"
#include "pb_omni_pid_pursuit_controller/critics/critic_function.hpp"
#include "pb_omni_pid_pursuit_controller/cbf_safety_layer.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_ros/buffer.h"
#include "tf2/time.h"

namespace pb_omni_pid_pursuit_controller
{

/**
 * @class pb_omni_pid_pursuit_controller::OmniPidPursuitController
 * @brief Omni-directional controller with PID pure pursuit + mini-MPPI sampling
 */
class OmniPidPursuitController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for
   * pb_omni_pid_pursuit_controller::OmniPidPursuitController
   */
  OmniPidPursuitController() = default;

  /**
   * @brief Destrructor for
   * pb_omni_pid_pursuit_controller::OmniPidPursuitController
   */
  ~OmniPidPursuitController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with
   * possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful
   * in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Transforms global plan into same frame as pose and clips poses
   * ineligible for lookaheadPoint Points ineligible to be selected as a
   * lookahead point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @return Path in new frame
   */
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(
    const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  /**
   * @brief Gets the maximum extent of the costmap
   * @return Maximum costmap extent in meters
   */
  double getCostmapMaxExtent() const;

  /**
   * @brief Creates a Carrot Point Marker message for visualization
   * @param carrot_pose Lookahead point pose
   * @return Unique pointer to the Carrot Point Marker message
   */
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Gets the lookahead point on the transformed plan
   * @param lookahead_dist Lookahead distance
   * @param transformed_plan Transformed local plan
   * @return Lookahead point pose
   */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan);

  /**
   * @brief Calculates the intersection point of a circle and a line segment
   * @param p1 Start point of the line segment
   * @param p2 End point of the line segment
   * @param r Radius of the circle
   * @return Intersection point (geometry_msgs::msg::Point)
   */
  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r);

  /**
   * @brief Callback function for dynamic parameter updates
   * @param parameters Vector of updated parameters
   * @return Result of parameter setting
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Calculates the lookahead distance based on current velocity
   * @param speed Current robot velocity
   * @return Lookahead distance
   */
  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed);

  /**
   * @brief Calculates the approach velocity scaling factor based on remaining path distance
   * @param path Transformed local path
   * @return Velocity scaling factor
   */
  double approachVelocityScalingFactor(const nav_msgs::msg::Path & path) const;

  /**
   * @brief Applies velocity scaling based on approach distance to the goal
   * @param path Transformed local path
   * @param linear_vel Linear velocity command (in out)
   */
  void applyApproachVelocityScaling(const nav_msgs::msg::Path & path, double & linear_vel) const;

  // 旧的碰撞检测接口，若你想继续用可以保留；mini-MPPI 版本主要用 evaluateTrajectory()
  bool isCollisionDetected(const nav_msgs::msg::Path & path);

private:
  // 曲率相关
  void applyCurvatureLimitation(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double & linear_vel);

  /**
   * @brief Calculates curvature using three-point circle fitting
   * @param path Transformed local path
   * @param lookahead_pose Lookahead pose (current point)
   * @param forward_dist
   * @param backward_dist
   * @return Curvature value
   */
  double calculateCurvature(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double forward_dist, double backward_dist) const;

  /**
   * @brief Calculates the radius of curvature using three points
   * @param near_point Pose before the current point
   * @param current_point Current pose (lookahead pose)
   * @param far_point Pose after the current point
   * @return Radius of curvature
   */
  double calculateCurvatureRadius(
    const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
    const geometry_msgs::msg::Point & far_point) const;

  /**
   * @brief Visualizes near and far points used for curvature calculation
   * @param backward_pose Near point pose
   * @param forward_pose Far point pose
   */
  void visualizeCurvaturePoints(
    const geometry_msgs::msg::PoseStamped & backward_pose,
    const geometry_msgs::msg::PoseStamped & forward_pose) const;

  /**
   * @brief Calculates cumulative distances along the path
   * @param path The path to calculate distances for
   * @return Vector of cumulative distances
   */
  std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path & path) const;

  /**
   * @brief Finds a pose on the path at a given distance
   * @param path The path to search on
   * @param cumulative_distances Vector of cumulative distances along the path
   * @param target_distance The target distance to find the pose at
   * @return Pose at the target distance, or empty pose if not found
   */
  geometry_msgs::msg::PoseStamped findPoseAtDistance(
    const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
    double target_distance) const;

  // ------------------ mini-MPPI 相关结构与函数 ------------------

  // Use ControlSample from trajectory.hpp instead of local definition
  // struct ControlSample is now in trajectory.hpp

  /**
   * @brief Sample control candidates around nominal control
   * @param vx_nom Nominal X velocity
   * @param vy_nom Nominal Y velocity
   * @param wz_nom Nominal angular velocity
   * @return Vector of control samples
   */
  std::vector<ControlSample> sampleControlCandidates(
    double vx_nom, double vy_nom, double wz_nom);

  /**
   * @brief Evaluate a trajectory using the critic system
   * @param trajectory Trajectory to evaluate
   * @param data Context data for critics
   * @param collision Output parameter for collision detection
   * @return Total cost from all enabled critics
   */
  double evaluateTrajectoryWithCritics(
    const Trajectory & trajectory,
    const CriticData & data,
    bool & collision) const;

  /**
   * @brief Roll out a trajectory from a control input
   * @param start_pose Starting pose in costmap frame
   * @param u Control input
   * @param data Context data
   * @param trajectory Output trajectory
   * @return true if rollout successful
   */
  bool rolloutTrajectory(
    const geometry_msgs::msg::Pose & start_pose,
    const ControlSample & u,
    const CriticData & data,
    Trajectory & trajectory) const;

  void stepDynamics(
    double & x, double & y, double & theta,
    double vx, double vy, double wz, double dt) const;

  /**
   * @brief Legacy evaluation function (kept for backward compatibility)
   */
  double evaluateTrajectory(
    const geometry_msgs::msg::Pose & start_pose_costmap_frame,
    const ControlSample & u,
    const nav_msgs::msg::Path & costmap_frame_local_plan,
    nav2_costmap_2d::Costmap2D * costmap,
    bool & collision) const;

  double costmapObstacleCost(
    double wx, double wy, nav2_costmap_2d::Costmap2D * costmap) const;

  double pathEndDistanceCost(
    double wx, double wy, const nav_msgs::msg::Path & costmap_frame_local_plan) const;

  /**
   * @brief Initialize the critic system
   * @param parent Parent node
   */
  void initializeCritics(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent);

private:
  // ROS 相关
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("OmniPidPursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  // PID 控制器
  std::shared_ptr<PID> move_pid_;
  std::shared_ptr<PID> heading_pid_;

  // Controller 参数
  double last_velocity_scaling_factor_{0.0};

  double translation_kp_, translation_ki_, translation_kd_;
  bool enable_rotation_;
  double rotation_kp_, rotation_ki_, rotation_kd_;
  double min_max_sum_error_;
  double control_duration_;
  double max_robot_pose_search_dist_;
  bool use_interpolation_;
  double lookahead_dist_;
  bool use_velocity_scaled_lookahead_dist_;
  double min_lookahead_dist_;
  double max_lookahead_dist_;
  double lookahead_time_;
  bool use_rotate_to_heading_;
  double use_rotate_to_heading_treshold_;
  double v_linear_min_;
  double v_linear_max_;
  double v_angular_min_;
  double v_angular_max_;
  double min_approach_linear_velocity_;
  double approach_velocity_scaling_dist_;
  double curvature_min_;
  double curvature_max_;
  double reduction_ratio_at_high_curvature_;
  double curvature_forward_dist_;
  double curvature_backward_dist_;
  double max_velocity_scaling_factor_rate_;
  tf2::Duration transform_tolerance_;

  // mini-MPPI 参数
  int time_steps_;
  double model_dt_;
  int batch_size_;
  double vx_std_, vy_std_, wz_std_;
  double path_weight_;
  double obstacle_weight_;
  double control_weight_;

  std::mt19937 rng_;

  // ------------------ 新增: 模糊自适应 PID ------------------
  bool enable_fuzzy_pid_;
  double fuzzy_error_max_;
  double fuzzy_error_change_max_;
  double fuzzy_kp_adjustment_ratio_;
  double fuzzy_ki_adjustment_ratio_;
  double fuzzy_kd_adjustment_ratio_;

  // Fuzzy PID controllers (created if enable_fuzzy_pid_ is true)
  std::shared_ptr<FuzzyPID> fuzzy_move_pid_;
  std::shared_ptr<FuzzyPID> fuzzy_heading_pid_;

  // ------------------ 新增: Critic 系统 ------------------
  bool enable_mppi_;
  bool use_enhanced_mppi_;
  bool adaptive_noise_;
  double adaptive_noise_min_std_;
  double adaptive_noise_max_std_;

  // Critic instances
  std::vector<std::shared_ptr<CriticFunction>> critics_;

  // ------------------ 新增: CBF 安全层 ------------------
  bool enable_cbf_safety_;
  std::string cbf_type_;  // "c3bf" or "dpcbf"
  double cbf_robot_radius_;
  double cbf_safety_margin_;
  double cbf_alpha_;
  double cbf_max_obstacle_dist_;
  int cbf_max_num_obstacles_;
  double cbf_k_lambda_;
  double cbf_k_mu_;

  // CBF 安全层实例
  std::shared_ptr<CBFSafetyLayer> cbf_safety_layer_;

  // 轨迹 / 可视化
  nav_msgs::msg::Path global_plan_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    curvature_points_pub_;

  // 动态参数
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_
