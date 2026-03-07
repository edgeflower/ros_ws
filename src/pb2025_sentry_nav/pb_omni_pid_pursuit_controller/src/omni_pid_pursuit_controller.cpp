

#include "pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp"

// New includes for fuzzy PID and critic system
#include "pb_omni_pid_pursuit_controller/fuzzy_pid.hpp"
#include "pb_omni_pid_pursuit_controller/critics/critic_function.hpp"
#include "pb_omni_pid_pursuit_controller/critics/path_align_critic.hpp"
#include "pb_omni_pid_pursuit_controller/critics/goal_angle_critic.hpp"
#include "pb_omni_pid_pursuit_controller/critics/prefer_forward_critic.hpp"
#include "pb_omni_pid_pursuit_controller/critics/obstacle_critic.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace pb_omni_pid_pursuit_controller
{

void OmniPidPursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 1.0;
  double control_frequency = 20.0;
  max_robot_pose_search_dist_ = getCostmapMaxExtent();
  last_velocity_scaling_factor_ = 0.0;

  // PID / pure-pursuit 参数
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kd", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_rotation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kd", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_max_sum_error", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_interpolation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading_treshold", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_min", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_min", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(getCostmapMaxExtent()));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_min", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_max", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".reduction_ratio_at_high_curvature", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_forward_dist", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_backward_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_velocity_scaling_factor_rate", rclcpp::ParameterValue(0.9));

  // mini-MPPI 参数
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".time_steps", rclcpp::ParameterValue(20));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".model_dt", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".batch_size", rclcpp::ParameterValue(200));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".vx_std", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".vy_std", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".wz_std", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_weight", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_weight", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".control_weight", rclcpp::ParameterValue(0.1));

  // Fuzzy PID 参数
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_fuzzy_pid", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".fuzzy_error_max", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".fuzzy_error_change_max", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".fuzzy_kp_adjustment_ratio", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".fuzzy_ki_adjustment_ratio", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".fuzzy_kd_adjustment_ratio", rclcpp::ParameterValue(0.4));

  // Enhanced MPPI 参数
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_mppi", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_enhanced_mppi", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".adaptive_noise", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".adaptive_noise_min_std", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".adaptive_noise_max_std", rclcpp::ParameterValue(1.0));

  // CBF 安全层参数
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_cbf_safety", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cbf_type", rclcpp::ParameterValue(std::string("c3bf")));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cbf_robot_radius", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cbf_safety_margin", rclcpp::ParameterValue(1.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cbf_alpha", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cbf_max_obstacle_dist", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cbf_max_num_obstacles", rclcpp::ParameterValue(10));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cbf_k_lambda", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cbf_k_mu", rclcpp::ParameterValue(0.5));

  // 读取参数
  node->get_parameter(plugin_name_ + ".translation_kp", translation_kp_);
  node->get_parameter(plugin_name_ + ".translation_ki", translation_ki_);
  node->get_parameter(plugin_name_ + ".translation_kd", translation_kd_);
  node->get_parameter(plugin_name_ + ".enable_rotation", enable_rotation_);
  node->get_parameter(plugin_name_ + ".rotation_kp", rotation_kp_);
  node->get_parameter(plugin_name_ + ".rotation_ki", rotation_ki_);
  node->get_parameter(plugin_name_ + ".rotation_kd", rotation_kd_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".min_max_sum_error", min_max_sum_error_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".use_interpolation", use_interpolation_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(
    plugin_name_ + ".use_rotate_to_heading_treshold", use_rotate_to_heading_treshold_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  if (approach_velocity_scaling_dist_ > costmap_->getSizeInMetersX() / 2.0) {
    RCLCPP_WARN(
      logger_,
      "approach_velocity_scaling_dist is larger than forward costmap extent, "
      "leading to permanent slowdown");
  }
  node->get_parameter(plugin_name_ + ".v_linear_max", v_linear_max_);
  node->get_parameter(plugin_name_ + ".v_linear_min", v_linear_min_);
  node->get_parameter(plugin_name_ + ".v_angular_max", v_angular_max_);
  node->get_parameter(plugin_name_ + ".v_angular_min", v_angular_min_);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
  node->get_parameter(plugin_name_ + ".curvature_min", curvature_min_);
  node->get_parameter(plugin_name_ + ".curvature_max", curvature_max_);
  node->get_parameter(
    plugin_name_ + ".reduction_ratio_at_high_curvature", reduction_ratio_at_high_curvature_);
  node->get_parameter(plugin_name_ + ".curvature_forward_dist", curvature_forward_dist_);
  node->get_parameter(plugin_name_ + ".curvature_backward_dist", curvature_backward_dist_);
  node->get_parameter(
    plugin_name_ + ".max_velocity_scaling_factor_rate", max_velocity_scaling_factor_rate_);

  node->get_parameter(plugin_name_ + ".time_steps", time_steps_);
  node->get_parameter(plugin_name_ + ".model_dt", model_dt_);
  node->get_parameter(plugin_name_ + ".batch_size", batch_size_);
  node->get_parameter(plugin_name_ + ".vx_std", vx_std_);
  node->get_parameter(plugin_name_ + ".vy_std", vy_std_);
  node->get_parameter(plugin_name_ + ".wz_std", wz_std_);
  node->get_parameter(plugin_name_ + ".path_weight", path_weight_);
  node->get_parameter(plugin_name_ + ".obstacle_weight", obstacle_weight_);
  node->get_parameter(plugin_name_ + ".control_weight", control_weight_);

  // Fuzzy PID 参数
  node->get_parameter(plugin_name_ + ".enable_fuzzy_pid", enable_fuzzy_pid_);
  node->get_parameter(plugin_name_ + ".fuzzy_error_max", fuzzy_error_max_);
  node->get_parameter(plugin_name_ + ".fuzzy_error_change_max", fuzzy_error_change_max_);
  node->get_parameter(plugin_name_ + ".fuzzy_kp_adjustment_ratio", fuzzy_kp_adjustment_ratio_);
  node->get_parameter(plugin_name_ + ".fuzzy_ki_adjustment_ratio", fuzzy_ki_adjustment_ratio_);
  node->get_parameter(plugin_name_ + ".fuzzy_kd_adjustment_ratio", fuzzy_kd_adjustment_ratio_);

  // Enhanced MPPI 参数
  node->get_parameter(plugin_name_ + ".enable_mppi", enable_mppi_);
  node->get_parameter(plugin_name_ + ".use_enhanced_mppi", use_enhanced_mppi_);
  node->get_parameter(plugin_name_ + ".adaptive_noise", adaptive_noise_);
  node->get_parameter(plugin_name_ + ".adaptive_noise_min_std", adaptive_noise_min_std_);
  node->get_parameter(plugin_name_ + ".adaptive_noise_max_std", adaptive_noise_max_std_);

  // CBF 安全层参数
  node->get_parameter(plugin_name_ + ".enable_cbf_safety", enable_cbf_safety_);
  node->get_parameter(plugin_name_ + ".cbf_type", cbf_type_);
  node->get_parameter(plugin_name_ + ".cbf_robot_radius", cbf_robot_radius_);
  node->get_parameter(plugin_name_ + ".cbf_safety_margin", cbf_safety_margin_);
  node->get_parameter(plugin_name_ + ".cbf_alpha", cbf_alpha_);
  node->get_parameter(plugin_name_ + ".cbf_max_obstacle_dist", cbf_max_obstacle_dist_);
  node->get_parameter(plugin_name_ + ".cbf_max_num_obstacles", cbf_max_num_obstacles_);
  node->get_parameter(plugin_name_ + ".cbf_k_lambda", cbf_k_lambda_);
  node->get_parameter(plugin_name_ + ".cbf_k_mu", cbf_k_mu_);

  node->get_parameter("controller_frequency", control_frequency);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;

  // Publisher
  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  curvature_points_pub_ =
    node_.lock()->create_publisher<visualization_msgs::msg::MarkerArray>(
      "curvature_points_marker_array", rclcpp::QoS(10));

  // PID 控制器
  move_pid_ = std::make_shared<PID>(
    control_duration_, v_linear_max_, v_linear_min_, translation_kp_, translation_kd_,
    translation_ki_);
  heading_pid_ = std::make_shared<PID>(
    control_duration_, v_angular_max_, v_angular_min_, rotation_kp_, rotation_kd_, rotation_ki_);

  // Fuzzy PID 控制器 (如果启用)
  if (enable_fuzzy_pid_) {
    fuzzy_move_pid_ = std::make_shared<FuzzyPID>(
      control_duration_, v_linear_max_, v_linear_min_, translation_kp_, translation_kd_,
      translation_ki_,
      fuzzy_error_max_, fuzzy_error_change_max_,
      fuzzy_kp_adjustment_ratio_, fuzzy_ki_adjustment_ratio_, fuzzy_kd_adjustment_ratio_);

    fuzzy_heading_pid_ = std::make_shared<FuzzyPID>(
      control_duration_, v_angular_max_, v_angular_min_, rotation_kp_, rotation_kd_, rotation_ki_,
      fuzzy_error_max_, fuzzy_error_change_max_,
      fuzzy_kp_adjustment_ratio_, fuzzy_ki_adjustment_ratio_, fuzzy_kd_adjustment_ratio_);

    RCLCPP_INFO(logger_, "Fuzzy adaptive PID enabled");
  }

  // 初始化 Critic 系统 (如果启用增强 MPPI)
  if (use_enhanced_mppi_) {
    initializeCritics(parent);
    RCLCPP_INFO(logger_, "Enhanced MPPI with critic system enabled (%zu critics loaded)",
                critics_.size());
  }

  // 初始化 CBF 安全层
  if (enable_cbf_safety_) {
    cbf_safety_layer_ = std::make_shared<CBFSafetyLayer>();

    CBFSafetyConfig cbf_config;
    if (cbf_type_ == "dpcbf") {
      cbf_config.cbf_type = CBFType::DPCBF;
    } else {
      cbf_config.cbf_type = CBFType::C3BF;
    }
    cbf_config.robot_radius = cbf_robot_radius_;
    cbf_config.safety_margin = cbf_safety_margin_;
    cbf_config.alpha = cbf_alpha_;
    cbf_config.max_obstacle_dist = cbf_max_obstacle_dist_;
    cbf_config.max_num_obstacles = cbf_max_num_obstacles_;
    cbf_config.k_lambda = cbf_k_lambda_;
    cbf_config.k_mu = cbf_k_mu_;

    cbf_safety_layer_->configure(cbf_config);
    RCLCPP_INFO(logger_, "CBF Safety Layer enabled (type: %s, radius: %.2f, margin: %.2f)",
                cbf_type_.c_str(), cbf_robot_radius_, cbf_safety_margin_);
  }

  // 随机数种子
  rng_ = std::mt19937(std::random_device{}());
}

void OmniPidPursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type pb_omni_pid_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_.reset();
  carrot_pub_.reset();
  curvature_points_pub_.reset();
}

void OmniPidPursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type pb_omni_pid_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_->on_activate();
  carrot_pub_->on_activate();
  curvature_points_pub_->on_activate();

  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&OmniPidPursuitController::dynamicParametersCallback, this, std::placeholders::_1));
}

void OmniPidPursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type pb_omni_pid_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  curvature_points_pub_->on_deactivate();
  dyn_params_handler_.reset();
}

geometry_msgs::msg::TwistStamped OmniPidPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // 1) 全局路径 → base_link frame 局部路径
  auto transformed_plan = transformGlobalPlan(pose);

  // 2) 计算 lookahead & carrot
  double lookahead_dist = getLookAheadDistance(velocity);
  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double lin_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  double theta_dist = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  double angle_to_goal = tf2::getYaw(carrot_pose.pose.orientation);

  if (use_rotate_to_heading_) {
    angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    if (fabs(angle_to_goal) > use_rotate_to_heading_treshold_) {
      lin_dist = 0.0;
    }
  }

  // 3) 用 PID 生成"标称控制" (使用 Fuzzy PID 如果启用)
  double lin_vel, angular_vel;

  if (enable_fuzzy_pid_ && fuzzy_move_pid_ && fuzzy_heading_pid_) {
    lin_vel = fuzzy_move_pid_->calculate(lin_dist, 0.0);
    angular_vel = enable_rotation_ ? fuzzy_heading_pid_->calculate(angle_to_goal, 0.0) : 0.0;
  } else {
    lin_vel = move_pid_->calculate(lin_dist, 0.0);
    angular_vel = enable_rotation_ ? heading_pid_->calculate(angle_to_goal, 0.0) : 0.0;
  }

  applyCurvatureLimitation(transformed_plan, carrot_pose, lin_vel);
  applyApproachVelocityScaling(transformed_plan, lin_vel);

  double vx_nom = lin_vel * std::cos(theta_dist);
  double vy_nom = lin_vel * std::sin(theta_dist);
  double wz_nom = angular_vel;

  // 4) 把局部路径采样转到 costmap frame，用于 path 末端代价
  nav_msgs::msg::Path costmap_frame_local_plan;
  costmap_frame_local_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  costmap_frame_local_plan.header.stamp = pose.header.stamp;

  int sample_points = 10;
  int plan_size = static_cast<int>(transformed_plan.poses.size());
  for (int i = 0; i < sample_points; ++i) {
    int index = std::min((i * plan_size) / sample_points, plan_size - 1);
    geometry_msgs::msg::PoseStamped map_pose;
    transformPose(costmap_ros_->getGlobalFrameID(), transformed_plan.poses[index], map_pose);
    costmap_frame_local_plan.poses.push_back(map_pose);
  }

  // 5) 获取机器人在 costmap frame 下当前姿态
  geometry_msgs::msg::PoseStamped robot_in_costmap;
  if (!transformPose(costmap_ros_->getGlobalFrameID(), pose, robot_in_costmap)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into costmap frame");
  }

  // 6) 采样一批控制候选
  auto candidates = sampleControlCandidates(vx_nom, vy_nom, wz_nom);

  // 7) 在 costmap 上滚动仿真 + 打分 (使用增强 Critic 系统或传统方法)
  ControlSample best = candidates.front();
  best.cost = std::numeric_limits<double>::infinity();
  best.collision = true;

  // Prepare critic data if using enhanced system
  CriticData critic_data;
  if (use_enhanced_mppi_ && !critics_.empty()) {
    critic_data.robot_pose = robot_in_costmap.pose;
    critic_data.reference_path = costmap_frame_local_plan;
    critic_data.costmap = costmap;
    critic_data.model_dt = model_dt_;
    critic_data.time_steps = time_steps_;
    critic_data.goal_pose = transformed_plan.poses.back().pose;
    critic_data.has_goal_pose = true;
    critic_data.v_linear_max = v_linear_max_;
    critic_data.v_angular_max = v_angular_max_;
  }

  for (auto & c : candidates) {
    bool collision = false;
    double cost_value = 0.0;

    if (use_enhanced_mppi_ && !critics_.empty()) {
      // Use enhanced critic system
      Trajectory trajectory;
      rolloutTrajectory(robot_in_costmap.pose, c, critic_data, trajectory);
      cost_value = evaluateTrajectoryWithCritics(trajectory, critic_data, collision);
    } else {
      // Use legacy evaluation
      cost_value = evaluateTrajectory(
        robot_in_costmap.pose, c, costmap_frame_local_plan, costmap, collision);
    }

    c.cost = cost_value;
    c.collision = collision;

    if (!collision && cost_value < best.cost) {
      best = c;
    }
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;

  // 8) 若所有轨迹都碰撞 → 停车
  if (best.collision || !std::isfinite(best.cost)) {
    RCLCPP_WARN(logger_, "All sampled trajectories in collision, stopping robot.");
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  // 9) 用最佳候选控制量作为输出
  cmd_vel.twist.linear.x = best.vx;
  cmd_vel.twist.linear.y = best.vy;
  cmd_vel.twist.angular.z = best.wz;

  cmd_vel.twist.linear.x =
    std::clamp(cmd_vel.twist.linear.x, v_linear_min_, v_linear_max_);
  cmd_vel.twist.linear.y =
    std::clamp(cmd_vel.twist.linear.y, v_linear_min_, v_linear_max_);
  cmd_vel.twist.angular.z =
    std::clamp(cmd_vel.twist.angular.z, v_angular_min_, v_angular_max_);

  // 10) CBF 安全层过滤 (如果启用)
  if (enable_cbf_safety_ && cbf_safety_layer_) {
    // 从 costmap 提取障碍物（使用聚类减少数量）
    std::vector<Obstacle> obstacles;
    auto costmap = costmap_ros_->getCostmap();

    unsigned int robot_mx, robot_my;
    if (costmap->worldToMap(robot_in_costmap.pose.position.x, robot_in_costmap.pose.position.y, robot_mx, robot_my)) {
      int scan_radius = static_cast<int>(cbf_max_obstacle_dist_ / costmap->getResolution());

      // 使用 visited 数组避免重复处理
      std::vector<bool> visited(scan_radius * 2 + 1 * scan_radius * 2 + 1, false);
      double cluster_resolution = costmap->getResolution() * 2.0;  // 聚类分辨率

      for (int dx = -scan_radius; dx <= scan_radius; ++dx) {
        for (int dy = -scan_radius; dy <= scan_radius; ++dy) {
          unsigned int mx = robot_mx + dx;
          unsigned int my = robot_my + dy;

          if (mx >= costmap->getSizeInCellsX() || my >= costmap->getSizeInCellsY()) {
            continue;
          }

          unsigned int cost = costmap->getCost(mx, my);
          if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
            // 转换为世界坐标
            double wx, wy;
            costmap->mapToWorld(mx, my, wx, wy);

            // 检查是否与已有障碍物足够远（聚类）
            bool is_new_cluster = true;
            for (const auto & existing_obs : obstacles) {
              double dist = std::hypot(existing_obs.x - wx, existing_obs.y - wy);
              if (dist < cluster_resolution) {
                is_new_cluster = false;
                break;
              }
            }

            if (is_new_cluster && obstacles.size() < static_cast<size_t>(cbf_max_num_obstacles_)) {
              // 障碍物半径设为 costmap 分辨率
              double obs_radius = costmap->getResolution();
              obstacles.emplace_back(wx, wy, obs_radius, 0.0, 0.0);

              // 限制障碍物数量
              if (obstacles.size() >= static_cast<size_t>(cbf_max_num_obstacles_)) {
                break;
              }
            }
          }
        }
        if (obstacles.size() >= static_cast<size_t>(cbf_max_num_obstacles_)) {
          break;
        }
      }
    }

    // 设置机器人状态和障碍物
    double current_v = std::hypot(velocity.linear.x, velocity.linear.y);
    cbf_safety_layer_->setRobotState(
      robot_in_costmap.pose.position.x,
      robot_in_costmap.pose.position.y,
      tf2::getYaw(robot_in_costmap.pose.orientation),
      current_v);
    cbf_safety_layer_->setObstacles(obstacles);

    // 应用 CBF 过滤
    Eigen::Vector3d u_ref(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
    Eigen::Vector3d u_safe = cbf_safety_layer_->filterControl(u_ref);

    cmd_vel.twist.linear.x = u_safe[0];
    cmd_vel.twist.linear.y = u_safe[1];
    cmd_vel.twist.angular.z = u_safe[2];
  }

  return cmd_vel;
}

void OmniPidPursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void OmniPidPursuitController::setSpeedLimit(
  const double & /*speed_limit*/, const bool & /*percentage*/)
{
  RCLCPP_WARN(logger_, "Speed limit is not implemented in this controller.");
}

nav_msgs::msg::Path OmniPidPursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  double max_costmap_extent = getCostmapMaxExtent();

  auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin = nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) { return euclidean_distance(pose, robot_pose) > max_costmap_extent; });

  // Lambda to transform a PoseStamped from global frame to local
  auto transform_global_pose_to_local = [&](const auto & global_plan_pose) {
    geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
    stamped_pose.header.frame_id = global_plan_.header.frame_id;
    stamped_pose.header.stamp = robot_pose.header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
    transformed_pose.pose.position.z = 0.0;
    return transformed_pose;
  };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
    transform_global_pose_to_local);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  local_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

std::unique_ptr<geometry_msgs::msg::PointStamped> OmniPidPursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;
  return carrot_msg;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::getLookAheadPoint(
  const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    return pose;
  }

  return *goal_pose_it;
}

geometry_msgs::msg::Point OmniPidPursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double d = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(std::max(0.0, r * r * dr2 - d * d));
  p.x = (d * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-d * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

double OmniPidPursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters =
    std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

bool OmniPidPursuitController::transformPose(
  const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

bool OmniPidPursuitController::isCollisionDetected(const nav_msgs::msg::Path & path)
{
  auto costmap = costmap_ros_->getCostmap();
  for (const auto & pose_stamped : path.poses) {
    const auto & pose = pose_stamped.pose;
    unsigned int mx, my;
    if (costmap->worldToMap(pose.position.x, pose.position.y, mx, my)) {
      if (costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return true;
      }
    } else {
      // RCLCPP_WARN(
      //   logger_,
      //   "The Local path is not in the costmap. Cannot check for collisions. "
      //   "Proceed at your own risk, slow the robot, or increase your costmap size.");
      return false;
    }
  }
  return false;
}

double OmniPidPursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;

  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = hypot(speed.linear.x, speed.linear.y) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}

double OmniPidPursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path) const
{
  // Waiting to apply the threshold based on integrated distance ensures we don't
  // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
  double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = transformed_path.poses.back();
    // Here we will use a regular euclidean distance from the robot frame (origin)
    // to get smooth scaling, regardless of path density.
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

void OmniPidPursuitController::applyApproachVelocityScaling(
  const nav_msgs::msg::Path & path, double & linear_vel) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

  // Use the lowest velocity between approach and other constraints, if all overlapping
  linear_vel = std::min(linear_vel, approach_vel);
}

void OmniPidPursuitController::applyCurvatureLimitation(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double & linear_vel)
{
  double curvature =
    calculateCurvature(path, lookahead_pose, curvature_forward_dist_, curvature_backward_dist_);

  double scaled_linear_vel = linear_vel;
  if (curvature > curvature_min_) {
    double reduction_ratio = 1.0;
    if (curvature > curvature_max_) {
      reduction_ratio = reduction_ratio_at_high_curvature_;
    } else {
      reduction_ratio = 1.0 - (curvature - curvature_min_) / (curvature_max_ - curvature_min_) *
                                (1.0 - reduction_ratio_at_high_curvature_);
    }

    double target_scaled_vel = linear_vel * reduction_ratio;
    scaled_linear_vel =
      last_velocity_scaling_factor_ + std::clamp(
                                        target_scaled_vel - last_velocity_scaling_factor_,
                                        -max_velocity_scaling_factor_rate_ * control_duration_,
                                        max_velocity_scaling_factor_rate_ * control_duration_);
  }
  scaled_linear_vel = std::max(scaled_linear_vel, 2.0 * min_approach_linear_velocity_);

  linear_vel = std::min(linear_vel, scaled_linear_vel);
  last_velocity_scaling_factor_ = linear_vel;
}

double OmniPidPursuitController::calculateCurvature(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double forward_dist, double backward_dist) const
{
  geometry_msgs::msg::PoseStamped backward_pose, forward_pose;
  std::vector<double> cumulative_distances = calculateCumulativeDistances(path);

  double lookahead_pose_cumulative_distance = 0.0;
  geometry_msgs::msg::PoseStamped robot_base_frame_pose;
  robot_base_frame_pose.pose = geometry_msgs::msg::Pose();
  lookahead_pose_cumulative_distance =
    nav2_util::geometry_utils::euclidean_distance(robot_base_frame_pose, lookahead_pose);

  backward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance - backward_dist);

  forward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance + forward_dist);

  double curvature_radius = calculateCurvatureRadius(
    backward_pose.pose.position, lookahead_pose.pose.position, forward_pose.pose.position);
  double curvature = 1.0 / curvature_radius;
  visualizeCurvaturePoints(backward_pose, forward_pose);
  return curvature;
}

double OmniPidPursuitController::calculateCurvatureRadius(
  const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
  const geometry_msgs::msg::Point & far_point) const
{
  double x1 = near_point.x, y1 = near_point.y;
  double x2 = current_point.x, y2 = current_point.y;
  double x3 = far_point.x, y3 = far_point.y;

  double denom = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
  if (std::fabs(denom) < 1e-6) {
    return 1e9;
  }

  double center_x = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
                     (x3 * x3 + y3 * y3) * (y1 - y2)) /
                    denom;
  double center_y = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
                     (x3 * x3 + y3 * y3) * (x2 - x1)) /
                    denom;
  double radius = std::hypot(x2 - center_x, y2 - center_y);
  if (std::isnan(radius) || std::isinf(radius) || radius < 1e-9) {
    return 1e9;
  }
  return radius;
}

void OmniPidPursuitController::visualizeCurvaturePoints(
  const geometry_msgs::msg::PoseStamped & backward_pose,
  const geometry_msgs::msg::PoseStamped & forward_pose) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker near_marker;
  near_marker.header = backward_pose.header;
  near_marker.ns = "curvature_points";
  near_marker.id = 0;
  near_marker.type = visualization_msgs::msg::Marker::SPHERE;
  near_marker.action = visualization_msgs::msg::Marker::ADD;
  near_marker.pose = backward_pose.pose;
  near_marker.scale.x = near_marker.scale.y = near_marker.scale.z = 0.1;
  near_marker.color.g = 1.0;
  near_marker.color.a = 1.0;

  visualization_msgs::msg::Marker far_marker;
  far_marker.header = forward_pose.header;
  far_marker.ns = "curvature_points";
  far_marker.id = 1;
  far_marker.type = visualization_msgs::msg::Marker::SPHERE;
  far_marker.action = visualization_msgs::msg::Marker::ADD;
  far_marker.pose = forward_pose.pose;
  far_marker.scale.x = far_marker.scale.y = far_marker.scale.z = 0.1;
  far_marker.color.r = 1.0;
  far_marker.color.a = 1.0;

  marker_array.markers.push_back(near_marker);
  marker_array.markers.push_back(far_marker);

  curvature_points_pub_->publish(marker_array);
}

std::vector<double> OmniPidPursuitController::calculateCumulativeDistances(
  const nav_msgs::msg::Path & path) const
{
  std::vector<double> cumulative_distances;
  if (path.poses.empty()) {
    return cumulative_distances;
  }

  cumulative_distances.reserve(path.poses.size());
  cumulative_distances.push_back(0.0);

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & prev_pose = path.poses[i - 1].pose.position;
    const auto & curr_pose = path.poses[i].pose.position;
    double distance = hypot(curr_pose.x - prev_pose.x, curr_pose.y - prev_pose.y);
    cumulative_distances.push_back(cumulative_distances.back() + distance);
  }
  return cumulative_distances;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::findPoseAtDistance(
  const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
  double target_distance) const
{
  geometry_msgs::msg::PoseStamped empty_pose;
  if (path.poses.empty() || cumulative_distances.empty()) {
    return empty_pose;
  }
  if (target_distance <= 0.0) {
    return path.poses.front();
  }
  if (target_distance >= cumulative_distances.back()) {
    return path.poses.back();
  }
  auto it =
    std::lower_bound(cumulative_distances.begin(), cumulative_distances.end(), target_distance);
  size_t index = static_cast<size_t>(std::distance(cumulative_distances.begin(), it));

  if (index == 0) {
    return path.poses.front();
  }

  double ratio = (target_distance - cumulative_distances[index - 1]) /
                 (cumulative_distances[index] - cumulative_distances[index - 1]);
  geometry_msgs::msg::PoseStamped pose1 = path.poses[index - 1];
  geometry_msgs::msg::PoseStamped pose2 = path.poses[index];

  geometry_msgs::msg::PoseStamped interpolated_pose;
  interpolated_pose.header = pose2.header;
  interpolated_pose.pose.position.x =
    pose1.pose.position.x + ratio * (pose2.pose.position.x - pose1.pose.position.x);
  interpolated_pose.pose.position.y =
    pose1.pose.position.y + ratio * (pose2.pose.position.y - pose1.pose.position.y);
  interpolated_pose.pose.position.z =
    pose1.pose.position.z + ratio * (pose2.pose.position.z - pose1.pose.position.z);
  interpolated_pose.pose.orientation = pose2.pose.orientation;

  return interpolated_pose;
}

// ---------------- mini-MPPI 相关实现 ----------------

std::vector<ControlSample>
OmniPidPursuitController::sampleControlCandidates(
  double vx_nom, double vy_nom, double wz_nom)
{
  std::vector<ControlSample> samples;
  samples.reserve(static_cast<size_t>(batch_size_));

  // Adaptive noise: scale noise based on nominal control magnitude
  double vx_noise_std = vx_std_;
  double vy_noise_std = vy_std_;
  double wz_noise_std = wz_std_;

  if (adaptive_noise_) {
    double control_magnitude = std::hypot(vx_nom, vy_nom);
    double noise_scale = adaptive_noise_min_std_ +
      (adaptive_noise_max_std_ - adaptive_noise_min_std_) *
      (control_magnitude / v_linear_max_);
    noise_scale = std::clamp(noise_scale, adaptive_noise_min_std_, adaptive_noise_max_std_);

    vx_noise_std = noise_scale;
    vy_noise_std = noise_scale;
    wz_noise_std = noise_scale * 0.5;  // Less noise on rotation
  }

  std::normal_distribution<double> dist_vx(0.0, vx_noise_std);
  std::normal_distribution<double> dist_vy(0.0, vy_noise_std);
  std::normal_distribution<double> dist_wz(0.0, wz_noise_std);

  // 标称控制 - 加入多次以提高选中概率 (约占 10% 的样本)
  int nominal_repeat_count = std::max(1, batch_size_ / 10);
  for (int i = 0; i < nominal_repeat_count; ++i) {
    ControlSample nominal;
    nominal.vx = std::clamp(vx_nom, v_linear_min_, v_linear_max_);
    nominal.vy = std::clamp(vy_nom, v_linear_min_, v_linear_max_);
    nominal.wz = std::clamp(wz_nom, v_angular_min_, v_angular_max_);
    nominal.cost = 0.0;
    nominal.collision = false;
    samples.push_back(nominal);
  }

  // 采样新的控制候选
  int noise_samples = batch_size_ - nominal_repeat_count;
  for (int i = 0; i < noise_samples; ++i) {
    ControlSample c;
    c.vx = vx_nom + dist_vx(rng_);
    c.vy = vy_nom + dist_vy(rng_);
    c.wz = wz_nom + dist_wz(rng_);

    c.vx = std::clamp(c.vx, v_linear_min_, v_linear_max_);
    c.vy = std::clamp(c.vy, v_linear_min_, v_linear_max_);
    c.wz = std::clamp(c.wz, v_angular_min_, v_angular_max_);

    c.cost = 0.0;
    c.collision = false;
    samples.push_back(c);
  }

  return samples;
}

void OmniPidPursuitController::stepDynamics(
  double & x, double & y, double & theta,
  double vx, double vy, double wz, double dt) const
{
  double cos_th = std::cos(theta);
  double sin_th = std::sin(theta);

  double vx_world = vx * cos_th - vy * sin_th;
  double vy_world = vx * sin_th + vy * cos_th;

  x += vx_world * dt;
  y += vy_world * dt;
  theta += wz * dt;
}

double OmniPidPursuitController::costmapObstacleCost(
  double wx, double wy, nav2_costmap_2d::Costmap2D * costmap) const
{
  unsigned int mx, my;
  if (!costmap->worldToMap(wx, wy, mx, my)) {
    return std::numeric_limits<double>::infinity();
  }

  unsigned char c = costmap->getCost(mx, my);

  if (c >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return std::numeric_limits<double>::infinity();
  }

  double norm_c = static_cast<double>(c) /
    static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE + 1);
  return norm_c;
}

double OmniPidPursuitController::pathEndDistanceCost(
  double wx, double wy, const nav_msgs::msg::Path & costmap_frame_local_plan) const
{
  if (costmap_frame_local_plan.poses.empty()) {
    return 0.0;
  }

  const auto & last = costmap_frame_local_plan.poses.back().pose.position;
  double dx = wx - last.x;
  double dy = wy - last.y;
  return std::hypot(dx, dy);
}

double OmniPidPursuitController::evaluateTrajectory(
  const geometry_msgs::msg::Pose & start_pose_costmap_frame,
  const ControlSample & u,
  const nav_msgs::msg::Path & costmap_frame_local_plan,
  nav2_costmap_2d::Costmap2D * costmap,
  bool & collision) const
{
  collision = false;
  double cost = 0.0;

  double x = start_pose_costmap_frame.position.x;
  double y = start_pose_costmap_frame.position.y;
  double yaw = tf2::getYaw(start_pose_costmap_frame.orientation);

  double max_obstacle_cost = 0.0;

  for (int t = 0; t < time_steps_; ++t) {
    stepDynamics(x, y, yaw, u.vx, u.vy, u.wz, model_dt_);

    double obs_c = costmapObstacleCost(x, y, costmap);
    if (!std::isfinite(obs_c)) {
      collision = true;
      cost += obstacle_weight_ * 1000.0;
      break;
    }
    max_obstacle_cost = std::max(max_obstacle_cost, obs_c);
  }

  double ctrl_energy = u.vx * u.vx + u.vy * u.vy + u.wz * u.wz;
  cost += control_weight_ * ctrl_energy * (time_steps_ * model_dt_);

  cost += obstacle_weight_ * max_obstacle_cost;

  double terminal_dist = pathEndDistanceCost(x, y, costmap_frame_local_plan);
  cost += path_weight_ * terminal_dist * terminal_dist;

  return cost;
}

// ---------------- 增强 MPPI 和 Critic 系统相关函数 ----------------

void OmniPidPursuitController::initializeCritics(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent)
{
  critics_.clear();

  // Create and configure each critic
  auto path_align = std::make_shared<PathAlignCritic>();
  path_align->configure(parent, plugin_name_, "PathAlignCritic");
  critics_.push_back(path_align);

  auto goal_angle = std::make_shared<GoalAngleCritic>();
  goal_angle->configure(parent, plugin_name_, "GoalAngleCritic");
  critics_.push_back(goal_angle);

  auto prefer_forward = std::make_shared<PreferForwardCritic>();
  prefer_forward->configure(parent, plugin_name_, "PreferForwardCritic");
  critics_.push_back(prefer_forward);

  auto obstacle = std::make_shared<ObstacleCritic>();
  obstacle->configure(parent, plugin_name_, "ObstacleCritic");
  critics_.push_back(obstacle);
}

bool OmniPidPursuitController::rolloutTrajectory(
  const geometry_msgs::msg::Pose & start_pose,
  const ControlSample & u,
  const CriticData & data,
  Trajectory & trajectory) const
{
  trajectory.clear();
  trajectory.reserve(static_cast<size_t>(time_steps_) + 1);

  double x = start_pose.position.x;
  double y = start_pose.position.y;
  double theta = tf2::getYaw(start_pose.orientation);

  // Add initial point
  TrajectoryPoint tp;
  tp.x = x;
  tp.y = y;
  tp.theta = theta;
  tp.vx = u.vx;
  tp.vy = u.vy;
  tp.wz = u.wz;
  tp.time_from_start = 0.0;
  trajectory.push_back(tp);

  // Roll out dynamics
  for (int t = 0; t < time_steps_; ++t) {
    stepDynamics(x, y, theta, u.vx, u.vy, u.wz, model_dt_);

    tp.x = x;
    tp.y = y;
    tp.theta = theta;
    tp.vx = u.vx;
    tp.vy = u.vy;
    tp.wz = u.wz;
    tp.time_from_start = (t + 1) * model_dt_;
    trajectory.push_back(tp);
  }

  return true;
}

double OmniPidPursuitController::evaluateTrajectoryWithCritics(
  const Trajectory & trajectory,
  const CriticData & data,
  bool & collision) const
{
  collision = false;
  double total_cost = 0.0;

  for (const auto & critic : critics_) {
    if (!critic->isEnabled()) {
      continue;
    }

    bool critic_collision = false;
    double critic_cost = critic->score(trajectory, data, critic_collision);

    if (critic_collision) {
      collision = true;
      return std::numeric_limits<double>::infinity();
    }

    total_cost += critic->getWeight() * critic_cost;
  }

  return total_cost;
}

// ---------------- 动态参数回调 ----------------

rcl_interfaces::msg::SetParametersResult OmniPidPursuitController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".translation_kp") {
        translation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".translation_ki") {
        translation_ki_ = parameter.as_double();
      } else if (name == plugin_name_ + ".translation_kd") {
        translation_kd_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_kp") {
        rotation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_ki") {
        rotation_ki_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_kd") {
        rotation_kd_ = parameter.as_double();
      } else if (name == plugin_name_ + ".transform_tolerance") {
        double transform_tolerance = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
      } else if (name == plugin_name_ + ".min_max_sum_error") {
        min_max_sum_error_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_dist") {
        lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_lookahead_dist") {
        min_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        max_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        lookahead_time_ = parameter.as_double();
      } else if (name == plugin_name_ + ".use_rotate_to_heading_treshold") {
        use_rotate_to_heading_treshold_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
        min_approach_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".approach_velocity_scaling_dist") {
        approach_velocity_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_max") {
        v_linear_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_min") {
        v_linear_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_max") {
        v_angular_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_min") {
        v_angular_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_min") {
        curvature_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_max") {
        curvature_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".reduction_ratio_at_high_curvature") {
        reduction_ratio_at_high_curvature_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_forward_dist") {
        curvature_forward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_backward_dist") {
        curvature_backward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_velocity_scaling_factor_rate") {
        max_velocity_scaling_factor_rate_ = parameter.as_double();
      } else if (name == plugin_name_ + ".time_steps") {
        time_steps_ = static_cast<int>(parameter.as_double());
      } else if (name == plugin_name_ + ".model_dt") {
        model_dt_ = parameter.as_double();
      } else if (name == plugin_name_ + ".batch_size") {
        batch_size_ = static_cast<int>(parameter.as_double());
      } else if (name == plugin_name_ + ".vx_std") {
        vx_std_ = parameter.as_double();
      } else if (name == plugin_name_ + ".vy_std") {
        vy_std_ = parameter.as_double();
      } else if (name == plugin_name_ + ".wz_std") {
        wz_std_ = parameter.as_double();
      } else if (name == plugin_name_ + ".path_weight") {
        path_weight_ = parameter.as_double();
      } else if (name == plugin_name_ + ".obstacle_weight") {
        obstacle_weight_ = parameter.as_double();
      } else if (name == plugin_name_ + ".control_weight") {
        control_weight_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_interpolation") {
        use_interpolation_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_rotate_to_heading") {
        use_rotate_to_heading_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace pb_omni_pid_pursuit_controller

// 注册为 nav2_core 插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pb_omni_pid_pursuit_controller::OmniPidPursuitController, nav2_core::Controller)
