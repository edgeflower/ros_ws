// goal_approach_controller.cpp
// Nav2控制器wrapper：在接近目标时限制线速度，防止高速冲过目标点
// 原理：透明代理内部控制器（如MPPI），仅在距目标 < approach_distance 时
//       将合速度钳位到 approach_velocity

#include <cmath>
#include <memory>
#include <string>

#include "nav2_core/controller.hpp"
#include "tf2/utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace goal_approach_controller
{

class GoalApproachController : public nav2_core::Controller
{
public:
  GoalApproachController() = default;
  ~GoalApproachController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    logger_ = node->get_logger();

    // 声明本wrapper的参数
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".inner_plugin",
      rclcpp::ParameterValue("nav2_mppi_controller::MPPIController"));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".approach_distance",
      rclcpp::ParameterValue(1.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".approach_velocity",
      rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".direct_approach_distance",
      rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".direct_approach_kp",
      rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".obstacle_safe_distance",
      rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".min_speed_scale",
      rclcpp::ParameterValue(0.2));

    std::string inner_plugin_type;
    node->get_parameter(name + ".inner_plugin", inner_plugin_type);
    node->get_parameter(name + ".approach_distance", approach_distance_);
    node->get_parameter(name + ".approach_velocity", approach_velocity_);
    node->get_parameter(name + ".direct_approach_distance", direct_approach_distance_);
    node->get_parameter(name + ".direct_approach_kp", direct_approach_kp_);
    node->get_parameter(name + ".obstacle_safe_distance", obstacle_safe_distance_);
    node->get_parameter(name + ".min_speed_scale", min_speed_scale_);

    costmap_ros_ = costmap_ros;

    // 通过pluginlib加载内部控制器
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core", "nav2_core::Controller");
    inner_controller_ = loader_->createUniqueInstance(inner_plugin_type);
    inner_controller_->configure(parent, name, tf, costmap_ros);

    RCLCPP_INFO(
      logger_,
      "GoalApproachController: 包装 [%s], approach_distance=%.2f m, approach_velocity=%.2f m/s, "
      "direct_approach_distance=%.2f m, direct_approach_kp=%.2f, "
      "obstacle_safe_distance=%.2f m, min_speed_scale=%.2f",
      inner_plugin_type.c_str(), approach_distance_, approach_velocity_,
      direct_approach_distance_, direct_approach_kp_,
      obstacle_safe_distance_, min_speed_scale_);
  }

  void cleanup() override
  {
    inner_controller_->cleanup();
  }

  void activate() override
  {
    inner_controller_->activate();
  }

  void deactivate() override
  {
    inner_controller_->deactivate();
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if (!path.poses.empty()) {
      goal_ = path.poses.back();
    }
    inner_controller_->setPlan(path);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    auto cmd = inner_controller_->computeVelocityCommands(pose, velocity, goal_checker);

    // 动态限速：根据到最近障碍物的距离缩放线速度
    float dist_to_obs = computeDistanceToObstacle(
      static_cast<float>(pose.pose.position.x),
      static_cast<float>(pose.pose.position.y));
    float speed_scale = std::clamp(
      dist_to_obs / static_cast<float>(obstacle_safe_distance_),
      static_cast<float>(min_speed_scale_), 1.0f);
    cmd.twist.linear.x *= speed_scale;
    cmd.twist.linear.y *= speed_scale;

    // 计算到目标的距离
    double dx = goal_.pose.position.x - pose.pose.position.x;
    double dy = goal_.pose.position.y - pose.pose.position.y;
    double dist = std::hypot(dx, dy);

    if (dist < direct_approach_distance_) {
      // 近距离直接驱动模式：绕过 MPPI 的弧线输出，直接朝目标点走
      double target_speed = std::min(approach_velocity_, dist * direct_approach_kp_);
      if (dist > 0.01) {
        double global_vx = target_speed * (dx / dist);
        double global_vy = target_speed * (dy / dist);
        double yaw = tf2::getYaw(pose.pose.orientation);
        cmd.twist.linear.x = global_vx * std::cos(yaw) + global_vy * std::sin(yaw);
        cmd.twist.linear.y = -global_vx * std::sin(yaw) + global_vy * std::cos(yaw);
      } else {
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
      }
      cmd.twist.angular.z = 0.0;
    } else if (dist < approach_distance_) {
      double speed = std::hypot(cmd.twist.linear.x, cmd.twist.linear.y);
      if (speed > approach_velocity_) {
        double scale = approach_velocity_ / speed;
        cmd.twist.linear.x *= scale;
        cmd.twist.linear.y *= scale;
        // 角速度也按比例降低，避免原地打转
        cmd.twist.angular.z *= scale;
      }
    }

    return cmd;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    inner_controller_->setSpeedLimit(speed_limit, percentage);
  }

  /**
   * @brief 在costmap中扫描机器人周围，找到到最近致命障碍物的距离
   */
  float computeDistanceToObstacle(float robot_x, float robot_y)
  {
    auto * costmap = costmap_ros_->getCostmap();
    unsigned int mx, my;
    if (!costmap->worldToMap(
        static_cast<double>(robot_x), static_cast<double>(robot_y), mx, my))
    {
      return static_cast<float>(obstacle_safe_distance_);
    }

    const float resolution = static_cast<float>(costmap->getResolution());
    const int search_cells = static_cast<int>(
      std::ceil(obstacle_safe_distance_ / resolution));
    const unsigned int size_x = costmap->getSizeInCellsX();
    const unsigned int size_y = costmap->getSizeInCellsY();

    float min_dist = static_cast<float>(obstacle_safe_distance_);

    for (int dx = -search_cells; dx <= search_cells; ++dx) {
      for (int dy = -search_cells; dy <= search_cells; ++dy) {
        const int nx = static_cast<int>(mx) + dx;
        const int ny = static_cast<int>(my) + dy;
        if (nx < 0 || ny < 0 ||
          static_cast<unsigned int>(nx) >= size_x ||
          static_cast<unsigned int>(ny) >= size_y)
        {
          continue;
        }
        if (costmap->getCost(nx, ny) == nav2_costmap_2d::LETHAL_OBSTACLE) {
          float dist = std::sqrt(static_cast<float>(dx * dx + dy * dy)) * resolution;
          if (dist < min_dist) {min_dist = dist;}
        }
      }
    }

    return min_dist;
  }

private:
  pluginlib::UniquePtr<nav2_core::Controller> inner_controller_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  rclcpp::Logger logger_{rclcpp::get_logger("goal_approach_controller")};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  geometry_msgs::msg::PoseStamped goal_;
  double approach_distance_{1.5};
  double approach_velocity_{0.5};
  double direct_approach_distance_{0.5};
  double direct_approach_kp_{1.0};
  double obstacle_safe_distance_{1.0};
  double min_speed_scale_{0.2};
};

}  // namespace goal_approach_controller

PLUGINLIB_EXPORT_CLASS(
  goal_approach_controller::GoalApproachController,
  nav2_core::Controller)
