#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PATROL_UTILS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PATROL_UTILS_HPP_

#include <chrono>
#include <cmath>
#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rm_behavior_tree/plugins/action/waypoint_utils.hpp"

namespace rm_behavior_tree
{

class LoadWaypoints : public BT::SyncActionNode
{
public:
  LoadWaypoints(const std::string & name, const BT::NodeConfiguration & config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<std::string>("waypoint_file", "航点 CSV 文件的绝对路径"),
        BT::InputPort<std::string>("frame_id", "map", "坐标系 ID"),
        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
            "waypoints", "航点列表"),
        BT::OutputPort<std::vector<double>>(
            "wait_times", "每个航点的等待时间 (秒)"),
        BT::OutputPort<int>("total_waypoints", "航点总数"),
        BT::OutputPort<int>("wp_idx", "航点索引（初始化为0）"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto file_res = getInput<std::string>("waypoint_file");
    if (!file_res) {
      RCLCPP_ERROR(
          rclcpp::get_logger("LoadWaypoints"), "读取端口 [waypoint_file] 时出错: %s",
          file_res.error().c_str());
      return BT::NodeStatus::FAILURE;
    }

    auto frame_res = getInput<std::string>("frame_id");
    const std::string frame_id =
        (frame_res && !frame_res.value().empty()) ? frame_res.value() : "map";

    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    std::vector<double> wait_times;
    if (!loadWaypointsFromCSV(file_res.value(), waypoints, frame_id, &wait_times)) {
      RCLCPP_ERROR(
          rclcpp::get_logger("LoadWaypoints"), "无法从文件加载航点: %s",
          file_res.value().c_str());
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("LoadWaypoints"), "已加载 %lu 个航点",
        static_cast<unsigned long>(waypoints.size()));

    setOutput("waypoints", waypoints);
    setOutput("wait_times", wait_times);
    setOutput("total_waypoints", static_cast<int>(waypoints.size()));
    setOutput("wp_idx", 0);
    return BT::NodeStatus::SUCCESS;
  }
};

class GetCurrentWaypoint : public BT::SyncActionNode
{
public:
  GetCurrentWaypoint(const std::string & name, const BT::NodeConfiguration & config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
            "waypoints", "航点列表"),
        BT::InputPort<std::vector<double>>(
            "wait_times", "每个航点的等待时间列表"),
        BT::InputPort<int>("wp_idx", "当前航点索引"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(
            "current_goal", "当前目标点"),
        BT::OutputPort<double>("current_wait_sec", "当前航点的等待时间"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto wp_res =
        getInput<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints");
    auto wt_res = getInput<std::vector<double>>("wait_times");
    auto idx_res = getInput<int>("wp_idx");

    if (!wp_res || !idx_res) {
      RCLCPP_ERROR(rclcpp::get_logger("GetCurrentWaypoint"), "读取端口失败");
      return BT::NodeStatus::FAILURE;
    }

    const auto & waypoints = wp_res.value();
    int idx = idx_res.value();

    if (waypoints.empty() || idx < 0 ||
        idx >= static_cast<int>(waypoints.size())) {
      RCLCPP_ERROR(
          rclcpp::get_logger("GetCurrentWaypoint"),
          "航点索引越界: idx=%d, size=%lu", idx,
          static_cast<unsigned long>(waypoints.size()));
      return BT::NodeStatus::FAILURE;
    }

    auto goal = waypoints[idx];
    goal.header.stamp = rclcpp::Clock().now();

    double wait_sec = 0.0;
    if (wt_res && idx < static_cast<int>(wt_res.value().size())) {
      wait_sec = wt_res.value()[idx];
    }

    RCLCPP_INFO(
        rclcpp::get_logger("GetCurrentWaypoint"),
        "当前目标航点[%d]: [%.2f, %.2f], 等待 %.1fs", idx,
        goal.pose.position.x, goal.pose.position.y, wait_sec);

    setOutput("current_goal", goal);
    setOutput("current_wait_sec", wait_sec);
    return BT::NodeStatus::SUCCESS;
  }
};

class WaitUntilReached : public BT::StatefulActionNode
{
public:
  WaitUntilReached(const std::string & name, const BT::NodeConfiguration & config)
      : BT::StatefulActionNode(name, config)
  {
    std::call_once(init_flag_, []() {
      node_ = std::make_shared<rclcpp::Node>("wait_until_reached");
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
      tf_listener_ =
          std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);
      std::thread([]() { rclcpp::spin(node_); }).detach();
    });
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "目标位姿"),
        BT::InputPort<double>("tolerance", 0.5, "到达判定距离阈值 (m)"),
    };
  }

  BT::NodeStatus onStart() override
  {
    auto goal_res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
    auto tol_res = getInput<double>("tolerance");

    if (!goal_res) {
      RCLCPP_ERROR(node_->get_logger(), "WaitUntilReached: 读取 goal_pose 失败");
      return BT::NodeStatus::FAILURE;
    }

    goal_ = goal_res.value();
    tolerance_ = (tol_res) ? tol_res.value() : 0.5;

    RCLCPP_INFO(
        node_->get_logger(),
        "WaitUntilReached: 等待到达 [%.2f, %.2f], 阈值=%.2f",
        goal_.pose.position.x, goal_.pose.position.y, tolerance_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "WaitUntilReached: TF 查询失败: %s", ex.what());
      return BT::NodeStatus::RUNNING;
    }

    double dx = transform.transform.translation.x - goal_.pose.position.x;
    double dy = transform.transform.translation.y - goal_.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < tolerance_) {
      RCLCPP_INFO(
          node_->get_logger(), "WaitUntilReached: 已到达目标点 (距离=%.2f)",
          distance);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    RCLCPP_WARN(node_->get_logger(), "WaitUntilReached: 被中断");
  }

private:
  inline static std::shared_ptr<rclcpp::Node> node_ = nullptr;
  inline static std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
  inline static std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;
  inline static std::once_flag init_flag_;
  geometry_msgs::msg::PoseStamped goal_;
  double tolerance_ = 0.5;
};

class WaitDuration : public BT::StatefulActionNode
{
public:
  WaitDuration(const std::string & name, const BT::NodeConfiguration & config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("duration_sec", 5.0, "等待时长 (秒)，0 则跳过"),
    };
  }

  BT::NodeStatus onStart() override
  {
    auto dur_res = getInput<double>("duration_sec");
    duration_ = (dur_res) ? dur_res.value() : 0.0;

    if (duration_ <= 0.0) {
      return BT::NodeStatus::SUCCESS;
    }

    start_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
        rclcpp::get_logger("WaitDuration"), "在航点等待 %.1f 秒", duration_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto elapsed = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - start_time_)
                       .count();

    if (elapsed >= duration_) {
      RCLCPP_INFO(rclcpp::get_logger("WaitDuration"), "等待完成");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    RCLCPP_WARN(rclcpp::get_logger("WaitDuration"), "等待被中断");
  }

private:
  double duration_ = 0.0;
  std::chrono::steady_clock::time_point start_time_;
};

class NextWaypoint : public BT::SyncActionNode
{
public:
  NextWaypoint(const std::string & name, const BT::NodeConfiguration & config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::BidirectionalPort<int>("wp_idx", "当前航点索引"),
        BT::InputPort<int>("total_waypoints", "航点总数"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto idx_res = getInput<int>("wp_idx");
    auto total_res = getInput<int>("total_waypoints");

    if (!idx_res || !total_res) {
      RCLCPP_ERROR(rclcpp::get_logger("NextWaypoint"), "读取端口失败");
      return BT::NodeStatus::FAILURE;
    }

    int idx = idx_res.value();
    int total = total_res.value();
    int next = (idx + 1) % total;

    RCLCPP_INFO(
        rclcpp::get_logger("NextWaypoint"), "航点切换: %d -> %d (共 %d 个)", idx,
        next, total);

    setOutput("wp_idx", next);
    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PATROL_UTILS_HPP_
