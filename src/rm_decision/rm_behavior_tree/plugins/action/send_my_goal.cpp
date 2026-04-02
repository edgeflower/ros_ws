#include "rm_behavior_tree/plugins/action/send_my_goal.hpp"
#include <sstream>
#include <vector>

namespace rm_behavior_tree
{

// 静态成员初始化
std::map<std::string, int> SendMyGoalAction::visit_counts_;
std::mutex SendMyGoalAction::visit_counts_mutex_;

SendMyGoalAction::SendMyGoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
  // 读取停滞检测参数
  getInput("stall_threshold", stall_threshold_);
  getInput("stall_check_interval", stall_check_interval_);
  getInput("min_distance_for_stall", min_distance_for_stall_);
}

void SendMyGoalAction::resetVisitCounts()
{
  std::lock_guard<std::mutex> lock(visit_counts_mutex_);
  visit_counts_.clear();
  RCLCPP_INFO(rclcpp::get_logger("SendMyGoalAction"), "All visit counts have been reset.");
}

bool SendMyGoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  // 获取点位ID和最大访问次数
  std::string goal_id;
  int max_visits = 100;  // 默认值

  // 检查是否提供了 goal_id
  if (getInput<std::string>("goal_id", goal_id)) {
    // 获取最大访问次数
    if (!getInput<int>("max_visits", max_visits)) {
      max_visits = 100;  // 如果未提供，使用默认值100
    }

    // 检查访问次数
    {
      std::lock_guard<std::mutex> lock(visit_counts_mutex_);
      int current_count = visit_counts_[goal_id];

      if (current_count >= max_visits) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Goal '%s' has reached max visits (%d). Skipping.",
          goal_id.c_str(), max_visits);
        return false;  // 返回false将不会发送目标
      }

      // 增加访问计数
      visit_counts_[goal_id] = current_count + 1;
      RCLCPP_INFO(
        node_->get_logger(),
        "Goal '%s' visit count: %d/%d",
        goal_id.c_str(), visit_counts_[goal_id], max_visits);
    }
  }

  std::string goal_str;
  if (!getInput<std::string>("my_goal_pose", goal_str)) {
    throw BT::RuntimeError("error reading port [my_goal_pose]");
  }

  // 手动解析字符串: "x;y;z;qx;qy;qz;qw"
  std::vector<std::string> parts;
  std::stringstream ss(goal_str);
  std::string token;
  while (std::getline(ss, token, ';')) {
    parts.push_back(token);
  }

  if (parts.size() != 7) {
    throw BT::RuntimeError("Invalid my_goal_pose format. Expected: x;y;z;qx;qy;qz;qw");
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = std::stod(parts[0]);
  pose.pose.position.y = std::stod(parts[1]);
  pose.pose.position.z = std::stod(parts[2]);
  pose.pose.orientation.x = std::stod(parts[3]);
  pose.pose.orientation.y = std::stod(parts[4]);
  pose.pose.orientation.z = std::stod(parts[5]);
  pose.pose.orientation.w = std::stod(parts[6]);

  goal.pose = pose;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = rclcpp::Clock().now();

  // 重置停滞检测状态（新目标）
  last_distance_ = -1.0;  // 标记为未初始化
  stall_detection_enabled_ = true;

  /*
  // clang-format off
  std::cout << "My_goal_pose: [ "
    << std::fixed << std::setprecision(1)
    << goal.pose.pose.position.x << ", "
    << goal.pose.pose.position.y << ", "
    << goal.pose.pose.position.z << ", "
    << goal.pose.pose.orientation.x << ", "
    << goal.pose.pose.orientation.y << ", "
    << goal.pose.pose.orientation.z << ", "
    << goal.pose.pose.orientation.w << " ]\n";
  // clang-format on
  */

  return true;
}

void SendMyGoalAction::onHalt()
{
  this->halt();  // 或 cancel goal
  RCLCPP_INFO(node_->get_logger(), "SendMyGoalAction has been halted.");
}

BT::NodeStatus SendMyGoalAction::onResultReceived(const WrappedResult & wr)
{
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Success!!!");
      return BT::NodeStatus::SUCCESS;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(node_->get_logger(), "Goal was aborted");
      return BT::NodeStatus::FAILURE;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(node_->get_logger(), "Goal was canceled");
      std::cout << "Goal was canceled" << '\n';
      return BT::NodeStatus::FAILURE;
      break;
    default:
      RCLCPP_INFO(node_->get_logger(), "Unknown result code");
      return BT::NodeStatus::FAILURE;
      break;
  }
}

BT::NodeStatus SendMyGoalAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  // 如果停滞检测被禁用，直接返回 RUNNING
  if (!stall_detection_enabled_) {
    return BT::NodeStatus::RUNNING;
  }

  double current_distance = feedback->distance_remaining;
  auto current_time = node_->now();

  // 首次调用：初始化状态
  if (last_distance_ < 0) {
    last_distance_ = current_distance;
    last_check_time_ = current_time;
    return BT::NodeStatus::RUNNING;
  }

  double elapsed = (current_time - last_check_time_).seconds();

  // 检测间隔未到
  if (elapsed < stall_check_interval_) {
    return BT::NodeStatus::RUNNING;
  }

  // 执行停滞检测
  double distance_change = std::abs(current_distance - last_distance_);

  // 条件：距离目标足够远 + 距离几乎没变
  if (current_distance > min_distance_for_stall_ && distance_change < stall_threshold_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[SendMyGoal] STALL DETECTED: dist=%.2fm, change=%.3fm in %.1fs (threshold=%.3fm)",
      current_distance, distance_change, elapsed, stall_threshold_);
    return BT::NodeStatus::FAILURE;
  }

  // 更新状态
  last_distance_ = current_distance;
  last_check_time_ = current_time;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SendMyGoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(node_->get_logger(), "SendGoalAction failed with error code: %d", error);
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SendMyGoalAction, "SendMyGoal");