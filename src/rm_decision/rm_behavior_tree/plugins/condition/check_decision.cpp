#include "rm_behavior_tree/plugins/condition/check_decision.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>

namespace rm_behavior_tree
{

CheckDecision::CheckDecision(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckDecision::tick()
{
  // ------------------------------------------------------------------
  // 第一步：从黑板读取 SentryDecision 消息
  // ------------------------------------------------------------------
  auto msg = getInput<rm_decision_interfaces::msg::SentryDecision>("sentry_decision");
  if (!msg) {
    RCLCPP_WARN(rclcpp::get_logger("CheckDecision"),
      "黑板中未找到 sentry_decision，请确保 SubSentryDecision 已在树中运行");
    return BT::NodeStatus::FAILURE;
  }

  // ------------------------------------------------------------------
  // 第二步：检查数据是否超时（0.5 秒内有效）
  // 使用消息 header 中的时间戳与当前时间对比
  // ------------------------------------------------------------------
  auto now = rclcpp::Clock().now();
  auto msg_time = rclcpp::Time(msg->header.stamp);
  double age_sec = (now - msg_time).seconds();

  if (age_sec > 0.5 || age_sec < -1.0) {
    // 超过 0.5 秒视为数据过期；负值过大说明时钟跳变，也视为无效
    RCLCPP_WARN(rclcpp::get_logger("CheckDecision"),
      "sentry_decision 数据超时 (%.2f 秒)，当前模式: %s",
      age_sec, msg->mode.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // ------------------------------------------------------------------
  // 第三步：读取期望的模式
  // ------------------------------------------------------------------
  std::string expected_mode;
  getInput("expected_mode", expected_mode);

  // ------------------------------------------------------------------
  // 第四步：比较当前模式与期望模式
  // ------------------------------------------------------------------
  const std::string & current_mode = msg->mode;

  if (current_mode != expected_mode) {
    // 模式不匹配，返回 FAILURE
    // 仍然输出 current_mode 和 reason，方便调试
    setOutput("current_mode", current_mode);
    setOutput("reason", msg->reason);
    return BT::NodeStatus::FAILURE;
  }

  // ------------------------------------------------------------------
  // 第五步：模式匹配 → 输出到黑板，返回 SUCCESS
  // ------------------------------------------------------------------
  setOutput("target_pose", msg->target_pose);
  setOutput("reason", msg->reason);
  setOutput("current_mode", current_mode);

  RCLCPP_DEBUG(rclcpp::get_logger("CheckDecision"),
    "模式匹配: %s, 原因: %s, 目标: (%.2f, %.2f)",
    current_mode.c_str(), msg->reason.c_str(),
    msg->target_pose.pose.position.x, msg->target_pose.pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::CheckDecision>("CheckDecision");
}
