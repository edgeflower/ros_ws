#include "rm_behavior_tree/plugins/action/sub_sentry_decision.hpp"

namespace rm_behavior_tree
{

SubSentryDecisionAction::SubSentryDecisionAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::SentryDecision>(name, conf, params)
{
}

BT::NodeStatus SubSentryDecisionAction::onTick(
  const std::shared_ptr<rm_decision_interfaces::msg::SentryDecision> & last_msg)
{
  // 如果收到了消息，把完整的 SentryDecision 存到黑板里
  if (last_msg) {
    RCLCPP_DEBUG(logger(), "[%s] decision: mode=%s, reason=%s",
      name().c_str(), last_msg->mode.c_str(), last_msg->reason.c_str());
    setOutput("sentry_decision", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubSentryDecisionAction, "SubSentryDecision");
