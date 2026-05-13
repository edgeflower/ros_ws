#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_SENTRY_DECISION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_SENTRY_DECISION_HPP_

#include <memory>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/sentry_decision.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 订阅 /sentry_decision 话题，将决策消息存入黑板
 *
 * 这个节点的作用是"数据桥梁"：
 * 把 utility_ai_node 发布的 SentryDecision 消息接收到行为树的黑板中，
 * 供后续的 CheckDecision 等条件节点读取。
 *
 * 使用方式（XML）：
 *   <SubSentryDecision topic_name="/sentry_decision" sentry_decision="{sentry_decision}"/>
 *
 * 输出端口：
 *   sentry_decision - 最新的 SentryDecision 消息（整个消息存入黑板）
 */
class SubSentryDecisionAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::SentryDecision>
{
public:
  SubSentryDecisionAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<rm_decision_interfaces::msg::SentryDecision>("sentry_decision"),
    };
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::SentryDecision> & last_msg) override;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_SENTRY_DECISION_HPP_
