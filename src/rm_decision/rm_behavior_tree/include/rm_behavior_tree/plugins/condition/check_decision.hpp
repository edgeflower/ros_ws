#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_DECISION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_DECISION_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/sentry_decision.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <behaviortree_cpp/basic_types.h>

namespace rm_behavior_tree
{

/**
 * @brief 条件节点：检查 Utility AI 决策模式是否匹配
 *
 * 这个节点从黑板读取 SubSentryDecision 存入的 SentryDecision 消息，
 * 判断当前模式是否与 expected_mode 匹配。
 *
 * == 工作流程 ==
 * 1. 从黑板读取最新的 SentryDecision 消息
 * 2. 检查消息是否在 0.5 秒内（超时 → FAILURE）
 * 3. 比较 current_mode 与 expected_mode
 * 4. 匹配 → 输出 target_pose/reason/current_mode 到黑板，返回 SUCCESS
 * 5. 不匹配 → 返回 FAILURE
 *
 * == 使用方式（XML） ==
 * <CheckDecision sentry_decision="{sentry_decision}"
 *                 expected_mode="ATTACK"
 *                 target_pose="{goal}"
 *                 reason="{reason}"
 *                 current_mode="{current_mode}"/>
 *
 * @param[in]  sentry_decision  黑板中的 SentryDecision 消息（由 SubSentryDecision 写入）
 * @param[in]  expected_mode    期望的模式名称（如 "ATTACK", "RETREAT" 等）
 * @param[out] target_pose      决策对应的目标点（map 坐标系）
 * @param[out] reason           决策原因（调试用）
 * @param[out] current_mode     当前实际的模式名称
 */
class CheckDecision : public BT::ConditionNode
{
public:
  CheckDecision(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      // 输入端口
      BT::InputPort<rm_decision_interfaces::msg::SentryDecision>(
        "sentry_decision", "黑板中的 SentryDecision 消息"),
      BT::InputPort<std::string>(
        "expected_mode", "期望的模式: IDLE/ATTACK/RETREAT/SUPPLY/PATROL/DEFEND_BASE"),
      // 输出端口
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "target_pose", "决策对应的目标点"),
      BT::OutputPort<std::string>(
        "reason", "决策原因"),
      BT::OutputPort<std::string>(
        "current_mode", "当前实际模式"),
    };
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_DECISION_HPP_
