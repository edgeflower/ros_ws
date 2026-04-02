#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ATTACKED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ATTACKED_HPP_

#include <rm_decision_interfaces/msg/detail/robot_status__struct.hpp>
#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/robot_status.hpp"

namespace rm_behavior_tree
{

/**
* @brief condition节点，用于判断机器人是否被攻击掉血
* @param[in] message 机器人状态话题 id
*/
class IsAttackedAction : public BT::ConditionNode
{
public:
  IsAttackedAction(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<rm_decision_interfaces::msg::RobotStatus>("message")};
  }

private:
  bool initialized_ = false;
  int robot_hp_hold_ = 0;
};
} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ATTACKED_HPP_