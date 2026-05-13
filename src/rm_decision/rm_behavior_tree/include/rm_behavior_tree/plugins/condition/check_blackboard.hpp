#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_BLACKBOARD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_BLACKBOARD_HPP_

#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{

class CheckBlackboard : public BT::ConditionNode
{
public:
  CheckBlackboard(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("value", "Blackboard bool value to check"),
      BT::InputPort<bool>("expected", true, "Expected value to compare against")
    };
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_BLACKBOARD_HPP_
