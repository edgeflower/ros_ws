#include "rm_behavior_tree/plugins/condition/check_blackboard.hpp"

namespace rm_behavior_tree
{

CheckBlackboard::CheckBlackboard(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckBlackboard::tick()
{
  auto value_opt = getInput<bool>("value");
  auto expected_opt = getInput<bool>("expected");

  if (!value_opt) {
    return BT::NodeStatus::FAILURE;
  }

  bool value = value_opt.value();
  bool expected = expected_opt ? expected_opt.value() : true;

  return (value == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::CheckBlackboard>("CheckBlackboard");
}
