#include "rm_behavior_tree/plugins/action/write_to_blackboard.hpp"

namespace rm_behavior_tree
{
// WriteToBlackboard is fully implemented in header
} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::WriteToBlackboard>(
      "WriteToBlackboard");
}
