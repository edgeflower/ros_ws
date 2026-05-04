#include "rm_behavior_tree/plugins/action/patrol_utils.hpp"

namespace rm_behavior_tree
{
// All patrol utility nodes are fully implemented in header
} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::LoadWaypoints>("LoadWaypoints");
  factory.registerNodeType<rm_behavior_tree::GetCurrentWaypoint>(
      "GetCurrentWaypoint");
  factory.registerNodeType<rm_behavior_tree::WaitUntilReached>(
      "WaitUntilReached");
  factory.registerNodeType<rm_behavior_tree::WaitDuration>("WaitDuration");
  factory.registerNodeType<rm_behavior_tree::NextWaypoint>("NextWaypoint");
}
