#include "rm_behavior_tree/plugins/action/abort_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

// Node is fully implemented in header as SyncActionNode
// Registration is done here

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::AbortNavigationAction>("AbortNavigation");
}
