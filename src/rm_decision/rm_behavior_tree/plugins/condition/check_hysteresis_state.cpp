#include "rm_behavior_tree/plugins/condition/check_hysteresis_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

// Node is fully implemented in header as SimpleConditionNode
// Registration is done here

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::CheckHysteresisState>("CheckHysteresisState");
}
