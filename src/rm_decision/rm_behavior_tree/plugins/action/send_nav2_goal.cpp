#include "rm_behavior_tree/plugins/action/send_nav2_goal.hpp"

namespace rm_behavior_tree
{
// SendNav2Goal is fully implemented in header
} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SendNav2Goal, "SendNav2Goal");
