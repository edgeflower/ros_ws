#include "rm_behavior_tree/plugins/action/follow_waypoints.hpp"

namespace rm_behavior_tree
{
// FollowWaypointsAction is fully implemented in header
} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::FollowWaypointsAction, "FollowWaypoints");
