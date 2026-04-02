#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_REACHED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_REACHED_HPP_

#include "behaviortree_cpp/condition_node.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <mutex>

namespace rm_behavior_tree
{

// Goal status enumeration (must match GoalManager)
enum GoalStatusEnum : uint8_t {
    IDLE = 0,
    VISITING = 1,
    DONE = 2,
    BLOCKED = 3,
    RETRYING = 4
};

// Goal status structure for Blackboard sharing
struct GoalStatusEntry {
    int32_t point_id;
    uint8_t status;

    GoalStatusEntry() : point_id(0), status(IDLE) {}
};

class IsGoalReachedCondition : public BT::SimpleConditionNode
{
public:
    IsGoalReachedCondition(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int32_t>("goal_id", "ID of the goal to check"),
            BT::InputPort<bool>("check_current", false, "Check if currently selected goal is reached"),
            BT::InputPort<int32_t>("current_goal_id", 0, "Current goal ID from GoalManager"),
            BT::InputPort<std::vector<GoalStatusEntry>>("goal_statuses", "Goal statuses from GoalManager"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose", "Current robot pose for approximate check"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("current_goal", "Current goal pose for approximate check"),
            BT::InputPort<bool>("allow_approximate", false, "Allow success based on proximity even if not DONE"),
            BT::InputPort<double>("approximate_distance", 0.2, "Distance threshold for approximate success (meters)")
        };
    }

    BT::NodeStatus checkGoalReached();

    // Helper to check if goal is reached (DONE status)
    bool isGoalReached(int32_t goal_id, const std::vector<GoalStatusEntry>& statuses) const;

    // Helper to check approximate arrival
    bool isApproximatelyReached(const geometry_msgs::msg::PoseStamped& robot_pose,
                               const geometry_msgs::msg::PoseStamped& goal_pose) const;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_REACHED_HPP_
