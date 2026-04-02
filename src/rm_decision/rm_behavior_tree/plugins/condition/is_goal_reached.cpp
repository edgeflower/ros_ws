// IsGoalReached Condition Node - Check if a goal point has been reached
// Now uses Blackboard for state sharing with GoalManager (dual-source sync)
// File: is_goal_reached.cpp

#include "rm_behavior_tree/plugins/condition/is_goal_reached.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

namespace rm_behavior_tree
{

IsGoalReachedCondition::IsGoalReachedCondition(
    const std::string & name,
    const BT::NodeConfiguration & config)
: BT::SimpleConditionNode(name, std::bind(&IsGoalReachedCondition::checkGoalReached, this), config)
{
    RCLCPP_INFO(rclcpp::get_logger("IsGoalReached"), "IsGoalReachedCondition initialized (Blackboard mode)");
}

BT::NodeStatus IsGoalReachedCondition::checkGoalReached()
{
    // Get goal_id from input port
    int32_t goal_id = 0;
    if (!getInput<int32_t>("goal_id", goal_id)) {
        RCLCPP_ERROR(rclcpp::get_logger("IsGoalReached"), "Failed to get goal_id from input port");
        return BT::NodeStatus::FAILURE;
    }

    // Get goal statuses from Blackboard (shared by GoalManager)
    std::vector<GoalStatusEntry> goal_statuses;
    if (!getInput<std::vector<GoalStatusEntry>>("goal_statuses", goal_statuses)) {
        RCLCPP_WARN(rclcpp::get_logger("IsGoalReached"), "Failed to get goal_statuses from Blackboard");
        // Fall back to checking approximate arrival if possible
    }

    // Check if we should verify the current goal
    bool check_current = false;
    getInput<bool>("check_current", check_current);

    if (check_current) {
        int32_t current_goal_id = 0;
        if (getInput<int32_t>("current_goal_id", current_goal_id)) {
            if (current_goal_id != goal_id) {
                // Not the current goal, return FAILURE
                RCLCPP_DEBUG(rclcpp::get_logger("IsGoalReached"), "Goal ID %u is not the current goal (%u)",
                             goal_id, current_goal_id);
                return BT::NodeStatus::FAILURE;
            }
        }
    }

    // Primary check: Is goal marked as DONE in GoalManager?
    if (isGoalReached(goal_id, goal_statuses)) {
        RCLCPP_DEBUG(rclcpp::get_logger("IsGoalReached"), "Goal ID %u marked as DONE in GoalManager", goal_id);
        return BT::NodeStatus::SUCCESS;
    }

    // Secondary check: Allow approximate arrival?
    bool allow_approximate = false;
    getInput<bool>("allow_approximate", allow_approximate);

    if (allow_approximate) {
        geometry_msgs::msg::PoseStamped robot_pose;
        geometry_msgs::msg::PoseStamped current_goal;

        if (getInput<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose) &&
            getInput<geometry_msgs::msg::PoseStamped>("current_goal", current_goal)) {

            if (isApproximatelyReached(robot_pose, current_goal)) {
                RCLCPP_INFO(rclcpp::get_logger("IsGoalReached"), "Goal ID %u approximately reached (proximity check)", goal_id);
                return BT::NodeStatus::SUCCESS;
            }
        }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("IsGoalReached"), "Goal ID %u not yet reached", goal_id);
    return BT::NodeStatus::FAILURE;
}

bool IsGoalReachedCondition::isGoalReached(int32_t goal_id, const std::vector<GoalStatusEntry>& statuses) const
{
    for (const auto& entry : statuses) {
        if (entry.point_id == goal_id) {
            // Status: 0=IDLE, 1=VISITING, 2=DONE, 3=BLOCKED, 4=RETRYING
            return entry.status == DONE;
        }
    }
    return false;
}

bool IsGoalReachedCondition::isApproximatelyReached(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const geometry_msgs::msg::PoseStamped& goal_pose) const
{
    // Get approximate distance threshold
    double threshold = 0.2;  // default
    const_cast<IsGoalReachedCondition*>(this)->getInput<double>("approximate_distance", threshold);

    // Calculate Euclidean distance
    double dx = robot_pose.pose.position.x - goal_pose.pose.position.x;
    double dy = robot_pose.pose.position.y - goal_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    return distance <= threshold;
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<rm_behavior_tree::IsGoalReachedCondition>("IsGoalReached");
}
