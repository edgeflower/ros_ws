#include "rm_behavior_tree/plugins/action/set_goal.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <rclcpp/logging.hpp>
#include <cmath>

namespace rm_behavior_tree
{

SetGoalAction::SetGoalAction(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus SetGoalAction::tick()
{
    std::string goal_str;
    if (!getInput<std::string>("goal_string", goal_str)) {
        RCLCPP_ERROR(rclcpp::get_logger("SetGoal"), "Missing required input [goal_string]");
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped goal_pose;

    if (!parseGoalString(goal_str, goal_pose)) {
        RCLCPP_ERROR(rclcpp::get_logger("SetGoal"), "Failed to parse goal string: %s", goal_str.c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (setOutput("goal", goal_pose)) {
        RCLCPP_DEBUG(rclcpp::get_logger("SetGoal"),
                   "SetGoal: x=%.2f, y=%.2f",
                   goal_pose.pose.position.x,
                   goal_pose.pose.position.y
                   ); // Store yaw in z for simplicity
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("SetGoal"), "Failed to set output port [goal]");
        return BT::NodeStatus::FAILURE;
    }
}

bool SetGoalAction::parseGoalString(const std::string& str, geometry_msgs::msg::PoseStamped& pose)
{
    // 解析格式: "x;y;yaw;..." (取前7个值)
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, ';')) {
        tokens.push_back(token);
    }

    if (tokens.size() < 7) {
        RCLCPP_ERROR(rclcpp::get_logger("SetGoal"),
                    "Invalid goal string format. Expected at least 7 values, got %zu",
                    tokens.size());
        return false;
    }

    try {
        // x, y, yaw 使用前3个值
        pose.pose.position.x = std::stod(tokens[0]);
        pose.pose.position.y = std::stod(tokens[1]);
        double yaw = std::stod(tokens[2]);

        // position.z 设为 0
        pose.pose.position.z = 0.0;

        // 将 yaw 转换为 quaternion，简单起见存储在 orientation.z 中
        // 完整实现应使用 tf2::Quaternion
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;  // 临时存储yaw值
        pose.pose.orientation.w = 1.0;

        // 设置 header
        pose.header.frame_id = "map";
        // 时间戳由接收方设置

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("SetGoal"), "Failed to parse numeric values: %s", e.what());
        return false;
    }
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::SetGoalAction>("SetGoal");
}
