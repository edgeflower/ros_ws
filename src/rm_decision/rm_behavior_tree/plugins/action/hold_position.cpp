#include "rm_behavior_tree/plugins/action/hold_position.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

HoldPositionAction::HoldPositionAction(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
    , hold_start_time_(0, 0, RCL_ROS_TIME)
{
    RCLCPP_INFO(rclcpp::get_logger("HoldPosition"), "HoldPositionAction initialized");
}

BT::NodeStatus HoldPositionAction::tick()
{
    rclcpp::Time now = rclcpp::Clock().now();

    // 检查是否启用
    bool enabled = true;
    auto enabled_result = getInput<bool>("enabled");
    if (enabled_result) {
        enabled = enabled_result.value();
    }

    if (!enabled) {
        // 未启用，清除保持状态
        if (is_holding_) {
            RCLCPP_INFO(rclcpp::get_logger("HoldPosition"),
                "Hold disabled, releasing position");
            is_holding_ = false;
        }
        setOutput("is_holding", false);
        return BT::NodeStatus::FAILURE;
    }

    // 获取当前目标
    armor_interfaces::msg::Target target;
    if (!getInput<armor_interfaces::msg::Target>("target", target)) {
        RCLCPP_WARN(rclcpp::get_logger("HoldPosition"), "No target input");
        setOutput("is_holding", false);
        return BT::NodeStatus::FAILURE;
    }

    // 获取机器人位姿
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header.frame_id = "map";
    robot_pose.pose.position.x = 0.0;
    robot_pose.pose.position.y = 0.0;
    robot_pose.pose.position.z = 0.0;
    robot_pose.pose.orientation.w = 1.0;

    auto pose_result = getInput<geometry_msgs::msg::PoseStamped>("robot_pose");
    bool has_robot_pose = pose_result.has_value();
    if (has_robot_pose) {
        robot_pose = pose_result.value();
    }

    // 检查是否超时
    if (is_holding_ && isTimeout(now)) {
        RCLCPP_INFO(rclcpp::get_logger("HoldPosition"),
                    "Hold timeout (%.1fs), releasing position",
                    (now - hold_start_time_).seconds());
        is_holding_ = false;
        setOutput("is_holding", false);
        return BT::NodeStatus::FAILURE;
    }

    // 如果没有在保持状态，或者目标发生了显著变化，更新保持的目标
    if (!is_holding_) {
        // 首次进入保持状态
        updateHeldGoal(target);
        hold_start_time_ = now;
        is_holding_ = true;

        RCLCPP_INFO(rclcpp::get_logger("HoldPosition"),
                    "Started holding position at [%.2f, %.2f], confidence: %.2f",
                    held_goal_.pose.position.x, held_goal_.pose.position.y, target.confidence);
    } else {
        // 检查目标是否发生了显著变化（可能需要切换目标）
        double dx = target.position.x - held_goal_.pose.position.x;
        double dy = target.position.y - held_goal_.pose.position.y;
        double distance_change = std::sqrt(dx * dx + dy * dy);

        // 如果目标移动超过2米，可能是一个新的敌人，更新保持的目标
        if (distance_change > 2.0) {
            RCLCPP_INFO(rclcpp::get_logger("HoldPosition"),
                        "Target changed significantly (%.2fm), updating held goal",
                        distance_change);
            updateHeldGoal(target);
        }
    }

    // 输出保持的目标
    setOutput("held_goal", held_goal_);
    setOutput("is_holding", true);

    return BT::NodeStatus::SUCCESS;
}

void HoldPositionAction::reset()
{
    is_holding_ = false;
    hold_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    RCLCPP_INFO(rclcpp::get_logger("HoldPosition"), "Reset - position released");
}

bool HoldPositionAction::isTimeout(const rclcpp::Time &now) const
{
    if (hold_start_time_.nanoseconds() == 0) {
        return false;
    }
    double elapsed = (now - hold_start_time_).seconds();
    return elapsed >= hold_timeout_;
}

void HoldPositionAction::updateHeldGoal(const armor_interfaces::msg::Target &target)
{
    held_goal_.header.stamp = rclcpp::Clock().now();
    held_goal_.header.frame_id = target.header.frame_id;

    // 如果有机器人位姿，可以调整目标为"朝向敌人的当前位姿"
    // 这里简化处理：直接使用目标的坐标
    held_goal_.pose.position.x = target.position.x;
    held_goal_.pose.position.y = target.position.y;
    held_goal_.pose.position.z = 0.0;

    // 保持朝向敌人的姿态
    double yaw = target.yaw;  // 使用目标提供的偏航角
    held_goal_.pose.orientation.x = 0.0;
    held_goal_.pose.orientation.y = 0.0;
    held_goal_.pose.orientation.z = std::sin(yaw / 2.0);
    held_goal_.pose.orientation.w = std::cos(yaw / 2.0);
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::HoldPositionAction>("HoldPosition");
}
