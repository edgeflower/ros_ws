#include "rm_behavior_tree/plugins/condition/is_status_ok.hpp"
#include "rclcpp/rclcpp.hpp"
#include <functional>

namespace rm_behavior_tree {

IsStatusOKAction::IsStatusOKAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsStatusOKAction::checkRobotStatus , this), config)
{}

BT::NodeStatus IsStatusOKAction::checkRobotStatus()
{
    int hp_threshold, heat_threshold;
    auto msg = getInput<rm_decision_interfaces::msg::RobotStatus>("message");
    getInput("hp_threshold", hp_threshold);
    getInput("heat_threshold", heat_threshold);

    if (!msg) {
        RCLCPP_WARN(rclcpp::get_logger("IsStatusOK"), "⚠️ 未接收到 robot_status");
        return BT::NodeStatus::FAILURE;
    }

    int current_hp = msg->current_hp;
    int current_heat = msg->shooter_heat;

    RCLCPP_INFO(rclcpp::get_logger("IsStatusOK"),
        "[🔍 检查] current_hp=%d (阈值=%d), shooter_heat=%d (阈值=%d)",
        current_hp, hp_threshold, current_heat, heat_threshold);

    if (current_hp < hp_threshold || current_heat > heat_threshold) {
        RCLCPP_WARN(rclcpp::get_logger("IsStatusOK"), "❌ 状态异常：血量过低或热量过高");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("IsStatusOK"), "✅ 状态正常");
    return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<rm_behavior_tree::IsStatusOKAction>("IsStatusOK");
}
