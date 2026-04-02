#include "rm_behavior_tree/plugins/condition/is_hp_ok.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

IsHpOkAction::IsHpOkAction(
    const std::string & name,
    const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsHpOkAction::checkHpStatus, this), config)
{
    RCLCPP_INFO(rclcpp::get_logger("IsHpOk"), "IsHpOk Condition initialized");
}

BT::NodeStatus IsHpOkAction::checkHpStatus()
{
    // 获取输入参数
    rm_decision_interfaces::msg::AllRobotHP all_robot_hp;
    if (!getInput("all_robot_hp", all_robot_hp)) {
        RCLCPP_WARN(rclcpp::get_logger("IsHpOk"), "No all_robot_hp input available");
        return BT::NodeStatus::FAILURE;
    }

    int hp_threshold = 100;
    getInput("hp_threshold", hp_threshold);

    // 获取哨兵 HP (7号机器人，固定使用红队)
    // TODO: 可改为从参数读取队伍颜色
    int current_hp = all_robot_hp.red_7_robot_hp;

    // 判断 HP 是否低于阈值
    if (current_hp < hp_threshold) {
        RCLCPP_WARN(rclcpp::get_logger("IsHpOk"),
                    "HP LOW: current_hp=%d < threshold=%d",
                    current_hp, hp_threshold);
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("IsHpOk"),
                 "HP OK: current_hp=%d >= threshold=%d",
                 current_hp, hp_threshold);

    return BT::NodeStatus::SUCCESS;
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::IsHpOkAction>("IsHpOk");
}
