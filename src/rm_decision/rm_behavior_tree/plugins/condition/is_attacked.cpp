#include "rm_behavior_tree/plugins/condition/is_attacked.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/condition_node.h>
#include <ostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rm_decision_interfaces/msg/detail/robot_status__struct.hpp>

namespace rm_behavior_tree {

IsAttackedAction::IsAttackedAction(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
{
}

BT::NodeStatus IsAttackedAction::tick()
{
    // 检测 halt/reset：如果节点状态是 IDLE，说明被 halt 或树被 reset 了
    // 需要重新初始化，防止跨局/跨阶段的数据污染
    if (status() == BT::NodeStatus::IDLE) {
        initialized_ = false;
    }

    auto msg = getInput<rm_decision_interfaces::msg::RobotStatus>("message");
    if (!msg) {
        // 配置错误：输入端口未连接，只警告一次
        RCLCPP_WARN(rclcpp::get_logger("IsAttacked"),
                    "Missing required input [RobotStatus]. Check BT configuration.");
        return BT::NodeStatus::FAILURE;
    }

    int current_hp = msg->current_hp;

    // 第一次初始化或被重置后重新同步
    if (!initialized_) {
        robot_hp_hold_ = current_hp;
        initialized_ = true;
        RCLCPP_DEBUG(rclcpp::get_logger("IsAttacked"),
                     "Initialized with HP: %d", robot_hp_hold_);
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus result = BT::NodeStatus::FAILURE;

    // 只有在血量【严格下降】时才返回 SUCCESS
    // 注意：回血掩盖伤害的场景下，这里会漏检
    // 但在大多数 RM 场景下，瞬间回血量 > 扣血量的情况较少
    if (current_hp < robot_hp_hold_) {
        RCLCPP_INFO(rclcpp::get_logger("IsAttacked"),
                    "机器人受到攻击! HP: %d -> %d (Δ%d)",
                    robot_hp_hold_, current_hp, current_hp - robot_hp_hold_);
        result = BT::NodeStatus::SUCCESS;
    }

    // 无论血量是掉了、涨了还是没变，都同步最新的血量基准
    robot_hp_hold_ = current_hp;

    return result;
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::IsAttackedAction>("IsAttacked");
}
