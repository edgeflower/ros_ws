#include "rm_behavior_tree/plugins/condition/is_attacked.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/condition_node.h>
#include <ostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rm_decision_interfaces/msg/detail/robot_status__struct.hpp>

namespace rm_behavior_tree
{

IsAttackedAction::IsAttackedAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsAttackedAction::checkRobotAttacked, this), config)
{
}

BT::NodeStatus IsAttackedAction::checkRobotAttacked()
{
    auto msg = getInput<rm_decision_interfaces::msg::RobotStatus>("message");
    if (!msg) {
        //std::cout << "Missing required input [game_status]" << '\n';
        RCLCPP_WARN(rclcpp::get_logger("[IsAttacked]"),"Missing required input [game_status]");
        return BT::NodeStatus::FAILURE;
    }

    if (msg->is_attacked) {
        // 机器人受到攻击
        std::cout << "机器人受到攻击" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}
}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::IsAttackedAction>("IsAttacked");
}