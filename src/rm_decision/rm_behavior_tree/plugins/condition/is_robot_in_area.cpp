#include "rm_behavior_tree/plugins/condition/is_robot_in_area.hpp"
#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <rclcpp/logging.hpp>
#include <rm_decision_interfaces/msg/detail/robot_area_status__struct.hpp>

namespace rm_behavior_tree {

IsRobotInArea::IsRobotInArea(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsRobotInArea::checkRobotInArea , this), config)
{}

BT::NodeStatus IsRobotInArea::checkRobotInArea()
{
    auto msg = getInput<rm_decision_interfaces::msg::RobotAreaStatus>("robot_area_status");

    std::string target_area_name;
    std::string target_area_type;

    getInput("target_area_name", target_area_name);
    getInput("target_area_type", target_area_type);

    if (!msg) {
        RCLCPP_WARN(rclcpp::get_logger("IsRobotInArea"),
                    "未接收到 robot_area_status");
        return BT::NodeStatus::FAILURE;
    }

    if (!msg->is_in_area || msg->matched_area_count <= 0) {
        return BT::NodeStatus::FAILURE;
    }

    for (size_t i = 0; i < msg->matched_area_names.size(); ++i) {
        const std::string & name = msg->matched_area_names[i];

        std::string type;
        if (i < msg->matched_area_types.size()) {
            type = msg->matched_area_types[i];
        }

        bool name_ok = target_area_name.empty() || name == target_area_name;
        bool type_ok = target_area_type.empty() || type == target_area_type;

        if (name_ok && type_ok) {
            RCLCPP_DEBUG(rclcpp::get_logger("IsRobotInArea"),
                         "机器人在目标区域内: name=%s, type=%s",
                         name.c_str(), type.c_str());
            return BT::NodeStatus::SUCCESS;
        }
    }

    return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<rm_behavior_tree::IsRobotInArea>("IsRobotInArea");
}
