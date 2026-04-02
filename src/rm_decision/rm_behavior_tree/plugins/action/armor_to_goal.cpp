#include "rm_behavior_tree/plugins/action/armor_to_goal.hpp"
#include <armor_interfaces/msg/detail/armor__struct.hpp>
#include <behaviortree_cpp/basic_types.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <cmath>

namespace rm_behavior_tree
{

ArmorToGoalAction::ArmorToGoalAction(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config), rclcpp::Node("armor_to_goal_node")
{
    RCLCPP_INFO(this->get_logger(), "ArmorToGoalAction initialized: direct enemy position as goal");
}

BT::NodeStatus ArmorToGoalAction::tick()
{
    // 1. 获取敌人位置 (target_message 已在 map 坐标系)
    armor_interfaces::msg::Target target;
    auto target_result = getInput<armor_interfaces::msg::Target>("target_message");
    if (!target_result.has_value()) {
        RCLCPP_DEBUG(this->get_logger(), "No target_message received");
        return BT::NodeStatus::FAILURE;
    }
    target = target_result.value();

    // 注意：置信度检查已移至 ConfidenceHysteresis 节点统一处理
    // 本节点只负责将敌人位置转换为导航目标点

    // 2. 构建目标点：直接使用敌人位置
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = this->now();
    goal.header.frame_id = target.header.frame_id;

    // 位置：直接使用敌人位置
    goal.pose.position.x = target.position.x;
    goal.pose.position.y = target.position.y;
    goal.pose.position.z = 0.0;

    // 朝向：使用敌人的 yaw（如果有的话），否则默认朝向
    // 如果 target 中没有 yaw，可以保持当前朝向或设为 0
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;  // 默认朝向

    // 3. 设置输出端口
    setOutput("goal_pose", goal);
    
    RCLCPP_INFO(this->get_logger(), "Goal set to enemy position:");
    RCLCPP_INFO(this->get_logger(), "  Enemy: (%.2f, %.2f)", target.position.x, target.position.y);
    RCLCPP_INFO(this->get_logger(), "  Goal: (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);

    return BT::NodeStatus::SUCCESS;
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<rm_behavior_tree::ArmorToGoalAction>("ArmorToGoal");
}
