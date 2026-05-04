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
    RCLCPP_INFO(this->get_logger(), "ArmorToGoalAction initialized: with offset and velocity prediction");
}

BT::NodeStatus ArmorToGoalAction::tick()
{
    // 1. 获取敌人位置
    armor_interfaces::msg::Target target;
    auto target_result = getInput<armor_interfaces::msg::Target>("target_message");
    if (!target_result.has_value()) {
        RCLCPP_DEBUG(this->get_logger(), "No target_message received");
        return BT::NodeStatus::FAILURE;
    }
    target = target_result.value();

    // 2. 获取参数
    double offset_distance = 2.0;
    double predict_time = 0.5;
    double max_chase_distance = 10.0;

    getInput("offset_distance", offset_distance);
    getInput("predict_time", predict_time);
    getInput("max_chase_distance", max_chase_distance);

    // // // 3. 获取机器人位置
    // geometry_msgs::msg::PoseStamped robot_pose;
    // auto robot_result = getInput<geometry_msgs::msg::PoseStamped>("robot_pose");
    // if (!robot_result.has_value()) {
    //     RCLCPP_WARN(this->get_logger(), "No robot_pose received");
    //     return BT::NodeStatus::FAILURE;
    // }
    // robot_pose = robot_result.value();

    // // 4. 计算机器人到敌人的距离和方向
    double enemy_x = target.position.x;
    double enemy_y = target.position.y;
    // double robot_x = robot_pose.pose.position.x;
    // double robot_y = robot_pose.pose.position.y;


    double distance = std::sqrt( enemy_x * enemy_x + enemy_y * enemy_y);

    // 5. 距离安全检查
    if (distance > max_chase_distance) {
        RCLCPP_INFO(this->get_logger(),
            "Enemy too far (%.2fm > %.2fm), aborting chase",
            distance, max_chase_distance);
        return BT::NodeStatus::FAILURE;
    }

    // 6. 计算导航目标
    double goal_x, goal_y;

    if (distance < offset_distance) {
        // 太近了，不动（保持当前距离）
        goal_x = enemy_x;
        goal_y = enemy_y;
        RCLCPP_DEBUG(this->get_logger(),
            "Too close to enemy (%.2fm < %.2fm), holding position", distance, offset_distance);
    } else {
        // 计算偏移目标: enemy - normalize(robot→enemy) × offset

        goal_x = enemy_x;
        goal_y = enemy_y;

        // 速度预测
        goal_x += target.velocity.x * predict_time;
        goal_y += target.velocity.y * predict_time;
    }

    // 7. 构建目标点
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = this->now();
    goal.header.frame_id = "gimbal_yaw"; // 视觉给我的就是这个，让 nav2 自动转去吧
    goal.pose.position.x = goal_x;
    goal.pose.position.y = goal_y;
    goal.pose.position.z = 0.0;

    // 朝向: 面向敌人
    // double yaw_to_enemy = std::atan2(goal_x, goal_y);
    // goal.pose.orientation.z = std::sin(yaw_to_enemy / 2.0);
    // goal.pose.orientation.w = std::cos(yaw_to_enemy / 2.0);
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    // 8. 输出
    setOutput("goal_pose", goal);

    // RCLCPP_INFO(this->get_logger(),
    //     "Enemy: (%.2f, %.2f) Robot: (%.2f, %.2f) dist=%.2fm → Goal: (%.2f, %.2f)",
    //     enemy_x, enemy_y, robot_x, robot_y, distance, goal_x, goal_y);
    

    return BT::NodeStatus::SUCCESS;
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<rm_behavior_tree::ArmorToGoalAction>("ArmorToGoal");
}
