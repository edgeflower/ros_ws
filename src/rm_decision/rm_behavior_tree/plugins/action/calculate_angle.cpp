#include "rm_behavior_tree/plugins/action/calculate_angle.hpp"
#include <rclcpp/logging.hpp>
#include <cmath>

namespace rm_behavior_tree
{

CalculateAngleAction::CalculateAngleAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params)
: RosTopicPubNode<std_msgs::msg::Float32>(name, conf, params)
{
    RCLCPP_INFO(node_->get_logger(), "[CalculateAngle] Initialized");
}

bool CalculateAngleAction::setMessage(std_msgs::msg::Float32 & msg)
{
    // 从 blackboard 读取 goal_pose
    if (!getInput<geometry_msgs::msg::PoseStamped>("goal_pose", goal_pose_)) {
        RCLCPP_WARN(node_->get_logger(), "[CalculateAngle] Failed to get goal_pose from blackboard");
        return false;
    }

    // 从 blackboard 读取 robot_pose
    if (!getInput<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose_)) {
        RCLCPP_WARN(node_->get_logger(), "[CalculateAngle] Failed to get robot_pose from blackboard");
        return false;
    }

    // 计算角度
    double angle = calculateAngle(goal_pose_, robot_pose_);

    msg.data = static_cast<float>(angle);

    RCLCPP_DEBUG(node_->get_logger(),
                "[CalculateAngle] Calculated angle: %.3f rad (%.1f deg)",
                angle, angle * 180.0 / M_PI);

    return true;
}

double CalculateAngleAction::calculateAngle(
    const geometry_msgs::msg::PoseStamped& goal,
    const geometry_msgs::msg::PoseStamped& robot)
{
    // 计算从机器人位置指向目标位置的向量
    double dx = goal.pose.position.x - robot.pose.position.x;
    double dy = goal.pose.position.y - robot.pose.position.y;

    // 计算角度（-π 到 π）
    double angle = std::atan2(dy, dx);

    // 如果需要考虑机器人当前朝向，可以减去机器人的 yaw
    // 这里假设 robot_pose.orientation.z 存储了 yaw（与 set_goal 一致）
    double robot_yaw = robot.pose.orientation.z;
    angle -= robot_yaw;

    // 归一化到 [-π, π]
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;

    return angle;
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::CalculateAngleAction, "CalculateAngle");
