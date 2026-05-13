#include "rm_behavior_tree/plugins/condition/is_yaw_in_range.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cmath>

namespace rm_behavior_tree
{

IsYawInRange::IsYawInRange(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{}

BT::NodeStatus IsYawInRange::tick()
{
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getInput<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose)) {
        RCLCPP_WARN(rclcpp::get_logger("IsYawInRange"), "Missing input port: robot_pose");
        return BT::NodeStatus::FAILURE;
    }

    double min_yaw_deg = 0.0;
    if (!getInput<double>("min_yaw_deg", min_yaw_deg)) {
        RCLCPP_WARN(rclcpp::get_logger("IsYawInRange"), "Missing input port: min_yaw_deg");
        return BT::NodeStatus::FAILURE;
    }

    double max_yaw_deg = 0.0;
    if (!getInput<double>("max_yaw_deg", max_yaw_deg)) {
        RCLCPP_WARN(rclcpp::get_logger("IsYawInRange"), "Missing input port: max_yaw_deg");
        return BT::NodeStatus::FAILURE;
    }

    const double yaw_deg = yawFromPoseDeg(robot_pose);
    const bool in_range = (min_yaw_deg <= yaw_deg && yaw_deg <= max_yaw_deg);

    RCLCPP_INFO(
        rclcpp::get_logger("IsYawInRange"),
        "yaw=%.2f deg, min_yaw_deg=%.2f, max_yaw_deg=%.2f, result=%s",
        yaw_deg, min_yaw_deg, max_yaw_deg, in_range ? "SUCCESS" : "FAILURE");

    return in_range ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

double IsYawInRange::yawFromPoseDeg(const geometry_msgs::msg::PoseStamped & pose)
{
    const auto & q = pose.pose.orientation;

    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw_rad = std::atan2(siny_cosp, cosy_cosp);
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

    return normalizeYawDeg(yaw_rad * kRadToDeg);
}

double IsYawInRange::normalizeYawDeg(double yaw_deg)
{
    yaw_deg = std::fmod(yaw_deg + 180.0, 360.0);
    if (yaw_deg < 0.0) {
        yaw_deg += 360.0;
    }
    return yaw_deg - 180.0;
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::IsYawInRange>("IsYawInRange");
}
