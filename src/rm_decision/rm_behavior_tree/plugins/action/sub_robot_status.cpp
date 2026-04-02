#include "rm_behavior_tree/plugins/action/sub_robot_status.hpp"

namespace rm_behavior_tree {

SubRobotStatusAction::SubRobotStatusAction(
    const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<rm_decision_interfaces::msg::RobotStatus>(name, conf, params)
{
}
BT::NodeStatus SubRobotStatusAction::onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::RobotStatus>& last_msg)
{
    if (last_msg) {
        RCLCPP_DEBUG(
            logger(), "[%s] new message, shooter_heat: %d", name().c_str(),
            last_msg->shooter_heat);
        setOutput("robot_status", *last_msg);
    }
    return BT::NodeStatus::SUCCESS;
}
} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubRobotStatusAction, "SubRobotStatus");