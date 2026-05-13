#include "rm_behavior_tree/plugins/action/sub_robot_area_status.hpp"

namespace rm_behavior_tree {

SubRobotAreaStatusAction::SubRobotAreaStatusAction(
    const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<rm_decision_interfaces::msg::RobotAreaStatus>(name, conf, params)
{
}
BT::NodeStatus SubRobotAreaStatusAction::onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::RobotAreaStatus>& last_msg)
{
    if (last_msg) {
        RCLCPP_DEBUG(
            logger(), "[%s] new message, shooter_heat: %s", name().c_str(),
            last_msg->area_name.c_str());
        setOutput("robot_area_status", *last_msg);
    }
    return BT::NodeStatus::SUCCESS;
}
} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubRobotAreaStatusAction, "SubRobotAreaStatus");