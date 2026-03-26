#include "rm_behavior_tree/plugins/action/sub_robot_posture.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rm_decision_interfaces/msg/detail/sentry_posture_status__struct.hpp>
namespace rm_behavior_tree {
SubRobotPostureAction::SubRobotPostureAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<rm_decision_interfaces::msg::SentryPostureStatus>(name, conf, params)
{
}

BT::NodeStatus SubRobotPostureAction::onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::SentryPostureStatus>& last_msg)
{
    if (last_msg) {
        RCLCPP_DEBUG(
            logger(), "[%s] new message, robot posture %d", name().c_str(),
            last_msg->reported_posture);
        setOutput("robot_posture_status", *last_msg);
    }
    return BT::NodeStatus::SUCCESS;
}
} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubRobotPostureAction, "SubRobotPosture");