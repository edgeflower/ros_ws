#include "rm_behavior_tree/plugins/action/sentry_posture_controller.hpp"
#include <behaviortree_cpp/basic_types.h>
namespace rm_behavior_tree {
SentryPostureControllerAction::SentryPostureControllerAction(
    const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosTopicPubNode<rm_decision_interfaces::msg::SentryPostureCmd>(name, conf, params)
{
}

BT::NodeStatus SentryPostureControllerAction::setMessage(
    rm_decision_interfaces::msg::SentryPostureCmd& msg)
{
    int8_t posture;
    if (!getInput<int8_t>("posture", posture)) {
        RCLCPP_ERROR(logger(), "[%s] missing required input [posture]", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    msg.posture_cmd = posture;
    return BT::NodeStatus::SUCCESS;
}
} // namespace rm_behavior_tree
