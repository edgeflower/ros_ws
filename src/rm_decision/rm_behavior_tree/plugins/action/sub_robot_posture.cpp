#include "rm_behavior_tree/plugins/action/sub_robot_posture.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rm_decision_interfaces/msg/detail/sentry_posture_status__struct.hpp>

namespace rm_behavior_tree {

SubRobotPostureAction::SubRobotPostureAction(
    const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<rm_decision_interfaces::msg::SentryPostureStatus>(name, conf, params)
{
}

BT::NodeStatus SubRobotPostureAction::onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::SentryPostureStatus>& last_msg)
{
    if (last_msg) {
        int current_posture = static_cast<int>(last_msg->current_posture);

        // 检测姿态变化，便于调试
        if (current_posture != last_posture_) {
            RCLCPP_INFO(
                logger(), "[%s] 姿态变化: %d -> %d", name().c_str(),
                last_posture_, current_posture);
            last_posture_ = current_posture;
        } else {
            RCLCPP_DEBUG(
                logger(), "[%s] 当前姿态: %d", name().c_str(), current_posture);
        }

        // 输出完整消息
        setOutput("posture_status", *last_msg);
        // 输出姿态值（方便直接使用）
        setOutput("posture", current_posture);
    }

    return BT::NodeStatus::SUCCESS;
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubRobotPostureAction, "SubRobotPosture");
