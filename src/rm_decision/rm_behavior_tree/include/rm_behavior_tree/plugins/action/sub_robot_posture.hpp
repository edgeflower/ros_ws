#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/sentry_posture_status.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <memory>

namespace rm_behavior_tree {

class SubRobotPostureAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::SentryPostureStatus> {
public:
    SubRobotPostureAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name", "sentry_posture_status", "订阅的话题名称"),
            BT::OutputPort<rm_decision_interfaces::msg::SentryPostureStatus>("posture_status", "完整的姿态状态消息"),
            BT::OutputPort<int>("posture", "当前姿态值（用于SetPosture确认）")
        };
    }

    BT::NodeStatus onTick(
        const std::shared_ptr<rm_decision_interfaces::msg::SentryPostureStatus>& last_msg) override;

private:
    enum Posture
    {
        POSTURE_ATTACK = 1,    // 进攻
        POSTURE_DEFENSE = 2,   // 防御
        POSTURE_MOVE = 3,      // 移动
    };

    // 上次的姿态，用于检测变化
    int last_posture_ {-1};
};

} // namespace rm_behavior_tree
