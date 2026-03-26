#pragma once
#include "rm_decision_interfaces/msg/sentry_posture_cmd.hpp"
#include "rm_decision_interfaces/msg/sentry_posture_status.hpp"
#include <rclcpp/node.hpp>
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <memory>
namespace rm_behavior_tree {
class SentryPostureControllerAction : public BT::RosTopicPubNode<rm_decision_interfaces::msg::SentryPostureCmd> {
public:
    SentryPostureControllerAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::InputPort<int8_t>("posture")
        };
    }

    bool setMessage(rm_decision_interfaces::msg::SentryPostureCmd& msg) override;

private:
    enum Posture 
    {
        POSTURE_ATTACK = 1,    // 进攻姿态
        POSTURE_DEFFENSE = 2,  // 防御姿态
        POSTURE_MOVE = 3,	   // 移动姿态（默认）
    }; // enum Posture 哨兵姿态
};
} // namespace rm_behavior_tree