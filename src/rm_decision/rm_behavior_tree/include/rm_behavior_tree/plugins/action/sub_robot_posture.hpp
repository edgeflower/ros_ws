#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/sentry_posture_status.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <memory>
#include <rm_decision_interfaces/msg/detail/sentry_posture_status__struct.hpp>

namespace rm_behavior_tree {
class SubRobotPostureAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::SentryPostureStatus> {
public:
    SubRobotPostureAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::OutputPort<rm_decision_interfaces::msg::SentryPostureStatus>("robot_posture_status")
        };
    }

    BT::NodeStatus onTick(
        const std::shared_ptr<rm_decision_interfaces::msg::SentryPostureStatus>& last_msg) override;
private:
	enum Posture
	{
		POSTURE_ATTACK = 1,    // 
		POSTURE_DEFFENSE = 2,  //
		POSTURE_MOVE = 3,	   //
	};

};

} // namespace rm_behavior_tree