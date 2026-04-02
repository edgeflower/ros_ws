#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__SHOULD_RESET_OBSERVATION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__SHOULD_RESET_OBSERVATION_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include <rm_decision_interfaces/msg/observation_points.hpp>
#include <mutex>

namespace rm_behavior_tree
{

class ShouldResetObservationCondition : public BT::RosTopicSubNode<rm_decision_interfaces::msg::ObservationPoints>
{
public:
    ShouldResetObservationCondition(const std::string & name,
                                   const BT::NodeConfiguration & config,
                                   const BT::RosNodeParams & params);

    static BT::PortsList providedPorts()
    {
        BT::PortsList custom_ports = {
            BT::InputPort<bool>("reset_signal", false, "External reset signal"),
            BT::InputPort<int32_t>("idle_count", 0, "Number of IDLE points from GoalManager"),
            BT::InputPort<int32_t>("done_count", 0, "Number of DONE points from GoalManager"),
            BT::InputPort<int32_t>("total_points", 0, "Total number of observation points"),
            BT::OutputPort<bool>("reset_ready", "True when reset is needed")
        };
        return BT::RosTopicSubNode<rm_decision_interfaces::msg::ObservationPoints>::providedBasicPorts(custom_ports);
    }

    BT::NodeStatus onTick(const std::shared_ptr<rm_decision_interfaces::msg::ObservationPoints>& last_msg) override;

private:
    int32_t total_points_;
    std::mutex data_mutex_;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__SHOULD_RESET_OBSERVATION_HPP_
