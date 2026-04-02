#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ALL_ROBOT_LOCATION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ALL_ROBOT_LOCATION_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/friend_location.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <memory>
#include <rm_decision_interfaces/msg/detail/friend_location__struct.hpp>

namespace rm_behavior_tree {
class SubAllRobotLocationAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::FriendLocation> {
public:
    SubAllRobotLocationAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::OutputPort<rm_decision_interfaces::msg::FriendLocation>("all_robot_location")
        };
    }

    BT::NodeStatus onTick(
        const std::shared_ptr<rm_decision_interfaces::msg::FriendLocation>& last_msg) override;
};
} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ALL_ROBOT_LOCATION_HPP_
// 7.12