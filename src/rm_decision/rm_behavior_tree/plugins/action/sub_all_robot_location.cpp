#include "rm_behavior_tree/plugins/action/sub_all_robot_location.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <memory>
#include <rm_decision_interfaces/msg/detail/friend_location__struct.hpp>

namespace rm_behavior_tree {
SubAllRobotLocationAction::SubAllRobotLocationAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<rm_decision_interfaces::msg::FriendLocation>(name, conf, params)
{
}
BT::NodeStatus SubAllRobotLocationAction::onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::FriendLocation>& last_msg)
{
    if (last_msg) {
        RCLCPP_DEBUG(
            logger(), "[%s] new message, hero_x: %f", name().c_str(), last_msg->hero_x);
        setOutput("all_robot_location", *last_msg);
    }
    return BT::NodeStatus::SUCCESS;
}
} // namespaace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubAllRobotLocationAction, "SubAllRobotLocation");
// 7.12