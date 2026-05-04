#pragma once
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rm_decision_interfaces/msg/detail/enemy_location__struct.hpp>
#include <rm_decision_interfaces/msg/enemy_location.hpp>

namespace rm_behavior_tree {
class SubAllEnemyLocationAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::EnemyLocation>{
public:
    SubAllEnemyLocationAction(const std::string &name, const BT::NodeConfig& conf, const BT::RosNodeParams & params);
    static BT::PortsList providedPorts(){
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::OutputPort<rm_decision_interfaces::msg::EnemyLocation>("all_enemy_location")
        };
    }

    BT::NodeStatus onTick(
        const std::shared_ptr<rm_decision_interfaces::msg::EnemyLocation>& last_msg
    ) override;
};
} // namespace rm_behavior_tree