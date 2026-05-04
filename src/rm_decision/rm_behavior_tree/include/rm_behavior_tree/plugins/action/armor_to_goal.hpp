#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_

#include "armor_interfaces/msg/armor.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <armor_interfaces/msg/detail/target__struct.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/node.hpp>
#include <armor_interfaces/msg/target.hpp>
#include <cmath>

namespace rm_behavior_tree {

class ArmorToGoalAction : public BT::SyncActionNode, public rclcpp::Node
{
public:
    ArmorToGoalAction(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<armor_interfaces::msg::Target>("target_message", "Target message with position"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose", "Current robot pose"),
            BT::InputPort<double>("offset_distance", 2.0, "Keep this distance from enemy (m)"),
            BT::InputPort<double>("predict_time", 0.5, "Velocity prediction time (s)"),
            BT::InputPort<double>("max_chase_distance", 10.0, "Max chase distance, return FAILURE if exceeded (m)"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "Output goal pose with offset")
        };
    }

    BT::NodeStatus tick() override;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_
