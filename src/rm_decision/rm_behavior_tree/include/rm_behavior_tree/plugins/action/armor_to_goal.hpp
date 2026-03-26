#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_

#include "armor_interfaces/msg/armor.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/publisher.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/node.hpp>
#include <nav_msgs/msg/odometry.hpp>
namespace rm_behavior_tree {
class ArmorToGoalAction : public BT::StatefulActionNode, rclcpp::Node
{
public:
    ArmorToGoalAction(const std::string &name, const BT::NodeConfig & config);

    static BT::PortsList providedPorts()
    {   
        return {BT::InputPort<armor_interfaces::msg::Armor>("armor_message"),
                BT::InputPort<geometry_msgs::msg::TransformStamped>("sentry_message"),
                BT::InputPort<std::pair<float, float>>("robot_pose")};
                
    }

    BT::NodeStatus onStart() override;
    
    BT::NodeStatus onRunning() override;

    void onHalted() override;

    void setMessage(geometry_msgs::msg::TransformStamped location ,
        geometry_msgs::msg::PoseStamped & armor_target_location);

    void sendGoalPose(geometry_msgs::msg::PoseStamped & msg);

private:

    int goal_count;
    geometry_msgs::msg::PoseStamped armor_target_location;
    armor_interfaces::msg::Armor armor_relative_current_location;
    nav_msgs::msg::Odometry sentry_current_location;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_goal_pose;
    std::pair<float, float> robot_location;
};

} // rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_