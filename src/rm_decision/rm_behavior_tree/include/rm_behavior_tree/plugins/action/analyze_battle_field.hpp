#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__ANALYZE_BATTLE_FIELD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__ANALYZE_BATTLE_FIELD_HPP_
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <cmath>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rm_decision_interfaces/msg/all_robot_hp.hpp>
#include <rm_decision_interfaces/msg/detail/all_robot_hp__struct.hpp>
#include <rm_decision_interfaces/msg/detail/friend_location__struct.hpp>
#include <rm_decision_interfaces/msg/detail/robot_status__struct.hpp>
#include <rm_decision_interfaces/msg/friend_location.hpp>
#include <rm_decision_interfaces/msg/robot_status.hpp>
#include <utility>
namespace rm_behavior_tree {
class AnalyzeBattleFieldAction : public BT::SyncActionNode {
public:
    AnalyzeBattleFieldAction(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<rm_decision_interfaces::msg::FriendLocation>("all_robot_location"),
            BT::InputPort<rm_decision_interfaces::msg::AllRobotHP>("robot_hp"),
            BT::InputPort<std::pair<float, float>>("robot_pose"),
            BT::InputPort<rm_decision_interfaces::msg::RobotStatus>("robot_status")
        };
    }
    BT::NodeStatus tick();
    void messageUpdate();
    BT::NodeStatus conservativeFactorCalculation();
    BT::NodeStatus analyze();

    void sendGoalPose(const geometry_msgs::msg::PoseStamped& msg);

private:
    rclcpp::Node::SharedPtr node_;
    float robot_x_, robot_y_;
    float robot_hp_;
    std::pair<float, float> robot_location;
    double ratio; // 保守因子
    float restriction_site[5] { 0, 11, 17, 28 }; // 把场地分为 4 个部分；
    geometry_msgs::msg::PoseStamped goal_pose;
    rm_decision_interfaces::msg::FriendLocation friend_location_;
    rm_decision_interfaces::msg::RobotStatus robot_status_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_goal_pose;
};

}

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__ANALYZE_BATTLE_FIELD_HPP_