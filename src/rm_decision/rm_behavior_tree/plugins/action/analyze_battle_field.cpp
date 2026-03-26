#include "rm_behavior_tree/plugins/action/analyze_battle_field.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <rm_decision_interfaces/msg/detail/all_robot_hp__struct.hpp>
#include <rm_decision_interfaces/msg/detail/friend_location__struct.hpp>
#include <rm_decision_interfaces/msg/detail/robot_status__struct.hpp>
#include <utility>

namespace rm_behavior_tree {
AnalyzeBattleFieldAction::AnalyzeBattleFieldAction(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{
    node_ = std::make_shared<rclcpp::Node>("analyze_battle_field");
    publisher_goal_pose = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    analyze();
}

void AnalyzeBattleFieldAction::messageUpdate()
{

    getInput<std::pair<float, float>>("robot_pose", robot_location);
    auto msg = getInput<rm_decision_interfaces::msg::AllRobotHP>("message");
    auto friend_location = getInput<rm_decision_interfaces::msg::FriendLocation>("all_robot_location");
    auto robot_status = getInput<rm_decision_interfaces::msg::RobotStatus>("robot_status");

    // 本机器人位置
    robot_x_ = robot_location.first;
    robot_y_ = robot_location.second;

    // 机器人状态
    robot_hp_ = robot_status->current_hp;
    robot_status_.robot_id = robot_status->robot_id;

    // 友方机器人位置
    friend_location_.hero_x = friend_location->hero_x;
    //    friend_location_.hero_y = friend_location->hero_y;
    friend_location_.engineer_x = friend_location->engineer_x;
    //    friend_location_.engineer_y = friend_location->engineer_y;
    friend_location_.standard_3_x = friend_location->standard_3_x;
    //    friend_location_.standard_3_y = friend_location->standard_3_y;
    friend_location_.standard_5_x = friend_location->standard_5_x;
    //    friend_location_.standard_5_y = friend_location->standard_5_y;
}

BT::NodeStatus AnalyzeBattleFieldAction::analyze()
{
    float site;
    messageUpdate();
    site = friend_location_.hero_x * 0.6 + friend_location_.standard_3_x * 0.2 + friend_location_.standard_4_x * 0.2;

    if (site > restriction_site[2] && site < restriction_site[3]) {
        conservativeFactorCalculation();
    }

    if (site > restriction_site[3]) {
        conservativeFactorCalculation();
    }

    return BT::NodeStatus::SUCCESS;
}

void AnalyzeBattleFieldAction::sendGoalPose(const geometry_msgs::msg::PoseStamped& msg)
{

    // 初始化
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = 0;
    goal_pose.pose.position.y = 0;
    goal_pose.pose.position.z = 0;
    goal_pose.pose.orientation.w = 1;

    // 赋值
    goal_pose.pose.position.x = msg.pose.position.x;
    goal_pose.pose.position.y = msg.pose.position.y;

    // 发布
    publisher_goal_pose->publish(goal_pose);
}

BT::NodeStatus AnalyzeBattleFieldAction::conservativeFactorCalculation()
{
    ratio = robot_hp_ / 400; // 保守因子

    if (ratio > 0.55) {
        goal_pose.pose.position.x = 5;
        goal_pose.pose.position.y = 5;
        sendGoalPose(goal_pose);
        // return BT::NodeStatus::RUNNING;
    }

    if (0.55 > ratio && ratio > 0.25) {
        goal_pose.pose.position.x = 0.0;
        goal_pose.pose.position.y = 0.0;
        sendGoalPose(goal_pose);
        // return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
}
}