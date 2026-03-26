#include "rm_behavior_tree/plugins/action/move_around.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <chrono>
#include <cmath>
#include <random>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;
using namespace std::chrono;

namespace rm_behavior_tree {

MoveAroundAction::MoveAroundAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
    , Node("move_around_node")
{
    publisher_goal_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    onStart();
    onRunning();
}

BT::NodeStatus MoveAroundAction::onStart()
{
    current_location.header.frame_id = "map";
    current_location.transform.translation.x = 0.0;
    current_location.transform.translation.y = 0.0;
    current_location.transform.translation.z = 0.0;

    current_location.transform.rotation.x = 0.0;
    current_location.transform.rotation.y = 0.0;
    current_location.transform.rotation.z = 0.0;
    current_location.transform.rotation.w = 1.0;

    expected_dis = 0.0;
    expected_nearby_goal_count = 0.0;
    goal_count = 0;
/*
    // 获取参数，机器人当前位置坐标的 blackboard 映射
    if (!getInput("message", current_location)){
        RCLCPP_DEBUG(this->get_logger(),"没有得到 current_location");
        return BT::NodeStatus::FAILURE;      
    }
    */
    if (getInput("robot_pose", robot_location)) {
        current_location.transform.translation.x = robot_location.first;
        current_location.transform.translation.y = robot_location.second;
    } else {
        RCLCPP_DEBUG(this->get_logger(),"没有得到 robot_location");
        return BT::NodeStatus::FAILURE;
    }

    // 获取参数：期望的距离
    if (!getInput("expected_dis", expected_dis)) {
        RCLCPP_DEBUG(this->get_logger(),"没有得到 expected_dis");
        return BT::NodeStatus::FAILURE;
    }

    // 获取参数：期望的点位数量
    if (!getInput("expected_nearby_goal_count", expected_nearby_goal_count)) {
        RCLCPP_DEBUG(this->get_logger(), "没有得到 expected_nearby_goal_count");
        return BT::NodeStatus::FAILURE;
    }

    if (expected_nearby_goal_count <= 0) {
        RCLCPP_DEBUG(this->get_logger(), "期望点数小于 0, 节点关闭");
        return BT::NodeStatus::FAILURE;
    } else {
        return BT::NodeStatus::RUNNING;
    }
}

BT::NodeStatus MoveAroundAction::onRunning()
{

    if (expected_nearby_goal_count == goal_count) {
        return BT::NodeStatus::SUCCESS;
    } else {
        goal_count++;
        generatePoints(current_location, expected_dis, nearby_random_point);
        sendGoalPose(nearby_random_point);
        std::this_thread::sleep_for(milliseconds(1000));
        return BT::NodeStatus::RUNNING;
    }
}

void MoveAroundAction::onHalted()
{
    // nothing to do here ...
    // std::cout << "MoveAroundAction interrupted" << '\n'
}

void MoveAroundAction::generatePoints(
    geometry_msgs::msg::TransformStamped location, double distance,
    geometry_msgs::msg::PoseStamped& nearby_random_point)
{
    // 创建随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 2 * M_PI);

    // 生成随机角度
    double angle = dis(gen);
    nearby_random_point.header.stamp = rclcpp::Clock().now();
    nearby_random_point.header.frame_id = "map";
    nearby_random_point.pose.position.x = location.transform.translation.x + distance * cos(angle);
    nearby_random_point.pose.position.y = location.transform.translation.y + distance * sin(angle);
    nearby_random_point.pose.position.z = location.transform.translation.z;

    nearby_random_point.pose.orientation.x = location.transform.rotation.x;
    nearby_random_point.pose.orientation.y = location.transform.rotation.y;
    nearby_random_point.pose.orientation.z = location.transform.rotation.z;
    nearby_random_point.pose.orientation.w = location.transform.rotation.w;
}

void MoveAroundAction::sendGoalPose(geometry_msgs::msg::PoseStamped& msg)
{
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = nearby_random_point.header.frame_id;
    msg.pose.position.x = nearby_random_point.pose.position.x;
    msg.pose.position.y = nearby_random_point.pose.position.y;
    msg.pose.position.z = nearby_random_point.pose.position.z;

    msg.pose.orientation.x = nearby_random_point.pose.orientation.x;
    msg.pose.orientation.y = nearby_random_point.pose.orientation.y;
    msg.pose.orientation.z = nearby_random_point.pose.orientation.z;
    msg.pose.orientation.w = nearby_random_point.pose.orientation.w;

    // DEBUG

    RCLCPP_DEBUG(this->get_logger(), "目标点位");
    RCLCPP_DEBUG(this->get_logger(), "pose.position.x %f", msg.pose.position.x);
    RCLCPP_DEBUG(this->get_logger(), "pose.position.y %f", msg.pose.position.y);
    RCLCPP_DEBUG(this->get_logger(), "pose.position.z %f", msg.pose.position.z);

    RCLCPP_DEBUG(this->get_logger(), "pose.orientation.x %f", msg.pose.orientation.x);
    RCLCPP_DEBUG(this->get_logger(), "pose.orientation.y %f", msg.pose.orientation.y);
    RCLCPP_DEBUG(this->get_logger(), "pose.orientation.z %f", msg.pose.orientation.z);
    RCLCPP_DEBUG(this->get_logger(), "pose.orientation.w %f", msg.pose.orientation.w);

    publisher_goal_pose->publish(msg);
}
}
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::MoveAroundAction>("MoveAround");
}