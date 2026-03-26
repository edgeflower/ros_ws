// 文件名：send_goal.cpp
// 描述：这是一个 ROS2 + BehaviorTree.CPP 自定义 SendGoal 行为树节点的完整实现
// 功能：连接 nav2 的 NavigateToPose 动作服务器并发送目标点，支持从行为树 XML 配置 action_name，加入了 ActionServer 等待机制

#include "rm_behavior_tree/plugins/action/send_goal.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "rm_behavior_tree/bt_conversions.hpp"
#include <iomanip>
#include <iostream>

namespace rm_behavior_tree
{

SendGoalAction::SendGoalAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
    // 从 XML 中读取 action_name，如果没有则使用默认路径
    if (!getInput<std::string>("action_name", action_name_)) {
        action_name_ = "/navigate_to_pose";
    }

    // 使用 ROS2 rclcpp_action 通用接口手动创建 client 等待
    auto client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, action_name_);
    RCLCPP_INFO(node_->get_logger(), "[%s] 等待 Action Server [%s] ...", name.c_str(), action_name_.c_str());
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), " [%s] Action Server [%s] 不可达！", name.c_str(), action_name_.c_str());
        throw BT::RuntimeError("Action server not available: ", action_name_);
    }
    RCLCPP_INFO(node_->get_logger(), " [%s] 已连接 Action Server: %s", name.c_str(), action_name_.c_str());
}

bool SendGoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
    auto res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
    if (!res) {
        throw BT::RuntimeError("error reading port [goal_pose]", res.error());
    }

    goal.pose = res.value();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = rclcpp::Clock().now();

    RCLCPP_INFO(node_->get_logger(), "发送目标点: x=%.2f, y=%.2f",
                goal.pose.pose.position.x, goal.pose.pose.position.y);
/*
    std::cout << "Goal_pose:["
              << std::fixed << std::setprecision(1)
              << goal.pose.pose.position.x << ", "
              << goal.pose.pose.position.y << ", "
              << goal.pose.pose.position.z << ", "
              << goal.pose.pose.orientation.x << ", "
              << goal.pose.pose.orientation.y << ", "
              << goal.pose.pose.orientation.z << ", "
              << goal.pose.pose.orientation.w << "]" << std::endl;
*/
    RCLCPP_DEBUG(node_->get_logger(),"Goal_pose:[ X :%f, Y:%f", goal.pose.pose.position.x, goal.pose.pose.position.y);

    return true;
}

void SendGoalAction::onHalt()
{
    RCLCPP_INFO(node_->get_logger(), "SendGoalAction has been halted");
}

BT::NodeStatus SendGoalAction::onResultReceived(const WrappedResult & wr)
{
    switch (wr.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "导航成功完成！");
            return BT::NodeStatus::SUCCESS;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(node_->get_logger(), "导航目标被中止！");
            return BT::NodeStatus::FAILURE;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node_->get_logger(), "导航目标被取消！");
            return BT::NodeStatus::FAILURE;
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "未知导航结果状态");
            return BT::NodeStatus::FAILURE;
            break;
    }
}

BT::NodeStatus SendGoalAction::onFeedback(
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> /*feedback*/)
{
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SendGoalAction::onFailure(BT::ActionNodeErrorCode error)
{
    RCLCPP_ERROR(node_->get_logger(), "SendGoalAction 执行失败，错误码: %d", static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SendGoalAction, "SendGoal");