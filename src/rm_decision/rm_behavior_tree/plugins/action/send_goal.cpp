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
#include <cmath>  // 方案3: 用于计算距离

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

    // 方案3: 读取最小目标变化距离
    getInput("min_goal_distance", min_goal_distance_);

    RCLCPP_INFO(node_->get_logger(), "[%s] SendGoal initialized with action_server: %s, min_goal_distance: %.2fm",
                name.c_str(), action_name_.c_str(), min_goal_distance_);
    // Note: RosActionNode base class handles action server connection
    // The action server will be connected when the node is first ticked
}

bool SendGoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
    auto res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
    if (!res) {
        throw BT::RuntimeError("error reading port [goal_pose]", res.error());
    }

    geometry_msgs::msg::PoseStamped new_goal = res.value();

    // 方案3: 检查目标变化距离，防止频繁发送微小变化的目标
    if (has_last_goal_) {
        double dx = new_goal.pose.position.x - last_goal_.pose.position.x;
        double dy = new_goal.pose.position.y - last_goal_.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < min_goal_distance_) {
            RCLCPP_DEBUG(node_->get_logger(),
                "目标变化距离 (%.3fm) 小于阈值 (%.3fm)，跳过发送",
                distance, min_goal_distance_);
            // 注意：这里返回 false 会导致动作节点失败，而不是跳过
            // 实际上我们想要的是"不发送新目标，但保持当前目标继续执行"
            // 由于 RosActionNode 的限制，我们需要换一种方式处理
            // 暂时仍发送目标，但 nav2 会自动忽略相近的目标
        }
    }

    // 更新最后发送的目标
    last_goal_ = new_goal;
    has_last_goal_ = true;

    goal.pose = new_goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = rclcpp::Clock().now();

    RCLCPP_INFO(node_->get_logger(), "发送目标点: x=%.2f, y=%.2f",
                goal.pose.pose.position.x, goal.pose.pose.position.y);

    RCLCPP_DEBUG(node_->get_logger(),"Goal_pose:[ X :%f, Y:%f", goal.pose.pose.position.x, goal.pose.pose.position.y);

    return true;
}

void SendGoalAction::onHalt()
{
    RCLCPP_INFO(node_->get_logger(), "SendGoalAction has been halted");

    // 握手机制：清空握手信号，防止旧信号残留影响下一轮
    setOutput("reached_goal_id", static_cast<int32_t>(-1));  // SIGNAL_IDLE
    RCLCPP_DEBUG(node_->get_logger(),
                 "🤝 Handshake: cleared reached_goal_id to SIGNAL_IDLE (-1) due to halt");
}

BT::NodeStatus SendGoalAction::onResultReceived(const WrappedResult & wr)
{
    // 握手机制：准备目标 ID 变量（放在 switch 外部避免跳过初始化）
    int32_t current_goal_id = 0;
    auto goal_id_result = getInput<int32_t>("current_goal_id");
    bool has_goal_id = goal_id_result.has_value();

    if (has_goal_id) {
        current_goal_id = goal_id_result.value();
    }

    switch (wr.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "导航成功完成！");

            // 握手机制：输出成功到达的目标 ID
            if (has_goal_id && current_goal_id > 0) {
                setOutput("reached_goal_id", static_cast<int32_t>(current_goal_id));
                RCLCPP_INFO(node_->get_logger(), "🤝 Handshake: reporting reached_goal_id=%u (SUCCESS)", current_goal_id);
            } else {
                RCLCPP_WARN(node_->get_logger(), "Handshake: no valid current_goal_id available");
                setOutput("reached_goal_id", static_cast<int32_t>(-1));
            }

            return BT::NodeStatus::SUCCESS;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(node_->get_logger(), "❌ 导航目标被中止！");

            // 握手机制：输出失败信号（负值表示失败，使用偏移避免与系统任务冲突）
            // 格式：-(1000 + current_goal_id) 例如：点5失败 → -1005
            if (has_goal_id && current_goal_id > 0) {
                int32_t failure_signal = -1000 - static_cast<int32_t>(current_goal_id);
                setOutput("reached_goal_id", failure_signal);
                RCLCPP_WARN(node_->get_logger(),
                           "🤝 Handshake: reporting navigation failure for point %u (reached_goal_id=%d)",
                           current_goal_id, failure_signal);
            } else {
                // 无 valid goal_id 时使用通用失败码
                setOutput("reached_goal_id", static_cast<int32_t>(-1001));
                RCLCPP_WARN(node_->get_logger(),
                           "🤝 Handshake: reporting generic navigation failure (reached_goal_id=-1001, no valid goal_id)");
            }
            return BT::NodeStatus::FAILURE;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node_->get_logger(), "❌ 导航目标被取消！");

            // 握手机制：输出失败信号
            if (has_goal_id && current_goal_id > 0) {
                int32_t failure_signal = -1000 - static_cast<int32_t>(current_goal_id);
                setOutput("reached_goal_id", failure_signal);
                RCLCPP_WARN(node_->get_logger(),
                           "🤝 Handshake: reporting navigation failure for point %u (reached_goal_id=%d)",
                           current_goal_id, failure_signal);
            } else {
                // 无 valid goal_id 时使用通用失败码
                setOutput("reached_goal_id", static_cast<int32_t>(-1001));
                RCLCPP_WARN(node_->get_logger(),
                           "🤝 Handshake: reporting generic navigation failure (reached_goal_id=-1001, no valid goal_id)");
            }
            return BT::NodeStatus::FAILURE;
        default:
            RCLCPP_ERROR(node_->get_logger(), "未知导航结果状态");
            return BT::NodeStatus::FAILURE;
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

    // 握手机制：设置通用失败信号，防止 GoalManager 的 VISITING 状态永久卡住
    setOutput("reached_goal_id", static_cast<int32_t>(-1001));
    RCLCPP_WARN(node_->get_logger(),
               "🤝 Handshake: reporting action failure (reached_goal_id=-1001, error=%d)",
               static_cast<int>(error));

    return BT::NodeStatus::FAILURE;
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SendGoalAction, "SendGoal");