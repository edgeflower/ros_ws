#include "rm_behavior_tree/plugins/action/cancel_navigation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <behaviortree_cpp/basic_types.h>

namespace rm_behavior_tree
{

CancelNavigationAction::CancelNavigationAction(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config)
{
    // 获取 action server 名称
    getInput<std::string>("action_name", action_name_);
    if (action_name_.empty()) {
        action_name_ = "/navigate_to_pose";
    }

    // 使用共享的节点
    node_ = params.nh;

    // 创建 action client
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        node_, action_name_);

    RCLCPP_INFO(node_->get_logger(),
               "[CancelNavigation] Initialized, targeting action server: %s",
               action_name_.c_str());
}

BT::NodeStatus CancelNavigationAction::tick()
{
    // 等待 action server 可用（短暂等待）
    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
        RCLCPP_WARN(node_->get_logger(),
                   "[CancelNavigation] Action server %s not available, assuming no active navigation",
                   action_name_.c_str());
        return BT::NodeStatus::SUCCESS;  // 没有 action server = 没有活动目标
    }

    // 调用标准的 ROS2 Action 取消接口
    // async_cancel_all_goals() 会取消该 action client 发送的所有活动目标
    // 这是 ROS2 推荐的取消方式
    auto cancel_future = action_client_->async_cancel_all_goals();

    // 等待取消完成（设置超时避免长时间阻塞）
    std::future_status status = cancel_future.wait_for(std::chrono::milliseconds(500));

    if (status == std::future_status::ready) {
        auto cancel_response = cancel_future.get();

        // CancelGoal_Response 结构：
        // int8 return_code
        // 其中 0 = SUCCESS, 其他值为错误
        if (cancel_response->return_code == 0) {
            RCLCPP_INFO(node_->get_logger(),
                       "[CancelNavigation] Navigation goals canceled successfully");
        } else {
            // 可能是没有活动目标或其他原因，但请求已发送
            RCLCPP_INFO(node_->get_logger(),
                       "[CancelNavigation] Cancel request sent (return_code=%d, may indicate no active goals)",
                       cancel_response->return_code);
        }

        return BT::NodeStatus::SUCCESS;

    } else {
        // 超时 - 但取消请求已经发送，可能只是响应慢
        RCLCPP_WARN(node_->get_logger(),
                   "[CancelNavigation] Cancel request sent, but response timeout (may still succeed)");
        return BT::NodeStatus::SUCCESS;
    }
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::CancelNavigationAction, "CancelNavigation");
