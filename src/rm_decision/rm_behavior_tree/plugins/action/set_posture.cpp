#include "rm_behavior_tree/plugins/action/set_posture.hpp"

namespace rm_behavior_tree
{

SetPosture::SetPosture(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosServiceNode<rm_decision_interfaces::srv::SetSentryPosture>(name, conf, params)
{
    RCLCPP_INFO(node_->get_logger(), "[%s] SetPosture 节点已创建", name.c_str());
}

bool SetPosture::setRequest(Request::SharedPtr & request)
{
    if (!getInput("posture", target_posture_)) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] 缺少姿态参数!", name().c_str());
        return false;
    }

    bool override_mode = false;
    getInput("override", override_mode);

    request->posture = static_cast<uint8_t>(target_posture_);
    request->override_mode = override_mode;

    RCLCPP_INFO(node_->get_logger(), "[%s] 发送姿态请求: posture = %d, override = %s",
                name().c_str(), target_posture_, override_mode ? "true" : "false");
    return true;
}

BT::NodeStatus SetPosture::onResponseReceived(const Response::SharedPtr & response)
{
    if (!response->accepted) {
        RCLCPP_WARN(node_->get_logger(), "[%s] 姿态切换请求被拒绝!", name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // 命令已被接受，直接返回成功（模拟模式，无需等待下位机确认）
    RCLCPP_INFO(node_->get_logger(), "[%s] 姿态命令已发送，返回 SUCCESS", name().c_str());

    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SetPosture::onFailure(BT::ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(node_->get_logger(), "[%s] 服务调用失败，错误码: %d", name().c_str(), static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SetPosture, "SetPosture");
