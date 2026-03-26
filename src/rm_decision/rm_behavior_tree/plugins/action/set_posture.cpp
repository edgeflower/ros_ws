#include "rm_behavior_tree/plugins/action/set_posture.hpp"

namespace rm_behavior_tree
{

SetPosture::SetPosture(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosServiceNode<rm_decision_interfaces::srv::SetSentryPosture>(name, conf, params)
{
    // 1. 获取服务名（默认为 /set_posture）
    std::string service_name;
    if (!getInput<std::string>("service_name", service_name)) {
        service_name = "set_posture";
    }

    // 2. 这里的 node_ 是基类 RosServiceNode 提供的
    RCLCPP_INFO(node_->get_logger(), "[%s] 正在连接服务: %s", name.c_str(), service_name.c_str());

    // 3. 阻塞等待一小段时间，确保服务上线
    // 注意：这里的等待只在树创建时发生一次
    auto client = node_->create_client<rm_decision_interfaces::srv::SetSentryPosture>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node_->get_logger(), " [%s] 服务 [%s] 暂时不可用，将在运行时重试。", 
                    name.c_str(), service_name.c_str());
    } else {
        RCLCPP_INFO(node_->get_logger(), " [%s] 服务已连接", name.c_str());
    }
}

bool SetPosture::setRequest(Request::SharedPtr & request)
{
    if (!getInput("posture", posture)) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] 缺少姿态参数!", name().c_str());
        return false;
    }
    getInput("override", override_mode);

    // 填充请求
    request->posture = posture;
    request->override_mode = override_mode;

    RCLCPP_INFO(node_->get_logger(), "[%s] 发送姿态请求: posture = %d, override = %s", name().c_str(), posture, override_mode ? "true" : "false");
    return true;
}

BT::NodeStatus SetPosture::onResponseReceived(const Response::SharedPtr & response)
{
    if (response->accepted) {
        RCLCPP_INFO(node_->get_logger(), "[%s] 服务请求成功接受", name().c_str());
        
        if ((node_->now() - start_time_).seconds() >= 15.0) {
                RCLCPP_INFO(node_->get_logger(), "15秒保持结束");
                return BT::NodeStatus::SUCCESS;
            }
        return BT::NodeStatus::RUNNING; 
    } else {
        RCLCPP_WARN(node_->get_logger(), "[%s] 姿态切换请求被拒绝!", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus SetPosture::onFailure(BT::ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(node_->get_logger(), "[%s] 服务调用失败，错误码: %d", name().c_str(), static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SetPosture, "SetPosture");