#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_POSTURE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_POSTURE_HPP_

#include "behaviortree_ros2/bt_service_node.hpp"
#include "rm_decision_interfaces/srv/set_sentry_posture.hpp"

namespace rm_behavior_tree
{

class SetPosture : public BT::RosServiceNode<rm_decision_interfaces::srv::SetSentryPosture>
{
public:
    SetPosture(const std::string & name, 
               const BT::NodeConfig & conf, 
               const BT::RosNodeParams & params);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("posture", "要切换的机器人姿态"),
            BT::InputPort<bool>("override", false, "是否强制覆盖当前状态"),
            BT::InputPort<std::string>("service_name", "/set_posture", "服务名称")
        };
    }

    // 发送请求前调用，用于填充请求数据
    bool setRequest(Request::SharedPtr & request) override;

    // 收到响应后调用
    BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;

    // 发生错误（如服务不可用）时调用
    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
    enum Posture
    {
        POSTURE_ATTACK = 1,
        POSTURE_DEFENSE = 2,
        POSTURE_MOVE = 3,
    };
    // 如果需要 15 秒保持逻辑，可以记录时间
    rclcpp::Time start_time_;
    bool is_executing_timer_ = false;
    // 从端口读取数据
    int posture;
    bool override_mode;
    
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_POSTURE_HPP_