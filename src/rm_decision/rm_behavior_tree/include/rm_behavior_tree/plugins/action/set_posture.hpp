#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_POSTURE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_POSTURE_HPP_

#include "behaviortree_ros2/bt_service_node.hpp"
#include "rm_decision_interfaces/srv/set_sentry_posture.hpp"

namespace rm_behavior_tree
{
enum Posture
{
    POSTURE_ATTACK = 1,    // 进攻
    POSTURE_DEFENSE = 2,   // 防御
    POSTURE_MOVE = 3,      // 移动
};
class SetPosture : public BT::RosServiceNode<rm_decision_interfaces::srv::SetSentryPosture>
{
public:
    SetPosture(const std::string & name,
               const BT::NodeConfig & conf,
               const BT::RosNodeParams & params);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<Posture>("robot_posture_status",
                "POSTURE_DEFENSE / POSTURE_ATTACK / POSTURE_MOVE 或数字"),
            BT::InputPort<bool>("override", false, "是否强制覆盖当前状态")
        };
    }

    bool setRequest(Request::SharedPtr & request) override;
    BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;
    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

private:
    Posture target_posture_ {POSTURE_DEFENSE};
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_POSTURE_HPP_
