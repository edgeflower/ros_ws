#include "rm_behavior_tree/plugins/action/set_posture_xin.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace rm_behavior_tree
{

BT::NodeStatus SetPostureXin::tick()
{
    Posture posture;
    bool override;

    if (!getInput("posture", posture)) {
        RCLCPP_ERROR(rclcpp::get_logger("SetPostureXin"), "缺少 posture 参数!");
        return BT::NodeStatus::FAILURE;
    }

    getInput("override", override);

    const char* posture_name = "";
    switch (posture) {
        case POSTURE_ATTACK:  posture_name = "ATTACK"; break;
        case POSTURE_DEFENSE: posture_name = "DEFENSE"; break;
        case POSTURE_MOVE:    posture_name = "MOVE"; break;
        default:              posture_name = "UNKNOWN"; break;
    }

    RCLCPP_INFO(rclcpp::get_logger("SetPostureXin"),
                "模拟设置姿态: posture = %s (%d), override = %s",
                posture_name, static_cast<int>(posture), override ? "true" : "false");

    return BT::NodeStatus::SUCCESS;
}

} // namespace rm_behavior_tree

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::SetPostureXin>("SetPostureXin");
}
