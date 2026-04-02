#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_POSTURE_XIN_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_POSTURE_XIN_HPP_

#include "behaviortree_cpp/action_node.h"
#include <rclcpp/logging.hpp>
#include <string>
#include <unordered_map>

namespace rm_behavior_tree
{

enum Posture
{
    POSTURE_ATTACK = 1,    // 进攻
    POSTURE_DEFENSE = 2,   // 防御
    POSTURE_MOVE = 3,      // 移动
};

class SetPostureXin : public BT::SyncActionNode
{
public:
    SetPostureXin(const std::string & name, const BT::NodeConfig & conf)
    : BT::SyncActionNode(name, conf)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<Posture>("robot_posture_status", "要切换的机器人姿态 (POSTURE_ATTACK/POSTURE_DEFENSE/POSTURE_MOVE)"),
            BT::InputPort<bool>("override", false, "是否强制覆盖当前状态")
        };
    }

    BT::NodeStatus tick() override;
};

} // namespace rm_behavior_tree

namespace BT
{

// 枚举字符串转换特化
template <>
inline rm_behavior_tree::Posture convertFromString(StringView key)
{
    static const std::unordered_map<std::string, rm_behavior_tree::Posture> posture_map = {
        {"POSTURE_ATTACK", rm_behavior_tree::POSTURE_ATTACK},
        {"POSTURE_DEFENSE", rm_behavior_tree::POSTURE_DEFENSE},
        {"POSTURE_MOVE", rm_behavior_tree::POSTURE_MOVE},
        {"1", rm_behavior_tree::POSTURE_ATTACK},
        {"2", rm_behavior_tree::POSTURE_DEFENSE},
        {"3", rm_behavior_tree::POSTURE_MOVE},
    };

    std::string key_str(key);
    auto it = posture_map.find(key_str);
    if (it != posture_map.end()) {
        return it->second;
    }

    throw RuntimeError("Invalid posture value: ", key_str);
}

} // namespace BT

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_POSTURE_XIN_HPP_
