#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_OK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_OK_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/all_robot_hp.hpp"

namespace rm_behavior_tree
{

/**
 * @brief Condition 节点，检测哨兵 HP 是否正常
 *
 * 功能说明：
 *   - 从 AllRobotHP 消息中获取 7 号机器人（英雄/哨兵）的 HP
 *   - 判断 HP 是否低于阈值
 *   - 低于阈值返回 FAILURE，否则返回 SUCCESS
 *
 * @param[in] all_robot_hp 所有机器人 HP 消息
 * @param[in] hp_threshold HP 阈值，默认 100
 *
 * 示例 XML:
 *   <IsHpOk all_robot_hp="{robot_hp}" hp_threshold="100"/>
 */
class IsHpOkAction : public BT::SimpleConditionNode
{
public:
    IsHpOkAction(const std::string & name, const BT::NodeConfig & config);

    // 检查 HP 状态
    BT::NodeStatus checkHpStatus();

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<rm_decision_interfaces::msg::AllRobotHP>("all_robot_hp"),
            BT::InputPort<int>("hp_threshold", 100, "HP阈值，低于此值返回FAILURE，默认100")
        };
    }
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_OK_HPP_
