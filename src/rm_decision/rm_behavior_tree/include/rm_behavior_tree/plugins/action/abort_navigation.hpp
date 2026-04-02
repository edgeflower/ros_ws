#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__ABORT_NAVIGATION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__ABORT_NAVIGATION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include <string>

namespace rm_behavior_tree
{

/**
 * @brief Abort Navigation Action Node (Semantic Marker)
 *
 * 中止当前导航的语义标记节点
 *
 * 功能说明：
 * - 在 Nav2 中，发送新目标会自动取消旧目标
 * - 此节点作为行为树中的语义标记，明确表达"停止导航"的意图
 * - 实际取消由下一个 SendGoal 节点处理
 *
 * 使用场景：
 * - ConfidenceHysteresis 从 HIGH/GRACE_PERIOD 切换到 LOW 时
 * - 需要明确标记导航意图改变的位置
 *
 * 使用示例:
 *   <AbortNavigation action_name="/navigate_to_pose"/>
 *
 * 注意：
 * - 此节点不执行实际的 ROS2 操作
 * - 主要用于行为树逻辑清晰和调试
 * - 下一个 SendGoal 会自动取消之前的导航目标
 */
class AbortNavigationAction : public BT::SyncActionNode
{
public:
    AbortNavigationAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("action_name", "/navigate_to_pose", "Navigation action server name (for logging/debugging)")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string action_name;
        if (!getInput<std::string>("action_name", action_name)) {
            action_name = "/navigate_to_pose";
        }

        // 记录中止意图（用于调试）
        RCLCPP_INFO(rclcpp::get_logger("AbortNavigation"),
                    "Navigation abort requested for: %s (will be canceled by next SendGoal)",
                    action_name.c_str());

        // 直接返回成功，实际取消由下一个 SendGoal 处理
        return BT::NodeStatus::SUCCESS;
    }
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__ABORT_NAVIGATION_HPP_
