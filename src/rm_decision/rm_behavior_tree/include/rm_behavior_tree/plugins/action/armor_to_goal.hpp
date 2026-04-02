#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_

#include "armor_interfaces/msg/armor.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <armor_interfaces/msg/detail/target__struct.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/node.hpp>
#include <armor_interfaces/msg/target.hpp>
#include <cmath>

namespace rm_behavior_tree {

/**
 * @brief ArmorToGoalAction - 将敌人位置转换为导航目标点
 *
 * 输入: target_message (已在 map 坐标系的敌人位置)
 * 输出: goal_pose (导航目标点，直接使用敌人位置)
 *
 * 功能: 直接使用敌人位置作为导航目标点，不做偏移计算
 *       置信度检查由 ConfidenceHysteresis 节点统一处理
 */
class ArmorToGoalAction : public BT::SyncActionNode, public rclcpp::Node
{
public:
    ArmorToGoalAction(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<armor_interfaces::msg::Target>("target_message", "Target message with position"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "Output goal pose (enemy position)")
        };
    }

    BT::NodeStatus tick() override;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__ARMOR_TO_GOAL_HPP_
