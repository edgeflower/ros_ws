#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__HOLD_POSITION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__HOLD_POSITION_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <armor_interfaces/msg/target.hpp>

namespace rm_behavior_tree
{

/**
 * @brief 在观察期保持最后目标位置的Action节点
 *
 * 用于配合 ConfidenceHysteresis 使用，在置信度抖动期间：
 * - 保持最后的有效目标点
 * - 防止机器人原地停止或盲目切换
 * - 提供平滑的模式过渡
 *
 * 使用场景：
 * - ConfidenceHysteresis 进入 GRACE_PERIOD 状态时
 * - 需要在原地短暂等待观察敌人是否重现
 * - 确保导航目标的平滑过渡
 */
class HoldPositionAction : public BT::SyncActionNode
{
public:
    HoldPositionAction(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<armor_interfaces::msg::Target>("target", "Current target message"),
            BT::InputPort<bool>("enabled", true, "Enable position holding"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose", "Current robot pose"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("held_goal", "Held goal position"),
            BT::OutputPort<bool>("is_holding", "Whether we're holding position")
        };
    }

    BT::NodeStatus tick() override;

    void reset();

private:
    geometry_msgs::msg::PoseStamped held_goal_;
    bool is_holding_ = false;
    rclcpp::Time hold_start_time_;
    double hold_timeout_ = 5.0;  // 最长保持时间（秒）

    // Helper functions
    bool isTimeout(const rclcpp::Time &now) const;
    void updateHeldGoal(const armor_interfaces::msg::Target &target);
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__HOLD_POSITION_HPP_
