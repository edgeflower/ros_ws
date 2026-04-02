#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__CANCEL_NAVIGATION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__CANCEL_NAVIGATION_HPP_

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <behaviortree_cpp/basic_types.h>

namespace rm_behavior_tree
{

/**
 * @brief Cancel Navigation Action Node
 *
 * 使用标准的 ROS2 Action 取消机制来中止当前导航
 *
 * 优点：
 * 1. 语义明确：直接取消而不是发送"假目标"
 * 2. 性能更好：避免不必要的路径规划
 * 3. 响应更快：立即停止，无延迟
 * 4. 符合 ROS2 Action 标准设计
 *
 * 使用方法：
 * <CancelNavigation action_name="/navigate_to_pose"/>
 *
 * 实现原理：
 * - 在构造函数中获取 action client
 * - 节点执行时调用 async_cancel_all_goals()
 * - 不发送新目标，直接取消所有活动目标
 */
class CancelNavigationAction : public BT::SyncActionNode
{
public:
    CancelNavigationAction(
        const std::string & name,
        const BT::NodeConfiguration & config,
        const BT::RosNodeParams & params);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("action_name", "/navigate_to_pose", "导航action server名称")
        };
    }

    BT::NodeStatus tick() override;

private:
    std::string action_name_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    rclcpp::Node::SharedPtr node_;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__CANCEL_NAVIGATION_HPP_
