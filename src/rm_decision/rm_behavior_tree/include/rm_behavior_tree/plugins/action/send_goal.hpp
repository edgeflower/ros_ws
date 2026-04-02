#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_GOAL_HPP_

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <nav2_msgs/action/detail/navigate_to_pose__struct.hpp>
#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace rm_behavior_tree
{

class SendGoalAction : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
    SendGoalAction(
        const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & param);
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
            BT::InputPort<std::string>("action_name"),
            BT::InputPort<int32_t>("current_goal_id", 0, "Current goal ID for handshake mechanism"),
            BT::OutputPort<int32_t>("reached_goal_id", "Output goal ID when navigation succeeds (for handshake)"),
            BT::InputPort<double>("min_goal_distance", 0.5, "最小目标变化距离(米)，默认0.5m")
        };
    }

    bool setGoal(Goal & goal) override;
    void onHalt() override;

    BT::NodeStatus onResultReceived(const WrappedResult & wr) override;

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

    BT::NodeStatus onFailure (BT::ActionNodeErrorCode error) override;

    private:
    std::string action_name_;

    // 方案3: 目标变化阈值检测，防止频繁发送微小变化的目标
    geometry_msgs::msg::PoseStamped last_goal_;
    bool has_last_goal_ = false;
    double min_goal_distance_ = 0.5;  // 默认0.5米
};
}


#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_GOAL_HPP_