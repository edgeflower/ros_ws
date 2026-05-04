#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__CHASE_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__CHASE_GOAL_HPP_

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <optional>
#include <cmath>

namespace rm_behavior_tree
{

class ChaseGoalAction : public BT::StatefulActionNode
{
public:
    ChaseGoalAction(const std::string &name, const BT::NodeConfig &config,
                    const BT::RosNodeParams &params);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "Current goal (read every tick)"),
            BT::InputPort<std::string>("action_name", "/navigate_to_pose", "Nav2 action server name"),
            BT::InputPort<double>("min_goal_distance", 0.3, "Min goal change to trigger resend (m)"),
            BT::InputPort<double>("max_resend_interval", 2.0, "Max time between resends (s)"),
            BT::InputPort<double>("chase_timeout", 15.0, "Max chase duration (s)"),
            BT::InputPort<double>("server_timeout", 5.0, "Goal response timeout (s)")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    // 共享 ROS 节点 + 独立 executor 处理 action 回调
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

    // 状态
    geometry_msgs::msg::PoseStamped last_sent_goal_;
    rclcpp::Time last_send_time_;
    rclcpp::Time chase_start_time_;
    bool has_active_goal_ = false;
    bool waiting_for_response_ = false;
    GoalHandle::SharedPtr goal_handle_;
    std::optional<rclcpp_action::ResultCode> result_;

    // 参数
    double min_goal_distance_ = 0.3;
    double max_resend_interval_ = 2.0;
    double chase_timeout_ = 15.0;
    double server_timeout_ = 5.0;

    bool ensureActionClient();
    bool sendGoal(const geometry_msgs::msg::PoseStamped &goal);
    void cancelCurrentGoal();
    double goalDistance(const geometry_msgs::msg::PoseStamped &a,
                        const geometry_msgs::msg::PoseStamped &b) const;

    void goalResponseCallback(GoalHandle::SharedPtr handle);
    void resultCallback(const GoalHandle::WrappedResult &result);
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__CHASE_GOAL_HPP_
