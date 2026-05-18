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
#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>

namespace rm_behavior_tree
{

class ChaseGoalAction : public BT::StatefulActionNode
{
public:
    ChaseGoalAction(const std::string &name, const BT::NodeConfig &config,
                    const BT::RosNodeParams &params);

    ~ChaseGoalAction() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "Current goal (read every tick)"),
            BT::InputPort<std::string>("action_name", "/navigate_to_pose", "Nav2 action server name"),
            BT::InputPort<double>("min_goal_distance", 0.3, "Min goal change to trigger resend (m)"),
            BT::InputPort<double>("max_resend_interval", 2.0, "Max time between resends (s)"),
            BT::InputPort<double>("chase_timeout", 15.0, "Max chase duration (s)"),
            BT::InputPort<double>("server_timeout", 5.0, "Goal response timeout (s)"),
            BT::InputPort<double>("stop_distance", 0.0, "Stop chasing when goal is within this distance (m, 0=disabled)")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    // ── ROS 接口 ──────────────────────────────────────────────
    rclcpp::Node::SharedPtr node_;
    // Reentrant callback group：允许回调并发执行
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    // 独立 executor，由后台线程持续 spin
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

    // ── 后台线程控制 ──────────────────────────────────────────
    // executor 后台 spin 线程
    std::thread spin_thread_;
    // 控制线程退出的原子标志
    std::atomic<bool> spinning_{false};

    // ── 线程安全状态 ──────────────────────────────────────────
    // 保护以下共享状态的所有读写：result_, has_active_goal_,
    // waiting_for_response_, goal_handle_, last_sent_goal_
    std::mutex state_mutex_;

    // 追击起始时间（仅在 BT tick 线程中读写，无需加锁）
    rclcpp::Time chase_start_time_;
    // 上次发送 goal 的时间（BT tick 线程写入，onRunning 读取，单线程无需锁）
    rclcpp::Time last_send_time_;

    // 以下状态可被回调线程和 BT tick 线程同时访问，必须加锁
    std::optional<rclcpp_action::ResultCode> result_;
    bool has_active_goal_ = false;
    bool waiting_for_response_ = false;
    GoalHandle::SharedPtr goal_handle_;
    geometry_msgs::msg::PoseStamped last_sent_goal_;

    // ── 行为参数（只在 onStart 中写入，onRunning 中读取，单线程安全）──
    double min_goal_distance_ = 0.3;
    double max_resend_interval_ = 2.0;
    double chase_timeout_ = 15.0;
    double server_timeout_ = 5.0;
    double stop_distance_ = 0.0;

    // ── 内部方法 ──────────────────────────────────────────────
    bool ensureActionClient();
    bool sendGoal(const geometry_msgs::msg::PoseStamped &goal);
    void cancelCurrentGoal();
    double goalDistance(const geometry_msgs::msg::PoseStamped &a,
                        const geometry_msgs::msg::PoseStamped &b) const;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__CHASE_GOAL_HPP_
