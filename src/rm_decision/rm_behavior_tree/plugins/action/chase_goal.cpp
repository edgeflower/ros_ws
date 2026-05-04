#include "rm_behavior_tree/plugins/action/chase_goal.hpp"
#include <rclcpp/logging.hpp>
#include <cmath>

namespace rm_behavior_tree
{

ChaseGoalAction::ChaseGoalAction(const std::string &name, const BT::NodeConfig &config,
                                 const BT::RosNodeParams &params)
    : BT::StatefulActionNode(name, config)
    , node_(params.nh)
    , last_send_time_(0, 0, RCL_ROS_TIME)
    , chase_start_time_(0, 0, RCL_ROS_TIME)
{
    // 独立 callback group + executor，不干扰主 executor
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_, node_->get_node_base_interface());

    RCLCPP_INFO(node_->get_logger(), "ChaseGoalAction initialized (shared node: %s)", node_->get_name());
}

bool ChaseGoalAction::ensureActionClient()
{
    std::string action_name;
    getInput("action_name", action_name);

    if (!action_client_) {
        action_client_ = rclcpp_action::create_client<NavigateToPose>(
            node_, action_name, callback_group_);
    }

    if (!action_client_->action_server_is_ready()) {
        if (!action_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "Action server '%s' not available", action_name.c_str());
            return false;
        }
    }
    return true;
}

BT::NodeStatus ChaseGoalAction::onStart()
{
    // 读取参数
    getInput("min_goal_distance", min_goal_distance_);
    getInput("max_resend_interval", max_resend_interval_);
    getInput("chase_timeout", chase_timeout_);
    getInput("server_timeout", server_timeout_);

    // 获取初始目标
    auto goal_res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
    if (!goal_res.has_value()) {
        RCLCPP_WARN(node_->get_logger(), "ChaseGoal: no goal_pose provided");
        return BT::NodeStatus::FAILURE;
    }

    if (!ensureActionClient()) {
        return BT::NodeStatus::FAILURE;
    }

    // 重置状态
    result_.reset();
    has_active_goal_ = false;
    waiting_for_response_ = false;
    goal_handle_ = nullptr;
    chase_start_time_ = node_->now();

    if (!sendGoal(goal_res.value())) {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ChaseGoalAction::onRunning()
{
    // 处理 action 回调
    executor_->spin_some(std::chrono::milliseconds(0));

    auto now = node_->now();

    // 1. 追击超时
    if ((now - chase_start_time_).seconds() > chase_timeout_) {
        RCLCPP_WARN(node_->get_logger(),
            "Chase timeout (%.1fs > %.1fs)", (now - chase_start_time_).seconds(), chase_timeout_);
        cancelCurrentGoal();
        return BT::NodeStatus::FAILURE;
    }

    // 2. 检查 result
    if (result_.has_value()) {
        auto code = result_.value();
        result_.reset();
        has_active_goal_ = false;

        if (code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(node_->get_logger(), "Chase goal reached");
            return BT::NodeStatus::SUCCESS;
        }
        if (code == rclcpp_action::ResultCode::CANCELED) {
            // 主动取消（目标更新），不返回 FAILURE，继续发送新目标
        } else {
            RCLCPP_WARN(node_->get_logger(), "Chase goal failed: code=%d", static_cast<int>(code));
            return BT::NodeStatus::FAILURE;
        }
    }

    // 3. 等待 goal response
    if (waiting_for_response_) {
        if ((now - last_send_time_).seconds() > server_timeout_) {
            RCLCPP_WARN(node_->get_logger(), "Goal response timeout");
            has_active_goal_ = false;
            waiting_for_response_ = false;
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    // 4. 判断是否需要更新目标
    auto goal_res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
    if (!goal_res.has_value()) {
        return BT::NodeStatus::RUNNING;
    }

    auto &current_goal = goal_res.value();
    double time_since_send = (now - last_send_time_).seconds();
    double dist_change = has_active_goal_
        ? goalDistance(current_goal, last_sent_goal_)
        : std::numeric_limits<double>::max();

    bool should_resend = !has_active_goal_
        || dist_change > min_goal_distance_
        || time_since_send > max_resend_interval_;

    if (should_resend) {
        cancelCurrentGoal();
        if (!sendGoal(current_goal)) {
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(),
            "Goal updated: dist=%.2fm time=%.1fs → (%.2f, %.2f)",
            dist_change, time_since_send,
            current_goal.pose.position.x, current_goal.pose.position.y);
    }

    return BT::NodeStatus::RUNNING;
}

void ChaseGoalAction::onHalted()
{
    cancelCurrentGoal();
    RCLCPP_INFO(node_->get_logger(), "ChaseGoal halted");
}

bool ChaseGoalAction::sendGoal(const geometry_msgs::msg::PoseStamped &goal)
{
    if (!action_client_ || !action_client_->action_server_is_ready()) {
        return false;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal;
    goal_msg.pose.header.frame_id = goal.header.frame_id; // gimbal_yaw   已在armor_to_goal 中赋值
    goal_msg.pose.header.stamp = node_->now();

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
        [this](GoalHandle::SharedPtr handle) {
            if (handle) {
                goal_handle_ = handle;
                has_active_goal_ = true;
                waiting_for_response_ = false;
            } else {
                waiting_for_response_ = false;
            }
        };
    options.result_callback =
        [this](const GoalHandle::WrappedResult &result) {
            result_ = result.code;
            has_active_goal_ = false;
        };

    waiting_for_response_ = true;
    last_sent_goal_ = goal;
    last_send_time_ = node_->now();
    result_.reset();

    action_client_->async_send_goal(goal_msg, options);
    return true;
}

void ChaseGoalAction::cancelCurrentGoal()
{
    if (goal_handle_ && has_active_goal_) {
        action_client_->async_cancel_goal(goal_handle_);
        has_active_goal_ = false;
        goal_handle_ = nullptr;
    }
}

double ChaseGoalAction::goalDistance(
    const geometry_msgs::msg::PoseStamped &a,
    const geometry_msgs::msg::PoseStamped &b) const
{
    double dx = a.pose.position.x - b.pose.position.x;
    double dy = a.pose.position.y - b.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::ChaseGoalAction, "ChaseGoal");
