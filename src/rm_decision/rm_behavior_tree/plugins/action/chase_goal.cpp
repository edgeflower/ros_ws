#include "rm_behavior_tree/plugins/action/chase_goal.hpp"
#include <rclcpp/logging.hpp>
#include <cmath>

namespace rm_behavior_tree
{

// ═══════════════════════════════════════════════════════════════
// 构造 / 析构
// ═══════════════════════════════════════════════════════════════

ChaseGoalAction::ChaseGoalAction(const std::string &name, const BT::NodeConfig &config,
                                 const BT::RosNodeParams &params)
    : BT::StatefulActionNode(name, config)
    , node_(params.nh)
    // 时间初始化为 ROS 时间 0（后续 onStart 会重新赋值）
    , last_send_time_(0, 0, RCL_ROS_TIME)
    , chase_start_time_(0, 0, RCL_ROS_TIME)
{
    // ── 创建 Reentrant callback group ──
    // Reentrant 允许多个回调同时执行，避免回调之间互相阻塞。
    // 对于 action client 的 goal_response 和 result 回调，
    // 使用 Reentrant 可以保证即使一个回调耗时较长也不会饿死另一个。
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    // ── 创建独立 executor 并注册 callback group ──
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_, node_->get_node_base_interface());

    // ── 启动后台 spin 线程 ──
    // spinning_ 原子标志为 true 时线程持续运行
    spinning_.store(true);
    spin_thread_ = std::thread([this]() {
        // 线程循环：只要 spinning_ 为 true 就持续调用 spin_some()
        // spin_some(100ms) 每次最多处理 100ms 的回调，然后返回继续循环
        // 这样既能及时处理回调，又能在 spinning_ 变为 false 时快速退出
        while (spinning_.load()) {
            executor_->spin_some(std::chrono::milliseconds(100));
        }
    });

    RCLCPP_INFO(node_->get_logger(),
        "ChaseGoalAction initialized (shared node: %s, background spin thread started)",
        node_->get_name());
}

ChaseGoalAction::~ChaseGoalAction()
{
    // ── 安全关闭后台线程 ──
    // 步骤 1：设置原子标志，通知线程退出循环
    spinning_.store(false);

    // 步骤 2：等待线程结束（join），最长等待 2 秒
    // 正常情况下线程在 100ms 内（spin_some 的超时）就会检测到
    // spinning_ == false 并退出，这里 2 秒是极端情况的安全上限
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }

    RCLCPP_INFO(node_->get_logger(), "ChaseGoalAction destroyed, spin thread stopped");
}

// ═══════════════════════════════════════════════════════════════
// Action Client 初始化
// ═══════════════════════════════════════════════════════════════

bool ChaseGoalAction::ensureActionClient()
{
    std::string action_name;
    getInput("action_name", action_name);

    if (!action_client_) {
        // 使用独立的 Reentrant callback group 创建 action client
        // 确保 action 回调在后台线程的 executor 中处理
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

// ═══════════════════════════════════════════════════════════════
// BT 生命周期：onStart / onRunning / onHalted
// ═══════════════════════════════════════════════════════════════

BT::NodeStatus ChaseGoalAction::onStart()
{
    // 读取参数（仅 BT tick 线程调用，无需加锁）
    getInput("min_goal_distance", min_goal_distance_);
    getInput("max_resend_interval", max_resend_interval_);
    getInput("chase_timeout", chase_timeout_);
    getInput("server_timeout", server_timeout_);
    getInput("stop_distance", stop_distance_);

    // 获取初始目标
    auto goal_res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
    if (!goal_res.has_value()) {
        RCLCPP_WARN(node_->get_logger(), "ChaseGoal: no goal_pose provided");
        return BT::NodeStatus::FAILURE;
    }

    // 近距离检查：敌人在 stop_distance 内直接 SUCCESS，不发 goal
    if (stop_distance_ > 0.0) {
        double dx = goal_res.value().pose.position.x;
        double dy = goal_res.value().pose.position.y;
        if (std::sqrt(dx * dx + dy * dy) <= stop_distance_) {
            return BT::NodeStatus::SUCCESS;
        }
    }

    if (!ensureActionClient()) {
        return BT::NodeStatus::FAILURE;
    }

    // 重置状态（加锁保护，因为后台线程可能正在访问这些变量）
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        result_.reset();
        has_active_goal_ = false;
        waiting_for_response_ = false;
        goal_handle_ = nullptr;
    }

    chase_start_time_ = node_->now();

    if (!sendGoal(goal_res.value())) {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ChaseGoalAction::onRunning()
{
    // ── 不再调用 executor_->spin_some() ──
    // 后台线程已经在持续 spin，回调会自动被处理。
    // 这里只做状态检查和决策逻辑。

    auto now = node_->now();

    // ── 1. 近距离检查 ──
    if (stop_distance_ > 0.0) {
        auto goal_res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
        if (goal_res.has_value()) {
            double dx = goal_res.value().pose.position.x;
            double dy = goal_res.value().pose.position.y;
            if (std::sqrt(dx * dx + dy * dy) <= stop_distance_) {
                cancelCurrentGoal();
                return BT::NodeStatus::SUCCESS;
            }
        }
    }

    // ── 2. 追击超时检查 ──
    if ((now - chase_start_time_).seconds() > chase_timeout_) {
        RCLCPP_WARN(node_->get_logger(),
            "Chase timeout (%.1fs > %.1fs)", (now - chase_start_time_).seconds(), chase_timeout_);
        cancelCurrentGoal();
        return BT::NodeStatus::FAILURE;
    }

    // ── 3. 检查 action result ──
    // 加锁读取 result_，因为后台线程的 result_callback 可能正在写入
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

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
    } // 释放锁

    // ── 4. 等待 goal response 超时检查 ──
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (waiting_for_response_) {
            if ((now - last_send_time_).seconds() > server_timeout_) {
                RCLCPP_WARN(node_->get_logger(), "Goal response timeout");
                has_active_goal_ = false;
                waiting_for_response_ = false;
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }
    } // 释放锁

    // ── 5. 判断是否需要更新目标 ──
    auto goal_res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
    if (!goal_res.has_value()) {
        return BT::NodeStatus::RUNNING;
    }

    auto &current_goal = goal_res.value();
    double time_since_send = (now - last_send_time_).seconds();

    // 加锁读取 last_sent_goal_ 和 has_active_goal_
    double dist_change;
    bool active;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        active = has_active_goal_;
        dist_change = active
            ? goalDistance(current_goal, last_sent_goal_)
            : std::numeric_limits<double>::max();
    }

    bool should_resend = !active
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

// ═══════════════════════════════════════════════════════════════
// Goal 发送 / 取消
// ═══════════════════════════════════════════════════════════════

bool ChaseGoalAction::sendGoal(const geometry_msgs::msg::PoseStamped &goal)
{
    if (!action_client_ || !action_client_->action_server_is_ready()) {
        return false;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal;
    goal_msg.pose.header.frame_id = "gimbal_yaw"; // gimbal_yaw   已在armor_to_goal 中赋值 手动加上
    goal_msg.pose.header.stamp = node_->now();

    // 将目标点缩放到距敌人 stop_distance_ 处（gimbal_yaw 原点即机器人自身）
    if (stop_distance_ > 0.0) {
        double dx = goal_msg.pose.pose.position.x;
        double dy = goal_msg.pose.pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > stop_distance_) {
            double scale = (dist - stop_distance_) / dist;
            goal_msg.pose.pose.position.x *= scale;
            goal_msg.pose.pose.position.y *= scale;
        }
    }

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    // ── goal_response_callback ──
    // 由后台 spin 线程中的 executor 触发，需要加锁保护共享状态
    options.goal_response_callback =
        [this](GoalHandle::SharedPtr handle) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (handle) {
                // 服务器接受了 goal，记录 handle
                goal_handle_ = handle;
                has_active_goal_ = true;
                waiting_for_response_ = false;
            } else {
                // 服务器拒绝了 goal
                waiting_for_response_ = false;
            }
        };

    // ── result_callback ──
    // 由后台 spin 线程中的 executor 触发，需要加锁保护共享状态
    options.result_callback =
        [this](const GoalHandle::WrappedResult &result) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            result_ = result.code;
            has_active_goal_ = false;
        };

    // ── 更新发送状态（加锁）──
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        waiting_for_response_ = true;
        last_sent_goal_ = goal;
        result_.reset();
    }

    // last_send_time_ 仅在 BT tick 线程中读写，不需要加锁
    last_send_time_ = node_->now();

    // async_send_goal 将请求投递到 ROS2 中间件，
    // 回调会在后台线程的 executor 中被处理
    action_client_->async_send_goal(goal_msg, options);
    return true;
}

void ChaseGoalAction::cancelCurrentGoal()
{
    // 加锁保护 goal_handle_ 和 has_active_goal_ 的读写
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (goal_handle_ && has_active_goal_) {
        action_client_->async_cancel_goal(goal_handle_);
        has_active_goal_ = false;
        goal_handle_ = nullptr;
    }
}

// ═══════════════════════════════════════════════════════════════
// 工具函数
// ═══════════════════════════════════════════════════════════════

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
