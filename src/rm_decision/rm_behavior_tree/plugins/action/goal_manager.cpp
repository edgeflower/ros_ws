// Goal Manager Action Node - Refactored to use RosTopicSubNode
// Now shares the main ROS node instead of creating a private one
// File: goal_manager.cpp

#include "rm_behavior_tree/plugins/action/goal_manager.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <rclcpp/executors.hpp>

namespace rm_behavior_tree
{

GoalManagerAction::GoalManagerAction(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::ObservationPoints>(name, config, params)
, all_points_completed_(false)
, locked_goal_id_(0)
, locked_goal_score_(0.0)
, last_completed_point_id_(0)
, completed_point_cooldown_(10.0)
, skipped_recovery_time_(30.0)
{
    // Get parameters from ports if provided, otherwise use defaults
    getInput<double>("lethal_threshold", lethal_threshold_);
    getInput<double>("high_cost_threshold", high_cost_threshold_);
    getInput<double>("visit_timeout", visit_timeout_);
    getInput<int>("max_retries", max_retries_);

    // Get new enhancement parameters
    getInput<double>("hysteresis_threshold", hysteresis_threshold_);
    getInput<double>("arrival_distance", arrival_distance_);
    getInput<double>("arrival_speed", arrival_speed_);
    getInput<double>("stay_duration", stay_duration_);
    getInput<double>("min_exclusion_radius", min_exclusion_radius_);
    getInput<double>("auto_reset_cooldown", auto_reset_cooldown_);
    getInput<double>("completed_point_cooldown", completed_point_cooldown_);
    getInput<double>("skipped_recovery_time", skipped_recovery_time_);

    // Set defaults if not provided
    if (lethal_threshold_ <= 0) lethal_threshold_ = 252.0;
    if (high_cost_threshold_ <= 0) high_cost_threshold_ = 200.0;
    if (visit_timeout_ <= 0) visit_timeout_ = 30.0;
    if (max_retries_ < 0) max_retries_ = 2;
    if (hysteresis_threshold_ <= 0) hysteresis_threshold_ = 0.3;
    if (arrival_distance_ <= 0) arrival_distance_ = 0.5;
    if (arrival_speed_ <= 0) arrival_speed_ = 0.1;
    if (stay_duration_ <= 0) stay_duration_ = 1.5;
    if (min_exclusion_radius_ <= 0) min_exclusion_radius_ = 1.0;
    if (auto_reset_cooldown_ <= 0) auto_reset_cooldown_ = 5.0;
    if (completed_point_cooldown_ <= 0) completed_point_cooldown_ = 10.0;
    if (skipped_recovery_time_ <= 0) skipped_recovery_time_ = 30.0;

    RCLCPP_INFO(node_->get_logger(), "GoalManagerAction initialized:");
    RCLCPP_INFO(node_->get_logger(), "  lethal_threshold: %.1f", lethal_threshold_);
    RCLCPP_INFO(node_->get_logger(), "  high_cost_threshold: %.1f", high_cost_threshold_);
    RCLCPP_INFO(node_->get_logger(), "  visit_timeout: %.1f seconds", visit_timeout_);
    RCLCPP_INFO(node_->get_logger(), "  max_retries: %d", max_retries_);
    RCLCPP_INFO(node_->get_logger(), "Enhanced features:");
    RCLCPP_INFO(node_->get_logger(), "  hysteresis_threshold: %.1f%%", hysteresis_threshold_ * 100);
    RCLCPP_INFO(node_->get_logger(), "  arrival_distance: %.2f m", arrival_distance_);
    RCLCPP_INFO(node_->get_logger(), "  arrival_speed: %.2f m/s", arrival_speed_);
    RCLCPP_INFO(node_->get_logger(), "  stay_duration: %.1f s", stay_duration_);
    RCLCPP_INFO(node_->get_logger(), "  min_exclusion_radius: %.2f m", min_exclusion_radius_);
    RCLCPP_INFO(node_->get_logger(), "  auto_reset_cooldown: %.1f s", auto_reset_cooldown_);
    RCLCPP_INFO(node_->get_logger(), "  completed_point_cooldown: %.1f s", completed_point_cooldown_);
    RCLCPP_INFO(node_->get_logger(), "  skipped_recovery_time: %.1f s", skipped_recovery_time_);
}

BT::NodeStatus GoalManagerAction::onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::ObservationPoints>& last_msg)
{
    // Check if we received observation points data
    if (!last_msg) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,   // 5 秒最多打印一次
        "Waiting for observation points..."
    );
    setOutput("should_reset", false);
    return BT::NodeStatus::FAILURE;
}

    RCLCPP_DEBUG(node_->get_logger(), "Received %zu observation points", last_msg->points.size());

    // ========== 握手机制：优先级最高 ==========
    // 在任何其他检测之前，先检查 SendGoal 是否报告了到达
    // 支持正数 ID（巡逻点）和负数 ID（系统任务）
    int32_t reached_goal_id = SystemGoalID::SIGNAL_IDLE;
    auto reached_result = getInput<int32_t>("reached_goal_id");
    if (reached_result.has_value()) {
        reached_goal_id = reached_result.value();
    }

    // 处理所有非空闲信号（包括正数巡逻点和负数系统任务）
    if (reached_goal_id != SystemGoalID::SIGNAL_IDLE) {
        bool should_clear_signal = true;  // 默认清空信号

        // ========== 导航失败信号：负数 < -1000 ==========
        // 格式：-(1000 + point_id)，例如：点5失败 → -1005
        if (reached_goal_id < -1000) {
            int32_t failed_point_id = -reached_goal_id - 1000;

            if (locked_goal_id_ != 0 && failed_point_id == locked_goal_id_) {
                RCLCPP_WARN(node_->get_logger(),
                           "🤝 Handshake: navigation failed for point %u, marking as BLOCKED",
                           locked_goal_id_);

                updateGoalStatus(locked_goal_id_, BLOCKED);
                incrementFailCount(locked_goal_id_);
                releaseGoalLock();

            } else if (locked_goal_id_ == 0) {
                RCLCPP_WARN(node_->get_logger(),
                           "Handshake: received failure signal for point %u but no goal is locked, clearing",
                           failed_point_id);
            } else {
                RCLCPP_DEBUG(node_->get_logger(),
                            "Handshake: expired failure signal (ID=%u vs locked=%u), ignoring",
                            failed_point_id, locked_goal_id_);
                should_clear_signal = false;
            }
        }
        else if (reached_goal_id > 0) {
            // ========== 正数 ID：巡逻点握手 ==========
            if (locked_goal_id_ != 0 && static_cast<int32_t>(reached_goal_id) == locked_goal_id_) {
                RCLCPP_INFO(node_->get_logger(),
                            "🤝 Handshake SUCCESS: patrol point %u completed, marking as DONE",
                            locked_goal_id_);

                updateGoalStatus(locked_goal_id_, DONE);
                resetFailCount(locked_goal_id_);
                releaseGoalLock();

            } else if (locked_goal_id_ == 0) {
                RCLCPP_WARN(node_->get_logger(),
                            "Handshake: received patrol point ID=%d but no goal is locked, clearing",
                            reached_goal_id);

            } else {
                // reached_goal_id != locked_goal_id_，过期信号
                RCLCPP_DEBUG(node_->get_logger(),
                             "Handshake: expired patrol signal (ID=%d vs locked=%u), ignoring",
                             reached_goal_id, locked_goal_id_);
                should_clear_signal = false;  // 过期信号不清空，可能是并发导致的
            }

        } else {
            // ========== 负数 ID：系统任务完成 ==========
            // 不更新任何巡逻点状态，只记录日志并清空信号
            const char* task_name = "unknown";
            switch (reached_goal_id) {
                case SystemGoalID::GO_HOME:
                    task_name = "GO_HOME (回血)";
                    break;
                case SystemGoalID::MANUAL_CONTROL:
                    task_name = "MANUAL_CONTROL (手动接管)";  // 视觉发现敌人追击任务也可以用这个 ID，具体任务类型可以通过日志区分
                    break;
                case SystemGoalID::EMERGENCY_STOP:
                    task_name = "EMERGENCY_STOP (紧急停止)";
                    break;
                default:
                    task_name = "UNKNOWN_SYSTEM_TASK";
                    break;
            }

            RCLCPP_INFO(node_->get_logger(),
                        "🏠 System task completed: %s (ID=%d), signal cleared for patrol logic",
                        task_name, reached_goal_id);

            // 系统任务不影响巡逻状态，但需要确保锁状态合理
            // 如果回家期间锁仍然存在，可能需要释放
            if (locked_goal_id_ != 0) {
                RCLCPP_DEBUG(node_->get_logger(),
                             "System task completed while patrol point %u still locked, keeping lock for now",
                             locked_goal_id_);
            }
        }

        // 阅后即焚：清空所有已处理的信号
        if (should_clear_signal) {
            setOutput("reached_goal_id", SystemGoalID::SIGNAL_IDLE);
            RCLCPP_DEBUG(node_->get_logger(), "Handshake: signal cleared to SIGNAL_IDLE");
        }
    }

    // Get robot pose and speed from input port
    geometry_msgs::msg::PoseStamped robot_pose;
    double current_speed = 0.0;

    if (!getInput<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get robot_pose from input port");
        return BT::NodeStatus::FAILURE;
    }

    // Try to get current speed (optional) 机器人自身速度对于语义到达判断很重要，如果没有提供则默认为0
    getInput<double>("robot_speed", current_speed);

    // Check if reset is requested
    bool reset_requested = false;
    getInput<bool>("reset_requested", reset_requested);
    if (reset_requested) {
        RCLCPP_INFO(node_->get_logger(), "Reset requested, resetting all points to IDLE");
        resetAllPoints();
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    // Update observation points from the latest message
    for (const auto & point : last_msg->points) {
        observation_points_[point.point_id] = point;

        // Initialize status if not exists
        if (goal_status_list_.find(point.point_id) == goal_status_list_.end()) {
            rm_decision_interfaces::msg::GoalStatus status;
            status.point_id = point.point_id;
            status.status = IDLE;
            status.last_visit_time.sec = 0;
            status.last_visit_time.nanosec = 0;
            goal_status_list_[point.point_id] = status;

            // Initialize visit info
            visit_info_map_[point.point_id] = PointVisitInfo();
            visit_info_map_[point.point_id].point_id = point.point_id;
            visit_info_map_[point.point_id].max_retries = max_retries_;
        }
    }

    RCLCPP_DEBUG(node_->get_logger(), "Total observation points stored: %zu", observation_points_.size());

    // Check for visiting timeouts and update statuses
    RCLCPP_DEBUG(node_->get_logger(), "Checking visiting timeouts...");
    checkVisitingTimeouts();
    RCLCPP_DEBUG(node_->get_logger(), "Visiting timeouts checked");

    // Check for SKIPPED points recovery
    RCLCPP_DEBUG(node_->get_logger(), "Checking SKIPPED points for recovery...");
    checkSkippedRecovery();
    RCLCPP_DEBUG(node_->get_logger(), "SKIPPED recovery checked, finding best point...");

    // ========== 语义到达检测已禁用 ==========
    // 原因：握手机制（基于 nav2 导航结果）更可靠，避免两套机制竞争
    // 如果 SendGoal 发布握手信号，会由握手检测逻辑处理
    // 语义到达检测可能导致：握手先释放锁 → 语义到达检测时 locked_goal_id_ 已为 0
    //
    // 如需启用语义到达作为备用机制，请确保与握手机制互斥（例如：只在 reached_goal_id == SIGNAL_IDLE 时执行）
    //
    // if (locked_goal_id_ != 0 && reached_goal_id == SystemGoalID::SIGNAL_IDLE) {
    //     if (checkSemanticArrival(locked_goal_id_, robot_pose, current_speed)) {
    //         RCLCPP_INFO(node_->get_logger(), "Semantic arrival detected for locked goal ID %u, immediately marking as DONE",
    //                     locked_goal_id_);
    //         updateGoalStatus(locked_goal_id_, DONE);
    //         resetFailCount(locked_goal_id_);
    //         releaseGoalLock();
    //     }
    // }


    // Check if all points are completed
    checkAllPointsCompleted();

    // Try to find best point with hysteresis (soft lock)
    RCLCPP_DEBUG(node_->get_logger(), "Finding best point with hysteresis... (robot_pose: %.2f, %.2f)",
                robot_pose.pose.position.x, robot_pose.pose.position.y);
    int32_t best_id = findBestPointWithHysteresis(robot_pose);
    RCLCPP_DEBUG(node_->get_logger(), "Best ID from hysteresis: %u", best_id);

    // If no IDLE points, check if we can retry blocked points
    if (best_id == 0) {
        RCLCPP_DEBUG(node_->get_logger(), "No IDLE points found, checking retry candidates...");
        best_id = findNearestPointConsideringRetry(robot_pose);
        RCLCPP_DEBUG(node_->get_logger(), "Best ID from retry: %u", best_id);
    }

    // Handle case when no points are available
    if (best_id == 0 && observation_points_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "No observation points available");
        setOutput("should_reset", false);
        return BT::NodeStatus::FAILURE;
    }

    // All points are done - auto reset for continuous patrol loop
    if (best_id == 0 && all_points_completed_) {
        // 统计完成的点数
        int32_t done_count = 0;
        for (const auto & status_pair : goal_status_list_) {
            if (status_pair.second.status == DONE) {
                done_count++;
            }
        }

        RCLCPP_INFO(node_->get_logger(), "✅ All %u observation points completed, auto-resetting for next patrol round...", done_count);

        // 自动重置所有点为 IDLE
        resetAllPoints();

        // 🔄 清除冷却惩罚，允许新一轮巡逻自由选择目标
        // (自动重置意味着新的一轮开始，不应该受上一轮的冷却限制)
        last_completed_point_id_ = 0;
        RCLCPP_DEBUG(node_->get_logger(), "Cooldown cleared after auto-reset for new patrol round");

        // 重置后重新尝试选择目标
        best_id = findBestPointWithHysteresis(robot_pose);

        // 如果重置后仍无点可选（异常情况）
        if (best_id == 0 && observation_points_.empty()) {
            RCLCPP_WARN(node_->get_logger(), "No observation points available even after reset");
            setOutput("should_reset", false);
            return BT::NodeStatus::FAILURE;
        }

        // 如果重置后有可用点，继续正常流程（不返回，继续执行下面的代码）
        RCLCPP_INFO(node_->get_logger(), "Patrol round reset complete, continuing with new targets");
    }

    // No available points but not all completed - check for deadlock
    if (best_id == 0) {
        // Count status of all points
        int32_t idle_count = 0, blocked_count = 0, visiting_count = 0, done_count = 0, skipped_count = 0;
        for (const auto & status_pair : goal_status_list_) {
            switch (status_pair.second.status) {
                case IDLE:     idle_count++;     break;
                case BLOCKED:  blocked_count++;  break;
                case VISITING: visiting_count++; break;
                case DONE:     done_count++;     break;
                case RETRYING: blocked_count++;  break;  // RETRYING 也是阻塞状态
                case SKIPPED:  skipped_count++;  break;  // SKIPPED 也是不可用状态
            }
        }

        // Check cooldown before auto-reset
        auto current_time = node_->now();
        auto cooldown_elapsed = (current_time - last_auto_reset_time_).seconds();

        // 死锁条件：
        // 1. 没有 IDLE 点
        // 2. 有 BLOCKED 或 SKIPPED 点（所有点都不可达或被跳过）
        // 3. 或者有 VISITING 点且有 DONE 点（说明某些点已完成，但当前点卡住了）
        bool is_deadlock = (idle_count == 0) &&
                          (blocked_count > 0 || skipped_count > 0 ||
                           (visiting_count > 0 && done_count > 0));

        // 只有在冷却时间过后才执行自动重置
        if (is_deadlock && cooldown_elapsed >= auto_reset_cooldown_) {
            RCLCPP_WARN(node_->get_logger(),
                        "Deadlock detected: idle=%u, blocked=%u, visiting=%u, done=%u, skipped=%u (cooldown: %.1fs elapsed). Auto-resetting all non-DONE points to IDLE...",
                        idle_count, blocked_count, visiting_count, done_count, skipped_count, cooldown_elapsed);

            // 自动重置所有非 DONE 的点为 IDLE
            for (auto & status_pair : goal_status_list_) {
                if (status_pair.second.status != DONE) {
                    updateGoalStatus(status_pair.first, IDLE);
                    resetFailCount(status_pair.first);
                }
            }
            releaseGoalLock();

            // 🔄 清除冷却惩罚，打破死锁循环
            // (死锁重置意味着异常恢复，不应该受冷却限制)
            last_completed_point_id_ = 0;
            RCLCPP_DEBUG(node_->get_logger(), "Cooldown cleared after deadlock reset");

            // 更新上次重置时间
            last_auto_reset_time_ = current_time;

            // 重置后重新尝试选择
            best_id = findBestPointWithHysteresis(robot_pose);
            if (best_id == 0) {
                best_id = findNearestPointConsideringRetry(robot_pose);
            }

            if (best_id != 0) {
                RCLCPP_INFO(node_->get_logger(), "Auto-reset successful, selected goal ID %u", best_id);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Auto-reset failed, still no available points");
            }
        } else if (is_deadlock) {
            RCLCPP_DEBUG(node_->get_logger(),
                        "Deadlock detected but cooldown active (%.1fs / %.1fs), waiting...",
                        cooldown_elapsed, auto_reset_cooldown_);
        }

        // 如果重置后仍无可选点，才返回 FAILURE
        if (best_id == 0) {
            setOutput("should_reset", false);
            setOutput("idle_count", idle_count);
            setOutput("done_count", done_count);
            return BT::NodeStatus::FAILURE;
        }
    }

    // Get the best observation point
    auto it = observation_points_.find(best_id);
    if (it == observation_points_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Point ID %u not found in observation points", best_id);
        return BT::NodeStatus::FAILURE;
    }

    // Create goal pose
    geometry_msgs::msg::PoseStamped best_goal;
    best_goal.header.stamp = node_->now();
    best_goal.header.frame_id = "map";
    best_goal.pose = it->second.pose;

    // Check costmap status
    double cost_value = 0.0;
    CostmapStatus cost_status = checkCostmapStatus(best_goal, cost_value);

    if (cost_status == LETHAL || cost_status == OUT_OF_BOUNDS) {
        RCLCPP_WARN(node_->get_logger(), "Goal ID %u is unreachable (cost status: %d, cost: %.2f), marking as BLOCKED",
                    best_id, static_cast<int>(cost_status), cost_value);
        updateGoalStatus(best_id, BLOCKED);

        // Increment failure count for penalty mechanism
        incrementFailCount(best_id);

        // Release goal lock when blocked
        if (locked_goal_id_ == best_id) {
            releaseGoalLock();
        }

        // Try to retry if max retries not reached
        if (!retryBlockedPoint(best_id)) {
            RCLCPP_ERROR(node_->get_logger(), "Point ID %u exceeded max retries, skipping", best_id);
        }

        setOutput("should_reset", false);
        return BT::NodeStatus::FAILURE;
    }

    if (cost_status == HIGH_COST) {
        RCLCPP_WARN(node_->get_logger(), "Goal ID %u has high cost (%.2f), delaying visit", best_id, cost_value);
        // For high cost points, we still mark as VISITING but log warning
        // The navigation stack will handle the difficulty
    }

    // Mark as VISITING and setup visit tracking
    updateGoalStatus(best_id, VISITING);
    visit_info_map_[best_id].point_id = best_id;
    visit_info_map_[best_id].visit_start_time = node_->now();
    visit_info_map_[best_id].max_duration_seconds = visit_timeout_;
    visit_info_map_[best_id].arrival_detected = false;

    // Lock this goal (soft lock with hysteresis)
    locked_goal_id_ = best_id;
    locked_goal_score_ = it->second.score;

    RCLCPP_DEBUG(node_->get_logger(), "Goal ID %u locked with score %.2f", best_id, locked_goal_score_);

    // Count IDLE, DONE, and SKIPPED points
    int32_t idle_count = 0, done_count = 0, skipped_count = 0;
    for (const auto & status_pair : goal_status_list_) {
        if (status_pair.second.status == IDLE) idle_count++;
        if (status_pair.second.status == DONE) done_count++;
        if (status_pair.second.status == SKIPPED) skipped_count++;
    }

    // Log skipped points warning
    if (skipped_count > 0) {
        RCLCPP_WARN(node_->get_logger(),
                   "Currently %u points are SKIPPED (unreachable after retries), they will be reset on next round",
                   skipped_count);
    }

    // Prepare goal statuses for Blackboard sharing
    std::vector<GoalStatusEntry> goal_statuses;
    for (const auto & status_pair : goal_status_list_) {
        GoalStatusEntry entry;
        entry.point_id = status_pair.first;
        entry.status = static_cast<uint8_t>(status_pair.second.status);
        goal_statuses.push_back(entry);
    }

    // Output results
    setOutput("best_goal", best_goal);
    setOutput("selected_id", best_id);
    setOutput("should_reset", false);
    setOutput("idle_count", idle_count);
    setOutput("done_count", done_count);
    setOutput("goal_statuses", goal_statuses);

    // 握手机制：切换新目标时清空握手信号，确保信号新鲜度
    setOutput("reached_goal_id", SystemGoalID::SIGNAL_IDLE);
    RCLCPP_DEBUG(node_->get_logger(), "Handshake: cleared reached_goal_id for new goal %u", best_id);

    RCLCPP_INFO(node_->get_logger(), "Selected goal ID %u at (%.2f, %.2f) with score %.2f, cost: %.2f",
                best_id, best_goal.pose.position.x, best_goal.pose.position.y,
                it->second.score, cost_value);

    return BT::NodeStatus::SUCCESS;
}

int32_t GoalManagerAction::findNearestIdlePoint(
    const geometry_msgs::msg::PoseStamped & robot_pose)
{
    double min_dist = std::numeric_limits<double>::max();
    int32_t best_id = 0;

    for (const auto & status_pair : goal_status_list_) {
        const auto & status = status_pair.second;

        // Only consider IDLE points
        if (status.status != IDLE) {
            continue;
        }

        // Find corresponding observation point
        auto it = observation_points_.find(status.point_id);
        if (it == observation_points_.end()) {
            continue;
        }

        // Apply dynamic proximity exclusion
        if (isPointTooClose(robot_pose, it->second)) {
            continue;
        }

        // Calculate distance
        double dist = euclideanDistance(robot_pose, it->second);

        // Update best if closer
        if (dist < min_dist) {
            min_dist = dist;
            best_id = status.point_id;
        }
    }
    RCLCPP_DEBUG(node_->get_logger(), "Nearest IDLE point: ID %u, distance %.2f", best_id, min_dist);

    return best_id;
}

int32_t GoalManagerAction::findNearestPointConsideringRetry(
    const geometry_msgs::msg::PoseStamped & robot_pose)
{
    double min_dist = std::numeric_limits<double>::max();
    int32_t best_id = 0;

    for (const auto & status_pair : goal_status_list_) {
        const auto & status = status_pair.second;

        // Consider RETRYING and SKIPPED points (SKIPPED can be reset and retried)
        if (status.status != RETRYING && status.status != SKIPPED) {
            continue;
        }

        // Find corresponding observation point
        auto it = observation_points_.find(status.point_id);
        if (it == observation_points_.end()) {
            continue;
        }

        // Calculate distance
        double dist = euclideanDistance(robot_pose, it->second);

        // Update best if closer
        if (dist < min_dist) {
            min_dist = dist;
            best_id = status.point_id;
        }
    }

    return best_id;
}

double GoalManagerAction::euclideanDistance(
    const geometry_msgs::msg::PoseStamped & pose1,
    const rm_decision_interfaces::msg::ObservationPoint & point2)
{
    double dx = pose1.pose.position.x - point2.pose.position.x;
    double dy = pose1.pose.position.y - point2.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

GoalManagerAction::CostmapStatus GoalManagerAction::checkCostmapStatus(
    const geometry_msgs::msg::PoseStamped & goal, double & cost_value)
{
    // NON-BLOCKING: Only check if service exists, don't actually call it
    // Calling spin_until_future_complete inside BT node causes Executor conflicts

    // Create costmap service client if not exists
    if (!costmap_client_) {
        costmap_client_ = node_->create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");
    }

    // NON-BLOCKING check: Only verify service availability without spinning
    if (!costmap_client_->service_is_ready()) {
        RCLCPP_DEBUG(node_->get_logger(), "Costmap service not ready, skipping check");
        cost_value = 0.0;
        return REACHABLE;
    }

    // Skip actual costmap checking to avoid blocking
    // The navigation stack will handle obstacle avoidance
    cost_value = 0.0;
    return REACHABLE;
}

void GoalManagerAction::updateGoalStatus(int32_t point_id, GoalStatusEnum status)
{
    auto it = goal_status_list_.find(point_id);
    if (it != goal_status_list_.end()) {
        it->second.status = status;
        it->second.last_visit_time = node_->now();
    }
}

void GoalManagerAction::checkVisitingTimeouts()
{
    auto current_time = node_->now();

    // 快速失败阈值：如果在这个时间内超时，认为是导航失败而不是超时
    const double QUICK_FAILURE_THRESHOLD = 5.0;  // 5秒

    for (auto & status_pair : goal_status_list_) {
        int32_t point_id = status_pair.first;
        auto & status = status_pair.second;

        // Check VISITING points for timeout
        if (status.status == VISITING) {
            auto visit_it = visit_info_map_.find(point_id);
            if (visit_it != visit_info_map_.end()) {
                auto elapsed = (current_time - visit_it->second.visit_start_time).seconds();

                if (elapsed > visit_it->second.max_duration_seconds) {
                    // 区分快速失败和正常超时
                    if (elapsed < QUICK_FAILURE_THRESHOLD) {
                        // 快速失败：可能是导航不可达（LETHAL）或路径规划失败
                        RCLCPP_WARN(node_->get_logger(),
                                   "🚫 Point ID %u failed quickly (%.1f s < %.1f s threshold), likely unreachable. Marking as BLOCKED instead of DONE",
                                   point_id, elapsed, QUICK_FAILURE_THRESHOLD);
                        updateGoalStatus(point_id, BLOCKED);
                        incrementFailCount(point_id);  // 增加失败计数

                        // 释放锁（如果当前锁定了这个点）
                        if (locked_goal_id_ == point_id) {
                            releaseGoalLock();
                        }
                    } else {
                        // 正常超时：机器人确实在导航，但超时了
                        RCLCPP_INFO(node_->get_logger(),
                                   "⏱️ Point ID %u visiting timeout (%.1f s), marking as DONE",
                                   point_id, elapsed);
                        updateGoalStatus(point_id, DONE);
                        resetFailCount(point_id);  // Reset failure count on successful completion
                    }
                }
            }
        }
    }
}

void GoalManagerAction::checkSkippedRecovery()
{
    auto current_time = node_->now();
    int32_t recovered_count = 0;

    for (auto & status_pair : goal_status_list_) {
        int32_t point_id = status_pair.first;
        auto & status = status_pair.second;

        // Check SKIPPED points for recovery
        if (status.status == SKIPPED) {
            auto visit_it = visit_info_map_.find(point_id);
            if (visit_it != visit_info_map_.end()) {
                // Check if skip_time is set (rclcpp::Time(0) has seconds() == 0)
                if (visit_it->second.skip_time.seconds() > 0) {
                    auto elapsed_since_skip = (current_time - visit_it->second.skip_time).seconds();

                    if (elapsed_since_skip >= skipped_recovery_time_) {
                        RCLCPP_INFO(node_->get_logger(),
                                   "🔄 SKIPPED point ID %u has recovered after %.1f s, marking as IDLE",
                                   point_id, elapsed_since_skip);

                        // Reset to IDLE and clear retry count
                        updateGoalStatus(point_id, IDLE);
                        visit_it->second.retry_count = 0;
                        resetFailCount(point_id);

                        recovered_count++;
                    }
                }
            }
        }
    }

    if (recovered_count > 0) {
        RCLCPP_INFO(node_->get_logger(), "Recovered %u SKIPPED points to IDLE status", recovered_count);
    }
}

void GoalManagerAction::checkAllPointsCompleted()
{
    if (observation_points_.empty()) {
        all_points_completed_ = false;
        return;
    }

    bool all_done = true;
    int32_t total_points = 0;
    int32_t skipped_count = 0;

    for (const auto & status_pair : goal_status_list_) {
        uint8_t status = status_pair.second.status;

        // SKIPPED 也视为"完成"，允许循环继续（防止僵尸状态导致死锁）
        if (status != DONE && status != SKIPPED) {
            all_done = false;
            break;
        }
        if (status == DONE) total_points++;
        if (status == SKIPPED) {
            total_points++;
            skipped_count++;
        }
    }

    all_points_completed_ = (all_done && total_points > 0);

    if (all_points_completed_) {
        if (skipped_count > 0) {
            RCLCPP_WARN(node_->get_logger(),
                       "All observation points processed! (%u done, %u skipped due to retry limit)",
                       total_points - skipped_count, skipped_count);
        } else {
            RCLCPP_INFO(node_->get_logger(), "All %u observation points completed!", total_points);
        }
    }
}

bool GoalManagerAction::retryBlockedPoint(int32_t point_id)
{
    auto visit_it = visit_info_map_.find(point_id);
    if (visit_it == visit_info_map_.end()) {
        return false;
    }

    if (visit_it->second.retry_count < visit_it->second.max_retries) {
        visit_it->second.retry_count++;
        updateGoalStatus(point_id, RETRYING);
        RCLCPP_INFO(node_->get_logger(), "Point ID %u set to RETRYING (attempt %d/%d)",
                    point_id, visit_it->second.retry_count, visit_it->second.max_retries);
        return true;
    }

    // 达到最大重试次数，强制跳过以防止死锁
    RCLCPP_WARN(node_->get_logger(),
               "🚫 Point ID %u exceeded max retries (%d), marking as SKIPPED to prevent deadlock",
               point_id, visit_it->second.max_retries);

    updateGoalStatus(point_id, SKIPPED);
    resetFailCount(point_id);

    // 记录跳过时间，用于后续恢复机制
    visit_it->second.skip_time = node_->now();
    RCLCPP_DEBUG(node_->get_logger(),
                "SKIPPED point ID %u skip_time set, will recover after %.1f seconds",
                point_id, skipped_recovery_time_);

    // 释放锁（如果当前锁定了这个点）
    if (locked_goal_id_ == point_id) {
        RCLCPP_DEBUG(node_->get_logger(),
                    "Releasing lock on skipped point ID %u", point_id);
        releaseGoalLock();
    }

    return false;
}

bool GoalManagerAction::isGoalReached(int32_t point_id) const
{
    auto it = goal_status_list_.find(point_id);
    return (it != goal_status_list_.end() && it->second.status == DONE);
}

bool GoalManagerAction::shouldResetAll() const
{
    return all_points_completed_;
}

void GoalManagerAction::resetAllPoints()
{
    // 🔄 冷却惩罚机制：记录最后完成的点，防止重置后立即返回
    last_completed_point_id_ = 0;
    for (const auto & status_pair : goal_status_list_) {
        if (status_pair.second.status == DONE) {
            last_completed_point_id_ = status_pair.first;
            break;
        }
    }
    if (last_completed_point_id_ != 0) {
        last_complete_time_ = node_->now();
        RCLCPP_INFO(node_->get_logger(), "🎯 Cooldown: Point ID %u was the last completed point, will be cooled for %.1f seconds",
                    last_completed_point_id_, completed_point_cooldown_);
    }

    for (auto & status_pair : goal_status_list_) {
        status_pair.second.status = IDLE;
        status_pair.second.last_visit_time = node_->now();
    }

    // Reset visit info
    for (auto & visit_pair : visit_info_map_) {
        visit_pair.second.retry_count = 0;
        visit_pair.second.arrival_detected = false;
    }

    // Reset failure counts
    for (auto & fail_pair : fail_count_map_) {
        fail_pair.second = 0;
    }

    // Release goal lock
    releaseGoalLock();

    all_points_completed_ = false;

    // 握手机制：清空握手信号，确保新的一轮巡逻从"白纸"开始
    setOutput("reached_goal_id", SystemGoalID::SIGNAL_IDLE);
    RCLCPP_INFO(node_->get_logger(), "Reset all %zu observation points to IDLE (handshake cleared)",
                goal_status_list_.size());
}

std::vector<int32_t> GoalManagerAction::sortPointsByNearestNeighbor(
    const geometry_msgs::msg::PoseStamped & robot_pose)
{
    std::vector<int32_t> sorted_ids;
    std::set<int32_t> unvisited;

    // Collect all unvisited (IDLE) points
    for (const auto & status_pair : goal_status_list_) {
        if (status_pair.second.status == IDLE) {
            unvisited.insert(status_pair.first);
        }
    }

    geometry_msgs::msg::PoseStamped current_pose = robot_pose;

    // Greedy nearest neighbor algorithm
    while (!unvisited.empty()) {
        double min_dist = std::numeric_limits<double>::max();
        int32_t nearest_id = 0;

        for (int32_t point_id : unvisited) {
            auto it = observation_points_.find(point_id);
            if (it != observation_points_.end()) {
                double dist = euclideanDistance(current_pose, it->second);
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest_id = point_id;
                }
            }
        }

        if (nearest_id != 0) {
            sorted_ids.push_back(nearest_id);
            unvisited.erase(nearest_id);

            // Update current pose to the selected point
            auto it = observation_points_.find(nearest_id);
            if (it != observation_points_.end()) {
                current_pose.pose.position.x = it->second.pose.position.x;
                current_pose.pose.position.y = it->second.pose.position.y;
            }
        } else {
            break;  // Should not happen
        }
    }

    return sorted_ids;
}

// ============================================================================
// Soft Lock / Hysteresis Implementation
// ============================================================================

int32_t GoalManagerAction::findBestPointWithHysteresis(
    const geometry_msgs::msg::PoseStamped & robot_pose)
{
    // If we have a locked goal, check if it's still valid
    if (locked_goal_id_ != 0) {
        auto status_it = goal_status_list_.find(locked_goal_id_);
        if (status_it != goal_status_list_.end() && status_it->second.status == VISITING) {
            // Locked goal is still active, keep using it
            RCLCPP_DEBUG(node_->get_logger(), "Keeping locked goal ID %u (hysteresis active)", locked_goal_id_);
            return locked_goal_id_;
        } else {
            // Locked goal is no longer VISITING (DONE/BLOCKED), release lock
            RCLCPP_DEBUG(node_->get_logger(), "Releasing lock on goal ID %u (status: %d)",
                        locked_goal_id_, status_it != goal_status_list_.end() ? status_it->second.status : -1);
            releaseGoalLock();
        }
    }

    // No locked goal, find best candidate with enhanced scoring
    double best_score = -1.0;
    double best_distance = std::numeric_limits<double>::max();
    int32_t best_id = 0;

    for (const auto & status_pair : goal_status_list_) {
        const auto & status = status_pair.second;

        // Only consider IDLE points
        if (status.status != IDLE) {
            RCLCPP_DEBUG(node_->get_logger(), "Point ID %u: status=%d (skipping, not IDLE)",
                        status.point_id, status.status);
            continue;
        }

        // 🎯 冷却惩罚机制：跳过刚刚完成的点（在冷却期内）
        if (last_completed_point_id_ != 0 && status.point_id == last_completed_point_id_) {
            auto elapsed = (node_->now() - last_complete_time_).seconds();
            if (elapsed < completed_point_cooldown_) {
                RCLCPP_DEBUG(node_->get_logger(),
                           "Point ID %u is in cooldown (%.1f / %.1f s), skipping to prevent immediate return",
                           status.point_id, elapsed, completed_point_cooldown_);
                continue;
            } else {
                RCLCPP_DEBUG(node_->get_logger(),
                           "Point ID %u cooldown expired (%.1f s), now available for selection",
                           status.point_id, elapsed);
            }
        }

        // Find corresponding observation point
        auto it = observation_points_.find(status.point_id);
        if (it == observation_points_.end()) {
            continue;
        }

        // Apply dynamic proximity exclusion
        if (isPointTooClose(robot_pose, it->second)) {
            continue;
        }

        // Calculate distance
        double dist = euclideanDistance(robot_pose, it->second);

        // Combined score: prioritize observation score, then distance
        // Improved distance normalization with smooth decay (exp-based)
        // exp(-dist / 5.0): 0m → 1.0, 5m → 0.37, 10m → 0.14, 20m → 0.02
        // This ensures distant points still have some distance differentiation
        double distance_score = std::exp(-dist / 5.0);

        // Final score: 70% observation score, 30% distance
        double combined_score = 0.7 * it->second.score + 0.3 * distance_score;

        // Apply failure penalty
        combined_score = getPenaltyScore(status.point_id, combined_score);

        // Update best if score is better, or if score is similar but closer
        if (combined_score > best_score ||
            (std::abs(combined_score - best_score) < 0.05 && dist < best_distance)) {
            best_score = combined_score;
            best_distance = dist;
            best_id = status.point_id;
        }
    }

    if (best_id != 0) {
        RCLCPP_INFO(node_->get_logger(), "Selected new candidate ID %u (score: %.2f, distance: %.2f)",
                    best_id, best_score, best_distance);
    } else {
        RCLCPP_WARN(node_->get_logger(), "No suitable IDLE point found (all points blocked or in cooldown)");
        // 注意：这里只做日志记录，不自动重置
        // 重置逻辑由主逻辑中的 "all_points_completed_" 或死锁检测处理
    }

    return best_id;
}

bool GoalManagerAction::shouldSwitchGoal(int32_t new_id, double new_score)
{
    if (locked_goal_id_ == 0) {
        return true;  // No locked goal, always switch
    }

    // Calculate score improvement
    double score_improvement = (new_score - locked_goal_score_) / (locked_goal_score_ + 1e-6);

    // Only switch if improvement exceeds hysteresis threshold
    if (score_improvement >= hysteresis_threshold_) {
        RCLCPP_INFO(node_->get_logger(), "Switching goal: %u -> %u (improvement: %.1f%%, threshold: %.1f%%)",
                    locked_goal_id_, new_id, score_improvement * 100, hysteresis_threshold_ * 100);
        return true;
    }

    return false;
}

void GoalManagerAction::releaseGoalLock()
{
    if (locked_goal_id_ != 0) {
        RCLCPP_DEBUG(node_->get_logger(), "Releasing goal lock on ID %u", locked_goal_id_);
        locked_goal_id_ = 0;
        locked_goal_score_ = 0.0;
    }
}

// ============================================================================
// Semantic Arrival Implementation
// ============================================================================

bool GoalManagerAction::checkSemanticArrival(
    int32_t point_id,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double current_speed)
{
    auto visit_it = visit_info_map_.find(point_id);
    if (visit_it == visit_info_map_.end() || visit_it->second.arrival_detected) {
        return false;
    }

    auto point_it = observation_points_.find(point_id);
    if (point_it == observation_points_.end()) {
        return false;
    }

    // Calculate distance to goal
    double dist = euclideanDistance(robot_pose, point_it->second);

    // Check arrival conditions: close enough AND moving slowly
    if (dist <= arrival_distance_ && current_speed <= arrival_speed_) {
        visit_it->second.arrival_detected = true;
        visit_it->second.arrival_time = node_->now();
        RCLCPP_INFO(node_->get_logger(),
                    "Semantic arrival at goal ID %u: distance=%.2fm (threshold=%.2fm), speed=%.2fm/s (threshold=%.2fm/s)",
                    point_id, dist, arrival_distance_, current_speed, arrival_speed_);
        return true;
    }

    return false;
}

bool GoalManagerAction::checkStayCompletion(int32_t point_id)
{
    auto visit_it = visit_info_map_.find(point_id);
    if (visit_it == visit_info_map_.end() || !visit_it->second.arrival_detected) {
        return false;
    }

    auto elapsed = (node_->now() - visit_it->second.arrival_time).seconds();

    if (elapsed >= stay_duration_) {
        RCLCPP_DEBUG(node_->get_logger(), "Stay duration complete for goal ID %u: %.2fs / %.1fs",
                    point_id, elapsed, stay_duration_);
        return true;
    }

    return false;
}

// ============================================================================
// Dynamic Proximity Exclusion Implementation
// ============================================================================

bool GoalManagerAction::isPointTooClose(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const rm_decision_interfaces::msg::ObservationPoint & point)
{
    double dist = euclideanDistance(robot_pose, point);

    if (dist < min_exclusion_radius_) {
        RCLCPP_DEBUG(node_->get_logger(), "Excluding point ID %u: too close (%.2fm < %.2fm)",
                    point.point_id, dist, min_exclusion_radius_);
        return true;
    }

    return false;
}

// ============================================================================
// Failure Penalty Mechanism Implementation
// ============================================================================

double GoalManagerAction::getPenaltyScore(int32_t point_id, double raw_score)
{
    auto fail_it = fail_count_map_.find(point_id);
    if (fail_it == fail_count_map_.end() || fail_it->second == 0) {
        return raw_score;  // No penalty
    }

    int fail_count = fail_it->second;
    // Apply exponential penalty: score *= (0.5^fail_count)
    double penalty_factor = std::pow(0.5, fail_count);
    double penalized_score = raw_score * penalty_factor;

    RCLCPP_DEBUG(node_->get_logger(), "Point ID %u: raw_score=%.2f, fail_count=%d, penalty_factor=%.2f, penalized_score=%.2f",
                point_id, raw_score, fail_count, penalty_factor, penalized_score);

    return penalized_score;
}

void GoalManagerAction::incrementFailCount(int32_t point_id)
{
    fail_count_map_[point_id]++;
    RCLCPP_DEBUG(node_->get_logger(), "Point ID %u: fail_count incremented to %d",
                point_id, fail_count_map_[point_id]);
}

void GoalManagerAction::resetFailCount(int32_t point_id)
{
    auto fail_it = fail_count_map_.find(point_id);
    if (fail_it != fail_count_map_.end()) {
        fail_it->second = 0;
        RCLCPP_DEBUG(node_->get_logger(), "Point ID %u: fail_count reset", point_id);
    }
}

} // namespace rm_behavior_tree

// Use CreateRosNodePlugin macro instead of BT_REGISTER_NODES
// This enables shared node registration
#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::GoalManagerAction, "GoalManager")
