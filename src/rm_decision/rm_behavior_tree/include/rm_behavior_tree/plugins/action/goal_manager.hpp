#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__GOAL_MANAGER_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__GOAL_MANAGER_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rm_decision_interfaces/msg/observation_points.hpp>
#include <rm_decision_interfaces/msg/observation_point.hpp>
#include <rm_decision_interfaces/msg/goal_status.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/client.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <vector>
#include <map>
#include <mutex>
#include <chrono>

namespace rm_behavior_tree
{

enum GoalStatusEnum : uint8_t {
    IDLE = 0,
    VISITING = 1,
    DONE = 2,
    BLOCKED = 3,
    RETRYING = 4,
    SKIPPED = 5  // 僵尸状态：重试次数超限后被跳过，视为完成以防止死锁
};

// System Goal IDs for special tasks (negative values to avoid conflict with patrol points)
enum SystemGoalID : int32_t {
    SIGNAL_IDLE = -1,    // 空闲/握手清空信号
    GO_HOME = -2,        // 回家补血任务
    MANUAL_CONTROL = -3, // 手动接管任务  // 视觉发现敌人追击任务也可以用这个 ID，具体任务类型可以通过日志区分
    EMERGENCY_STOP = -4  // 紧急停止任务
};

// Goal status structure for Blackboard sharing (must match IsGoalReachedCondition)
struct GoalStatusEntry {
    int32_t point_id;
    uint8_t status;

    GoalStatusEntry() : point_id(0), status(IDLE) {}
    GoalStatusEntry(int32_t id, uint8_t s) : point_id(id), status(s) {}
};

// Point visit tracking
struct PointVisitInfo {
    int32_t point_id;
    rclcpp::Time visit_start_time;
    rclcpp::Time arrival_time;      // Time when semantic arrival was detected
    double max_duration_seconds;
    int retry_count;
    int max_retries;
    bool arrival_detected;          // Flag for semantic arrival detection
    rclcpp::Time skip_time;         // Time when point was marked as SKIPPED

    PointVisitInfo()
        : point_id(0)
        , max_duration_seconds(30.0)
        , retry_count(0)
        , max_retries(2)
        , arrival_detected(false)
    {}
};

// GoalManagerAction now inherits from RosTopicSubNode
// It will receive observation_points messages automatically
class GoalManagerAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::ObservationPoints>
{
public:
    GoalManagerAction(const std::string & name,
                     const BT::NodeConfiguration & config,
                     const BT::RosNodeParams & params);

    // Must provide ports including topic_name port from base class
    static BT::PortsList providedPorts()
    {
        // Combine custom ports with base class ports
        BT::PortsList custom_ports = {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),
            BT::InputPort<double>("lethal_threshold", 252, "Lethal cost threshold"),
            BT::InputPort<double>("high_cost_threshold", 200, "High cost threshold for delay"),
            BT::InputPort<double>("visit_timeout", 30.0, "Max time to mark point as DONE (seconds)"),
            BT::InputPort<int>("max_retries", 2, "Maximum retry attempts for blocked points"),
            BT::InputPort<bool>("reset_requested", false, "Request to reset all points to IDLE"),
            BT::InputPort<double>("hysteresis_threshold", 0.3, "Score improvement threshold for switching targets (0.0-1.0)"),
            BT::InputPort<double>("arrival_distance", 0.5, "Distance threshold for semantic arrival (meters)"),
            BT::InputPort<double>("arrival_speed", 0.1, "Speed threshold for semantic arrival (m/s)"),
            BT::InputPort<double>("stay_duration", 1.5, "Time to wait at goal after arrival (seconds)"),
            BT::InputPort<double>("min_exclusion_radius", 1.0, "Minimum radius to exclude near points (meters)"),
            BT::InputPort<double>("robot_speed", 0.0, "Current robot speed for semantic arrival (m/s)"),
            BT::InputPort<double>("auto_reset_cooldown", 5.0, "Minimum time between auto-resets for deadlock (seconds)"),
            BT::InputPort<double>("completed_point_cooldown", 10.0, "Cooldown time for last completed point to prevent immediate return (seconds)"),
            BT::InputPort<double>("skipped_recovery_time", 30.0, "Time before SKIPPED points automatically recover to IDLE (seconds)"),
            BT::InputPort<int32_t>("reached_goal_id", SystemGoalID::SIGNAL_IDLE, "Handshake ID from SendGoal: patrol (>0) or system task (<0)"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("best_goal"),
            BT::OutputPort<int32_t>("selected_id"),
            BT::OutputPort<bool>("should_reset", "True if all points completed"),
            BT::OutputPort<int32_t>("idle_count", "Number of IDLE points remaining"),
            BT::OutputPort<int32_t>("done_count", "Number of DONE points"),
            BT::OutputPort<std::vector<GoalStatusEntry>>("goal_statuses", "All goal statuses for sharing with BT")
        };
        return BT::RosTopicSubNode<rm_decision_interfaces::msg::ObservationPoints>::providedBasicPorts(custom_ports);
    }

    // onTick is called automatically when the behavior tree ticks this node
    // last_msg contains the latest ObservationPoints message (or nullptr if no message yet)
    BT::NodeStatus onTick(const std::shared_ptr<rm_decision_interfaces::msg::ObservationPoints>& last_msg) override;

    // Public methods for BT condition nodes
    bool isGoalReached(int32_t point_id) const;
    bool shouldResetAll() const;
    void resetAllPoints();

private:
    // Costmap service client (created lazily)
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;

    // Data storage
    std::map<int32_t, rm_decision_interfaces::msg::ObservationPoint> observation_points_;
    std::map<int32_t, rm_decision_interfaces::msg::GoalStatus> goal_status_list_;
    std::map<int32_t, PointVisitInfo> visit_info_map_;
    std::map<int32_t, int> fail_count_map_;  // Failure penalty tracking
    mutable std::mutex data_mutex_;

    // Parameters
    double lethal_threshold_;
    double high_cost_threshold_;
    double visit_timeout_;
    int max_retries_;
    bool all_points_completed_;

    // Soft lock / Hysteresis parameters
    double hysteresis_threshold_;
    int32_t locked_goal_id_;          // Currently locked goal (0 = no lock)
    double locked_goal_score_;         // Score of locked goal for comparison

    // Semantic arrival parameters
    double arrival_distance_;
    double arrival_speed_;
    double stay_duration_;

    // Dynamic proximity exclusion
    double min_exclusion_radius_;

    // Auto-reset deadlock recovery
    double auto_reset_cooldown_;        // Minimum time between auto-resets (seconds)
    rclcpp::Time last_auto_reset_time_; // Last time auto-reset was triggered

    // Completed point cooldown mechanism (prevent immediate return)
    int32_t last_completed_point_id_;  // Last point that was completed
    rclcpp::Time last_complete_time_;     // Time when last point was completed
    double completed_point_cooldown_;     // Cooldown time for last completed point (seconds)

    // SKIPPED point recovery mechanism
    double skipped_recovery_time_;        // Time before SKIPPED points auto-recover to IDLE

    // Helper functions
    int32_t findNearestIdlePoint(const geometry_msgs::msg::PoseStamped & robot_pose);
    int32_t findNearestPointConsideringRetry(const geometry_msgs::msg::PoseStamped & robot_pose);
    double euclideanDistance(const geometry_msgs::msg::PoseStamped & pose1,
                              const rm_decision_interfaces::msg::ObservationPoint & point2);

    // Soft lock / Hysteresis functions
    int32_t findBestPointWithHysteresis(const geometry_msgs::msg::PoseStamped & robot_pose);
    bool shouldSwitchGoal(int32_t new_id, double new_score);
    void releaseGoalLock();

    // Semantic arrival functions
    bool checkSemanticArrival(int32_t point_id,
                              const geometry_msgs::msg::PoseStamped & robot_pose,
                              double current_speed);
    bool checkStayCompletion(int32_t point_id);

    // Dynamic proximity exclusion
    bool isPointTooClose(const geometry_msgs::msg::PoseStamped & robot_pose,
                         const rm_decision_interfaces::msg::ObservationPoint & point);

    // Failure penalty
    double getPenaltyScore(int32_t point_id, double raw_score);
    void incrementFailCount(int32_t point_id);
    void resetFailCount(int32_t point_id);

    // Enhanced costmap checking with delay logic
    enum CostmapStatus { REACHABLE, HIGH_COST, LETHAL, OUT_OF_BOUNDS };
    CostmapStatus checkCostmapStatus(const geometry_msgs::msg::PoseStamped & goal, double & cost_value);

    // Status management
    void updateGoalStatus(int32_t point_id, GoalStatusEnum status);
    void checkVisitingTimeouts();
    void checkSkippedRecovery();
    void checkAllPointsCompleted();
    bool retryBlockedPoint(int32_t point_id);

    // TSP nearest neighbor algorithm
    std::vector<int32_t> sortPointsByNearestNeighbor(const geometry_msgs::msg::PoseStamped & robot_pose);
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__GOAL_MANAGER_HPP_
