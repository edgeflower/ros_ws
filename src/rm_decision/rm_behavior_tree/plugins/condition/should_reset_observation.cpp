// ShouldResetObservation Condition Node - Check if all observation points are completed
// Refactored to use RosTopicSubNode
// File: should_reset_observation.cpp

#include "rm_behavior_tree/plugins/condition/should_reset_observation.hpp"

namespace rm_behavior_tree
{

ShouldResetObservationCondition::ShouldResetObservationCondition(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::ObservationPoints>(name, config, params)
, total_points_(0)
{
    RCLCPP_INFO(node_->get_logger(), "ShouldResetObservationCondition initialized");
}

BT::NodeStatus ShouldResetObservationCondition::onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::ObservationPoints>& last_msg)
{
    // Update total_points if we received observation points
    if (last_msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        total_points_ = static_cast<int32_t>(last_msg->points.size());
        RCLCPP_DEBUG(node_->get_logger(), "Total observation points: %u", total_points_);
    }

    // Check for external reset signal
    bool reset_signal = false;
    getInput<bool>("reset_signal", reset_signal);

    if (reset_signal) {
        RCLCPP_INFO(node_->get_logger(), "External reset signal received");
        setOutput("reset_ready", true);
        return BT::NodeStatus::SUCCESS;
    }

    // Get counts from GoalManager
    int32_t idle_count = 0;
    int32_t done_count = 0;
    int32_t total_points_input = 0;

    getInput<int32_t>("idle_count", idle_count);
    getInput<int32_t>("done_count", done_count);
    getInput<int32_t>("total_points", total_points_input);

    // Use the larger of total_points_input or internally tracked total_points_
    int32_t effective_total = std::max(total_points_input, total_points_);

    // Check if all points are done (no IDLE points remaining and done_count equals total)
    bool should_reset = (idle_count == 0) && (done_count > 0) &&
                        (done_count >= effective_total || effective_total == 0);

    std::lock_guard<std::mutex> lock(data_mutex_);

    if (should_reset) {
        RCLCPP_INFO(node_->get_logger(),
                    "All observation points completed (done: %u, total: %u), reset needed",
                    done_count, effective_total);
        setOutput("reset_ready", true);
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG(node_->get_logger(),
                 "Reset not needed yet (idle: %u, done: %u, total: %u)",
                 idle_count, done_count, effective_total);
    setOutput("reset_ready", false);
    return BT::NodeStatus::FAILURE;
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::ShouldResetObservationCondition, "ShouldResetObservation")
