#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CONFIDENCE_HYSTERESIS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CONFIDENCE_HYSTERESIS_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "armor_interfaces/msg/target.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace rm_behavior_tree
{

/**
 * @brief Confidence Hysteresis Condition Node
 *
 * 防止置信度在阈值附近抖动时导致的模式频繁切换
 *
 * 工作原理:
 * - 当置信度从高变低时，不立即返回 FAILURE
 * - 进入"观察期" (grace period)，在原地等待 1-2 秒
 * - 如果观察期内置信度回升，继续追逐模式
 * - 如果观察期内置信度保持低位，才切换到巡逻模式
 *
 * 状态机:
 *
 *   HIGH_CONFIDENCE (置信度 >= 0.4)
 *         ↓ (置信度下降到 0.4 以下)
 *   GRACE_PERIOD (观察期，等待 1-2 秒)
 *         ↓ (观察期结束 && 置信度仍低)
 *   LOW_CONFIDENCE (返回 FAILURE)
 *         ↓ (置信度回升到 0.4 以上)
 *   HIGH_CONFIDENCE (立即返回 SUCCESS)
 *
 * 使用场景:
 * 在 BT 中放在 IsTracking 节点之前，提供模式切换的迟滞效果
 */
class ConfidenceHysteresis : public BT::ConditionNode
{
public:
    enum class State : uint8_t {
        HIGH_CONFIDENCE = 0,  // 置信度高，正常追逐
        GRACE_PERIOD = 1,     // 观察期，等待确认
        LOW_CONFIDENCE = 2    // 置信度低，切换到巡逻
    };

    ConfidenceHysteresis(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<armor_interfaces::msg::Target>("target", "Current target message"),
            BT::InputPort<double>("upper_threshold", 0.4, "Upper threshold for entering grace period"),
            BT::InputPort<double>("lower_threshold", 0.3, "Lower threshold for grace period check"),
            BT::InputPort<double>("grace_period", 1.5, "Wait time before switching to patrol (seconds)"),
            BT::InputPort<bool>("enable_hold_position", true, "Hold last goal position during grace period"),
            BT::OutputPort<double>("current_confidence", "Output current confidence value"),
            BT::OutputPort<std::string>("hysteresis_state", "Current state: HIGH/GRACE/LOW")
        };
    }

    // 重置状态（例如检测到新敌人时）
    void reset();

    // 清除目标缓存（用于切换目标场景）
    void clearTargetCache();

private:
    State current_state_ = State::LOW_CONFIDENCE;
    State previous_state_ = State::LOW_CONFIDENCE;  // 用于检测状态迁移
    rclcpp::Time grace_period_start_;
    rclcpp::Time last_high_confidence_time_;

    // 上一帧的状态，用于检测 halt（节点状态从非 IDLE 变为 IDLE 说明被 halt 了）
    BT::NodeStatus previous_status_ = BT::NodeStatus::IDLE;

    // Parameters (cached)
    double upper_threshold_ = 0.4;
    double lower_threshold_ = 0.3;
    double grace_period_ = 1.5;
    bool enable_hold_position_ = true;

    // 最后一帧的目标（用于保持位置）
    armor_interfaces::msg::Target last_valid_target_;

    // Helper functions
    std::string stateToString(State state) const;
    bool isInGracePeriod(const rclcpp::Time &now) const;
    double getGracePeriodElapsed(const rclcpp::Time &now) const;

    // 检查是否应该从高置信度进入观察期
    bool shouldEnterGracePeriod(double confidence);

    // 检查观察期是否应该结束（切换到低置信度）
    bool shouldExitGracePeriod(const rclcpp::Time &now, double confidence);

    // 检查是否应该从低置信度直接回到高置信度
    bool shouldReturnToHighConfidence(double confidence);

    // 状态迁移辅助方法
    void transitionTo(State new_state);

    // 状态迁移日志（仅在状态改变时打印）
    void logStateTransition(State new_state, double confidence, const rclcpp::Time &now);

    // 检测并处理 halt（ConditionNode::halt() 是 final 的，无法 override）
    void checkAndHandleHalt();
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CONFIDENCE_HYSTERESIS_HPP_
