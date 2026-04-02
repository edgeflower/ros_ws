#include "rm_behavior_tree/plugins/condition/confidence_hysteresis.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

namespace rm_behavior_tree
{

ConfidenceHysteresis::ConfidenceHysteresis(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config)
{
    // Get parameters with defaults
    getInput<double>("upper_threshold", upper_threshold_);
    getInput<double>("lower_threshold", lower_threshold_);
    getInput<double>("grace_period", grace_period_);
    getInput<bool>("enable_hold_position", enable_hold_position_);

    // Validate thresholds
    if (upper_threshold_ <= lower_threshold_) {
        upper_threshold_ = lower_threshold_ + 0.1;
        RCLCPP_WARN(rclcpp::get_logger("ConfidenceHysteresis"),
                    "Upper threshold <= lower threshold, adjusted to %.2f", upper_threshold_);
    }

    RCLCPP_INFO(rclcpp::get_logger("ConfidenceHysteresis"),
                "Initialized with thresholds: [%.2f, %.2f], grace_period: %.1fs",
                lower_threshold_, upper_threshold_, grace_period_);
}

BT::NodeStatus ConfidenceHysteresis::tick()
{
    // 检测 halt：如果上一帧状态不是 IDLE，现在变成了 IDLE，说明节点被 halt 了
    checkAndHandleHalt();

    // 支持 Groot2 在线调参（动态更新参数，开销极小）
    getInput("upper_threshold", upper_threshold_);
    getInput("lower_threshold", lower_threshold_);
    getInput("grace_period", grace_period_);
    getInput("enable_hold_position", enable_hold_position_);

    rclcpp::Time now = rclcpp::Clock().now();

    // Get current target
    auto target_result = getInput<armor_interfaces::msg::Target>("target");
    bool has_target = target_result.has_value();
    armor_interfaces::msg::Target target;
    if (has_target) {
        target = target_result.value();
    }

    double confidence = has_target ? target.confidence : 0.0;
    setOutput("current_confidence", confidence);

    // 状态机逻辑
    switch (current_state_) {
        case State::HIGH_CONFIDENCE:
            // 高置信度状态：检查是否应该进入观察期
            if (shouldEnterGracePeriod(confidence)) {
                transitionTo(State::GRACE_PERIOD);
                grace_period_start_ = now;

                // 保存最后有效目标用于保持位置
                if (enable_hold_position_ && has_target) {
                    last_valid_target_ = target;
                }

                logStateTransition(State::GRACE_PERIOD, confidence, now);
            }
            setOutput("hysteresis_state", stateToString(current_state_));
            previous_status_ = BT::NodeStatus::SUCCESS;
            return BT::NodeStatus::SUCCESS;

        case State::GRACE_PERIOD:
            // 观察期：检查是否应该回到高置信度或进入低置信度
            if (shouldReturnToHighConfidence(confidence)) {
                // 置信度回升，立即回到高置信度状态
                transitionTo(State::HIGH_CONFIDENCE);
                last_high_confidence_time_ = now;

                logStateTransition(State::HIGH_CONFIDENCE, confidence, now);
                setOutput("hysteresis_state", stateToString(current_state_));
                previous_status_ = BT::NodeStatus::SUCCESS;
                return BT::NodeStatus::SUCCESS;
            }

            if (shouldExitGracePeriod(now, confidence)) {
                // 观察期结束，置信度仍然低，切换到巡逻模式
                transitionTo(State::LOW_CONFIDENCE);

                double elapsed = getGracePeriodElapsed(now);
                RCLCPP_INFO(rclcpp::get_logger("ConfidenceHysteresis"),
                            "GRACE_PERIOD -> LOW (grace period ended: %.1fs, final confidence: %.2f)",
                            elapsed, confidence);

                setOutput("hysteresis_state", stateToString(current_state_));
                previous_status_ = BT::NodeStatus::SUCCESS;
                return BT::NodeStatus::SUCCESS;  // 状态计算器始终返回SUCCESS，由CheckHysteresisState判断
            }

            // 仍在观察期
            setOutput("hysteresis_state", stateToString(current_state_));
            previous_status_ = BT::NodeStatus::SUCCESS;
            return BT::NodeStatus::SUCCESS;

        case State::LOW_CONFIDENCE:
            // 低置信度状态：检查是否应该回到高置信度
            if (shouldReturnToHighConfidence(confidence)) {
                // 置信度回升，立即切换回追逐模式
                transitionTo(State::HIGH_CONFIDENCE);
                last_high_confidence_time_ = now;

                logStateTransition(State::HIGH_CONFIDENCE, confidence, now);
                setOutput("hysteresis_state", stateToString(current_state_));
                previous_status_ = BT::NodeStatus::SUCCESS;
                return BT::NodeStatus::SUCCESS;
            }

            setOutput("hysteresis_state", stateToString(current_state_));
            previous_status_ = BT::NodeStatus::SUCCESS;
            return BT::NodeStatus::SUCCESS;  // 状态计算器始终返回SUCCESS，由CheckHysteresisState判断
    }

    previous_status_ = BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::SUCCESS;  // Should not reach here, but default to SUCCESS
}

void ConfidenceHysteresis::checkAndHandleHalt()
{
    // 检测 halt：如果上一帧状态不是 IDLE，现在变成了 IDLE，说明节点被 halt 了
    if (previous_status_ != BT::NodeStatus::IDLE && status() == BT::NodeStatus::IDLE) {
        State old_state = current_state_;
        current_state_ = State::LOW_CONFIDENCE;
        previous_state_ = State::LOW_CONFIDENCE;
        grace_period_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_high_confidence_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

        if (old_state != State::LOW_CONFIDENCE) {
            RCLCPP_DEBUG(rclcpp::get_logger("ConfidenceHysteresis"),
                         "Node halted (status IDLE detected), state reset from %s to LOW",
                         stateToString(old_state).c_str());
        }
    }
}

void ConfidenceHysteresis::reset()
{
    current_state_ = State::LOW_CONFIDENCE;
    previous_state_ = State::LOW_CONFIDENCE;
    grace_period_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_high_confidence_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // 清除目标缓存，防止切换目标后旧坐标影响决策
    last_valid_target_ = armor_interfaces::msg::Target();

    RCLCPP_INFO(rclcpp::get_logger("ConfidenceHysteresis"), "State reset to LOW, target cache cleared");
}

void ConfidenceHysteresis::clearTargetCache()
{
    last_valid_target_ = armor_interfaces::msg::Target();
    RCLCPP_DEBUG(rclcpp::get_logger("ConfidenceHysteresis"), "Target cache cleared");
}

void ConfidenceHysteresis::transitionTo(State new_state)
{
    previous_state_ = current_state_;
    current_state_ = new_state;
}

void ConfidenceHysteresis::logStateTransition(State new_state, double confidence, const rclcpp::Time &now)
{
    const char* old_str = stateToString(previous_state_).c_str();
    const char* new_str = stateToString(new_state).c_str();

    switch (new_state) {
        case State::HIGH_CONFIDENCE:
            RCLCPP_INFO(rclcpp::get_logger("ConfidenceHysteresis"),
                        "%s -> HIGH (confidence recovered: %.2f)", old_str, confidence);
            break;

        case State::GRACE_PERIOD:
            RCLCPP_INFO(rclcpp::get_logger("ConfidenceHysteresis"),
                        "%s -> GRACE_PERIOD (confidence: %.2f <= %.2f)",
                        old_str, confidence, upper_threshold_);
            break;

        case State::LOW_CONFIDENCE:
            {
                double elapsed = getGracePeriodElapsed(now);
                RCLCPP_INFO(rclcpp::get_logger("ConfidenceHysteresis"),
                            "%s -> LOW (grace period ended: %.1fs, final confidence: %.2f)",
                            old_str, elapsed, confidence);
            }
            break;
    }
}

std::string ConfidenceHysteresis::stateToString(State state) const
{
    switch (state) {
        case State::HIGH_CONFIDENCE: return "HIGH";
        case State::GRACE_PERIOD: return "GRACE_PERIOD";
        case State::LOW_CONFIDENCE: return "LOW";
        default: return "UNKNOWN";
    }
}

bool ConfidenceHysteresis::isInGracePeriod(const rclcpp::Time &now) const
{
    return current_state_ == State::GRACE_PERIOD;
}

double ConfidenceHysteresis::getGracePeriodElapsed(const rclcpp::Time &now) const
{
    if (grace_period_start_.nanoseconds() == 0) {
        return 0.0;
    }
    return (now - grace_period_start_).seconds();
}

bool ConfidenceHysteresis::shouldEnterGracePeriod(double confidence)
{
    // 从高置信度进入观察期的条件：置信度低于上阈值
    return confidence < upper_threshold_;
}

bool ConfidenceHysteresis::shouldExitGracePeriod(const rclcpp::Time &now, double confidence)
{
    // 从观察期切换到低置信度的条件：
    // 1. 观察期时间已满
    // 2. 置信度没有回到 HIGH（避免"中间地带"卡死）
    double elapsed = getGracePeriodElapsed(now);
    return (elapsed >= grace_period_) && !shouldReturnToHighConfidence(confidence);
}

bool ConfidenceHysteresis::shouldReturnToHighConfidence(double confidence)
{
    // 从任何状态回到高置信度的条件：置信度高于上阈值
    // 这提供了即时响应（不用担心抖动，因为我们刚从高置信度掉下来）
    return confidence >= upper_threshold_;
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::ConfidenceHysteresis>("ConfidenceHysteresis");
}
