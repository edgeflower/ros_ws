#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_HYSTERESIS_STATE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_HYSTERESIS_STATE_HPP_

#include "behaviortree_cpp/condition_node.h"
#include <string>

namespace rm_behavior_tree
{

/**
 * @brief Check Hysteresis State Condition Node
 *
 * 检查迟滞状态是否匹配预期状态
 *
 * 配合 ConfidenceHysteresis 使用，用于在行为树中根据状态分支
 *
 * 状态值:
 * - "HIGH": 高置信度，正常追踪
 * - "GRACE_PERIOD": 观察期，等待确认
 * - "LOW": 低置信度，切换到巡逻
 *
 * 使用示例:
 *   <CheckHysteresisState state="{hysteresis_state}" expected_state="HIGH"/>
 */
class CheckHysteresisState : public BT::SimpleConditionNode
{
public:
    CheckHysteresisState(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SimpleConditionNode(name, std::bind(&CheckHysteresisState::checkState, this), config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("state", "Current hysteresis state (HIGH/GRACE_PERIOD/LOW)"),
            BT::InputPort<std::string>("expected_state", "HIGH", "Expected state to match")
        };
    }

    BT::NodeStatus checkState()
    {
        std::string state;
        if (!getInput<std::string>("state", state)) {
            // No state input, return FAILURE
            return BT::NodeStatus::FAILURE;
        }

        std::string expected_state;
        if (!getInput<std::string>("expected_state", expected_state)) {
            expected_state = "HIGH";  // Default
        }

        // Case-insensitive comparison
        bool matches = (state.size() == expected_state.size());
        if (matches) {
            for (size_t i = 0; i < state.size(); ++i) {
                if (std::tolower(state[i]) != std::tolower(expected_state[i])) {
                    matches = false;
                    break;
                }
            }
        }

        return matches ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_HYSTERESIS_STATE_HPP_
