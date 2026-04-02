#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_HPP_

#include "behaviortree_cpp/action_node.h"
#include <chrono>

namespace rm_behavior_tree
{

/**
 * @brief Wait Action Node
 *
 * 等待指定的时间（秒）
 *
 * 使用示例:
 *   <Wait duration="3.0"/>  <!-- 等待 3 秒 -->
 *
 * 注意：这是 SleepNode 的包装器，使用 duration（秒）而不是 msec（毫秒）
 */
class WaitAction : public BT::StatefulActionNode
{
public:
    WaitAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), started_(false)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("duration", "Wait duration in seconds")
        };
    }

    BT::NodeStatus onStart() override
    {
        double duration_sec = 1.0;  // Default 1 second
        getInput<double>("duration", duration_sec);

        // Convert to milliseconds
        duration_ms_ = static_cast<uint64_t>(duration_sec * 1000.0);

        start_time_ = std::chrono::steady_clock::now();
        started_ = true;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!started_) {
            return BT::NodeStatus::FAILURE;
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();

        if (elapsed >= static_cast<int64_t>(duration_ms_)) {
            started_ = false;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        started_ = false;
    }

private:
    std::chrono::steady_clock::time_point start_time_;
    uint64_t duration_ms_;
    bool started_;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_HPP_
