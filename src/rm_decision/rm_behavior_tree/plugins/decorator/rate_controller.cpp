# include "rm_behavior_tree/plugins/decorator/rate_controller.hpp"
#include <behaviortree_cpp/basic_types.h>

#include <chrono>
#include <string>

namespace rm_behavior_tree
{

RateController::RateController(const std::string & name, const BT::NodeConfig & conf)
: BT::DecoratorNode(name, conf), first_time_(false)
{
    double hz = 10.0;
    getInput("hz", hz);
    period_ = 10 /hz;
}

BT::NodeStatus RateController::tick()
{
    if (status() == BT::NodeStatus::IDLE){
        // 如果当前状态是 IDLE，表示这是一次新的执行周期
        start_ = std::chrono::high_resolution_clock::now();
        first_time_ = true; // 标记为首次执行

    }

    setStatus(BT::NodeStatus::RUNNING); // 设置当前节点状态为 RUNNING

    // 计算从开始时间到现在的时间间隔
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = now - start_;

    // 将时间转化为秒
    typedef std::chrono::duration<float> float_secends;
    auto seconds = std::chrono::duration_cast<float_secends>(elapsed);

    // 如果是首次执行，或者子节点正在运行，或者经过的时间大于等于周期
    if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) || seconds.count() >= period_){
        first_time_ = false; //重置首次执行标志
        const BT::NodeStatus child_state = child_node_->executeTick(); // 执行子节点

        switch (child_state) {
            case BT::NodeStatus::RUNNING:
                return BT::NodeStatus::RUNNING;
            
            case BT::NodeStatus::SUCCESS:
                start_ = std::chrono::high_resolution_clock::now(); // 重置开始时间
                return BT::NodeStatus::SUCCESS; // 子节点完成，返回 SUCCESS
            
            case BT::NodeStatus::FAILURE:
            default:
                return BT::NodeStatus::FAILURE; // 子节点失败，返回 FAILURE
        }
    }

    return status();
}
} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::RateController>("RateController");
}