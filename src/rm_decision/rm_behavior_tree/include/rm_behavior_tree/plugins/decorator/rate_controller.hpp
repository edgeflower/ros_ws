//! 行为树 发布频率
#ifndef RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <chrono>
#include <string>

#include "behaviortree_cpp/decorator_node.h"
namespace rm_behavior_tree
{
/**
*  @brief A BT::DecoratorNode that ticks its child at a specified rate.
*
* 该装饰器节点控制其子节点以指定频率 （hz）被调用。
* 它使用高精度时钟换算时间间隔，确保子节点按预期速率执行。
*/
class RateController : public BT::DecoratorNode
{
public:
/**
* @brief 构造函数。
* 初始化 RateController 节点，并设置节点名称与配置信息。
* 
* @param name 节点名称，用于 XML 配置中的标签标识
* @param conf 节点配置信息，包含行为树所需的参数
*/

RateController(const std::string & name, const BT::NodeConfig &conf);

/**
* @brief 提供节点所需的端口列表
* 注册一个输入端口 "hz"，用于配置执行频率
* 默认值 10.0 hz。
* 
* @return BT::PortsList 包含节点特定端口的列表。
* 
*/
static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("hz", 10.0, "rate")};
}

private:
    /**
    * @brief 行为树核心执行函数
    *
    * 每当行为树调用该节点时， tick() 函数被调用。
    * 该函数首先判断是否为第一次调用，如是则直接执行子节点，
    * 否则通过高精度计时计算自上次执行以来的时间是否满足期望的时间间隔 （period）。
    * 如果时间充足，则调用子节点的 tick(), 否则延迟调用
    *
    * @return BT::NodeStatus 当前节点执行后的状态
    */
    BT::NodeStatus tick() override;

    /// @brief 上次调用 tick() 之间的时间间隔， 使用高精度计时器
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;

    /// @brief 两次连续调用 tick() 之间的时间间隔（单位：秒），通常计算方式为 1 / hz
    double period_;

    /// @brief 标记是否为第一次调用 tick(), 第一次调用时不进行时间判断

    bool first_time_;

};
} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_