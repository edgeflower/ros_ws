#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GAME_TIME_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GAME_TIME_HPP_

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <rm_decision_interfaces/msg/detail/game_status__struct.hpp>
#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/game_status.hpp"

namespace rm_behavior_tree
{


/**
* @brief condition 节点，用于判断比赛阶段与剩余时间是否符合预期
* {0, "未开始比赛"}, {1, “准备阶段”}, {2, "十五秒裁判系统自检阶段"},
* {3, "五秒倒计时"}, {4, "比赛开始"}, {5, "比赛结束中"}
* @param[in] message 比赛阶段话题
* @param[in] game_prosess 期望的比赛阶段
* @param[in] lower_remain_time 期望的时间下限
* @param[in] higher_remain_time 期望的剩余时间上限
*/
class IsGameTimeCondition : public BT::SimpleConditionNode
{
public:
    IsGameTimeCondition(const std::string  &name, const BT::NodeConfig & config);

    BT::NodeStatus CheckGameStart();

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<rm_decision_interfaces::msg::GameStatus>("message"),
            BT::InputPort<int>("game_progress"), BT::InputPort<int>("lower_remain_time"),
            BT::InputPort<int>("higher_remain_time")};
    }
};
} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GAME_TIME_HPP_