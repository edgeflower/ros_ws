#include "rm_behavior_tree/plugins/condition/is_game_time.hpp"

namespace rm_behavior_tree
{
    IsGameTimeCondition::IsGameTimeCondition(const std::string & name, const BT::NodeConfig &config)
    : BT::SimpleConditionNode(name, std::bind(&IsGameTimeCondition::CheckGameStart, this),config){

    }

BT::NodeStatus IsGameTimeCondition::CheckGameStart()
{
    int game_progress, lower_remain_time, higher_remain_time;
    auto msg = getInput<rm_decision_interfaces::msg::GameStatus>("message");
    getInput("game_progress", game_progress);
    getInput("lower_remain_time", lower_remain_time);
    getInput("higher_remain_time", higher_remain_time);

    if(!msg) {
        // std::cout << "Missing required input [game_status]" << '\n';
        return BT::NodeStatus::FAILURE;
    }

    if (
        msg->game_progress  == game_progress && msg->stage_remain_time >= lower_remain_time &&
        msg->stage_remain_time <= higher_remain_time)
        {return BT::NodeStatus::SUCCESS;}
    else {
        return BT::NodeStatus::FAILURE;
    }
}
} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory){
    factory.registerNodeType<rm_behavior_tree::IsGameTimeCondition>("IsGameTime");
}