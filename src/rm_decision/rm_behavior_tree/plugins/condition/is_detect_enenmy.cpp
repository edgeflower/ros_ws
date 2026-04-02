#include "rm_behavior_tree/plugins/condition/is_detect_enemy.hpp"

namespace rm_behavior_tree{
IsDetectEnemyAction::IsDetectEnemyAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode( name,std::bind(&IsDetectEnemyAction::detectEnemyStatus ,this),config){ 

}

BT::NodeStatus IsDetectEnemyAction::detectEnemyStatus(){
    auto msg = getInput<armor_interfaces::msg::Target>("message");
    if (!msg) {
        std::cerr << "Missing required input [message]" << '\n';
        return BT::NodeStatus::FAILURE;
    }

    if(msg->tracking_status == 2 || msg->tracking_status == 3){
        std::cout<< "检测到敌方机器人" << '\n';
        return BT::NodeStatus::SUCCESS;
    } else {
        std::cerr << "未检测到机器人" << '\n';
        return BT::NodeStatus::FAILURE;
    }
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory){
    factory.registerNodeType<rm_behavior_tree::IsDetectEnemyAction>("IsDetectEnemy");
}