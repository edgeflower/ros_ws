#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DETECTED_ENENMY_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DETECTED_ENENMY_HPP_

#include <armor_interfaces/msg/armors.hpp>
#include <armor_interfaces/msg/target.hpp>
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree{
/**
* @brief condition 节点，用于判断实业内是否存在敌人
* @param[in] message 识别模块的检测结果序列
*/
class IsDetectEnemyAction : public BT::SimpleConditionNode {
public:
    IsDetectEnemyAction(const std::string & name, const BT::NodeConfig & config);
    
    BT::NodeStatus detectEnemyStatus();
    
    static BT::PortsList providedPorts(){
        return {
            BT::InputPort<armor_interfaces::msg::Target>("message")};
    }
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DETECTED_ENENMY_HPP_