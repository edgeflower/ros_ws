#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_OUTPOST_OK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_OUTPOST_OK_HPP_

#include <behaviortree_cpp/basic_types.h>
#include <rm_decision_interfaces/msg/detail/all_robot_hp__struct.hpp>
#include <rm_decision_interfaces/msg/detail/robot_status__struct.hpp>
#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include "rm_decision_interfaces/msg/all_robot_hp.hpp"

namespace rm_behavior_tree
{
enum  TeamColor{
    RED = 0,
    BLUE = 1
};

class IsOutPostOKCondition : public BT::SimpleConditionNode
{

public:
    IsOutPostOKCondition(const std::string & name, const BT::NodeConfig & config);

    // BT::NodeStatus checkGameStart(BT::TreeNode & self_node)
    BT::NodeStatus checkRobotStatus();

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<rm_decision_interfaces::msg::AllRobotHP>("all_robot_hp"),
            BT::InputPort<rm_decision_interfaces::msg::RobotStatus>("robot_status"),
            BT::InputPort<int>("hp_threshold")
        };
    }

};
} // namespace rm_behavior_tree

#endif //RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_OUTPOST_OK_HPP_