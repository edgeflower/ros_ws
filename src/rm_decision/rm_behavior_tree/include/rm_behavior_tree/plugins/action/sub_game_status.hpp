#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GAME_STATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GAME_STATUS_HPP_

#include <memory>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_behavior_tree/plugins/action/sub_all_robot_hp.hpp"
#include "rm_decision_interfaces/msg/game_status.hpp"

namespace rm_behavior_tree
{
class SubGameStatusAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::GameStatus>
{
public:
    SubGameStatusAction(
        const std::string & name , const BT::NodeConfig & conf , const BT::RosNodeParams & params);
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::OutputPort<rm_decision_interfaces::msg::GameStatus>("game_status")};
        
    }
    BT::NodeStatus onTick(
            const std::shared_ptr<rm_decision_interfaces::msg::GameStatus> & last_msg) override;
};
}

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GAME_STATUS_HPP_