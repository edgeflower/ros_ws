#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ARMORS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ARMORS_HPP_

#include <armor_interfaces/msg/detail/armors__struct.hpp>
#include <armor_interfaces/msg/detail/target__struct.hpp>
#include <behaviortree_cpp/basic_types.h>
//#include "armor_interfaces/msg/armors.hpp"
#include <armor_interfaces/msg/target.hpp>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"


namespace rm_behavior_tree
{

class SubArmorsAction : public BT::RosTopicSubNode<armor_interfaces::msg::Target>
{
    public:
    SubArmorsAction(
        const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);
    
    static BT::PortsList providedPorts(){
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::OutputPort<armor_interfaces::msg::Target>("armors")};

    }

    BT::NodeStatus onTick(
        const std::shared_ptr<armor_interfaces::msg::Target> &last_msg) override;


}; 
} // namespace rm_behavior_tree


#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ARMORS_HPP_