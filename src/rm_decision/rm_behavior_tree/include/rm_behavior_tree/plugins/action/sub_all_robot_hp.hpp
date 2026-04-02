#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ALL_ROBOT_HP_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ALL_ROBOT_HP_HPP_


#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/all_robot_hp.hpp"

namespace rm_behavior_tree
{
class SubAllRobotHPAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::AllRobotHP>
{
public:
SubAllRobotHPAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);
    static BT::PortsList providedPorts()
    {
        return {
            // 在使用RosTopicSubNode时InputPort的topic_name即输入端口名字只能为topic_name,在xml中可以把任意名字赋给端口
            BT::InputPort<std::string>("topic_name"),
            BT::OutputPort<rm_decision_interfaces::msg::AllRobotHP>("robot_hp")
        };
    }

    BT::NodeStatus onTick(
        const std::shared_ptr<rm_decision_interfaces::msg::AllRobotHP> & last_msg) override;

};
} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ALL_ROBOT_HP_HPP_