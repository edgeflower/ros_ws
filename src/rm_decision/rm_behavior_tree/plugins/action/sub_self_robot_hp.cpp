#include "rm_behavior_tree//plugins/action/sub_self_robot_hp.hpp"
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <rclcpp/logging.hpp>
#include <rm_decision_interfaces/msg/detail/self_robot_hp__struct.hpp>
 

namespace rm_behavior_tree
{

SubSelfRobotHPAction::SubSelfRobotHPAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::SelfRobotHP> (name, conf, params)
{

}
BT::NodeStatus SubSelfRobotHPAction::onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::SelfRobotHP> & last_msg)
{
    if (last_msg)
    {
        RCLCPP_DEBUG(
            logger(), "[%s] new message, hero_hp: %d", name().c_str(),
            last_msg->hero_hp);
        setOutput("robot_hp",*last_msg );
    }
    return BT::NodeStatus::SUCCESS;
}
} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubSelfRobotHPAction, "SubSelfRobotHP");
