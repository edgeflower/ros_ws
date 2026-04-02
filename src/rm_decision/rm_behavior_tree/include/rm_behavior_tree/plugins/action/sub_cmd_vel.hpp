#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_CMD_VEL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_CMD_VEL_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace rm_behavior_tree
{

/**
 * @brief Action node that subscribes to cmd_vel topic and outputs to blackboard
 * @param[in] topic_name cmd_vel topic name (default: /cmd_vel_chassis)
 * @param[out] cmd_vel geometry_msgs::msg::Twist velocity command
 */
class SubCmdVelAction : public BT::RosTopicSubNode<geometry_msgs::msg::Twist>
{
public:
  SubCmdVelAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    BT::PortsList custom_ports = {
      BT::OutputPort<geometry_msgs::msg::Twist>("cmd_vel")
    };
    return BT::RosTopicSubNode<geometry_msgs::msg::Twist>::providedBasicPorts(custom_ports);
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<geometry_msgs::msg::Twist> & last_msg) override;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_CMD_VEL_HPP_
