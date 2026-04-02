#include "rm_behavior_tree/plugins/action/sub_cmd_vel.hpp"

namespace rm_behavior_tree
{

SubCmdVelAction::SubCmdVelAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicSubNode<geometry_msgs::msg::Twist>(name, conf, params)
{
}

BT::NodeStatus SubCmdVelAction::onTick(
  const std::shared_ptr<geometry_msgs::msg::Twist> & last_msg)
{
  if (last_msg) {
    setOutput("cmd_vel", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubCmdVelAction, "SubCmdVel");
