#include "rm_behavior_tree/plugins/action/robot_control.hpp"
#include <rclcpp/logging.hpp>

namespace rm_behavior_tree
{

RobotControlAction::RobotControlAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<rm_decision_interfaces::msg::RobotControl>(name, conf, params)
{
}

bool RobotControlAction::setMessage(rm_decision_interfaces::msg::RobotControl & msg)
{
  getInput("stop_gimbal_scan", msg.stop_gimbal_scan);
  getInput("chassis_spin_vel", msg.chassis_spin_vel);
  getInput("gimbal_big_yaw_vel",msg.gimbal_big_yaw_vel);

//   std::cout << "stop_gimbal_scan: " << msg.stop_gimbal_scan << '\n';
//   std::cout << "chassis_spin_vel: " << msg.chassis_spin_vel << '\n';

  //RCLCPP_INFO(node_->get_logger(),"stop_gimbal_scan: %d", msg.stop_gimbal_scan);
  //RCLCPP_INFO(node_->get_logger(),"chassis_spin_vel: %f", msg.chassis_spin_vel);
  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RobotControlAction, "RobotControl");