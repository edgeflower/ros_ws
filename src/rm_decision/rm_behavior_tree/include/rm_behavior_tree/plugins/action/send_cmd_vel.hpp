#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_CMD_VEL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_CMD_VEL_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>

#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{

class SendCmdVelAction : public BT::StatefulActionNode, public rclcpp::Node
{
public:
  SendCmdVelAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("linear_x", 0.0, "线速度 x (m/s)"),
      BT::InputPort<double>("linear_y", 0.0, "线速度 y (m/s)"),
      BT::InputPort<double>("angular_z", 0.0, "角速度 z (rad/s)"),
      BT::InputPort<std::string>("topic_name", "/cmd_vel_chassis", "cmd_vel 话题名"),
      BT::InputPort<double>("frequency", 10.0, "发布频率 (Hz)"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void publishCmdVel();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double linear_x_{0.0};
  double linear_y_{0.0};
  double angular_z_{0.0};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_CMD_VEL_HPP_
