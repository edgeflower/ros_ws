#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__ROBOT_CONTROL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__ROBOT_CONTROL_HPP_

#include "rm_decision_interfaces/msg/robot_control.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include <behaviortree_cpp/basic_types.h>

namespace rm_behavior_tree{

class RobotControlAction : public BT::RosTopicPubNode<rm_decision_interfaces::msg::RobotControl>
{
public:
  RobotControlAction(
    const std::string & name,const BT::NodeConfig & conf, const BT::RosNodeParams & params);
  bool setMessage(rm_decision_interfaces::msg::RobotControl & msg) override ;

  static BT::PortsList providedPorts()
    {
        return{BT::InputPort<bool>("stop_gimbal_scan"),
               BT::InputPort<float>("chassis_spin_vel"),
               BT::InputPort<float>("gimbal_big_yaw_vel"),
               BT::InputPort<float>("gimbal_big_yaw_angle"),
              BT::InputPort<float>("follow_gimbal_big"),
            BT::InputPort<float>("chassis_vel_multi"),
          BT::InputPort<bool>("stop_chassis_vel_multi")};
     }
private:
 bool chassis_vel_multi_;
};
} // namespace rm_behavior_tree

#endif //RM_BEHAVIOR_TREE__PLUGINS__ACTION__ROBOT_CONTROL_HPP_


