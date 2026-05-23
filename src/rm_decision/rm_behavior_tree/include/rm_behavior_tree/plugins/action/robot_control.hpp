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
        return{ BT::InputPort<bool>("stop_gimbal_scan"),       // 没用到
                BT::InputPort<float>("chassis_spin_vel"),      // 底盘转速
                BT::InputPort<float>("gimbal_big_yaw_vel"),    // 大云台转速
                BT::InputPort<float>("gimbal_big_yaw_angle"),  // 大云台角度
                BT::InputPort<bool>("follow_gimbal_big"),      // 是否启动底盘跟随大云台
                BT::InputPort<float>("chassis_vel_multi"),     // 底盘速率倍增
                BT::InputPort<bool>("stop_chassis_vel_multi"), // 是否停止底盘速率倍增
                BT::InputPort<bool>("track_status"),           // 是否启动履带
                BT::InputPort<bool>("perception_status"),      // 大云台是否跟随全向感知 是否屏蔽全向感知
                BT::InputPort<bool>("start_gimbal_big_spin")   // 是否启动大云台旋转
      };
     }
private:
 bool chassis_vel_multi_;
};
} // namespace rm_behavior_tree

#endif //RM_BEHAVIOR_TREE__PLUGINS__ACTION__ROBOT_CONTROL_HPP_


