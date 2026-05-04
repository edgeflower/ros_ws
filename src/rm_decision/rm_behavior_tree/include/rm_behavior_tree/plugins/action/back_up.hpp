#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__BACK_UP_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__BACK_UP_HPP_

#include <behaviortree_cpp/basic_types.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "behaviortree_cpp/action_node.h"
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace rm_behavior_tree
{

/**
 * @brief 后退动作节点
 *        发布 cmd_vel 让机器人后退指定距离
 * @param[in] backup_dist 后退距离 (m)
 * @param[in] backup_speed 后退速度 (m/s)
 * @param[in] time_allowance 最大执行时间 (s)
 * @param[in] global_frame 全局坐标系 (默认: odom)
 * @param[in] robot_base_frame 机器人坐标系 (默认: base_footprint)
 */
class BackUpAction : public BT::StatefulActionNode, public rclcpp::Node
{
public:
  BackUpAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("backup_dist", 0.3, "后退距离 (m)"),
      BT::InputPort<double>("backup_speed", 0.15, "后退速度 (m/s)"),
      BT::InputPort<double>("time_allowance", 5.0, "最大执行时间 (s)"),
      BT::InputPort<std::string>("global_frame", "odom", "全局坐标系"),
      BT::InputPort<std::string>("robot_base_frame", "base_footprint", "机器人坐标系"),
    };
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  bool getCurrentPose(geometry_msgs::msg::PoseStamped & pose);

  void stopRobot();

  void publishVelocity(double linear_x);

  // ROS 接口
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 状态
  bool started_{false};
  rclcpp::Time start_time_;
  geometry_msgs::msg::PoseStamped start_pose_;

  // 参数
  double backup_dist_{0.3};
  double backup_speed_{0.15};
  double time_allowance_{5.0};
  std::string global_frame_{"odom"};
  std::string robot_base_frame_{"base_footprint"};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__BACK_UP_HPP_
