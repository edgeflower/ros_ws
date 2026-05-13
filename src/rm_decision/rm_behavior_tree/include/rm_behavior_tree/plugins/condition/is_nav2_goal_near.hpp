#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAV2_GOAL_NEAR_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAV2_GOAL_NEAR_HPP_

#include <behaviortree_cpp/basic_types.h>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rm_behavior_tree
{

class IsNav2GoalNear : public BT::ConditionNode
{
public:
  IsNav2GoalNear(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params);

  ~IsNav2GoalNear() override;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "导航目标位置"),
      BT::InputPort<std::string>("global_frame", "map", "全局坐标系"),
      BT::InputPort<std::string>("robot_frame", "chassis", "机器人坐标系"),
      BT::InputPort<double>(
        "distance_threshold", 0.5, "距离目标点小于该值时返回SUCCESS"),
      BT::InputPort<double>("tf_timeout", 0.2, "等待 TF 的超时时间（秒）"),
      BT::InputPort<std::string>(
        "feedback_topic", "/navigate_to_pose/_action/feedback", "Nav2 feedback topic"),
      BT::InputPort<double>("feedback_timeout", 0.5, "等待反馈的超时时间（秒）")
    };
  }

private:
  double last_distance_remaining_ = 9999.0;
  bool has_feedback_ = false;
  rclcpp::Time last_feedback_time_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<
    nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage
  >::SharedPtr feedback_sub_;
  std::thread spin_thread_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex feedback_mutex_;
  bool was_near_ = false;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAV2_GOAL_NEAR_HPP_
