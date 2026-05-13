#include "rm_behavior_tree/plugins/condition/is_nav2_goal_near.hpp"

#include <algorithm>
#include <cmath>

#include "behaviortree_ros2/plugins.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rm_behavior_tree
{

IsNav2GoalNear::IsNav2GoalNear(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & /* params */)
: BT::ConditionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("is_nav2_goal_near");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

  std::string feedback_topic = "/navigate_to_pose/_action/feedback";
  getInput("feedback_topic", feedback_topic);

  feedback_sub_ = node_->create_subscription<
    nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
      feedback_topic,
      rclcpp::SystemDefaultsQoS(),
      [this](
        nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        last_distance_remaining_ = msg->feedback.distance_remaining;
        last_feedback_time_ = node_->now();
        has_feedback_ = true;
      });

  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() {
    executor_.spin();
  });
}

IsNav2GoalNear::~IsNav2GoalNear()
{
  executor_.cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  executor_.remove_node(node_);
}

BT::NodeStatus IsNav2GoalNear::tick()
{
  double distance_threshold = 0.5;
  getInput("distance_threshold", distance_threshold);

  geometry_msgs::msg::PoseStamped goal_pose;
  const auto goal_res = getInput("goal_pose", goal_pose);
  if (goal_res) {
    std::string global_frame = "map";
    std::string robot_frame = "chassis";
    double tf_timeout = 0.2;
    getInput("global_frame", global_frame);
    getInput("robot_frame", robot_frame);
    getInput("tf_timeout", tf_timeout);

    if (goal_pose.header.frame_id.empty()) {
      goal_pose.header.frame_id = global_frame;
    }

    try {
      geometry_msgs::msg::PoseStamped goal_in_global = goal_pose;
      const auto timeout = tf2::durationFromSec(std::max(0.0, tf_timeout));
      if (goal_pose.header.frame_id != global_frame) {
        goal_in_global = tf_buffer_->transform(goal_pose, global_frame, timeout);
      }

      const auto robot_tf = tf_buffer_->lookupTransform(
        global_frame, robot_frame, tf2::TimePointZero, timeout);

      const double dx =
        goal_in_global.pose.position.x - robot_tf.transform.translation.x;
      const double dy =
        goal_in_global.pose.position.y - robot_tf.transform.translation.y;
      const double distance = std::hypot(dx, dy);

      if (distance <= distance_threshold) {
        if (!was_near_) {
          RCLCPP_INFO(
            node_->get_logger(),
            "Nav2 goal near by TF: distance=%.3f, threshold=%.3f, global_frame=%s, robot_frame=%s",
            distance, distance_threshold, global_frame.c_str(), robot_frame.c_str());
          was_near_ = true;
        }
        return BT::NodeStatus::SUCCESS;
      }

      was_near_ = false;
      return BT::NodeStatus::FAILURE;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "IsNav2GoalNear TF lookup failed: %s",
        ex.what());
      was_near_ = false;
      return BT::NodeStatus::FAILURE;
    }
  }

  double feedback_timeout = 0.5;
  getInput("feedback_timeout", feedback_timeout);

  double distance_remaining = 9999.0;
  bool has_feedback = false;
  rclcpp::Time last_feedback_time;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    distance_remaining = last_distance_remaining_;
    has_feedback = has_feedback_;
    last_feedback_time = last_feedback_time_;
  }

  if (!has_feedback) {
    was_near_ = false;
    return BT::NodeStatus::FAILURE;
  }

  const double feedback_age = (node_->now() - last_feedback_time).seconds();
  if (feedback_timeout > 0.0 && feedback_age > feedback_timeout) {
    was_near_ = false;
    return BT::NodeStatus::FAILURE;
  }

  if (distance_remaining < distance_threshold) {
    if (!was_near_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Nav2 goal near detected: distance_remaining=%.3f, threshold=%.3f",
        distance_remaining, distance_threshold);
      was_near_ = true;
    }
    return BT::NodeStatus::SUCCESS;
  }

  was_near_ = false;
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(
  rm_behavior_tree::IsNav2GoalNear,
  "IsNav2GoalNear");
