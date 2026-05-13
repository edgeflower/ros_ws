#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_NAV2_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_NAV2_GOAL_HPP_

#include <behaviortree_cpp/basic_types.h>
#include <iomanip>
#include <iostream>
#include <rclcpp/logging.hpp>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace rm_behavior_tree
{

class SendNav2Goal : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  static BT::RosNodeParams configureParams(BT::RosNodeParams params)
  {
    params.server_timeout = std::chrono::milliseconds(30000);
    params.wait_for_server_timeout = std::chrono::milliseconds(5000);
    return params;
  }

  SendNav2Goal(
      const std::string & name, const BT::NodeConfiguration & conf,
      BT::RosNodeParams params)
      : RosActionNode<nav2_msgs::action::NavigateToPose>(
            name, conf, configureParams(std::move(params)))
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "导航目标位置"),
        BT::InputPort<double>("min_distance", 0.5, "距离目标点多近返回成功")
    });
  }

  bool setGoal(Goal & goal) override
  {
    auto res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
    if (!res) {
      throw BT::RuntimeError("读取端口[goal_pose]时出错:", res.error());
    }

    goal.pose = *res;
    goal.pose.header.stamp = node_->now();

    if (goal.pose.header.frame_id.empty()) {
      goal.pose.header.frame_id = "map";
    }

    RCLCPP_INFO(node_->get_logger(),
                "SendNav2Goal: frame_id=%s, x=%.2f, y=%.2f",
                goal.pose.header.frame_id.c_str(),
                goal.pose.pose.position.x,
                goal.pose.pose.position.y);

    return true;
  }

  void onHalt() override
  {
    RCLCPP_INFO(node_->get_logger(), "SendGoalAction has been halted.");

    try {
      BT::RosActionNode<nav2_msgs::action::NavigateToPose>::onHalt();
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError & e) {
      RCLCPP_WARN(
          node_->get_logger(), "Ignored UnknownGoalHandleError during halt: %s",
          e.what());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "Exception caught during halt: %s", e.what());
    }
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    switch (wr.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(), "Success!!!");
        return BT::NodeStatus::SUCCESS;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(node_->get_logger(), "Goal was aborted");
        return BT::NodeStatus::FAILURE;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(node_->get_logger(), "Goal was canceled");
        return BT::NodeStatus::FAILURE;

      default:
        RCLCPP_INFO(node_->get_logger(), "Unknown result code");
        return BT::NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFeedback(
      const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>
          feedback) override
  {
    RCLCPP_INFO(node_->get_logger(), "SendNav2Goal feedback: distance_remaining = %.2f",
                feedback->distance_remaining);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(
        node_->get_logger(), "SendNav2Goal failed: %s (%d)",
        BT::toStr(error), static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
  }

private:
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_NAV2_GOAL_HPP_
