#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_NAV2_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_NAV2_GOAL_HPP_

#include <iomanip>
#include <iostream>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace rm_behavior_tree
{

class SendNav2Goal : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  SendNav2Goal(
      const std::string & name, const BT::NodeConfiguration & conf,
      const BT::RosNodeParams & params)
      : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "导航目标位置"),
    };
  }

  bool setGoal(Goal & goal) override
  {
    auto res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
    if (!res) {
      throw BT::RuntimeError("读取端口[goal_pose]时出错:", res.error());
    }

    goal.pose = *res;
    goal.pose.header.stamp = rclcpp::Clock().now();

    // clang-format off
    std::cout << "Goal_pose: [ "
        << std::fixed << std::setprecision(1)
        << goal.pose.pose.position.x << ", "
        << goal.pose.pose.position.y << ", "
        << goal.pose.pose.position.z << ", " << " ]\n";
    // clang-format on

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
    std::cout << "Distance remaining: " << feedback->distance_remaining << '\n';
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(
        node_->get_logger(), "SendGoalAction failed with error code: %d", error);
    return BT::NodeStatus::FAILURE;
  }
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_NAV2_GOAL_HPP_
