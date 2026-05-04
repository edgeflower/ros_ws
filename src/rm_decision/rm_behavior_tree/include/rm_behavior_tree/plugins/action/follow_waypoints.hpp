#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_WAYPOINTS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_WAYPOINTS_HPP_

#include <sstream>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

#include "rm_behavior_tree/plugins/action/waypoint_utils.hpp"

namespace rm_behavior_tree
{

class FollowWaypointsAction
    : public BT::RosActionNode<nav2_msgs::action::FollowWaypoints>
{
public:
  FollowWaypointsAction(
      const std::string & name, const BT::NodeConfiguration & conf,
      const BT::RosNodeParams & params)
      : RosActionNode<nav2_msgs::action::FollowWaypoints>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("waypoint_file", "航点 CSV 文件的绝对路径"),
        BT::InputPort<std::string>("frame_id", "map", "坐标系 ID，默认为 map"),
        BT::OutputPort<int>("current_waypoint", "当前正在前往的航点索引"),
    });
  }

  bool setGoal(Goal & goal) override
  {
    auto file_res = getInput<std::string>("waypoint_file");
    if (!file_res) {
      RCLCPP_ERROR(
          node_->get_logger(), "读取端口 [waypoint_file] 时出错: %s",
          file_res.error().c_str());
      return false;
    }
    const std::string & waypoint_file = file_res.value();

    auto frame_res = getInput<std::string>("frame_id");
    const std::string & frame_id =
        (frame_res && !frame_res.value().empty()) ? frame_res.value() : "map";

    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    if (!loadWaypointsFromCSV(waypoint_file, waypoints, frame_id)) {
      RCLCPP_ERROR(
          node_->get_logger(), "无法从文件加载航点: %s", waypoint_file.c_str());
      return false;
    }

    if (waypoints.empty()) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "FollowWaypointsAction: 航点列表为空，取消发送 goal");
      return false;
    }

    rclcpp::Time now = rclcpp::Clock().now();
    for (auto & wp : waypoints) {
      if (wp.header.frame_id.empty()) wp.header.frame_id = frame_id;
      wp.header.stamp = now;
    }

    goal.poses = waypoints;
    RCLCPP_INFO(
        node_->get_logger(), "FollowWaypointsAction: 已加载 %lu 个航点，准备下发",
        static_cast<unsigned long>(waypoints.size()));
    for (size_t i = 0; i < waypoints.size(); ++i) {
      RCLCPP_INFO(
          node_->get_logger(), "  航点[%lu]: [%.2f, %.2f]",
          static_cast<unsigned long>(i), waypoints[i].pose.position.x,
          waypoints[i].pose.position.y);
    }
    return true;
  }

  void onHalt() override
  {
    RCLCPP_WARN(node_->get_logger(), "FollowWaypointsAction: 多点导航已被中断");
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    RCLCPP_INFO(node_->get_logger(), "FollowWaypointsAction: 所有航点导航完成!");
    if (wr.result && !wr.result->missed_waypoints.empty()) {
      RCLCPP_WARN(
          node_->get_logger(), "  有 %lu 个航点未到达",
          static_cast<unsigned long>(wr.result->missed_waypoints.size()));
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "  所有航点均已成功到达");
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFeedback(
      const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback>
          feedback) override
  {
    RCLCPP_DEBUG(
        node_->get_logger(), "FollowWaypointsAction: 正在前往航点 %u",
        static_cast<unsigned>(feedback->current_waypoint));
    setOutput("current_waypoint", static_cast<int>(feedback->current_waypoint));
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "FollowWaypointsAction 失败，错误码: " << BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_WAYPOINTS_HPP_
