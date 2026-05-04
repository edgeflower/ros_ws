#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_THROUGH_POSES_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_THROUGH_POSES_HPP_

#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

#include "rm_behavior_tree/plugins/action/waypoint_utils.hpp"

namespace rm_behavior_tree
{

class NavigateThroughPosesAction
    : public BT::RosActionNode<nav2_msgs::action::NavigateThroughPoses>
{
public:
  NavigateThroughPosesAction(
      const std::string & name, const BT::NodeConfiguration & conf,
      const BT::RosNodeParams & params)
      : RosActionNode<nav2_msgs::action::NavigateThroughPoses>(name, conf, params),
        total_waypoints_(0)
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
          "NavigateThroughPosesAction: 航点列表为空，取消发送 goal");
      return false;
    }

    rclcpp::Time now = rclcpp::Clock().now();
    for (auto & wp : waypoints) {
      if (wp.header.frame_id.empty()) wp.header.frame_id = frame_id;
      wp.header.stamp = now;
    }

    total_waypoints_ = static_cast<int>(waypoints.size());
    goal.poses = waypoints;

    RCLCPP_INFO(
        node_->get_logger(),
        "NavigateThroughPosesAction: 已加载 %d 个航点，准备穿越导航",
        total_waypoints_);
    for (size_t i = 0; i < waypoints.size(); ++i) {
      RCLCPP_INFO(
          node_->get_logger(), "  航点[%zu]: [%.2f, %.2f]", i,
          waypoints[i].pose.position.x, waypoints[i].pose.position.y);
    }
    return true;
  }

  void onHalt() override
  {
    RCLCPP_WARN(node_->get_logger(), "NavigateThroughPosesAction: 穿越导航已被中断");
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    RCLCPP_INFO(node_->get_logger(), "NavigateThroughPosesAction: 穿越导航完成!");
    if (wr.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(
          node_->get_logger(), "  穿越导航未成功，结果码: %d",
          static_cast<int>(wr.code));
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "  所有航点均已成功穿越");
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFeedback(
      const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback>
          feedback) override
  {
    int remaining = feedback->number_of_poses_remaining;
    int current = total_waypoints_ - remaining;
    RCLCPP_DEBUG(
        node_->get_logger(),
        "NavigateThroughPosesAction: 正在前往航点 %d/%d", current,
        total_waypoints_);
    setOutput("current_waypoint", current);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "NavigateThroughPosesAction 失败，错误码: " << BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }

private:
  int total_waypoints_;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_THROUGH_POSES_HPP_
