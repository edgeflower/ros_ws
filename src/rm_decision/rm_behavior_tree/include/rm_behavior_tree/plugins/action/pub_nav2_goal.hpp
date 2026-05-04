#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_NAV2_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_NAV2_GOAL_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <thread>

namespace rm_behavior_tree
{

class PubNav2Goal : public BT::RosTopicPubNode<geometry_msgs::msg::PoseStamped>
{
public:
  PubNav2Goal(
      const std::string & name, const BT::NodeConfiguration & conf,
      const BT::RosNodeParams & params)
      : RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "导航目标位置"),
        BT::InputPort<std::string>("frame_id", "map", "坐标系ID，默认为map"),
        BT::InputPort<unsigned>("min_pub_interval_ms"),
    });
  }

  bool setMessage(geometry_msgs::msg::PoseStamped & msg) override
  {
    auto res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
    if (!res) {
      RCLCPP_ERROR(
          node_->get_logger(), "读取端口[goal_pose]时出错: %s",
          res.error().c_str());
      return false;
    }

    msg = res.value();

    if (msg.header.frame_id.empty()) {
      auto frame_res = getInput<std::string>("frame_id");
      msg.header.frame_id =
          (frame_res && !frame_res.value().empty()) ? frame_res.value() : "map";
    }

    msg.header.stamp = node_->get_clock()->now();

    RCLCPP_INFO(
        node_->get_logger(),
        "PubNav2Goal: 发布目标点 [%.2f, %.2f, %.2f] frame: %s",
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
        msg.header.frame_id.c_str());

    auto interval_res = getInput<unsigned>("min_pub_interval_ms");
    if (interval_res) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(interval_res.value()));
    }

    return true;
  }
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_NAV2_GOAL_HPP_
