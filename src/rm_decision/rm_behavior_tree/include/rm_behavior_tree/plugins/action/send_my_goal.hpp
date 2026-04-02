#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_MY_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_MY_GOAL_HPP_

#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <map>
#include <string>
#include <mutex>
#include <rclcpp/time.hpp>

namespace rm_behavior_tree
{

class SendMyGoalAction : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  SendMyGoalAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("my_goal_pose"),
      BT::InputPort<std::string>("action_name"),
      BT::InputPort<std::string>("goal_id"),
      BT::InputPort<int>("max_visits"),
      // 停滞检测参数
      BT::InputPort<double>("stall_threshold", 0.2, "距离变化阈值(m)，小于此值视为停滞"),
      BT::InputPort<double>("stall_check_interval", 2.0, "停滞检测间隔(s)"),
      BT::InputPort<double>("min_distance_for_stall", 0.5, "启用停滞检测的最小距离(m)")
    };
  }


  bool setGoal(Goal & goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  // 静态成员：跟踪所有点位的访问次数
  static std::map<std::string, int> visit_counts_;
  static std::mutex visit_counts_mutex_;

  // 重置访问计数（用于测试或重新开始）
  static void resetVisitCounts();

private:
  // 停滞检测状态
  double last_distance_{-1.0};       // 上次检测的剩余距离，-1 表示未初始化
  rclcpp::Time last_check_time_;     // 上次检测时间
  bool stall_detection_enabled_{true};

  // 停滞检测参数（从端口读取后缓存）
  double stall_threshold_{0.05};
  double stall_check_interval_{2.0};
  double min_distance_for_stall_{0.5};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_MY_GOAL_HPP_