#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STUCK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STUCK_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

/**
 * @brief Condition node that detects if robot is stuck
 *
 * Compares velocity commands against actual movement to detect stuck state.
 * Returns FAILURE when stuck (velocity commands high but no movement),
 * SUCCESS otherwise.
 *
 * @param[in] cmd_vel Velocity command from blackboard
 * @param[in] robot_pose Robot pose from blackboard (from GetRobotLocation)
 * @param[in] velocity_threshold Minimum velocity to consider as "trying to move" (default: 1.0 m/s)
 * @param[in] distance_threshold Maximum distance moved to consider as "stuck" (default: 0.5 m)
 * @param[in] time_window Time window for detection (default: 3.0 s)
 */
class IsStuckAction : public BT::ConditionNode
{
public:
  IsStuckAction(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Twist>("cmd_vel"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),
      BT::InputPort<double>("velocity_threshold", 1.0, "Min velocity to trigger detection (m/s)"),
      BT::InputPort<double>("distance_threshold", 0.5, "Max distance for stuck detection (m)"),
      BT::InputPort<double>("time_window", 3.0, "Time window for detection (s)")
    };
  }

private:
  bool initialized_ = false;
  geometry_msgs::msg::Point start_position_;
  rclcpp::Time window_start_time_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STUCK_HPP_
