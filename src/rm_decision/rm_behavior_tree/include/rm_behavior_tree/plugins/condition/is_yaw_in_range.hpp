#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_YAW_IN_RANGE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_YAW_IN_RANGE_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace rm_behavior_tree
{

class IsYawInRange : public BT::ConditionNode
{
public:
    IsYawInRange(const std::string & name, const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose", "Current robot pose"),
            BT::InputPort<double>("min_yaw_deg", "Minimum yaw angle in degrees"),
            BT::InputPort<double>("max_yaw_deg", "Maximum yaw angle in degrees")
        };
    }

    BT::NodeStatus tick() override;

private:
    static double yawFromPoseDeg(const geometry_msgs::msg::PoseStamped & pose);
    static double normalizeYawDeg(double yaw_deg);
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_YAW_IN_RANGE_HPP_
