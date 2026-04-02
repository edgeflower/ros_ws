#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_GOAL_HPP_

#include <behaviortree_cpp/basic_types.h>
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <sstream>
#include <vector>

namespace rm_behavior_tree
{

class SetGoalAction : public BT::SyncActionNode
{
public:
    SetGoalAction(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("goal_string", "目标位置字符串，格式: x;y;yaw;... (前7个值)"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "解析后的目标点位姿")
        };
    }

    BT::NodeStatus tick() override;

private:
    bool parseGoalString(const std::string& str, geometry_msgs::msg::PoseStamped& pose);
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SET_GOAL_HPP_
