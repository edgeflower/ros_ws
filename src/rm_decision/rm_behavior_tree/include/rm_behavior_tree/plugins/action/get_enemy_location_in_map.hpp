#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__GET_ENEMY_LOCATION_IN_MAP_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__GET_ENEMY_LOCATION_IN_MAP_HPP_

#include "behaviortree_cpp/action_node.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rm_decision_interfaces/msg/enemy_location.hpp>
#include <rm_decision_interfaces/msg/friend_location.hpp>

namespace rm_behavior_tree
{

class GetEnemyLocationInMapAction : public BT::SyncActionNode
{
public:
    GetEnemyLocationInMapAction(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<rm_decision_interfaces::msg::EnemyLocation>("enemy_location",
                "Enemy positions from referee system"),
            BT::InputPort<rm_decision_interfaces::msg::FriendLocation>("friend_location",
                "Friend positions from referee system (need sentry_x/y)"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose",
                "Robot current position in map frame"),
            BT::OutputPort<rm_decision_interfaces::msg::EnemyLocation>("enemy_location_map",
                "Enemy positions transformed to map frame")
        };
    }

    BT::NodeStatus tick() override;

private:
    // enemy_map = robot_map + (enemy_ref - sentry_ref)
    double transform(double enemy_ref, double sentry_ref, double robot_map) const;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__GET_ENEMY_LOCATION_IN_MAP_HPP_
