#include "rm_behavior_tree/plugins/action/get_enemy_location_in_map.hpp"
#include <rclcpp/logging.hpp>

namespace rm_behavior_tree
{

GetEnemyLocationInMapAction::GetEnemyLocationInMapAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{}

double GetEnemyLocationInMapAction::transform(
    double enemy_ref, double sentry_ref, double robot_map) const
{
    return robot_map + (enemy_ref - sentry_ref);
}

BT::NodeStatus GetEnemyLocationInMapAction::tick()
{
    auto enemy_res = getInput<rm_decision_interfaces::msg::EnemyLocation>("enemy_location");
    auto friend_res = getInput<rm_decision_interfaces::msg::FriendLocation>("friend_location");
    auto robot_res = getInput<geometry_msgs::msg::PoseStamped>("robot_pose");

    if (!enemy_res.has_value() || !friend_res.has_value() || !robot_res.has_value()) {
        return BT::NodeStatus::FAILURE;
    }

    const auto &enemy = enemy_res.value();
    const auto &friend_loc = friend_res.value();
    double rx = robot_res.value().pose.position.x;
    double ry = robot_res.value().pose.position.y;

    double sx = friend_loc.sentry_x;
    double sy = friend_loc.sentry_y;

    rm_decision_interfaces::msg::EnemyLocation out;
    out.hero_x      = transform(enemy.hero_x, sx, rx);
    out.hero_y      = transform(enemy.hero_y, sy, ry);
    out.engineer_x  = transform(enemy.engineer_x, sx, rx);
    out.engineer_y  = transform(enemy.engineer_y, sy, ry);
    out.standard_3_x = transform(enemy.standard_3_x, sx, rx);
    out.standard_3_y = transform(enemy.standard_3_y, sy, ry);
    out.standard_4_x = transform(enemy.standard_4_x, sx, rx);
    out.standard_4_y = transform(enemy.standard_4_y, sy, ry);
    out.sentry_x    = transform(enemy.sentry_x, sx, rx);
    out.sentry_y    = transform(enemy.sentry_y, sy, ry);

    setOutput("enemy_location_map", out);
    return BT::NodeStatus::SUCCESS;
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::GetEnemyLocationInMapAction>("GetEnemyLocationInMap");
}
