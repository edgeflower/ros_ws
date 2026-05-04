#include "rm_behavior_tree/plugins/action/sub_all_enemy_location.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <memory>
#include <rm_decision_interfaces/msg/detail/enemy_location__struct.hpp>


namespace rm_behavior_tree {
SubAllEnemyLocationAction::SubAllEnemyLocationAction(const std::string & name,const BT::NodeConfig & conf, const BT::RosNodeParams & params ) :
 BT::RosTopicSubNode<rm_decision_interfaces::msg::EnemyLocation>(name, conf, params) {

}

BT::NodeStatus SubAllEnemyLocationAction::onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::EnemyLocation> &last_msg){
        if (last_msg){
            RCLCPP_DEBUG(
            logger(), "[%s] new message,enemy's  hero_x: %f", name().c_str(), last_msg->hero_x);
        setOutput("all_enemy_location", *last_msg);
        }
        return BT::NodeStatus::SUCCESS;
    }

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubAllEnemyLocationAction, "SubAllEnemyLocation");