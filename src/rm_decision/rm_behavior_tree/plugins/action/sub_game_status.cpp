#include "rm_behavior_tree/plugins/action/sub_game_status.hpp"

namespace rm_behavior_tree
{

SubGameStatusAction::SubGameStatusAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
    : BT::RosTopicSubNode<rm_decision_interfaces::msg::GameStatus>(name, conf, params){
    }
BT::NodeStatus SubGameStatusAction::onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::GameStatus> & last_msg
){
    if (last_msg)
    {
        RCLCPP_DEBUG(logger(), "[%s] new message, game_progress: %d",
             name().c_str(),
             last_msg->game_progress);

        setOutput("game_status", *last_msg);
    }
    return BT::NodeStatus::SUCCESS;
}
} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubGameStatusAction, "SubGameStatus");