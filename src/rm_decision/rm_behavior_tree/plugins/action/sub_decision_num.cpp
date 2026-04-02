#include "rm_behavior_tree/plugins/action/sub_decision_num.hpp"

/*
*至于为什么没有写
*/

namespace rm_behavior_tree {
SubDecisionNumAction::SubDecisionNumAction(
    const std::string &name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
    : BT::RosTopicSubNode<rm_decision_interfaces::msg::DecisionNum>(name, conf, params){

    }

    BT::NodeStatus  SubDecisionNumAction::onTick(const std::shared_ptr<rm_decision_interfaces::msg::DecisionNum> & last_msg){

        if(last_msg){
            RCLCPP_DEBUG(
                logger(), "[%s] new message, decision_num:%d", name().c_str(),
                last_msg->decision_num);
            setOutput("decision_num",*last_msg);
        }

        return BT::NodeStatus::SUCCESS;
    }

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubDecisionNumAction, "SubDecisonNum");