#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__WRITE_TO_BLACKBOARD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__WRITE_TO_BLACKBOARD_HPP_

#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include <rm_interfaces/msg/serial_receive_data.hpp>

namespace rm_behavior_tree
{

class WriteToBlackboard : public BT::SyncActionNode
{
public:
  WriteToBlackboard(const std::string & name, const BT::NodeConfiguration & config)
      : BT::SyncActionNode(name, config)
  {
    global_node_ = rclcpp::Node::make_shared("WriteToBlackboard");
    sub_ = global_node_->create_subscription<rm_interfaces::msg::SerialReceiveData>(
        "/SerialReceiveData", 10,
        std::bind(&WriteToBlackboard::callback, this, std::placeholders::_1));
    is_ReadInterface_ = false;
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::OutputPort<float>("Hp"),
        BT::OutputPort<bool>("Zone_status"),
        BT::OutputPort<bool>("Is_defence"),
        BT::OutputPort<bool>("Is_attack"),
        BT::OutputPort<bool>("Self_status"),
        BT::OutputPort<bool>("Is_recover"),
    };
  }

  BT::NodeStatus tick() override
  {
    rclcpp::spin_some(global_node_);
    if (!is_ReadInterface_) return BT::NodeStatus::FAILURE;

    setOutput("Hp", hp_);
    setOutput("Zone_status", zone_status_);
    setOutput("Is_defence", is_defence_);
    setOutput("Is_attack", is_attack_);
    setOutput("Self_status", self_status_);
    setOutput("Is_recover", is_recover_);

    return BT::NodeStatus::SUCCESS;
  }

private:
  void callback(const rm_interfaces::msg::SerialReceiveData::SharedPtr msg)
  {
    hp_ = msg->judge_system_data.hp;
    zone_status_ = msg->judge_system_data.zone_status;
    is_defence_ = msg->judge_system_data.is_defence;
    is_attack_ = msg->judge_system_data.is_attack;
    self_status_ = msg->judge_system_data.self_status;
    is_recover_ = msg->judge_system_data.is_recover;

    is_ReadInterface_ = true;
    RCLCPP_INFO(global_node_->get_logger(), "Callback hp = %f", hp_);
  }

  std::shared_ptr<rclcpp::Node> global_node_;
  rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr sub_;
  bool is_ReadInterface_;

  float hp_ = 400;
  bool zone_status_ = false;
  bool is_defence_ = false;
  bool is_attack_ = false;
  bool self_status_ = false;
  bool is_recover_ = false;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__WRITE_TO_BLACKBOARD_HPP_
