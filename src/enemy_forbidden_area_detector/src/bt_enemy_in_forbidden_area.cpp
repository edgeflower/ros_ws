#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "enemy_forbidden_area_detector/msg/enemy_forbidden_area.hpp"

namespace enemy_forbidden_area_detector {

/**
 * @class EnemyInForbiddenAreaCondition
 * @brief BehaviorTree 条件节点：检测敌人是否在 forbidden 禁区内
 *
 * 订阅 /enemy_in_forbidden_area 话题（EnemyForbiddenArea 消息）
 * 根据 is_forbidden 字段返回 SUCCESS 或 FAILURE
 *
 * 使用方式（在 BehaviorTree XML 中）：
 * <EnemyInForbiddenArea topic="/enemy_in_forbidden_area"/>
 *
 * 无消息或 armors_num == 0 时返回 FAILURE
 */
class EnemyInForbiddenAreaCondition : public BT::ConditionNode {
 public:
  EnemyInForbiddenAreaCondition(const std::string& name,
                                 const BT::NodeConfig& config)
      : BT::ConditionNode(name, config),
        node_(std::make_shared<rclcpp::Node>("bt_enemy_in_forbidden_area")),
        last_is_forbidden_(false),
        has_message_(false) {
    // 从 BT 端口获取 topic 名称，默认 "/enemy_in_forbidden_area"
    BT::Expected<std::string> topic_param = getInput<std::string>("topic");
    std::string topic = topic_param ? topic_param.value() : "/enemy_in_forbidden_area";

    // 订阅检测结果
    sub_ = node_->create_subscription<msg::EnemyForbiddenArea>(
        topic, 10,
        [this](const msg::EnemyForbiddenArea::SharedPtr msg) {
          last_is_forbidden_ = msg->is_forbidden;
          last_armors_num_ = msg->armors_num;
          has_message_ = true;
        });

    RCLCPP_INFO(node_->get_logger(),
                "BT EnemyInForbiddenAreaCondition created, topic: %s", topic.c_str());
  }

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>(
          "topic", "/enemy_in_forbidden_area",
          "Topic name for EnemyForbiddenArea message")
    };
  }

  BT::NodeStatus tick() override {
    // 处理待执行的回调（更新订阅消息）
    rclcpp::spin_some(node_);

    // 无消息时返回 FAILURE
    if (!has_message_) {
      return BT::NodeStatus::FAILURE;
    }

    // armors_num == 0（无目标）时返回 FAILURE
    if (last_armors_num_ == 0) {
      return BT::NodeStatus::FAILURE;
    }

    // is_forbidden == true → SUCCESS，否则 FAILURE
    return last_is_forbidden_ ? BT::NodeStatus::SUCCESS
                               : BT::NodeStatus::FAILURE;
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<msg::EnemyForbiddenArea>::SharedPtr sub_;
  bool last_is_forbidden_;
  int32_t last_armors_num_ = 0;
  bool has_message_;
};

}  // namespace enemy_forbidden_area_detector

// BT 插件注册入口
extern "C" {

void registerNodes(BT::BehaviorTreeFactory& factory) {
  factory.registerNodeType<
      enemy_forbidden_area_detector::EnemyInForbiddenAreaCondition>(
      "EnemyInForbiddenArea");
}

}
