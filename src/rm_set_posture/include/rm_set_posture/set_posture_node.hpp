#ifndef RM_SET_POSTURE__SET_POSTURE_NODE_HPP_
#define RM_SET_POSTURE__SET_POSTURE_NODE_HPP_

#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rm_decision_interfaces/msg/sentry_decision.hpp"
#include "rm_decision_interfaces/srv/set_sentry_posture.hpp"

using SentryDecision = rm_decision_interfaces::msg::SentryDecision;
using SetSentryPosture = rm_decision_interfaces::srv::SetSentryPosture;

enum Posture : uint8_t
{
  POSTURE_ATTACK = 1,
  POSTURE_DEFENSE = 2,
  POSTURE_MOVE = 3,
};

class SetPostureNode : public rclcpp::Node
{
public:
  explicit SetPostureNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onDecision(const SentryDecision::SharedPtr msg);

  std::map<std::string, uint8_t> mode_to_posture_;
  uint8_t last_posture_ = 0;
  bool override_mode_;

  rclcpp::Client<SetSentryPosture>::SharedPtr client_;
  rclcpp::Subscription<SentryDecision>::SharedPtr sub_;
};

#endif  // RM_SET_POSTURE__SET_POSTURE_NODE_HPP_
