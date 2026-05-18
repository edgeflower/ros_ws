#include "rm_set_posture/set_posture_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

SetPostureNode::SetPostureNode(const rclcpp::NodeOptions & options)
: Node("set_posture_node", options)
{
  mode_to_posture_ = {
    {"ATTACK",      POSTURE_ATTACK},
    {"RETREAT",     POSTURE_MOVE},
    {"SUPPLY",      POSTURE_MOVE},
    {"PATROL",      POSTURE_MOVE},
    {"DEFEND_BASE", POSTURE_DEFENSE},
    {"IDLE",        POSTURE_DEFENSE},
  };

  declare_parameter("service_name", std::string("/set_sentry_posture"));
  declare_parameter("override_mode", false);

  std::string service_name = get_parameter("service_name").as_string();
  override_mode_ = get_parameter("override_mode").as_bool();

  client_ = create_client<SetSentryPosture>(service_name);

  sub_ = create_subscription<SentryDecision>(
    "/sentry_decision", rclcpp::QoS(10),
    [this](const SentryDecision::SharedPtr msg) {
      onDecision(msg);
    });

  RCLCPP_INFO(get_logger(), "SetPostureNode 已启动，订阅 /sentry_decision，服务: %s",
              service_name.c_str());
}

void SetPostureNode::onDecision(const SentryDecision::SharedPtr msg)
{
  auto it = mode_to_posture_.find(msg->mode);
  if (it == mode_to_posture_.end()) {
    RCLCPP_WARN(get_logger(), "未知模式: '%s'，跳过", msg->mode.c_str());
    return;
  }

  uint8_t posture = it->second;

  if (posture == last_posture_) {
    return;
  }

  if (!client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "服务 %s 未就绪，等待中...", client_->get_service_name());
    return;
  }

  auto request = std::make_shared<SetSentryPosture::Request>();
  request->posture = posture;
  request->override_mode = override_mode_;

  RCLCPP_INFO(get_logger(), "模式切换: %s → 姿态 %d，发送服务请求",
              msg->mode.c_str(), static_cast<int>(posture));

  client_->async_send_request(request,
    [this, mode = msg->mode, posture](rclcpp::Client<SetSentryPosture>::SharedFuture future) {
      auto response = future.get();
      if (response->accepted) {
        last_posture_ = posture;
        RCLCPP_INFO(get_logger(), "姿态切换成功: %s → %d", mode.c_str(), static_cast<int>(posture));
      } else {
        RCLCPP_WARN(get_logger(), "姿态切换被拒绝: %s (原因: %s)",
                    mode.c_str(), response->message.c_str());
      }
    });
}

RCLCPP_COMPONENTS_REGISTER_NODE(SetPostureNode)
