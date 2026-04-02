#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rm_decision_interfaces/srv/set_sentry_posture.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <chrono>
#include <cstdint>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/node.hpp>
#include <rm_decision_interfaces/srv/detail/set_sentry_posture__struct.hpp>

class SetSentryPostureNode : public BT::SyncActionNode {
public:
    SetSentryPostureNode(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("bt_set_sentry_posture_node");
        client_ = node_->create_client<rm_decision_interfaces::srv::SetSentryPosture>("/set_sentry_posture");
    }

    // 端口
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<uint8_t>("posture"),
            BT::InputPort<bool>("override")
        };
    }

    BT::NodeStatus tick() override
    {
        uint8_t posture;
        bool override_mode;

        if (!getInput("posture", posture)) {
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput("override", override_mode)) {
            override_mode = false;
        }

        auto request = std::make_shared<rm_decision_interfaces::srv::SetSentryPosture::Request>();
        request->posture = posture;
        request->override_mode = override_mode;

        if (!client_->wait_for_service(std::chrono::seconds(2))) {
            return BT::NodeStatus::FAILURE;
        }
        auto future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future)
            != rclcpp::FutureReturnCode::SUCCESS) {
            return BT::NodeStatus::FAILURE;
        }

        auto response = future.get();
        if(response->accepted){
          return BT::NodeStatus::SUCCESS;
        }
      return BT::NodeStatus::FAILURE;
    }

    enum SentryPosture : uint8_t {
        POSTURE_ATTACK = 1,
        POSTURE_DEFENSE = 2,
        POSTURE_MOVE = 3
    };

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<rm_decision_interfaces::srv::SetSentryPosture>::SharedPtr client_;
};
