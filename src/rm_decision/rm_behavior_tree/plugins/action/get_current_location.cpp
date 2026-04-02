#include "rm_behavior_tree/plugins/action/get_current_location.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

namespace rm_behavior_tree {
GetCurrentLocationAction::GetCurrentLocationAction(
    const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{
    node_ = std::make_shared<rclcpp::Node>("get_current_location");
    // exec_.add_node(node_);
    // std::thread([this]{exec_.spin();}).detach();
    if (!node_) {
        throw std::runtime_error("Failed to create node 'get_current_location'");
    }

    auto clock = node_->get_clock();
    tf2::Duration buffer_duration(tf2::durationFromSec(10.0));
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock, buffer_duration, node_);
    tf_buffer_->setUsingDedicatedThread(true);
    // 检测 tf 树是否转换
    if (!tf_buffer_) {
        throw std::runtime_error("Fail to create tf2_ros::Buffer ");
    }

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
    if (!tf_listener_) {
        throw std::runtime_error("Fail to create tf2_ros::transformListener");
    }

    std::thread([this]() {
        rclcpp::spin(node_);
    }).detach();
}

BT::NodeStatus GetCurrentLocationAction::tick()
{   tf2::Duration timeout = tf2::durationFromSec(1.0);
    if (!tf_buffer_->canTransform("map", "base_footprint", tf2::TimePointZero, timeout)) {
        RCLCPP_WARN(node_->get_logger(),
                    "Transform map → base_footprint 不可用，等待超时");
        return BT::NodeStatus::FAILURE;
    }
    try {
        auto tf = tf_buffer_->lookupTransform("map", "base_footprint",
            tf2::TimePointZero, tf2::durationFromSec(1.0));
            setOutput("current_location", tf);

            RCLCPP_DEBUG(
                logger_,
                "Current Location:"
                "\nTranslation:"
                "\nx: %f"
                "\ny: %f"
                "\nz: %f"
                "\nRotation:"
                "\nx: %f"
                "\ny: %f"
                "\nz: %f"
                "\nw: %f",
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w

            );

            return BT::NodeStatus::SUCCESS;

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(logger_, "TF lookup failed : %s", ex.what());
        return BT::NodeStatus::FAILURE;
    }
}

}
//    RCLCPP_ERROR(logger_, "[GetCurrentLocation] 所有备选 base 坐标查询失败,请换base坐标");
// namespace rm_behavior_tree

// 注册插件

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::GetCurrentLocationAction>("GetCurrentLocation");
}