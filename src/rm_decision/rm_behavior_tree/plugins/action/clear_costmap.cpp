#include "rm_behavior_tree/plugins/action/clear_costmap.hpp"
#include <algorithm>
#include <chrono>

namespace rm_behavior_tree
{

ClearCostmapAction::ClearCostmapAction(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config)
{
    node_ = params.nh;

    // 创建服务客户端
    clear_global_client_ = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "/global_costmap/clear_entirely_global_costmap");

    clear_local_client_ = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "/local_costmap/clear_entirely_local_costmap");

    clear_around_client_ = node_->create_client<nav2_msgs::srv::ClearCostmapAroundRobot>(
        "/local_costmap/clear_around_robot_costmap");

    RCLCPP_INFO(node_->get_logger(),
               "[ClearCostmap] Initialized with services: global, local, around");
}

BT::NodeStatus ClearCostmapAction::tick()
{
    // 获取参数
    std::string which;
    if (!getInput<std::string>("which", which)) {
        RCLCPP_ERROR(node_->get_logger(), "[ClearCostmap] Missing 'which' port");
        return BT::NodeStatus::FAILURE;
    }

    // 转换为小写
    std::transform(which.begin(), which.end(), which.begin(), ::tolower);

    double reset_distance = 1.0; // 清除半径
    getInput<double>("reset_distance", reset_distance);

    int timeout_ms = 2000;
    getInput<int>("timeout_ms", timeout_ms);

    RCLCPP_INFO(node_->get_logger(), "[ClearCostmap] Clearing '%s' costmap...", which.c_str());

    if (which == "global") {
        // 清理全局代价地图
        if (!clear_global_client_->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(node_->get_logger(),
                       "[ClearCostmap] Global costmap service not available");
            return BT::NodeStatus::FAILURE;
        }

        auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
        auto future = clear_global_client_->async_send_request(request);

        auto status = future.wait_for(std::chrono::milliseconds(timeout_ms));
        if (status == std::future_status::ready) {
            RCLCPP_INFO(node_->get_logger(), "[ClearCostmap] Global costmap cleared");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(), "[ClearCostmap] Global clear timeout");
            return BT::NodeStatus::FAILURE;
        }

    } else if (which == "local") {
        // 清理局部代价地图
        if (!clear_local_client_->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(node_->get_logger(),
                       "[ClearCostmap] Local costmap service not available");
            return BT::NodeStatus::FAILURE;
        }

        auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
        auto future = clear_local_client_->async_send_request(request);

        auto status = future.wait_for(std::chrono::milliseconds(timeout_ms));
        if (status == std::future_status::ready) {
            RCLCPP_INFO(node_->get_logger(), "[ClearCostmap] Local costmap cleared");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(), "[ClearCostmap] Local clear timeout");
            return BT::NodeStatus::FAILURE;
        }

    } else if (which == "around") {
        // 清理机器人周边代价地图
        if (!clear_around_client_->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(node_->get_logger(),
                       "[ClearCostmap] Around-robot service not available");
            return BT::NodeStatus::FAILURE;
        }

        auto request = std::make_shared<nav2_msgs::srv::ClearCostmapAroundRobot::Request>();
        request->reset_distance = reset_distance;
        auto future = clear_around_client_->async_send_request(request);

        auto status = future.wait_for(std::chrono::milliseconds(timeout_ms));
        if (status == std::future_status::ready) {
            RCLCPP_INFO(node_->get_logger(),
                       "[ClearCostmap] Around-robot costmap cleared (radius=%.2fm)", reset_distance);
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(), "[ClearCostmap] Around clear timeout");
            return BT::NodeStatus::FAILURE;
        }

    } else {
        RCLCPP_ERROR(node_->get_logger(),
                    "[ClearCostmap] Invalid 'which' value: '%s'. Must be global/local/around",
                    which.c_str());
        return BT::NodeStatus::FAILURE;
    }
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::ClearCostmapAction, "ClearCostmap");
