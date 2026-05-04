#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_HPP_

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include <behaviortree_cpp/basic_types.h>

namespace rm_behavior_tree
{

/**
 * @brief Clear Costmap Action Node
 *
 * 清理 Nav2 代价地图，支持三种模式：
 * - global: 清理全局代价地图
 * - local: 清理局部代价地图
 * - around: 清理机器人周边代价地图
 *
 * 使用方法：
 * <ClearCostmap which="global"/>
 * <ClearCostmap which="local"/>
 * <ClearCostmap which="around" reset_distance="2.0"/>
 */
class ClearCostmapAction : public BT::SyncActionNode
{
public:
    ClearCostmapAction(
        const std::string & name,
        const BT::NodeConfiguration & config,
        const BT::RosNodeParams & params);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("which", "local",
                "清理目标: global / local / around"),
            BT::InputPort<double>("reset_distance", 1.0,
                "清理机器人周边半径(米)，仅 which=around 时有效"),
            BT::InputPort<int>("timeout_ms", 2000,
                "服务调用超时时间(毫秒)")
        };
    }

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;

    // 服务客户端
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_client_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_client_;
    rclcpp::Client<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_around_client_;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_HPP_
