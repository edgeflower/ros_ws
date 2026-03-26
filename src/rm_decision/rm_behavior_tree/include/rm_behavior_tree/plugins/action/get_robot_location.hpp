#ifndef RM_BEHAVIOR_TREE_PLUGINS_ACTION_GET_ROBOT_LOCATION_
#define RM_BEHAVIOR_TREE_PLUGINS_ACTION_GET_ROBOT_LOCATION_

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <utility>
namespace rm_behavior_tree {
class GetRobotLocationAction : public BT::SyncActionNode {
public:
    GetRobotLocationAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::pair<float, float>>("robot_pose")
        };
    }
    BT::NodeStatus tick();

private:
    rclcpp::Node::SharedPtr node_;
    float robot_x_, robot_y_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    void updateOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
    }
};

}

#endif // RM_BEHAVIOR_TREE_PLUGINS_ACTION_GET_ROBOT_LOCATION_