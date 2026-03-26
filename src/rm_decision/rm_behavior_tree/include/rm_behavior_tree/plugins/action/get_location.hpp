#ifndef RM_BEHAVIOR_TREE__PLUGINS__BEHAVIORS__GET_LOCATION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__BEHAVIORS__GET_LOCATION_HPP_

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace rm_behavior_tree {

    class GetLocationAction : public BT::RosTopicSubNode<nav_msgs::msg::Odometry>
    {
        public:
        GetLocationAction(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

        static BT::PortsList providedPorts(){
            return {
                BT::InputPort<std::string>("topic_name"),
                BT::OutputPort<nav_msgs::msg::Odometry>("robot_location")};
        }

        BT::NodeStatus onTick(const std::shared_ptr<nav_msgs::msg::Odometry> &last_msg) override;
    };

}  // namespace rm_behavior_tree   


#endif // RM_BEHAVIOR_TREE__PLUGINS__BEHAVIORS__GET_LOCATION_HPP_