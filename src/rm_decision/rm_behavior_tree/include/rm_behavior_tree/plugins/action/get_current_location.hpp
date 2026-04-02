#ifndef RM_BEHAVIOR_TREE_PLUGINS_ACTION_GET_CURRENT_LOCATION_HPP_
#define RM_BEHAVIOR_TREE_PLUGINS_ACTION_GET_CURRENT_LOCATION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace rm_behavior_tree
{

class GetCurrentLocationAction : public BT::SyncActionNode
{
public:
    GetCurrentLocationAction(const std::string &name,const BT::NodeConfig & config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_location")};
    }


private:
    rclcpp::Node::SharedPtr node_;
    //rclcpp::executors::MultiThreadedExecutor exec_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Logger logger_{rclcpp::get_logger("GetCurrentLocationAction")};
    std::vector<std::string> base_frames_{
        "odom","chassis", "base_footprint", "gimbal_big", "gimbal_yaw_fake"};
    std::string global_frame_{"map"};

};
}

#endif