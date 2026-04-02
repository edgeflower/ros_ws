#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__CALCULATE_ANGLE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__CALCULATE_ANGLE_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <behaviortree_cpp/basic_types.h>

namespace rm_behavior_tree
{

class CalculateAngleAction : public BT::RosTopicPubNode<std_msgs::msg::Float32>
{
public:
    CalculateAngleAction(
        const std::string & name,
        const BT::NodeConfig & conf,
        const BT::RosNodeParams & params);

    static BT::PortsList providedPorts()
    {
        return BT::RosTopicPubNode<std_msgs::msg::Float32>::providedBasicPorts({
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "目标点位姿"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose", "机器人位姿")
        });
    }

    bool setMessage(std_msgs::msg::Float32 & msg) override;

private:
    double calculateAngle(
        const geometry_msgs::msg::PoseStamped& goal,
        const geometry_msgs::msg::PoseStamped& robot);

    geometry_msgs::msg::PoseStamped goal_pose_;
    geometry_msgs::msg::PoseStamped robot_pose_;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__CALCULATE_ANGLE_HPP_
