#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_BEST_ARMOR_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_BEST_ARMOR_HPP_

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "armor_interfaces/msg/armors.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace rm_behavior_tree
{
    /**
    * @brief 追击敌人
    *
    * @param[in] message 地方机器人位置信息
    */

    class PickBestArmorAction : public BT::SyncActionNode
    {
        public:
        PickBestArmorAction(const std::string & name, const BT::NodeConfig & confing);

        static BT::PortsList providedPorts()
        {
            return 
            {
                BT::InputPort<armor_interfaces::msg::Armors>("armors_message"),
                BT::OutputPort<armor_interfaces::msg::Armor>("armor_message")
            };
        }
        BT::NodeStatus tick() override;
        private:
        armor_interfaces::msg::Armors armors;
    };
}

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_BEST_ARMOR_HPP_