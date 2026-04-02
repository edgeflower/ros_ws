#ifndef RM_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
#define RM_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_

#include "behaviortree_cpp/basic_types.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <behaviortree_cpp/exceptions.h>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>

namespace BT{
/**
* @brief parse XML string to gometry_msgs::msg::PoseStamped
* @param key XML string
* @return geometry_msg::msg::PoseStamped
*/ 
template <>
inline geometry_msgs::msg::PoseStamped convertFromString(const StringView key){


    auto parts = splitString(key,';');
    if(parts.size()!= 7){
        throw RuntimeError("invalid input ");

    }else{
        geometry_msgs::msg::PoseStamped msg;
        msg.pose.position.x = convertFromString<double>(parts[0]);
        msg.pose.position.y = convertFromString<double>(parts[1]);
        msg.pose.position.z = convertFromString<double>(parts[2]);

        msg.pose.orientation.x = convertFromString<double>(parts[3]);
        msg.pose.orientation.y = convertFromString<double>(parts[4]);
        msg.pose.orientation.z = convertFromString<double>(parts[5]);
        msg.pose.orientation.w = convertFromString<double>(parts[6]);
        return msg;

    }

}






}
#endif // RM_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_