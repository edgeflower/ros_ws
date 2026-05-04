#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__WAYPOINT_UTILS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__WAYPOINT_UTILS_HPP_

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

inline geometry_msgs::msg::PoseStamped loadPoseStamped(
    const rclcpp::Node::SharedPtr & node, const std::string & prefix)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = node->get_parameter(prefix + ".frame_id").as_string();
  pose.pose.position.x = node->get_parameter(prefix + ".position.x").as_double();
  pose.pose.position.y = node->get_parameter(prefix + ".position.y").as_double();
  pose.pose.position.z = node->get_parameter(prefix + ".position.z").as_double();
  pose.pose.orientation.w = node->get_parameter(prefix + ".orientation.w").as_double();
  return pose;
}

// CSV format: id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,wait_sec
// wait_sec is optional, defaults to 0
inline bool loadWaypointsFromCSV(
    const std::string & filename,
    std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
    const std::string & frame_id = "map",
    std::vector<double> * wait_times = nullptr)
{
  std::ifstream file(filename);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  std::getline(file, line); // skip header

  while (std::getline(file, line)) {
    if (line.empty()) continue;

    std::istringstream ss(line);
    std::string token;
    std::vector<std::string> tokens;

    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }

    if (tokens.size() < 8) continue;

    try {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame_id;
      pose.header.stamp = rclcpp::Time(0);

      pose.pose.position.x = std::stod(tokens[1]);
      pose.pose.position.y = std::stod(tokens[2]);
      pose.pose.position.z = std::stod(tokens[3]);
      pose.pose.orientation.x = std::stod(tokens[4]);
      pose.pose.orientation.y = std::stod(tokens[5]);
      pose.pose.orientation.z = std::stod(tokens[6]);
      pose.pose.orientation.w = std::stod(tokens[7]);

      waypoints.push_back(pose);

      if (wait_times) {
        double wt = 0.0;
        if (tokens.size() > 9 && !tokens[9].empty()) {
          wt = std::stod(tokens[9]);
        }
        wait_times->push_back(wt);
      }
    } catch (const std::exception &) {
      continue;
    }
  }

  file.close();
  return !waypoints.empty();
}

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__WAYPOINT_UTILS_HPP_
