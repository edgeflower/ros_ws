#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "openvdb/openvdb.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_layer.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace dynamic_spatio_temporal_layer
{

class DynamicSpatioTemporalLayer
    : public spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer
{
public:
  DynamicSpatioTemporalLayer() = default;
  ~DynamicSpatioTemporalLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

private:
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void flushBuffers();

  // Parameters
  double near_decay_{3.0};
  double far_decay_{1.5};
  double min_threshold_{0.55};
  double near_threshold_{1.5};
  double far_threshold_{8.0};
  double base_min_z_{-1.0};
  double base_max_z_{2.0};
  double odom_timeout_{0.5};

  // Odom tracking
  double origin_z_{0.0};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  bool odom_valid_{false};

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  // Thread safety
  std::mutex data_mutex_;

  // Segmented point buffers (world frame, accumulated between flushes)
  std::vector<openvdb::Vec3d> near_buffer_;
  std::vector<openvdb::Vec3d> far_buffer_;
};

}  // namespace dynamic_spatio_temporal_layer
