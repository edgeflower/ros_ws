#include "dynamic_spatio_temporal_layer/dynamic_spatio_temporal_layer.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <nav2_util/node_utils.hpp>
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace dynamic_spatio_temporal_layer
{

void DynamicSpatioTemporalLayer::onInitialize()
{
  // Initialize parent STVL (creates voxel grid, TF, logger, etc.)
  SpatioTemporalVoxelLayer::onInitialize();

  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "Failed to lock node for dynamic layer initialization");
    return;
  }

  // Declare and read parameters
  nav2_util::declare_parameter_if_not_declared(
    node.get(), getName() + ".near_decay", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), getName() + ".far_decay", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), getName() + ".min_threshold", rclcpp::ParameterValue(0.55));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), getName() + ".near_threshold", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), getName() + ".far_threshold", rclcpp::ParameterValue(8.0));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), getName() + ".base_min_z", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), getName() + ".base_max_z", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), getName() + ".odom_timeout", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), getName() + ".odom_topic", rclcpp::ParameterValue(std::string("/odin1/odometry_highfreq")));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), getName() + ".pointcloud_topic", rclcpp::ParameterValue(std::string("/livox/lidar")));

  node->get_parameter(getName() + ".near_decay", near_decay_);
  node->get_parameter(getName() + ".far_decay", far_decay_);
  node->get_parameter(getName() + ".min_threshold", min_threshold_);
  node->get_parameter(getName() + ".near_threshold", near_threshold_);
  node->get_parameter(getName() + ".far_threshold", far_threshold_);
  node->get_parameter(getName() + ".base_min_z", base_min_z_);
  node->get_parameter(getName() + ".base_max_z", base_max_z_);
  node->get_parameter(getName() + ".odom_timeout", odom_timeout_);

  std::string odom_topic, cloud_topic;
  node->get_parameter(getName() + ".odom_topic", odom_topic);
  node->get_parameter(getName() + ".pointcloud_topic", cloud_topic);

  // Create subscriptions
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&DynamicSpatioTemporalLayer::odomCallback, this, std::placeholders::_1));

  rclcpp::SubscriptionOptions cloud_opts;
  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic, rclcpp::SensorDataQoS().best_effort(),
    std::bind(&DynamicSpatioTemporalLayer::pointcloudCallback, this, std::placeholders::_1),
    cloud_opts);

  // Pre-allocate buffers
  near_buffer_.reserve(40000);
  far_buffer_.reserve(40000);

  RCLCPP_INFO(
    logger_,
    "DynamicSpatioTemporalLayer initialized: near_decay=%.1f far_decay=%.1f "
    "min_thresh=%.2f near_thresh=%.1f far_thresh=%.1f z=[%.2f,%.2f] odom_timeout=%.1f",
    near_decay_, far_decay_, min_threshold_, near_threshold_, far_threshold_,
    base_min_z_, base_max_z_, odom_timeout_);
}

void DynamicSpatioTemporalLayer::odomCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  origin_z_ = msg->pose.pose.position.z;
  last_odom_time_ = msg->header.stamp;
  odom_valid_ = true;
}

void DynamicSpatioTemporalLayer::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // Check odom validity
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!odom_valid_) {
      return;
    }
    double dt = (rclcpp::Time(msg->header.stamp) - last_odom_time_).seconds();
    if (std::abs(dt) > odom_timeout_) {
      return;
    }
  }

  // Use latest available TF to avoid extrapolation errors with Livox timestamps
  const tf2::TimePoint tf_time = tf2::TimePointZero;

  // TF: base_footprint <- sensor frame
  Eigen::Affine3d T_base_lidar;
  try {
    auto tf_msg = tf_->lookupTransform(
      "base_footprint", msg->header.frame_id, tf_time);
    T_base_lidar = tf2::transformToEigen(tf_msg);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "TF base_footprint lookup failed: %s", ex.what());
    return;
  }

  // TF: odom <- sensor frame
  Eigen::Affine3d T_odom_lidar;
  try {
    auto tf_msg = tf_->lookupTransform(
      "odom", msg->header.frame_id, tf_time);
    T_odom_lidar = tf2::transformToEigen(tf_msg);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "TF odom lookup failed: %s", ex.what());
    return;
  }

  // Current Z range
  double cur_min_z, cur_max_z, cur_origin_z;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cur_origin_z = origin_z_;
  }
  cur_min_z = base_min_z_ + cur_origin_z;
  cur_max_z = base_max_z_ + cur_origin_z;

  const double min_thresh_sq = min_threshold_ * min_threshold_;
  const double near_thresh_sq = near_threshold_ * near_threshold_;
  const double far_thresh_sq = far_threshold_ * far_threshold_;

  std::vector<openvdb::Vec3d> near_pts, far_pts;
  near_pts.reserve(20000);
  far_pts.reserve(20000);

  // Iterate point cloud
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float lx = *iter_x, ly = *iter_y, lz = *iter_z;
    if (!std::isfinite(lx) || !std::isfinite(ly) || !std::isfinite(lz)) {
      continue;
    }

    Eigen::Vector3d p_lidar(lx, ly, lz);

    // Distance check in base frame
    Eigen::Vector3d p_base = T_base_lidar * p_lidar;
    double d_sq = p_base.x() * p_base.x() + p_base.y() * p_base.y();
    if (d_sq < min_thresh_sq || d_sq > far_thresh_sq) {
      continue;
    }

    // Z filter in odom frame
    Eigen::Vector3d p_odom = T_odom_lidar * p_lidar;
    if (p_odom.z() < cur_min_z || p_odom.z() > cur_max_z) {
      continue;
    }

    // Segment by distance
    if (d_sq < near_thresh_sq) {
      near_pts.emplace_back(p_odom.x(), p_odom.y(), p_odom.z());
    } else {
      far_pts.emplace_back(p_odom.x(), p_odom.y(), p_odom.z());
    }
  }

  // Append to buffers
  std::lock_guard<std::mutex> lock(data_mutex_);
  near_buffer_.insert(near_buffer_.end(), near_pts.begin(), near_pts.end());
  far_buffer_.insert(far_buffer_.end(), far_pts.begin(), far_pts.end());
}

void DynamicSpatioTemporalLayer::flushBuffers()
{
  std::vector<openvdb::Vec3d> near_pts, far_pts;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::swap(near_pts, near_buffer_);
    std::swap(far_pts, far_buffer_);
  }

  if (near_pts.empty() && far_pts.empty()) {
    return;
  }

  auto * grid = getVoxelGrid();
  if (!grid) {
    return;
  }

  const double now_sec = clock_->now().seconds();
  const double delta = near_decay_ - far_decay_;

  // Mark far points first (shorter lifetime)
  if (!far_pts.empty()) {
    grid->MarkRawPoints(far_pts, now_sec - delta);
  }
  // Mark near points second (longer lifetime, overwrites shared voxels)
  if (!near_pts.empty()) {
    grid->MarkRawPoints(near_pts, now_sec);
  }
}

void DynamicSpatioTemporalLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  flushBuffers();
  SpatioTemporalVoxelLayer::updateBounds(robot_x, robot_y, robot_yaw,
    min_x, min_y, max_x, max_y);
}

void DynamicSpatioTemporalLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  SpatioTemporalVoxelLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace dynamic_spatio_temporal_layer

PLUGINLIB_EXPORT_CLASS(
  dynamic_spatio_temporal_layer::DynamicSpatioTemporalLayer,
  nav2_costmap_2d::Layer)
